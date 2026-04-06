/*
   AP_IntegrityFilter — autonomous GPS spoof detection via velocity cross-check.
   Compares GPS velocity against EKF (IMU-fused) velocity.
   Trust score 0.0-1.0 with escalating response levels.
*/

#include "Plane.h"
#include "AP_IntegrityFilter.h"
#include <AP_GPS/AP_GPS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

const AP_Param::GroupInfo AP_IntegrityFilter::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Integrity filter enable
    // @Description: Enable GPS integrity velocity cross-check filter
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("ENABLE", 1, AP_IntegrityFilter, _enable, 0),

    // @Param: VEL_THR
    // @DisplayName: Velocity divergence threshold
    // @Description: GPS vs EKF horizontal velocity difference threshold to start trust decay (m/s)
    // @Range: 1.0 10.0
    // @Units: m/s
    // @User: Advanced
    AP_GROUPINFO("VEL_THR", 2, AP_IntegrityFilter, _vel_thresh, 3.0f),

    // @Param: DECAY
    // @DisplayName: Trust decay rate
    // @Description: Trust score decrease per second when velocity divergence exceeds threshold
    // @Range: 0.01 1.0
    // @Units: 1/s
    // @User: Advanced
    AP_GROUPINFO("DECAY", 3, AP_IntegrityFilter, _decay_rate, 0.1f),

    // @Param: RECOVER
    // @DisplayName: Trust recovery rate
    // @Description: Trust score increase per second when velocity agrees
    // @Range: 0.005 0.5
    // @Units: 1/s
    // @User: Advanced
    AP_GROUPINFO("RECOVER", 4, AP_IntegrityFilter, _recover_rate, 0.02f),

    // @Param: CAUT_THR
    // @DisplayName: Caution threshold
    // @Description: Trust score below which CAUTION level is triggered
    // @Range: 0.3 0.9
    // @User: Advanced
    AP_GROUPINFO("CAUT_THR", 5, AP_IntegrityFilter, _caution_thresh, 0.7f),

    // @Param: WARN_THR
    // @DisplayName: Warning threshold
    // @Description: Trust score below which WARNING level is triggered (GPS disabled)
    // @Range: 0.1 0.6
    // @User: Advanced
    AP_GROUPINFO("WARN_THR", 6, AP_IntegrityFilter, _warn_thresh, 0.4f),

    // @Param: EMER_THR
    // @DisplayName: Emergency threshold
    // @Description: Trust score below which EMERGENCY level is triggered (full lockdown)
    // @Range: 0.05 0.3
    // @User: Advanced
    AP_GROUPINFO("EMER_THR", 7, AP_IntegrityFilter, _emerg_thresh, 0.15f),

    AP_GROUPEND
};

AP_IntegrityFilter::AP_IntegrityFilter()
{
    AP_Param::setup_object_defaults(this, var_info);
}

float AP_IntegrityFilter::compute_velocity_divergence() const
{
    const auto &gps = AP::gps();
    const auto &ahrs = AP::ahrs();

    // need at least 3D fix
    if (gps.status() < AP_GPS::GPS_OK_FIX_3D) {
        return 0.0f;
    }

    float max_div = 0.0f;

    // 1. Velocity cross-check: GPS velocity vs EKF velocity
    const Vector3f &gps_vel = gps.velocity();
    Vector3f ekf_vel;
    if (ahrs.get_velocity_NED(ekf_vel)) {
        Vector2f vel_diff(gps_vel.x - ekf_vel.x, gps_vel.y - ekf_vel.y);
        max_div = MAX(max_div, vel_diff.length());
    }

    // 2. Position cross-check: GPS position vs EKF position
    //    Convert both to NE offset from home, compare distance
    Location gps_loc = gps.location();
    Location ekf_loc;
    if (ahrs.get_location(ekf_loc)) {
        Vector2f pos_diff = gps_loc.get_distance_NE(ekf_loc);
        float pos_div_m = pos_diff.length();
        // scale position divergence to equivalent velocity metric
        // 50m offset → equivalent to 5 m/s velocity divergence
        max_div = MAX(max_div, pos_div_m * 0.1f);
    }

    // 3. EKF health check — if EKF is unhealthy, treat as divergence
    if (!ahrs.healthy()) {
        max_div = MAX(max_div, _vel_thresh + 1.0f);
    }

    return max_div;
}

AP_IntegrityFilter::Level AP_IntegrityFilter::compute_level(float trust) const
{
    if (trust < _emerg_thresh) {
        return Level::EMERGENCY;
    }
    if (trust < _warn_thresh) {
        return Level::WARNING;
    }
    if (trust < _caution_thresh) {
        return Level::CAUTION;
    }
    return Level::NOMINAL;
}

void AP_IntegrityFilter::execute_level_change(Level new_level, Level old_level)
{
    // escalating up
    if (new_level > old_level) {
        switch (new_level) {
        case Level::CAUTION:
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "INTEG: CAUTION trust=%.0f%% div=%.1fm/s",
                _trust * 100.0f, _last_divergence);
            break;

        case Level::WARNING:
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "INTEG: WARNING — GPS DISABLED trust=%.0f%%",
                _trust * 100.0f);
            AP::gps().force_disable(true);
            break;

        case Level::EMERGENCY:
            GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "INTEG: EMERGENCY — FULL LOCKDOWN trust=%.0f%%",
                _trust * 100.0f);
            plane.engage_spoof_emergency();
            break;

        case Level::NOMINAL:
            break;
        }
    }
    // de-escalating down
    else {
        switch (old_level) {
        case Level::EMERGENCY:
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "INTEG: recovering from EMERGENCY trust=%.0f%%",
                _trust * 100.0f);
            plane.disengage_spoof_emergency();
            break;

        case Level::WARNING:
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "INTEG: recovering from WARNING — GPS ENABLED trust=%.0f%%",
                _trust * 100.0f);
            AP::gps().force_disable(false);
            break;

        case Level::CAUTION:
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "INTEG: NOMINAL trust=%.0f%%", _trust * 100.0f);
            break;

        case Level::NOMINAL:
            break;
        }
    }
}

void AP_IntegrityFilter::update()
{
    if (!_enable) {
        return;
    }

    const uint32_t now = AP_HAL::millis();
    const float dt = (now - _last_update_ms) * 0.001f;
    if (dt <= 0 || dt > 1.0f) {
        _last_update_ms = now;
        return;
    }
    _last_update_ms = now;

    // skip if not armed — reset state
    if (!hal.util->get_soft_armed()) {
        _trust = 1.0f;
        _level = Level::NOMINAL;
        _pending_level = Level::NOMINAL;
        _arm_time_ms = 0;
        return;
    }

    // record arm time, skip first 30s to let EKF settle
    if (_arm_time_ms == 0) {
        _arm_time_ms = now;
    }
    if (now - _arm_time_ms < 30000) {
        return;
    }

    // skip if pilot has manual RC override active (not auto-triggered)
    if (plane._spoof_override_active && _level < Level::EMERGENCY) {
        return;
    }

    // compute divergence
    _last_divergence = compute_velocity_divergence();

    // update trust score
    if (_last_divergence > _vel_thresh) {
        _trust -= _decay_rate * dt;
    } else if (_level < Level::WARNING) {
        // only recover trust if we haven't disabled GPS yet
        // once in WARNING+, trust stays low — recovery requires pilot RC override
        // or future GPS_RAW velocity monitoring
        _trust += _recover_rate * dt;
    }
    // in WARNING+: trust slowly decays toward EMERGENCY even without divergence data
    if (_level >= Level::WARNING) {
        _trust -= _decay_rate * 0.1f * dt;  // slow decay: 10x slower than active divergence
    }
    _trust = constrain_float(_trust, 0.0f, 1.0f);

    // determine target level
    Level target = compute_level(_trust);

    // hysteresis: require level to be sustained for 2 seconds
    if (target != _level) {
        if (target != _pending_level) {
            _pending_level = target;
            _pending_level_start_ms = now;
        } else if (now - _pending_level_start_ms > 2000) {
            execute_level_change(target, _level);
            _level = target;
        }
    } else {
        _pending_level = _level;
    }

    // periodic status every 5 seconds (always when enabled, for debugging)
    static uint32_t last_status_ms;
    if (now - last_status_ms > 5000) {
        last_status_ms = now;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "INTEG: trust=%.0f%% div=%.1f lvl=%u",
            _trust * 100.0f, _last_divergence, (unsigned)_level);
    }
}
