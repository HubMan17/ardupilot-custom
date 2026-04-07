/*
   AP_IntegrityFilter — echelon 2 GPS spoof & sensor degradation detection.
   Cross-checks GPS against independent sensors (outside EKF):
     1. GPS velocity vs airspeed+heading (pitot+compass — spoofer can't fake these)
     2. GPS altitude vs barometer altitude (pressure sensor — independent)
     3. GPS velocity jitter (noise monitoring — catches degradation)
   Echelon 1 (EKF3 pre-filter) handles the same checks inside EKF at fusion rate.
   This filter catches slow drift that leaks through the pre-filter.
   Trust score 0.0-1.0 with instant lockdown for large divergence.
*/

#include "Plane.h"
#include "AP_IntegrityFilter.h"
#include <AP_GPS/AP_GPS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

const AP_Param::GroupInfo AP_IntegrityFilter::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Integrity filter enable
    // @Description: Enable GPS integrity cross-check filter
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("ENABLE", 1, AP_IntegrityFilter, _enable, 0),

    // @Param: VEL_THR
    // @DisplayName: Divergence threshold
    // @Description: Normalized divergence threshold to start trust decay
    // @Range: 1.0 10.0
    // @User: Advanced
    AP_GROUPINFO("VEL_THR", 2, AP_IntegrityFilter, _vel_thresh, 3.0f),

    // @Param: DECAY
    // @DisplayName: Trust decay rate
    // @Description: Trust score decrease per second when divergence exceeds threshold
    // @Range: 0.01 1.0
    // @Units: 1/s
    // @User: Advanced
    AP_GROUPINFO("DECAY", 3, AP_IntegrityFilter, _decay_rate, 0.1f),

    // @Param: RECOVER
    // @DisplayName: Trust recovery rate
    // @Description: Trust score increase per second when sensors agree
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

/*
  Primary spoof & degradation detection.
  Returns a normalized divergence score — higher = more suspicious.

  Checks (in order):
    1a. Direct speed limit: |GPS_speed - airspeed| > 15 m/s (no baseline, instant)
    1b. Wind baseline shift: tracks wind vector, detects sudden changes (needs 5s warmup)
    2.  Altitude: GPS alt vs baro alt divergence (needs 5s warmup)
    3.  GPS jitter: sensor degradation (needs baseline)
  Note: GPS vs EKF checks removed — not independent, now in EKF3 pre-filter.
*/
float AP_IntegrityFilter::compute_velocity_divergence()
{
    const auto &gps = AP::gps();
    const auto &ahrs = AP::ahrs();
    const uint32_t now = AP_HAL::millis();

    if (gps.status() < AP_GPS::GPS_OK_FIX_3D) {
        return 0.0f;
    }

    float max_div = 0.0f;
    const Vector3f &gps_vel = gps.velocity();

    // ---------------------------------------------------------------
    // 1. AIRSPEED CROSS-CHECKS
    //    1a: Direct speed limit — no baseline, works immediately
    //    1b: Wind baseline shift — sensitive, needs 5s warmup
    // ---------------------------------------------------------------
    auto *arspd = AP::airspeed();
    if (arspd && arspd->healthy()) {
        float airspeed = arspd->get_airspeed();

        // 1a. DIRECT SPEED LIMIT (no baseline needed, always active)
        //     |GPS_groundspeed - airspeed| should be bounded by wind speed.
        //     Max realistic wind: ~20 m/s. Beyond that = spoofing.
        //     Threshold 15 m/s: above transition speed to avoid prop wash on pitot.
        if (airspeed > 15.0f) {
            float speed_diff = fabsf(gps.ground_speed() - airspeed);
            if (speed_diff > 15.0f) {
                // 20 m/s diff → div = 2.5, 30 m/s → div = 7.5 (instant lockdown)
                max_div = MAX(max_div, (speed_diff - 15.0f) * 0.5f);
            }
        }

        // 1b. WIND BASELINE SHIFT (sensitive detection, needs warmup)
        //     Only in stable cruise (airspeed > 12 m/s, past transition)
        //     Invalidate baseline on yaw reset (mag anomaly causes heading jump
        //     which looks like a huge wind shift — false positive)
        if (airspeed > 12.0f) {
            float heading = ahrs.get_yaw();

            // Detect yaw discontinuity: >15 deg change between updates
            // (mag anomaly yaw reset causes heading jump → false wind shift)
            if (_yaw_primed) {
                float yaw_delta = fabsf(wrap_PI(heading - _prev_yaw));
                if (yaw_delta > radians(15.0f) && _baseline_wind_valid) {
                    _baseline_wind_valid = false;
                    _baseline_wind_start_ms = 0;
                }
            }
            _prev_yaw = heading;
            _yaw_primed = true;
            Vector2f expected_vel(airspeed * cosf(heading), airspeed * sinf(heading));
            Vector2f gps_vel_h(gps_vel.x, gps_vel.y);
            Vector2f wind = gps_vel_h - expected_vel;

            // Build baseline — use trust > 0.7 (not 0.9) so it can self-correct
            if (_trust > 0.7f && _baseline_wind_valid) {
                float ema = (now - _baseline_wind_start_ms < 10000) ? 0.15f : 0.03f;
                _baseline_wind = _baseline_wind * (1.0f - ema) + wind * ema;
            } else if (_trust > 0.7f && !_baseline_wind_valid) {
                _baseline_wind = wind;
                _baseline_wind_valid = true;
                _baseline_wind_start_ms = now;
            }

            // Only use after 5s of baseline building
            if (_baseline_wind_valid && (now - _baseline_wind_start_ms > 5000)) {
                Vector2f wind_shift = wind - _baseline_wind;
                float shift_mag = wind_shift.length();
                // 6 m/s shift → div = 3.0 (at threshold)
                max_div = MAX(max_div, shift_mag * 0.5f);
            }
        }
    }

    // ---------------------------------------------------------------
    // 2. ALTITUDE CROSS-CHECK: GPS altitude vs barometer altitude
    //    Barometer measures pressure — spoofer can't fake it.
    // ---------------------------------------------------------------
    {
        float baro_alt = AP::baro().get_altitude();
        const Location &home = AP::ahrs().get_home();
        float gps_alt_rel = (gps.location().alt - home.alt) * 0.01f;
        float alt_diff = gps_alt_rel - baro_alt;

        if (_trust > 0.7f && _baseline_alt_valid) {
            float ema = (now - _baseline_alt_start_ms < 10000) ? 0.15f : 0.03f;
            _baseline_alt_diff = _baseline_alt_diff * (1.0f - ema) + alt_diff * ema;
        } else if (_trust > 0.7f && !_baseline_alt_valid) {
            _baseline_alt_diff = alt_diff;
            _baseline_alt_valid = true;
            _baseline_alt_start_ms = now;
        }

        // Use after 5s of baseline
        if (_baseline_alt_valid && (now - _baseline_alt_start_ms > 5000)) {
            float alt_shift = fabsf(alt_diff - _baseline_alt_diff);
            // 30m shift → div = 3.0, 60m → div = 6.0 (instant lockdown)
            max_div = MAX(max_div, alt_shift * 0.1f);
        }
    }

    // ---------------------------------------------------------------
    // 3. GPS VELOCITY JITTER: sensor degradation detection
    //    Track sample-to-sample velocity changes.
    //    3× baseline jitter → contributes to divergence.
    // ---------------------------------------------------------------
    {
        Vector2f gps_vel_h(gps_vel.x, gps_vel.y);
        if (_jitter_primed) {
            Vector2f delta = gps_vel_h - _prev_gps_vel_detect;
            float jitter = delta.length();
            _gps_jitter = _gps_jitter * 0.9f + jitter * 0.1f;

            if (_trust > 0.7f && _baseline_jitter_valid) {
                _baseline_gps_jitter = _baseline_gps_jitter * 0.99f + _gps_jitter * 0.01f;
            } else if (_trust > 0.7f && _gps_jitter > 0.01f && !_baseline_jitter_valid) {
                _baseline_gps_jitter = _gps_jitter;
                _baseline_jitter_valid = true;
            }

            if (_baseline_jitter_valid && _baseline_gps_jitter > 0.01f) {
                float jitter_ratio = _gps_jitter / _baseline_gps_jitter;
                if (jitter_ratio > 3.0f) {
                    max_div = MAX(max_div, (jitter_ratio - 2.0f) * 1.5f);
                }
            }
        }
        _prev_gps_vel_detect = gps_vel_h;
        _jitter_primed = true;
    }

    // Checks 4 & 5 (GPS vs EKF vel/pos) removed — they are not independent
    // from the EKF state and are now handled by the EKF3 pre-filter.

    return max_div;
}

/*
  Recovery divergence: compare velocity CHANGES (deltas) between GPS and AHRS
  over 1-second windows. In DR mode, absolute velocities drift apart, but
  velocity *changes* (accelerations) still agree because IMU bias is slow.
*/
float AP_IntegrityFilter::compute_recovery_divergence()
{
    const auto &gps = AP::gps();
    const auto &ahrs = AP::ahrs();

    const Vector3f &gps_vel = gps.velocity();
    if (gps_vel.length() < 0.01f && gps.ground_speed() < 0.01f) {
        _prev_recovery_ms = 0;
        return 99.0f;
    }

    Vector3f ahrs_vel;
    if (!ahrs.get_velocity_NED(ahrs_vel)) {
        _prev_recovery_ms = 0;
        return 99.0f;
    }

    const uint32_t now = AP_HAL::millis();

    if (_prev_recovery_ms == 0 || (now - _prev_recovery_ms) > 2000) {
        _prev_gps_vel = gps_vel;
        _prev_ahrs_vel = ahrs_vel;
        _prev_recovery_ms = now;
        return _last_recovery_result;
    }

    float dt = (now - _prev_recovery_ms) * 0.001f;
    if (dt < 0.8f) {
        return _last_recovery_result;
    }

    Vector2f delta_gps(gps_vel.x - _prev_gps_vel.x, gps_vel.y - _prev_gps_vel.y);
    Vector2f delta_ahrs(ahrs_vel.x - _prev_ahrs_vel.x, ahrs_vel.y - _prev_ahrs_vel.y);

    _prev_gps_vel = gps_vel;
    _prev_ahrs_vel = ahrs_vel;
    _prev_recovery_ms = now;

    Vector2f accel_diff = delta_gps - delta_ahrs;
    _last_recovery_result = accel_diff.length() / dt;
    return _last_recovery_result;
}

void AP_IntegrityFilter::update_snapshots()
{
    const uint32_t now = AP_HAL::millis();

    if (now - _last_snapshot_ms < 1000) {
        return;
    }
    if (_trust < 0.8f) {
        return;
    }

    const auto &gps = AP::gps();
    if (gps.status() < AP_GPS::GPS_OK_FIX_3D) {
        return;
    }

    _last_snapshot_ms = now;
    Snapshot &s = _snapshots[_snapshot_idx];
    s.time_ms = now;
    const Location &loc = gps.location();
    s.lat = loc.lat;
    s.lng = loc.lng;
    s.alt_cm = loc.alt;
    s.velocity = gps.velocity();
    s.trust = _trust;

    _snapshot_idx = (_snapshot_idx + 1) % SNAPSHOT_SIZE;
}

bool AP_IntegrityFilter::get_clean_snapshot(Snapshot &snap) const
{
    for (uint8_t i = 0; i < SNAPSHOT_SIZE; i++) {
        uint8_t idx = (_snapshot_idx + SNAPSHOT_SIZE - 1 - i) % SNAPSHOT_SIZE;
        const Snapshot &s = _snapshots[idx];
        if (s.time_ms != 0 && s.trust > 0.8f) {
            snap = s;
            return true;
        }
    }
    return false;
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
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "INTEG: CAUTION trust=%.0f%% div=%.1f",
                _trust * 100.0f, _last_divergence);
            break;

        case Level::WARNING:
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "INTEG: WARNING trust=%.0f%%",
                _trust * 100.0f);
            // GPS stays enabled — per-core pre-filter handles rejection.
            // Core 0 keeps monitoring GPS for reconvergence when spoof ends.
            _prev_recovery_ms = 0;
            break;

        case Level::EMERGENCY:
            GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "INTEG: EMERGENCY — FULL LOCKDOWN trust=%.0f%%",
                _trust * 100.0f);
            plane.engage_spoof_emergency();
            _prev_recovery_ms = 0;
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
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "INTEG: recovering from WARNING trust=%.0f%%",
                _trust * 100.0f);
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
        _baseline_wind_valid = false;
        _baseline_wind_start_ms = 0;
        _prev_yaw = 0.0f;
        _yaw_primed = false;
        _baseline_alt_valid = false;
        _baseline_alt_start_ms = 0;
        _baseline_jitter_valid = false;
        _jitter_primed = false;
        _high_div_since_ms = 0;
        return;
    }

    // record arm time, skip first 10s to let EKF settle
    if (_arm_time_ms == 0) {
        _arm_time_ms = now;
    }
    if (now - _arm_time_ms < 10000) {
        return;
    }

    // skip if pilot has manual RC override active (not auto-triggered)
    if (plane._spoof_override_active && _level < Level::EMERGENCY) {
        return;
    }

    // save clean GPS snapshots while trust is high
    update_snapshots();

    // compute divergence — different methods for normal vs lockdown
    if (_level >= Level::WARNING) {
        _last_divergence = 0.0f;
        _recovery_divergence = compute_recovery_divergence();

        if (_recovery_divergence < _vel_thresh) {
            if (_recovery_good_since_ms == 0) {
                _recovery_good_since_ms = now;
            }
            if (now - _recovery_good_since_ms > 10000) {
                _trust += _recover_rate * dt;
            }
        } else {
            _recovery_good_since_ms = 0;
            _trust -= _decay_rate * 0.1f * dt;
        }
    } else {
        // Normal mode: cross-check GPS vs independent sensors
        _last_divergence = compute_velocity_divergence();
        _recovery_divergence = 0.0f;
        _recovery_good_since_ms = 0;

        if (_last_divergence > _vel_thresh) {
            // scale decay by divergence magnitude
            float decay_mult = _last_divergence / _vel_thresh;
            _trust -= _decay_rate * decay_mult * dt;
        } else {
            _trust += _recover_rate * dt;
            _high_div_since_ms = 0;
        }

        // INSTANT LOCKDOWN: div > 2× threshold sustained > 1s → straight to EMERGENCY
        if (_last_divergence > _vel_thresh * 2.0f) {
            if (_high_div_since_ms == 0) {
                _high_div_since_ms = now;
            } else if (now - _high_div_since_ms > 1000 && _level < Level::EMERGENCY) {
                GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY,
                    "INTEG: INSTANT LOCKDOWN div=%.1f (>%.1f for >1s)",
                    _last_divergence, _vel_thresh * 2.0f);
                _trust = 0.0f;
                execute_level_change(Level::EMERGENCY, _level);
                _level = Level::EMERGENCY;
                _pending_level = Level::EMERGENCY;
            }
        } else {
            _high_div_since_ms = 0;
        }
    }
    _trust = constrain_float(_trust, 0.0f, 1.0f);

    // determine target level (for gradual escalation path)
    Level target = compute_level(_trust);

    // hysteresis: require level to be sustained for 1 second
    if (target != _level) {
        if (target != _pending_level) {
            _pending_level = target;
            _pending_level_start_ms = now;
        } else if (now - _pending_level_start_ms > 1000) {
            execute_level_change(target, _level);
            _level = target;
        }
    } else {
        _pending_level = _level;
    }

    // periodic status: 5s at NOMINAL/CAUTION, 30s at WARNING/EMERGENCY
    static uint32_t last_status_ms;
    const uint32_t status_interval = (_level >= Level::WARNING) ? 30000 : 5000;
    if (now - last_status_ms > status_interval) {
        last_status_ms = now;
        if (_level >= Level::WARNING) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "INTEG: trust=%.0f%% recov=%.1f lvl=%u%s",
                _trust * 100.0f, _recovery_divergence, (unsigned)_level,
                _recovery_good_since_ms ? " RECOVERING" : "");
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "INTEG: trust=%.0f%% div=%.1f lvl=%u",
                _trust * 100.0f, _last_divergence, (unsigned)_level);
        }
    }
}
