/*
   Anti-spoof emergency override for ArduPlane.
   Shared logic for RC_OPTION and AP_IntegrityFilter auto-trigger.
*/

#include "Plane.h"

void Plane::engage_spoof_emergency()
{
    if (_spoof_override_active) {
        return;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "ANTI-SPOOF: ENGAGED");

    // 1. Disable GPS
    AP::gps().force_disable(true);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ANTI-SPOOF: GPS disabled");

    // 2. Disable airspeed sensor use
    auto *arspd = AP::airspeed();
    if (arspd) {
        arspd->force_disable_use(true);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ANTI-SPOOF: airspeed disabled");
    }

    // 3. Disable QuadPlane assist
#if HAL_QUADPLANE_ENABLED
    quadplane.set_q_assist_state(quadplane.Q_ASSIST_STATE_ENUM::Q_ASSIST_DISABLED);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ANTI-SPOOF: Q_ASSIST disabled");
#endif

    // 4. Reset compass to WMM + noise override for DR
    AP::ahrs().EKF3.resetMagFieldToWMM();
    AP::ahrs().EKF3.setMagNoiseOverride(0.15f);

    // 5. Switch EKF to SRC2 (DR, no GPS) before position reset
    AP::ahrs().EKF3.setPosVelYawSourceSet(1);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ANTI-SPOOF: EKF SRC2 (DR)");

    // 6. Force EKF position reset — use clean snapshot if available
    AP_IntegrityFilter::Snapshot snap;
    if (g2.integrity_filter.get_clean_snapshot(snap)) {
        Location clean_loc;
        clean_loc.lat = snap.lat;
        clean_loc.lng = snap.lng;
        clean_loc.alt = snap.alt_cm;
        // accuracy grows with age: base 10m + 0.5m/s IMU drift
        float age_s = (AP_HAL::millis() - snap.time_ms) * 0.001f;
        float accuracy = 10.0f + 0.5f * age_s;
        AP::ahrs().EKF3.forcePositionReset(clean_loc, accuracy);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ANTI-SPOOF: EKF reset to snapshot (%.0fs ago, acc=%.0fm)",
            age_s, accuracy);
    } else {
        AP::ahrs().EKF3.forcePositionReset(current_loc, 50.0f);
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ANTI-SPOOF: no clean snapshot, reset to current loc");
    }

    // 7. Switch to FBWA — safe manual mode without GPS dependency
    set_mode(mode_fbwa, ModeReason::EMERGENCY_OVERRIDE);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ANTI-SPOOF: mode FBWA");

    _spoof_override_active = true;
    _spoof_engage_ms = AP_HAL::millis();
}

void Plane::disengage_spoof_emergency()
{
    if (!_spoof_override_active) {
        return;
    }

    // minimum lockdown time 5 seconds to prevent instant disengage from RC bounce
    const uint32_t lockdown_time = AP_HAL::millis() - _spoof_engage_ms;
    if (lockdown_time < 5000) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ANTI-SPOOF: disengage blocked, lockdown %us",
            (unsigned)(lockdown_time / 1000));
        return;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ANTI-SPOOF: DISENGAGED after %us",
        (unsigned)(lockdown_time / 1000));

    // 1. Switch EKF back to SRC1 (GPS) before re-enabling GPS
    AP::ahrs().EKF3.setPosVelYawSourceSet(0);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ANTI-SPOOF: EKF SRC1 (GPS)");

    // 2. Re-enable GPS
    AP::gps().force_disable(false);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ANTI-SPOOF: GPS enabled");

    // 3. Re-enable airspeed
    auto *arspd = AP::airspeed();
    if (arspd) {
        arspd->force_disable_use(false);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ANTI-SPOOF: airspeed enabled");
    }

    // 4. Re-enable QuadPlane assist
#if HAL_QUADPLANE_ENABLED
    quadplane.set_q_assist_state(quadplane.Q_ASSIST_STATE_ENUM::Q_ASSIST_ENABLED);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ANTI-SPOOF: Q_ASSIST enabled");
#endif

    // 5. Clear mag noise override
    AP::ahrs().EKF3.clearMagNoiseOverride();

    // 6. Do NOT auto-restore flight mode — pilot decides

    _spoof_override_active = false;
}
