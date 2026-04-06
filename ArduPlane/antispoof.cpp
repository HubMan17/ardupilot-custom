/*
   Anti-spoof emergency override for ArduPlane.
   Shared logic for RC_OPTION and future AP_IntegrityFilter auto-trigger.
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

    // 5. Force EKF position reset to current location
    AP::ahrs().EKF3.forcePositionReset(current_loc, 10.0f);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ANTI-SPOOF: EKF pos reset to current loc");

    // 6. Switch to FBWA — safe manual mode without GPS dependency
    set_mode(mode_fbwa, ModeReason::EMERGENCY_OVERRIDE);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ANTI-SPOOF: mode FBWA");

    _spoof_override_active = true;
}

void Plane::disengage_spoof_emergency()
{
    if (!_spoof_override_active) {
        return;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ANTI-SPOOF: DISENGAGED");

    // 1. Re-enable GPS
    AP::gps().force_disable(false);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ANTI-SPOOF: GPS enabled");

    // 2. Re-enable airspeed
    auto *arspd = AP::airspeed();
    if (arspd) {
        arspd->force_disable_use(false);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ANTI-SPOOF: airspeed enabled");
    }

    // 3. Re-enable QuadPlane assist
#if HAL_QUADPLANE_ENABLED
    quadplane.set_q_assist_state(quadplane.Q_ASSIST_STATE_ENUM::Q_ASSIST_ENABLED);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ANTI-SPOOF: Q_ASSIST enabled");
#endif

    // 4. Clear mag noise override
    AP::ahrs().EKF3.clearMagNoiseOverride();

    // 5. Do NOT auto-restore flight mode — pilot decides

    _spoof_override_active = false;
}
