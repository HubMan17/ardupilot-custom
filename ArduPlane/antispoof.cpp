/*
   Anti-spoof emergency override for ArduPlane.
   Shared logic for RC_OPTION and AP_IntegrityFilter auto-trigger.

   Per-core architecture:
     Core 0: always GPS-aided — runs pre-filter, reconverges after spoof ends
     Core 1: DR mode during lockdown — clean attitude, never sees spoofed GPS
   On engage: force Core 1 as primary (instant clean switch, no position reset)
   On disengage: wait for Core 0 healthy, return to auto lane selection
*/

#include "Plane.h"

void Plane::engage_spoof_emergency()
{
    if (_spoof_override_active) {
        return;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "ANTI-SPOOF: ENGAGED");

    // 1. Force Core 1 to DR (no GPS fusion) and make it primary
    //    Core 0 keeps GPS — pre-filter rejects spoofed data, reconverges when clean
    //    Core 1 provides clean attitude via IMU+compass for FBWA
    AP::ahrs().EKF3.setCoreNoGPS(1, true);
    AP::ahrs().EKF3.forcePrimaryCore(1);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ANTI-SPOOF: Core1 DR primary, Core0 GPS monitoring");

    // 2. Disable airspeed sensor use (spoofer might affect pitot via proximity)
    auto *arspd = AP::airspeed();
    if (arspd) {
        arspd->force_disable_use(true);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ANTI-SPOOF: airspeed disabled");
    }

    // 3. Disable QuadPlane assist (no GPS-dependent VTOL)
#if HAL_QUADPLANE_ENABLED
    quadplane.set_q_assist_state(quadplane.Q_ASSIST_STATE_ENUM::Q_ASSIST_DISABLED);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ANTI-SPOOF: Q_ASSIST disabled");
#endif

    // 4. Reset compass to WMM + noise override for DR heading accuracy
    AP::ahrs().EKF3.resetMagFieldToWMM();
    AP::ahrs().EKF3.setMagNoiseOverride(0.15f);

    // 5. Save current mode and switch to FBWA (attitude-only, no position needed)
    _pre_spoof_mode = control_mode;
    set_mode(mode_fbwa, ModeReason::EMERGENCY_OVERRIDE);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ANTI-SPOOF: mode FBWA (was %s)", _pre_spoof_mode->name());

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

    // 1. Return to automatic core selection (Core 0 should have reconverged)
    AP::ahrs().EKF3.forcePrimaryCore(-1);

    // 2. Clear Core 1 DR mode (both cores back to normal GPS fusion)
    AP::ahrs().EKF3.setCoreNoGPS(1, false);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ANTI-SPOOF: Core selection auto, GPS fusion restored");

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

    // 6. Restore previous flight mode
    if (_pre_spoof_mode != nullptr) {
        set_mode(*_pre_spoof_mode, ModeReason::EMERGENCY_OVERRIDE);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ANTI-SPOOF: restored mode %s", _pre_spoof_mode->name());
        _pre_spoof_mode = nullptr;
    }

    _spoof_override_active = false;
}
