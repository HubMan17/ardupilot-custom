#include "AP_DisarmProtect.h"
#include <AP_Arming/AP_Arming.h>
#include <GCS_MAVLink/GCS.h>

const AP_Param::GroupInfo AP_DisarmProtect::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Disarm protection enable
    // @Description: Enable disarm protection at altitude. When enabled, disarm via aux switch is blocked above minimum altitude until a toggle combo is performed.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_DisarmProtect, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: MIN_ALT
    // @DisplayName: Minimum altitude for protection
    // @Description: Disarm is blocked when relative altitude is above this value in meters
    // @Units: m
    // @Range: 1 100
    // @User: Standard
    AP_GROUPINFO("MIN_ALT", 2, AP_DisarmProtect, _min_alt, 5),

    // @Param: TOGGLES
    // @DisplayName: Toggle count to unlock
    // @Description: Number of rapid arm/disarm switch toggles required to unlock disarm
    // @Range: 2 10
    // @User: Standard
    AP_GROUPINFO("TOGGLES", 3, AP_DisarmProtect, _toggle_count, 5),

    // @Param: TIMEOUT
    // @DisplayName: Combo timeout
    // @Description: Time window in seconds to complete the toggle combo before counter resets
    // @Units: s
    // @Range: 1.0 10.0
    // @User: Standard
    AP_GROUPINFO("TIMEOUT", 4, AP_DisarmProtect, _timeout, 3.0f),

    AP_GROUPEND
};

bool AP_DisarmProtect::should_block_disarm(uint8_t method, float rel_alt_m)
{
    // only active when enabled
    if (_enable.get() == 0) {
        return false;
    }

    // only block aux switch disarm
    if (method != (uint8_t)AP_Arming::Method::AUXSWITCH) {
        return false;
    }

    // below min altitude — allow disarm, reset state
    if (rel_alt_m < _min_alt.get()) {
        reset();
        return false;
    }

    // already unlocked — let the disarm through
    if (_unlocked) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "EMERGENCY DISARM (unlocked)");
        reset();
        return false;
    }

    // combo logic
    const uint32_t now_ms = AP_HAL::millis();

    // check timeout — reset if combo expired
    if (_toggle_counter > 0 && (now_ms - _first_toggle_ms) > (uint32_t)(_timeout.get() * 1000.0f)) {
        _toggle_counter = 0;
    }

    // first toggle in a new combo
    if (_toggle_counter == 0) {
        _first_toggle_ms = now_ms;
    }

    _toggle_counter++;

    if (_toggle_counter >= _toggle_count.get()) {
        // combo complete — unlock, but block THIS attempt
        _unlocked = true;
        gcs().send_text(MAV_SEVERITY_WARNING, "DISARM UNLOCKED! Toggle again to disarm");
        return true;
    }

    // rate-limit block messages (500ms)
    if (now_ms - _last_block_msg_ms >= 500) {
        _last_block_msg_ms = now_ms;
        gcs().send_text(MAV_SEVERITY_WARNING, "Disarm BLOCKED at %.0fm (%u/%u)",
                        (double)rel_alt_m,
                        (unsigned)_toggle_counter,
                        (unsigned)_toggle_count.get());
    }
    return true;
}

void AP_DisarmProtect::reset()
{
    _toggle_counter = 0;
    _first_toggle_ms = 0;
    _last_block_msg_ms = 0;
    _unlocked = false;
}
