#pragma once

#include <AP_Param/AP_Param.h>

class AP_DisarmProtect {
public:
    AP_DisarmProtect() { AP_Param::setup_object_defaults(this, var_info); }

    static const struct AP_Param::GroupInfo var_info[];

    // returns true if disarm should be blocked
    // rel_alt_m: relative altitude in meters (from caller)
    bool should_block_disarm(uint8_t method, float rel_alt_m);

    // reset state (called after successful disarm or when below min alt)
    void reset();

private:
    // parameters
    AP_Int8  _enable;
    AP_Int8  _min_alt;
    AP_Int8  _toggle_count;
    AP_Float _timeout;

    // state
    uint8_t  _toggle_counter;
    uint32_t _first_toggle_ms;
    uint32_t _last_block_msg_ms;
    bool     _unlocked;
};
