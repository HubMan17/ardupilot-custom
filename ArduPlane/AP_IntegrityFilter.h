#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>

class AP_IntegrityFilter {
public:
    AP_IntegrityFilter();

    static const struct AP_Param::GroupInfo var_info[];

    // called from scheduler at 10Hz
    void update();

    // accessors
    float get_trust() const { return _trust; }
    bool is_enabled() const { return _enable.get() != 0; }

    enum class Level : uint8_t {
        NOMINAL = 0,
        CAUTION = 1,
        WARNING = 2,
        EMERGENCY = 3,
    };

    Level get_level() const { return _level; }

private:
    // Parameters
    AP_Int8  _enable;
    AP_Float _vel_thresh;     // velocity divergence threshold (m/s)
    AP_Float _decay_rate;     // trust decay per second when divergent
    AP_Float _recover_rate;   // trust recovery per second when convergent
    AP_Float _caution_thresh; // trust threshold for CAUTION
    AP_Float _warn_thresh;    // trust threshold for WARNING
    AP_Float _emerg_thresh;   // trust threshold for EMERGENCY

    // State
    float _trust = 1.0f;
    Level _level = Level::NOMINAL;
    Level _pending_level = Level::NOMINAL;
    uint32_t _last_update_ms = 0;
    uint32_t _pending_level_start_ms = 0;
    float _last_divergence = 0.0f;
    uint32_t _arm_time_ms = 0;

    // Methods
    float compute_velocity_divergence() const;
    Level compute_level(float trust) const;
    void execute_level_change(Level new_level, Level old_level);
};
