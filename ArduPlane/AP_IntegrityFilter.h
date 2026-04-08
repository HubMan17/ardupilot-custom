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

    // Clean GPS snapshot structure (public for access from antispoof.cpp)
    struct Snapshot {
        uint32_t time_ms;
        int32_t lat;        // degE7
        int32_t lng;        // degE7
        int32_t alt_cm;
        Vector3f velocity;  // NED m/s
        float trust;
    };

    // get last clean snapshot (trust > 0.8), returns false if none available
    bool get_clean_snapshot(Snapshot &snap) const;

private:
    // Parameters
    AP_Int8  _enable;
    AP_Float _vel_thresh;     // divergence threshold
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
    float _recovery_divergence = 0.0f;
    uint32_t _arm_time_ms = 0;
    uint32_t _recovery_good_since_ms = 0;

    // Clean GPS snapshot ring buffer
    static constexpr uint8_t SNAPSHOT_SIZE = 10;
    Snapshot _snapshots[SNAPSHOT_SIZE];
    uint8_t _snapshot_idx = 0;
    uint32_t _last_snapshot_ms = 0;

    // Airspeed cross-check: baseline wind vector (GPS_vel - airspeed*heading)
    Vector2f _baseline_wind;
    bool _baseline_wind_valid = false;
    uint32_t _baseline_wind_start_ms = 0;  // when baseline was first set
    float _prev_yaw = 0.0f;               // previous heading for yaw discontinuity detection
    bool _yaw_primed = false;              // true after first yaw sample

    // Altitude cross-check: baseline GPS-baro altitude difference
    float _baseline_alt_diff = 0.0f;
    bool _baseline_alt_valid = false;
    uint32_t _baseline_alt_start_ms = 0;

    // GPS velocity jitter monitoring (sensor degradation)
    Vector2f _prev_gps_vel_detect;
    float _gps_jitter = 0.0f;
    float _baseline_gps_jitter = 0.0f;
    bool _baseline_jitter_valid = false;
    bool _jitter_primed = false;

    // Instant lockdown: high-divergence sustained timer
    uint32_t _high_div_since_ms = 0;

    // Rapid trust drop tracker: skip hysteresis when trust crashes fast
    float _trust_5s_ago = 1.0f;
    uint32_t _trust_history_ms = 0;

    // Pre-filter shortcut: both EKF cores trust < 40% sustained timer
    uint32_t _prefilter_low_since_ms = 0;

    // Recovery delta-comparison state
    Vector3f _prev_gps_vel;
    Vector3f _prev_ahrs_vel;
    uint32_t _prev_recovery_ms = 0;
    float _last_recovery_result = 99.0f;

    // Methods
    float compute_velocity_divergence();
    float compute_recovery_divergence();
    Level compute_level(float trust) const;
    bool execute_level_change(Level new_level, Level old_level);  // false if blocked
    void update_snapshots();
};
