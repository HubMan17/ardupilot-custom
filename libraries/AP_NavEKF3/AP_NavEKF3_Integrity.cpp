/*
  GPS integrity pre-filter for EKF3 (anti-spoof)

  Runs BEFORE fusion in SelectVelPosFusion(). Cross-checks GPS data
  against independent sensors (airspeed, barometer, velocity jitter).
  Adjusts measurement noise or rejects GPS data entirely to prevent
  EKF state contamination from GPS spoofing.

  Checks 1a-3 require established forward flight (wind baseline valid 5+ sec).
  Checks 4-5 (innovation CUSUM) run in ALL flight phases including hover.

  Thresholds calibrated from real flight data (see prefilter_baseline_reference.md):
    Check 1a |GS-AS|:     p99=4.92, MAX=8.43 → decay=7.0, reject=15.0
    Check 1b wind shift:  p99=3.15, MAX=4.64 → decay=5.0, reject=12.0
    Check 1c frozen ref:  frozen wind ref     → decay=5.0, reject=12.0
    Check 1d wind CUSUM:  persistent bias     → CUSUM threshold=8.0
    Check 2  alt shift:   p99=3.60, MAX=7.47 → decay=8.0, reject=30.0
    Check 3  jitter:      MAX~4.8×baseline    → decay=4.0×, reject=8.0×
    Check 4  pos CUSUM:   GPS-EKF position    → CUSUM threshold=8.0
    Check 5  vel CUSUM:   GPS-EKF velocity    → CUSUM threshold=8.0
 */

#include <AP_HAL/AP_HAL.h>

#include "AP_NavEKF3.h"
#include "AP_NavEKF3_core.h"
#include <AP_DAL/AP_DAL.h>
#include <GCS_MAVLink/GCS.h>

// Minimum airspeed for wind baseline building (m/s)
#define INTEGRITY_MIN_AIRSPEED 15.0f

// Time after wind baseline first established before checks are active (ms)
#define INTEGRITY_BASELINE_MATURE_MS 5000

// Per-check thresholds (from real flight data analysis)
// Check 1a: |groundspeed - airspeed| (m/s)
#define CHK1A_DECAY_START   7.0f    // > p99.5 (5.30)
#define CHK1A_HARD_REJECT  15.0f    // impossible wind in most conditions

// Check 1b: wind vector shift from EMA baseline (m/s)
#define CHK1B_DECAY_START   5.0f    // > p99 (3.15), 2x margin
#define CHK1B_HARD_REJECT  12.0f    // extreme shift

// Check 1c: wind shift from frozen reference (m/s)
#define CHK1C_DECAY_START   5.0f    // same scale as 1b
#define CHK1C_HARD_REJECT  12.0f    // same scale as 1b

// Check 2: GPS-baro altitude shift from baseline (m)
#define CHK2_DECAY_START    8.0f    // > MAX (7.47), margin for weather
#define CHK2_HARD_REJECT   30.0f    // huge altitude disagreement

// Check 3: jitter ratio (current / baseline)
#define CHK3_DECAY_START    4.0f    // > real MAX (~4.8x) with margin
#define CHK3_HARD_REJECT    8.0f    // extreme jitter

// CUSUM parameters
#define CUSUM_WIND_SIGMA    2.0f    // expected wind variation (m/s)
#define CUSUM_WIND_ALLOW    0.5f    // allowance in sigma units
#define CUSUM_WIND_THRESH   8.0f    // detection threshold in sigma units

#define CUSUM_POS_SIGMA     3.0f    // expected position innovation (m)
#define CUSUM_POS_ALLOW     0.5f
#define CUSUM_POS_THRESH    8.0f

#define CUSUM_VEL_SIGMA     0.5f    // expected velocity innovation (m/s)
#define CUSUM_VEL_ALLOW     0.5f
#define CUSUM_VEL_THRESH    8.0f

// Grace period after CUSUM reset before innovation checks re-arm (ms)
// Allows EKF to reconverge with clean GPS after DR mode
#define CUSUM_GRACE_MS      10000

// Helper: compute normalized score 0.0 (at decay_start) to 1.0 (at hard_reject)
static float check_score(float value, float decay_start, float hard_reject)
{
    if (value <= decay_start) {
        return 0.0f;
    }
    return (value - decay_start) / (hard_reject - decay_start);
}

// Two-sided CUSUM on NE axes
// cusum[0]=N+, cusum[1]=N-, cusum[2]=E+, cusum[3]=E-
// Returns max of all 4 accumulators (normalized by threshold)
static float cusum_update_2sided(float cusum[4], float resid_n, float resid_e,
                                  float sigma, float allowance, float threshold)
{
    float zn = resid_n / sigma;
    float ze = resid_e / sigma;

    cusum[0] = MAX(0.0f, cusum[0] + zn - allowance);   // N+
    cusum[1] = MAX(0.0f, cusum[1] - zn - allowance);   // N-
    cusum[2] = MAX(0.0f, cusum[2] + ze - allowance);   // E+
    cusum[3] = MAX(0.0f, cusum[3] - ze - allowance);   // E-

    float peak = MAX(MAX(cusum[0], cusum[1]), MAX(cusum[2], cusum[3]));
    return peak / threshold;
}

void NavEKF3_core::updateIntegrityPreFilter()
{
    // Only act when there is fresh GPS data flagged for fusion
    if (!gpsDataToFuse) {
        return;
    }

    // Only act in GPS-aided mode
    if (PV_AidingMode != AID_ABSOLUTE) {
        return;
    }

    const uint32_t now_ms = imuSampleTime_ms;

    // ---- BUILD WIND BASELINE (forward flight gate) ----
    const auto *arspd = dal.airspeed();
    float airspeed = 0.0f;
    bool have_airspeed = false;
    if (arspd && arspd->healthy(selected_airspeed)) {
        airspeed = arspd->get_airspeed(selected_airspeed);
        have_airspeed = (airspeed > INTEGRITY_MIN_AIRSPEED);
    }

    float heading = 0.0f;
    Vector2f gps_vel_h(gpsDataDelayed.vel.x, gpsDataDelayed.vel.y);
    Vector2f wind;

    if (have_airspeed) {
        heading = atan2F(prevTnb[0][1], prevTnb[0][0]);
        Vector2f expected_gs(airspeed * cosF(heading),
                             airspeed * sinF(heading));
        wind = gps_vel_h - expected_gs;

        if (_integrity.trust > 0.7f) {
            if (!_integrity.baseline_wind_valid) {
                _integrity.baseline_wind = wind;
                _integrity.baseline_wind_valid = true;
                _integrity.baseline_wind_start_ms = now_ms;
            } else {
                const float ema = (now_ms - _integrity.baseline_wind_start_ms < 10000) ? 0.15f : 0.03f;
                _integrity.baseline_wind = _integrity.baseline_wind * (1.0f - ema) + wind * ema;
            }
        }

        // Set frozen wind reference once during first forward flight, never update
        if (!_integrity.reference_wind_set && _integrity.trust > 0.9f) {
            _integrity.reference_wind = wind;
            _integrity.reference_wind_set = true;
        }
    }

    // ---- BUILD ALT BASELINE (always) ----
    {
        float baro_alt = dal.baro().get_altitude();
        float gps_alt = gpsDataDelayed.hgt - float(EKF_origin.alt) * 0.01f;
        float alt_diff = gps_alt - baro_alt;

        if (_integrity.trust > 0.7f) {
            if (!_integrity.baseline_alt_valid) {
                _integrity.baseline_alt_diff = alt_diff;
                _integrity.baseline_alt_valid = true;
                _integrity.baseline_alt_start_ms = now_ms;
            } else {
                const float ema = (now_ms - _integrity.baseline_alt_start_ms < 10000) ? 0.15f : 0.03f;
                _integrity.baseline_alt_diff = _integrity.baseline_alt_diff * (1.0f - ema) + alt_diff * ema;
            }
        }
    }

    // ---- BUILD JITTER BASELINE (always) ----
    {
        if (_integrity.jitter_primed) {
            float jitter = (gps_vel_h - _integrity.prev_gps_vel).length();
            _integrity.jitter_ema = _integrity.jitter_ema * 0.9f + jitter * 0.1f;

            if (_integrity.trust > 0.7f && _integrity.jitter_ema > 0.01f) {
                if (!_integrity.baseline_jitter_valid) {
                    _integrity.baseline_jitter = _integrity.jitter_ema;
                    _integrity.baseline_jitter_valid = true;
                } else {
                    _integrity.baseline_jitter =
                        _integrity.baseline_jitter * 0.99f +
                        _integrity.jitter_ema * 0.01f;
                }
            }
        }
        _integrity.prev_gps_vel = gps_vel_h;
        _integrity.jitter_primed = true;
    }

    // ---- INNOVATION CUSUM (checks 4,5 — run in ALL flight phases) ----
    // Grace period after CUSUM reset: EKF needs time to reconverge after DR mode.
    // Without this, clean GPS after recovery shows large innovations (EKF state
    // is stale from DR drift) and CUSUM false-triggers → oscillating lockdowns.
    bool cusum_armed = (_integrity.cusum_reset_ms == 0) ||
                       (now_ms - _integrity.cusum_reset_ms >= CUSUM_GRACE_MS);

    float innovation_score = 0.0f;
    float cusum_vel_score = 0.0f;
    float cusum_pos_score = 0.0f;
    float innov_vel_n = 0.0f, innov_vel_e = 0.0f;
    float innov_pos_n = 0.0f, innov_pos_e = 0.0f;

    if (cusum_armed) {
        // Compute GPS vs EKF-predicted innovations from velPosObs and stateStruct
        // velPosObs[0..2] = GPS vel NED, stateStruct.velocity = EKF vel
        // velPosObs[3..4] = GPS pos NE,  stateStruct.position = EKF pos
        innov_vel_n = velPosObs[0] - stateStruct.velocity.x;
        innov_vel_e = velPosObs[1] - stateStruct.velocity.y;
        innov_pos_n = velPosObs[3] - stateStruct.position.x;
        innov_pos_e = velPosObs[4] - stateStruct.position.y;

        cusum_vel_score = cusum_update_2sided(_integrity.cusum_vel,
            innov_vel_n, innov_vel_e,
            CUSUM_VEL_SIGMA, CUSUM_VEL_ALLOW, CUSUM_VEL_THRESH);

        cusum_pos_score = cusum_update_2sided(_integrity.cusum_pos,
            innov_pos_n, innov_pos_e,
            CUSUM_POS_SIGMA, CUSUM_POS_ALLOW, CUSUM_POS_THRESH);

        innovation_score = MAX(cusum_vel_score, cusum_pos_score);
    }

    // ---- GATE: forward flight check ----
    bool run_airspeed_checks = _integrity.baseline_wind_valid &&
        (now_ms - _integrity.baseline_wind_start_ms >= INTEGRITY_BASELINE_MATURE_MS);

    if (!run_airspeed_checks && innovation_score < 1.0f) {
        // No forward flight AND no innovation alarm — hold current state.
        // CRITICAL: do NOT reset trust to 1.0 here.  During VTOL transitions
        // airspeed drops below gate → this path fires → any accumulated trust
        // decay gets wiped, letting spoofed GPS re-enter at full weight.
        // This was the root cause of trust oscillating at ~45% forever:
        // detect → transition → reset → detect → transition → reset...
        // Natural recovery (0.1/s) will restore trust if GPS is clean.
        if (_integrity.trust >= 1.0f) {
            // Only set noise scaling to 1× if trust was never degraded
            _integrity.pos_noise_scale = 1.0f;
            _integrity.vel_noise_scale = 1.0f;
        }
        _integrity.divergence = 0.0f;
        return;
    }

    // ---- DIVERGENCE CHECKS ----
    float max_score = 0.0f;
    float max_raw = 0.0f;

    // Check 4: position innovation CUSUM (all phases)
    if (cusum_pos_score > max_score) {
        max_score = cusum_pos_score;
        max_raw = MAX(fabsF(innov_pos_n), fabsF(innov_pos_e));
    }

    // Check 5: velocity innovation CUSUM (all phases)
    if (cusum_vel_score > max_score) {
        max_score = cusum_vel_score;
        max_raw = MAX(fabsF(innov_vel_n), fabsF(innov_vel_e));
    }

    // Airspeed-based checks only in forward flight
    if (run_airspeed_checks) {
        // 1a. |groundspeed - airspeed| — catches large/fast spoofing
        if (have_airspeed) {
            float speed_diff = fabsF(gps_vel_h.length() - airspeed);
            float score = check_score(speed_diff, CHK1A_DECAY_START, CHK1A_HARD_REJECT);
            if (score > max_score) {
                max_score = score;
                max_raw = speed_diff;
            }
        }

        // 1b. Wind vector shift from EMA baseline — catches medium spoofing
        if (have_airspeed) {
            float shift = (wind - _integrity.baseline_wind).length();
            float score = check_score(shift, CHK1B_DECAY_START, CHK1B_HARD_REJECT);
            if (score > max_score) {
                max_score = score;
                max_raw = shift;
            }
        }

        // 1c. Wind shift from frozen reference — catches slow drift EMA absorbs
        if (have_airspeed && _integrity.reference_wind_set) {
            float shift = (wind - _integrity.reference_wind).length();
            float score = check_score(shift, CHK1C_DECAY_START, CHK1C_HARD_REJECT);
            if (score > max_score) {
                max_score = score;
                max_raw = shift;
            }
        }

        // 1d. CUSUM on wind residual from EMA baseline — catches persistent small bias
        if (have_airspeed && _integrity.baseline_wind_valid) {
            Vector2f wind_resid = wind - _integrity.baseline_wind;
            float cusum_wind_score = cusum_update_2sided(_integrity.cusum_wind,
                wind_resid.x, wind_resid.y,
                CUSUM_WIND_SIGMA, CUSUM_WIND_ALLOW, CUSUM_WIND_THRESH);
            if (cusum_wind_score > max_score) {
                max_score = cusum_wind_score;
                max_raw = wind_resid.length();
            }
        }

        // 2. GPS-baro altitude shift — catches altitude spoofing
        if (_integrity.baseline_alt_valid &&
            (now_ms - _integrity.baseline_alt_start_ms > 5000)) {
            float baro_alt = dal.baro().get_altitude();
            float gps_alt = gpsDataDelayed.hgt - float(EKF_origin.alt) * 0.01f;
            float alt_diff = gps_alt - baro_alt;
            float alt_shift = fabsF(alt_diff - _integrity.baseline_alt_diff);
            float score = check_score(alt_shift, CHK2_DECAY_START, CHK2_HARD_REJECT);
            if (score > max_score) {
                max_score = score;
                max_raw = alt_shift;
            }
        }

        // 3. GPS velocity jitter — catches GPS degradation/jamming
        if (_integrity.baseline_jitter_valid &&
            _integrity.baseline_jitter > 0.01f) {
            float ratio = _integrity.jitter_ema / _integrity.baseline_jitter;
            float score = check_score(ratio, CHK3_DECAY_START, CHK3_HARD_REJECT);
            if (score > max_score) {
                max_score = score;
                max_raw = ratio;
            }
        }
    }

    // ---- TRUST UPDATE ----
    _integrity.divergence = max_raw;

    const float gps_dt = 0.1f;  // approximate GPS rate (~10 Hz)

    if (max_score > 0.0f) {
        // Decay proportional to score: score=1.0 → 0.3/s, score=2.0 → 0.6/s
        float decay = 0.3f * max_score;
        _integrity.trust -= decay * gps_dt;
    } else {
        // Recovery: 0.1/s → ~10 seconds from 0% to 100%
        _integrity.trust += 0.1f * gps_dt;
    }
    _integrity.trust = constrain_float(_integrity.trust, 0.0f, 1.0f);

    // ---- BASELINE RESET ----
    if (_integrity.trust < 0.25f || max_score > 5.0f) {
        _integrity.baseline_wind_valid = false;
        _integrity.baseline_alt_valid = false;
        _integrity.baseline_jitter_valid = false;
        _integrity.reference_wind_set = false;
        memset(_integrity.cusum_wind, 0, sizeof(_integrity.cusum_wind));
        memset(_integrity.cusum_pos, 0, sizeof(_integrity.cusum_pos));
        memset(_integrity.cusum_vel, 0, sizeof(_integrity.cusum_vel));
        _integrity.cusum_reset_ms = now_ms;
    }

    // ---- NOISE SCALING / REJECTION ----
    // Reject GPS entirely when trust < 25%.  At 25% the spoof is confirmed —
    // continuing to fuse with 4× noise still corrupts the EKF state.
    // Old threshold (5%) let spoofed data leak through for too long.
    if (_integrity.trust < 0.25f) {
        // Full rejection — do not fuse GPS at all
        gpsDataToFuse = false;
        fusePosData = false;
        fuseVelData = false;

        if (now_ms - _integrity.last_msg_ms > 3000) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                "EKF3 IMU%u: GPS rejected (trust=%.0f%%)",
                (unsigned)imu_index, (double)(_integrity.trust * 100.0f));
            _integrity.last_msg_ms = now_ms;
        }

        _integrity.pos_noise_scale = 1.0f;
        _integrity.vel_noise_scale = 1.0f;
    } else {
        float scale = 1.0f / MAX(_integrity.trust, 0.25f);
        _integrity.pos_noise_scale = scale;
        _integrity.vel_noise_scale = scale;

        if (_integrity.trust < 0.5f && (now_ms - _integrity.last_msg_ms > 3000)) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                "EKF3 IMU%u: GPS trust=%.0f%% div=%.1f scr=%.1f",
                (unsigned)imu_index, (double)(_integrity.trust * 100.0f),
                (double)max_raw, (double)max_score);
            _integrity.last_msg_ms = now_ms;
        }
    }
}
