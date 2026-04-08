# Anti-Spoof System — Changelog & Known Issues

## Architecture Overview

Three-echelon GPS anti-spoofing system for ArduPlane quadplane:

| Echelon | Component | Rate | Scope |
|---------|-----------|------|-------|
| 1 | EKF3 pre-filter (`AP_NavEKF3_Integrity.cpp`) | GPS rate, per-core | Inside EKF, before fusion |
| 2 | IntegrityFilter (`AP_IntegrityFilter.cpp`) | 10 Hz | Outside EKF, cross-checks |
| 3 | Antispoof lockdown (`antispoof.cpp`) | On-demand | Mode/sensor management |

### Per-Core EKF Architecture

```
Normal:    Core 0 (GPS, primary)  |  Core 1 (GPS)
Lockdown:  Core 0 (GPS, pre-filter rejects spoof)  |  Core 1 (DR, primary)
Recovery:  Core 0 reconverges with clean GPS → disengage when Core 0 trust > 80%
```

Key custom additions to AP_NavEKF3 (ported/written for v4.4.3):
- `setCoreNoGPS(core, bool)` — per-core GPS fusion disable
- `forcePrimaryCore(core)` — override automatic lane selection
- `getCoreIntegrityTrust(core)` — read pre-filter trust per core
- `_force_no_gps` flag in `NavEKF3_core` — blocks GPS fusion in `SelectVelPosFusion`
- `_forced_primary` in `NavEKF3` — skips automatic core selection when >= 0
- `_integrity` struct in `NavEKF3_core` — full pre-filter state per core

### Key Files

- `ArduPlane/antispoof.cpp` — engage/disengage lockdown
- `ArduPlane/AP_IntegrityFilter.h/.cpp` — echelon 2 detection + trust + snapshots
- `ArduPlane/Plane.h:1228-1233` — state variables
- `libraries/AP_NavEKF3/AP_NavEKF3_Integrity.cpp` — echelon 1 pre-filter (8 checks + CUSUM)
- `libraries/AP_NavEKF3/AP_NavEKF3_core.h:1513-1558` — `_force_no_gps`, `_integrity` struct
- `libraries/AP_NavEKF3/AP_NavEKF3.h:382-396` — per-core API + `_forced_primary`
- `libraries/AP_NavEKF3/AP_NavEKF3.cpp:937` — `_forced_primary` blocks auto lane switch
- `libraries/AP_NavEKF3/AP_NavEKF3.cpp:1110-1142` — per-core methods
- `libraries/AP_NavEKF3/AP_NavEKF3_PosVelFusion.cpp:561-565` — `_force_no_gps` blocks fusion

---

## Session: 2026-04-07/08 — Critical Fixes

### Problem 1: Pre-filter trust reset to 1.0 (ROOT CAUSE)

**File:** `AP_NavEKF3_Integrity.cpp:228-234`

**Bug:** When airspeed checks are not available (VTOL transition, airspeed < 15 m/s) AND
CUSUM innovation score < 1.0, the pre-filter unconditionally reset `trust = 1.0`.

**Impact:** During spoofing, trust would drop to ~45% → aircraft does VTOL transition →
airspeed drops → `trust = 1.0` reset → after transition, baseline needs 5s warmup →
trust drops again → next transition resets again. Trust NEVER reached rejection threshold.
This caused the "trust oscillating at 45%" pattern seen in all SITL tests.

**Fix:** Removed unconditional `trust = 1.0`. Now preserves current trust and only sets
noise scaling to 1× if trust was never degraded. Natural recovery (0.1/s) handles
restoration when GPS is actually clean.

### Problem 2: Pre-filter rejection threshold too high (5%)

**File:** `AP_NavEKF3_Integrity.cpp:351-370`

**Bug:** GPS was only fully rejected when pre-filter trust < 5%. At trust=45% (typical
during spoof), noise was scaled by only 2.2× — spoofed GPS still fused and corrupted
EKF state.

**Fix:** Lowered rejection threshold from 5% to 25%. At 25%, spoof is confirmed with
high confidence. Noise scaling capped at 4× max. Baseline reset also at 25%.

### Problem 3: IntegrityFilter WARNING mode 10× slower decay

**File:** `AP_IntegrityFilter.cpp:486` (old line)

**Bug:** In WARNING/EMERGENCY mode, decay rate had a `* 0.1f` multiplier:
`_trust -= _decay_rate * 0.1f * dt` → 0.01/s instead of 0.1/s.
This stalled trust at 18% for ~3 seconds, during which the aircraft crashed.

**Fix:** Removed 0.1× multiplier. Now uses full decay rate, with additional logic:
if Core 0 pre-filter still shows bad GPS, trust is pinned to `emerg_thresh * 0.5`.

### Problem 4: Lockdown/recovery oscillation

**File:** `antispoof.cpp:55-117`, `AP_IntegrityFilter.cpp:474-500`

**Bug:** During active spoofing:
1. `recovery_divergence = 0.0` (metric broken — both velocity deltas ≈ 0 during steady flight)
2. After 10s of "good" recovery + 7.5s trust increase → trust reaches 15% → de-escalate
3. `disengage_spoof_emergency()` restores GPS fusion → EKF immediately poisoned → crash
4. Re-engage → repeat cycle every ~24s, each cycle risking crash

**Observed pattern:**
```
21:57:08  ENGAGE
21:57:32  DISENGAGE → immediate re-ENGAGE
21:57:41  DISENGAGE → immediate re-ENGAGE
21:57:43  ENGAGE
21:58:05  DISENGAGE → SIM Hit ground → re-ENGAGE
```

**Fix (3 parts):**

**A. Recovery requires Core 0 pre-filter trust > 80%:**
Core 0 keeps GPS fusion during lockdown (pre-filter rejects spoofed data). If Core 0
trust is still low, GPS is still bad — recovery blocked. Trust pinned to `emerg_thresh * 0.5`.

**B. Minimum lockdown increased from 5s to 30s:**
Prevents rapid oscillation. Old 5s was insufficient.

**C. `disengage_spoof_emergency()` validates Core 0 before restoring GPS:**
Returns `bool` (false = blocked). Checks:
- Minimum 30 seconds elapsed
- Core 0 pre-filter trust > 80% (GPS demonstrably clean)

`execute_level_change()` also returns `bool` — if disengage blocked, level stays EMERGENCY.

### Problem 5: Detection too slow (echelon 2 lagged echelon 1 by ~60s)

**File:** `AP_IntegrityFilter.cpp:442-470, 547-565`

**Bug:** Pre-filter detected spoof at T+0s (trust=46%), but IntegrityFilter didn't
reach EMERGENCY until T+69s. Three causes:
1. IntegrityFilter runs different checks (wind shift baseline), less sensitive
2. Level escalation requires 1s hysteresis per level (×3 levels = 3s minimum)
3. No cross-echelon communication

**Fix (2 parts):**

**A. Pre-filter shortcut:** If both EKF cores show pre-filter trust < 40% sustained
for > 2s → instant EMERGENCY lockdown, bypassing IntegrityFilter trust decay entirely.
Requires `activeCores() >= 2` to prevent false trigger on single-core configs.

**B. Rapid-drop hysteresis bypass:** If IntegrityFilter trust drops > 50% in 5 seconds,
skip 1s hysteresis for EMERGENCY — engage immediately.

---

## Test Results (2026-04-07)

### Test 1: Before all fixes (firmware 1ef9e469)

**Scenario:** SITL Circle spoof, Extreme ~2km, Fast

| Metric | Result |
|--------|--------|
| First pre-filter detection | T+0s (trust=46%) |
| IntegrityFilter detection | T+60s |
| Lockdown engage | T+69s |
| Aircraft crash | T+12s (before lockdown!) |
| Outcome | **CRASH** — lockdown 4s after crash |

### Test 2: Before all fixes, strong spoof

| Metric | Result |
|--------|--------|
| Lockdown engage | ~5s after detection |
| Lockdown/recovery oscillation | Every ~24s |
| SIM Hit ground events | Multiple (3.3 m/s, 15.8 m/s) |
| Outcome | **CRASH** — oscillating engage/disengage |

### Test 3: After fixes (pending validation)

**Expected behavior:**
```
T+0s:   Pre-filter sees div=17, score=4.5, trust drops
T+1s:   Pre-filter trust ≈ 25% → GPS REJECTED on both cores
T+2s:   Pre-filter shortcut: both cores < 40% for >2s → EMERGENCY
T+2s:   Core 1 DR primary, Core 0 GPS monitoring, mode FBWA
T+30s+: Minimum lockdown holds
T+??:   Spoof stops → Core 0 trust recovers > 80% → disengage allowed
```

---

## Known Issues / TODO

1. **Recovery divergence metric is broken** — `compute_recovery_divergence()` returns 0.0
   during steady spoofing because both GPS and AHRS velocity deltas are small. Currently
   mitigated by Core 0 pre-filter gate, but the metric itself needs rethinking.

2. **SITL Fault Injection persistence** — After stopping the spoof tool, GPS data may
   remain corrupted until SITL is fully restarted. The spoof tool must be disconnected
   and SITL restarted for clean state.

3. **Airspeed disable during lockdown** — Currently disables pitot. This may be too
   conservative for GPS-only spoofing scenarios. Consider only disabling if airspeed
   itself shows anomalies.

4. **Single GPS receiver** — System assumes single GPS. Dual-GPS cross-check not implemented.

5. **Pre-filter `gps_dt` hardcoded** — `gps_dt = 0.1f` assumes 10Hz GPS. Should derive
   from actual GPS update rate.

---

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| INTEG_ENABLE | 0 | Enable integrity filter (0/1) |
| INTEG_VEL_THR | 3.0 | Divergence threshold for trust decay |
| INTEG_DECAY | 0.1 | Trust decay rate (1/s) when divergent |
| INTEG_RECOVER | 0.02 | Trust recovery rate (1/s) when convergent |
| INTEG_CAUT_THR | 0.7 | Trust threshold for CAUTION level |
| INTEG_WARN_THR | 0.4 | Trust threshold for WARNING level |
| INTEG_EMER_THR | 0.15 | Trust threshold for EMERGENCY level |

Pre-filter thresholds are compile-time `#define` in `AP_NavEKF3_Integrity.cpp`.
