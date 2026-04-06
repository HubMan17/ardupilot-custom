#include "mode.h"
#include "Plane.h"
#include <AP_AHRS/AP_AHRS.h>

#if HAL_QUADPLANE_ENABLED

bool ModeQLoiter::_enter()
{
    quadplane.throttle_wait = false;
    loiter_nav->init_target();
    quadplane._ahl_loiter_active = false;
    return true;
}

void ModeQLoiter::_exit()
{
    // Обязательно вернуть EKF на исходный source при выходе из QLoiter
    if (AP::ahrs().get_posvelyaw_source_set() == 2) {
        AP::ahrs().set_posvelyaw_source_set(quadplane._ahl_prev_ekf_src);
    }
    quadplane._ahl_loiter_active = false;
}

void ModeQLoiter::update()
{
    plane.mode_qstabilize.update();
}

// run QLoiter: stick-based attitude control + auto-help descent limiting
void ModeQLoiter::run()
{
    if (quadplane.esc_calibration != 0) {
        quadplane.run_esc_calibration();
        return;
    }

    float pilot_throttle_scaled = quadplane.get_pilot_throttle();
    quadplane.hold_auto_help_land(pilot_throttle_scaled);
}

#endif
