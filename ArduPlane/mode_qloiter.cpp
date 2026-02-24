#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

bool ModeQLoiter::_enter()
{
    quadplane.throttle_wait = false;
    return true;
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
