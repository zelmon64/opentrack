#pragma once

#include "cv/translation-calibrator.hpp"
#include "api/plugin-support.hpp"

enum yaw_action {
    y_center, y_left, y_right,
};

enum pitch_action {
    p_center, p_top, p_bottom,
};

struct action
{
    yaw_action yaw_state;
    pitch_action pitch_state;
};


