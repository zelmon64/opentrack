#include "wizard-logic.hpp"

static constexpr action actions[] = {
    // centered, yaw left/right
    { y_left, p_center, },
    { y_center, p_center },
    { y_right, p_center },

    // upward, yaw left/right
    { y_center, p_center },
    { y_center, p_top },
    { y_left, p_top },
    { y_center, p_top },
    { y_right, p_top },

    // downward, yaw left/right
    { y_center, p_center },
    { y_center, p_bottom },
    { y_left, p_bottom },
    { y_center, p_bottom },
    { y_right, p_bottom },

    // done
    { y_center, p_center },
};

