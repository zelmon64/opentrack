#include "main-settings.hpp"

main_settings::main_settings() :
    b(make_bundle("opentrack-ui")),
    b_map(make_bundle("opentrack-mappings")),
    a_x(b, b_map, "x", TX),
    a_y(b, b_map, "y", TY),
    a_z(b, b_map, "z", TZ),
    a_yaw(b, b_map, "yaw", Yaw),
    a_pitch(b, b_map, "pitch", Pitch),
    a_roll(b, b_map, "roll", Roll),
    tray_enabled(b, "use-system-tray", false),
    tray_start(b, "start-in-tray", false),
    key_start_tracking1(b, "start-tracking"),
    key_start_tracking2(b, "start-tracking-alt"),
    key_stop_tracking1(b, "stop-tracking"),
    key_stop_tracking2(b, "stop-tracking-alt"),
    key_toggle_tracking1(b, "toggle-tracking"),
    key_toggle_tracking2(b, "toggle-tracking-alt"),
    key_restart_tracking1(b, "restart-tracking"),
    key_restart_tracking2(b, "restart-tracking-alt"),
    key_center1(b, "center"),
    key_center2(b, "center-alt"),
    key_toggle1(b, "toggle"),
    key_toggle2(b, "toggle-alt")
{
}

module_settings::module_settings() :
    b(make_bundle("modules")),
    tracker_dll(b, "tracker-dll", "PointTracker 1.1"),
    filter_dll(b, "filter-dll", "Accela"),
    protocol_dll(b, "protocol-dll", "freetrack 2.0 Enhanced")
{
}

key_opts::key_opts(bundle b, const QString& name) :
    keycode(b, QString("keycode-%1").arg(name), ""),
    guid(b, QString("guid-%1").arg(name), ""),
    button(b, QString("button-%1").arg(name), -1)
{}

