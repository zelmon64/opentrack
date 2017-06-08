/* Copyright (c) 2015, Stanislaw Halik <sthalik@misaki.pl>

 * Permission to use, copy, modify, and/or distribute this
 * software for any purpose with or without fee is hereby granted,
 * provided that the above copyright notice and this permission
 * notice appear in all copies.
 */

#pragma once

#include <QString>
#include "options/options.hpp"
#include "api/plugin-api.hpp"

using namespace options;

#include "export.hpp"

namespace axis_clamp_opts
{

} // ns axis-clamp-opts

struct OTR_LOGIC_EXPORT axis_opts final
{
    enum max_clamp
    {
        r180 = 180,
        r90 = 90,
        r60 = 60,
        r45 = 45,
        r30 = 30,
        r25 = 25,
        r20 = 20,
        r15 = 15,
        r10 = 10,

        t100 = 100,
        t30 = 30,
        t20 = 20,
        t15 = 15,
        t10 = 10,
    };

    // note, these two bundles can be the same value with no issues
    bundle b_settings_window, b_mapping_window;
    double zero;
    int src;
    bool invert;
    bool altp;
    max_clamp clamp;
    axis_opts(bundle b_settings_window, bundle b_mapping_window, QString pfx, Axis idx) :
        b_settings_window(b_settings_window),
        b_mapping_window(b_mapping_window),
        zero(0),
        src(idx),
        invert(false),
        altp(false),
        clamp(idx >= Yaw ? r180 : t30)
    {}
private:
    static inline QString n(QString pfx, QString name);
};

struct OTR_LOGIC_EXPORT key_opts
{
    value<QString> keycode, guid;
    value<int> button;

    key_opts(bundle b, const QString& name);
};

struct OTR_LOGIC_EXPORT module_settings
{
    bundle b;
    value<QString> tracker_dll, filter_dll, protocol_dll;
    module_settings();
};

struct OTR_LOGIC_EXPORT main_settings final
{
    bundle b, b_map;
    axis_opts a_x, a_y, a_z;
    axis_opts a_yaw, a_pitch, a_roll;
    value<bool> tray_enabled, tray_start;
    key_opts key_start_tracking1, key_start_tracking2;
    key_opts key_stop_tracking1, key_stop_tracking2;
    key_opts key_toggle_tracking1, key_toggle_tracking2;
    key_opts key_restart_tracking1, key_restart_tracking2;
    key_opts key_center1, key_center2;
    key_opts key_toggle1, key_toggle2;
    main_settings();
};
