/* Copyright (c) 2014-2015, Stanislaw Halik <sthalik@misaki.pl>

 * Permission to use, copy, modify, and/or distribute this
 * software for any purpose with or without fee is hereby granted,
 * provided that the above copyright notice and this permission
 * notice appear in all copies.
 */

#pragma once

#include <vector>

#include "opentrack-compat/pi-constant.hpp"
#include "opentrack-compat/nan.hpp"
#include "opentrack-compat/timer.hpp"
#include "opentrack/plugin-support.hpp"
#include "mappings.hpp"
#include "simple-mat.hpp"
#include "selected-libraries.hpp"

#include "spline-widget/functionconfig.h"
#include "main-settings.hpp"
#include "opentrack-compat/options.hpp"
#include "tracklogger.hpp"

#include <QMutex>
#include <QThread>

#include <new>

#include "export.hpp"

class OPENTRACK_LOGIC_EXPORT Tracker : private QThread
{
    Q_OBJECT

    using vec6 = Mat<double, 6, 1>;
    using rmat = euler::rmat;
    using vec3 = euler::euler_t;

    static constexpr double pi = OPENTRACK_PI;
    static constexpr double r2d = 180. / OPENTRACK_PI;
    static constexpr double d2r = OPENTRACK_PI / 180.;

    // note: float exponent base is 2
    static constexpr double c_mult = 4;
    static constexpr double c_div = 1./c_mult;

    class state final
    {
    public:
        rmat r_center_scaled, r_center_real;
        rmat r_pose_scaled, r_pose_real;
        rmat camera_offset_scaled, camera_offset_real;
        vec3 t_center;
        vec6 pose, tracker_input;

    private:
        bool is_nan_;

    public:
        state() :
            r_center_scaled(rmat::eye()),
            r_center_real(rmat::eye()),
            r_pose_scaled(rmat::eye()),
            r_pose_real(rmat::eye()),
            camera_offset_scaled(rmat::eye()),
            camera_offset_real(rmat::eye()),
            is_nan_(false)
        {
        }

        bool is_nan() const { return is_nan_; }

        void reset() { new (this) state(); }

        template<typename t> t& check_nan(t& val) { is_nan_ |= is_nan(val); return val; }
    };

    void stage1_camera_offset(state& st);
    void stage2_raw(state& st);
    void stage3_rmat(state& st);
    void stage4_apply_camera_offset(state& st);
    void stage5_maybe_set_center(state& st);
    void stage6_center(state& st);
    void stage7_update_euler(state& st);
    void stage8_filter(state& st);
    void stage9_map_rotation(state& st);
    void stage11_transform(state& st);
    void stage10_map_translation(state& st);

    void run_pipeline();

    QMutex mtx;
    main_settings s;
    Mappings& m;

    Timer t;
    state last_state;

    volatile bool centerp;
    volatile bool enabledp;
    volatile bool zero_;
    volatile bool should_quit;
    SelectedLibraries const& libs;
    // The owner of the reference is the main window.
    // This design might be usefull if we decide later on to swap out
    // the logger while the tracker is running.
    TrackLogger &logger;

    double map(double pos, Mapping& axis);
    static void t_compensate(const rmat& rmat, const vec3& ypr, vec3& output, bool rz);
    void logic();
    void run() override;

    template<int h, int w>
    static bool is_nan(const Mat<double, h, w>& r)
    {
        for (unsigned i = 0; i < h; i++)
            for (unsigned j = 0; j < w; j++)
                if (nanp(r(i, j)))
                    return true;
        return false;
    }

public:
    Tracker(Mappings& m, SelectedLibraries& libs, TrackLogger &logger);
    ~Tracker();

    rmat get_camera_offset_matrix(double c);
    void get_raw_and_mapped_poses(double* mapped, double* raw) const;
    void start() { QThread::start(); }

    void set_center() { centerp = !centerp; }
    void set_toggle_pressed() { enabledp = !enabledp; }
    void set_toggle_held(bool value) { enabledp = value; }
    void set_zero_held(bool value) { zero_ = value; }
    void set_zero_pressed() { zero_ = !zero_; }
};
