/* Copyright (c) 2012-2015 Stanislaw Halik <sthalik@misaki.pl>
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 */

/*
 * this file appeared originally in facetracknoir, was rewritten completely
 * following opentrack fork.
 *
 * originally written by Wim Vriend.
 */


#include "tracker.h"
#include <cmath>
#include <algorithm>

#if defined(_WIN32)
#   include <windows.h>
#endif

constexpr double Tracker::pi;
constexpr double Tracker::r2d;
constexpr double Tracker::d2r;

constexpr double Tracker::c_mult;
constexpr double Tracker::c_div;

Tracker::Tracker(Mappings &m, SelectedLibraries &libs, TrackLogger &logger) :
    m(m),
    newpose {0,0,0, 0,0,0},
    centerp(s.center_at_startup),
    enabledp(true),
    zero_(false),
    should_quit(false),
    libs(libs),
    logger(logger),
    r_b(get_camera_offset_matrix(c_div).t()),
    r_b_real(get_camera_offset_matrix(1).t())
{
}

Tracker::~Tracker()
{
    should_quit = true;
    wait();
}

Tracker::rmat Tracker::get_camera_offset_matrix(double c)
{
    const double off[] =
    {
        d2r * c * (double)-s.camera_yaw,
        d2r * c * (double)-s.camera_pitch,
        d2r * c * (double)-s.camera_roll
    };

    return euler::euler_to_rmat(off);
}

double Tracker::map(double pos, Mapping& axis)
{
    bool altp = (pos < 0) && axis.opts.altp;
    axis.curve.setTrackingActive(!altp);
    axis.curveAlt.setTrackingActive(altp);
    Map& fc = altp ? axis.curveAlt : axis.curve;
    return fc.getValue(float(pos));
}

void Tracker::t_compensate(const rmat& rmat, const vec3& xyz_, vec3& output, bool rz)
{
    // TY is really yaw axis. need swapping accordingly.
    const vec3 ret = rmat * vec3(xyz_(TZ), -xyz_(TX), -xyz_(TY));
    if (!rz)
        output(2) = ret(0);
    else
        output(2) = xyz_(2);
    output(1) = -ret(2);
    output(0) = -ret(1);
}

void Tracker::stage1_camera_offset(Tracker::state& s)
{
    s.camera_offset_real = get_camera_offset_matrix(1);
    s.camera_offset_scaled = get_camera_offset_matrix(c_div);

    s.check_nan(s.camera_offset_real);
    s.check_nan(s.camera_offset_scaled);
}

void Tracker::stage2_raw(state& st)
{
    for (unsigned i = 0; i < 6; i++)
    {
        Mapping& axis = m(i);
        const int k = axis.opts.src;
        if (k < 0 || k >= 6)
            st.pose(i) = 0;
        else
            st.pose(i) = st.tracker_input(k);
    }

    for (unsigned i = 3; i < 6; i++)
        if (std::fabs(st.pose(i)) > 180)
            st.pose(i) = std::copysign(360. - std::fabs(st.pose(i)), -st.pose(i));

    logger.write_pose(st.tracker_input); // raw

    st.check_nan(st.tracker_input);
    st.check_nan(st.pose);
}

void Tracker::stage3_rmat(state& st)
{
    vec3 tmp = d2r * vec3(&st.pose[Yaw]);

    using namespace euler;

    st.r_pose_real = euler_to_rmat(tmp);
    st.r_pose_scaled = euler_to_rmat(c_div * tmp);

    st.check_nan(st.r_pose_real);
    st.check_nan(st.r_pose_scaled);
}

void Tracker::stage4_apply_camera_offset(state& st)
{
    st.r_pose_real = st.r_pose_real * st.camera_offset_real;
    st.r_pose_scaled = st.r_pose_scaled * st.camera_offset_scaled;
}

void Tracker::stage5_maybe_set_center(state& st)
{
    const bool do_center_now = centerp &&
            progn(
                for (unsigned i = 0; i < 6; i++)
                    if (st.tracker_input(i) != 0.0)
                        return true;
                return false;
            );

    if (do_center_now)
    {
        centerp = false;

        if (libs.pFilter)
            libs.pFilter->center();

        if (libs.pTracker->center())
        {
            st.r_center_real = st.camera_offset_real.t();
            st.r_center_scaled = st.camera_offset_scaled.t();
            st.r_pose_real = rmat::eye();
            st.r_pose_scaled = rmat::eye();
        }
        else
        {
            st.r_center_real = st.r_pose_real.t();
            st.r_center_scaled = st.r_pose_scaled.t();
        }

        st.t_center = vec3(&st.pose[0]);
    }
}

void Tracker::stage6_center(state& st)
{
    switch (s.center_method)
    {
    // inertial
    case 0:
    default:
        st.r_pose_real = st.r_center_real * st.r_pose_real;
        st.r_pose_scaled = st.r_center_scaled * st.r_pose_scaled;
        break;
        // camera
    case 1:
        st.r_pose_real = st.r_pose_real * st.r_center_real;
        st.r_pose_scaled = st.r_pose_scaled * st.r_center_scaled;
        break;
    }
}

void Tracker::stage7_update_euler(state& st)
{
    using namespace euler;

    const vec3 rot(r2d * c_mult * rmat_to_euler(st.r_pose_scaled));
    vec3 pos(vec3(&st.pose[0]) - st.t_center);

    if (s.use_camera_offset_from_centering)
        t_compensate(st.r_center_real.t() * st.camera_offset_real.t(), pos, pos, false);
    else
        t_compensate(st.camera_offset_real.t(), pos, pos, false);

    st.check_nan(rot);
    st.check_nan(pos);

    for (int i = 0; i < 3; i++)
    {
        st.pose(i) = pos(i);
        st.pose(i+3) = rot(i);
    }

    logger.write_pose(st.pose); // "corrected" - after various transformations to account for camera position
}

void Tracker::stage8_filter(state& st)
{
    const vec6 tmp(st.pose);

    if (libs.pFilter)
        libs.pFilter->filter(tmp, st.pose);

    st.check_nan(st.pose);

    logger.write_pose(st.pose);
}

void Tracker::stage9_map_rotation(state& st)
{
    // CAVEAT rotation only, due to tcomp
    for (unsigned i = 3; i < 6; i++)
        st.pose(i) = map(st.pose(i), m(i));

    st.check_nan(st.pose);
}

// CAVEAT translation only, due to tcomp
void Tracker::stage10_map_translation(state& st)
{
    if (s.tcomp_p)
    {
        vec3 value_(st.pose(TX), st.pose(TY), st.pose(TZ));

        using namespace euler;

        t_compensate(euler_to_rmat(vec3(st.pose(Yaw) * d2r, st.pose(Pitch) * d2r, st.pose(Roll) * d2r)),
                     value_,
                     value_,
                     s.tcomp_tz);

        st.check_nan(value_);

        for (unsigned i = 0; i < 3; i++)
            st.pose(i) = value_(i);
    }

    for (unsigned i = 0; i < 3; i++)
        st.pose(i) = map(st.pose(i), m(i));

    st.check_nan(st.pose);

    // logger.write_pose(value); // "mapped"
}

void Tracker::stage11_transform(state& st)
{
    for (unsigned i = 0; i < 6; i++)
        st.pose(i) += m(i).opts.zero;

    for (unsigned i = 0; i < 6; i++)
        st.pose(i) *= int(m(i).opts.invert) * -2 + 1;

    if (zero_)
        for (unsigned i = 0; i < 6; i++)
            st.pose(i) = 0;
}

void Tracker::run_pipeline()
{
    {

    }

    // whenever something can corrupt its internal state due to nan/inf, elide the call
    if (is_nan(value))
    {
        nan = true;
         // "filtered"
    }
    else
    {
        {

        }
        logger.write_pose(value); // "filtered"



        if (is_nan(value))
            nan = true;
    }

    if (s.tcomp_p)
    {
        euler_t value_(value(TX), value(TY), value(TZ));
        t_compensate(euler_to_rmat(euler_t(value(Yaw) * d2r, value(Pitch) * d2r, value(Roll) * d2r)),
                     value_,
                     value_,
                     s.tcomp_tz);
        if (is_nan(r, value_))
            nan = true;
        for (int i = 0; i < 3; i++)
            value(i) = value_(i);
    }

    // CAVEAT translation only, due to tcomp
    for (int i = 0; i < 3; i++)
        value(i) = map(value(i), m(i));



    if (nan)
    {
        value = last_mapped;

        // for widget last value display
        for (int i = 0; i < 6; i++)
            (void) map(value(i), m(i));
    }

    logger.next_line();

    libs.pProtocol->pose(value);

    last_mapped = value;
    last_raw = raw;

    QMutexLocker foo(&mtx);
    output_pose = value;
    raw_6dof = raw;
}

void Tracker::run()
{
    const int sleep_ms = 3;

#if defined(_WIN32)
    (void) timeBeginPeriod(1);
#endif

    {
        const char* posechannels[6] = { "TX", "TY", "TZ", "Yaw", "Pitch", "Roll" };
        const char* datachannels[5] = { "dt", "raw", "corrected", "filtered", "mapped" };
        logger.write(datachannels[0]);
        char buffer[128];
        for (int j = 1; j < 5; ++j)
        {
            for (int i = 0; i < 6; ++i)
            {
                snprintf(buffer, 128, "%s%s", datachannels[j], posechannels[i]);
                logger.write(buffer);
            }
        }
    }
    logger.next_line();

    while (!should_quit)
    {
        {
            double dt = t.elapsed_seconds();
            logger.write(&dt, 1);
        }
        t.start();

        double tmp[6] {0,0,0, 0,0,0};
        libs.pTracker->data(tmp);

        if (enabledp)
            for (int i = 0; i < 6; i++)
                newpose[i] = elide_nan(tmp[i], newpose[i]);

        logic();

        long q = long(sleep_ms * 1000L - t.elapsed()/1000L);
        using std::max;
        using ulong = unsigned long;
        usleep(ulong(max(1L, q)));
    }

    {
        // filter may inhibit exact origin
        vec6 p;
        libs.pProtocol->pose(p);
    }

#if defined(_WIN32)
    (void) timeEndPeriod(1);
#endif

    for (int i = 0; i < 6; i++)
    {
        m(i).curve.setTrackingActive(false);
        m(i).curveAlt.setTrackingActive(false);
    }
}

void Tracker::get_raw_and_mapped_poses(double* mapped, double* raw) const
{
    QMutexLocker foo(&const_cast<Tracker&>(*this).mtx);

    for (int i = 0; i < 6; i++)
    {
        raw[i] = raw_6dof(i);
        mapped[i] = output_pose(i);
    }
}

