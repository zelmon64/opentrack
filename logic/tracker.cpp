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

#include "compat/sleep.hpp"
#include "compat/util.hpp"
#include "compat/timer-resolution.hpp"

#include "tracker.h"

#include <cmath>
#include <algorithm>
#include <cstdio>

using namespace euler;
using namespace gui_tracker_impl;
using namespace time_units;

constexpr double Tracker::r2d;
constexpr double Tracker::d2r;

Tracker::Tracker(Mappings& m, SelectedLibraries& libs, TrackLogger& logger) :
    m(m),
    libs(libs),
    logger(logger),
    backlog_time(ns(0)),
    tracking_started(false)
{
}

Tracker::~Tracker()
{
    set(f_should_quit, true);
    wait();
}

Tracker::rmat Tracker::camera_offset(double c)
{
    const double off[] =
    {
        0, 0, 0,
    };

    return euler::euler_to_rmat(off);
}

double Tracker::map(double pos, Map& axis)
{
    bool altp = (pos < 0) && axis.opts.altp;
    axis.spline_main.set_tracking_active( !altp );
    axis.spline_alt.set_tracking_active( altp );
    auto& fc = altp ? axis.spline_alt : axis.spline_main;
    return double(fc.get_value(pos));
}

void Tracker::t_compensate(const rmat& rmat, const euler_t& xyz, euler_t& output,
                           bool disable_tx, bool disable_ty, bool disable_tz)
{
    enum { tb_Z, tb_X, tb_Y };

    // TY is really yaw axis. need swapping accordingly.
    // sign changes are due to right-vs-left handedness of coordinate system used
    const euler_t ret = rmat * euler_t(xyz(TZ), -xyz(TX), -xyz(TY));

    if (disable_tz)
        output(TZ) = xyz(TZ);
    else
        output(TZ) = ret(tb_Z);

    if (disable_ty)
        output(TY) = xyz(TY);
    else
        output(TY) = -ret(tb_Y);

    if (disable_tx)
        output(TX) = xyz(TX);
    else
        output(TX) = -ret(tb_X);
}

#include "compat/nan.hpp"

static inline double elide_nan(double value, double def)
{
    if (nanp(value))
    {
        if (nanp(def))
            return 0;
        return def;
    }
    return value;
}

template<int u, int w>
static bool is_nan(const dmat<u,w>& r)
{
    for (int i = 0; i < u; i++)
        for (int j = 0; j < w; j++)
            if (nanp(r(i, j)))
                return true;

    return false;
}

constexpr double Tracker::c_mult;
constexpr double Tracker::c_div;

void Tracker::logic()
{
    using namespace euler;

    logger.write_dt();
    logger.reset_dt();

    const bool center_ordered = get(f_center) && tracking_started;
    set(f_center, false);
    const bool own_center_logic = center_ordered && libs.pTracker->center();

    {
        Pose tmp;
        libs.pTracker->data(tmp);

        if (get(f_enabled_p) ^ !get(f_enabled_h))
            for (int i = 0; i < 6; i++)
                newpose[i] = elide_nan(tmp(i), newpose(i));
    }

    Pose value, raw;

    for (int i = 0; i < 6; i++)
    {
        auto& axis = m(i);
        int k = axis.opts.src;
        if (k < 0 || k >= 6)
            value(i) = 0;
        else
            value(i) = newpose(k);
        raw(i) = newpose(i);
    }

    // hatire, udp, and freepie trackers can mess up here
    for (unsigned i = 3; i < 6; i++)
    {
        using std::fmod;
        using std::copysign;
        using std::fabs;

        const double x = value(i);
        if (fabs(x) - 1e-2 > 180)
            value(i) = fmod(x + copysign(180, x), 360) - copysign(180, x);
        else
            value(i) = clamp(x, -180, 180);
    }

    logger.write_pose(raw); // raw

    bool nanp = is_nan(raw) | is_nan(value);

    // TODO split this function, it's too big

    {
        euler_t tmp = d2r * euler_t(&value[Yaw]);
        scaled_rotation.rotation = euler_to_rmat(c_div * tmp);
        real_rotation.rotation = euler_to_rmat(tmp);
    }

    scaled_rotation.camera = camera_offset(c_div);
    real_rotation.camera = camera_offset(1);

    nanp |= is_nan(value) || is_nan(scaled_rotation.rotation) || is_nan(real_rotation.rotation);

    if (!tracking_started)
    {
        using std::fabs;

        for (int i = 0; i < 6; i++)
            if (fabs(newpose(i)) != 0)
            {
                tracking_started = true;
                break;
            }

        tracking_started &= !nanp;

        if (tracking_started)
        {
            set(f_center, true);
        }
    }

    if (center_ordered)
    {
        if (libs.pFilter)
            libs.pFilter->center();

        if (own_center_logic)
        {
            scaled_rotation.rot_center = rmat::eye();
            real_rotation.rot_center = rmat::eye();

            t_center = euler_t();
        }
        else
        {
            real_rotation.rot_center = real_rotation.rotation.t();
            scaled_rotation.rot_center = scaled_rotation.rotation.t();

            t_center = euler_t(&value(TX));
        }
    }

    {
        rmat rotation;

        switch (1)
        {
        // inertial
        case 0:
            //scaled_rotation.rotation = scaled_rotation
            rotation = scaled_rotation.rot_center * scaled_rotation.rotation;
            break;
        // camera
        default:
        case 1:
            rotation = scaled_rotation.rotation * scaled_rotation.rot_center;
            break;
        }

        euler_t rot = r2d * c_mult * rmat_to_euler(rotation);
        euler_t pos = euler_t(&value[TX]) - t_center;

        for (int i = 0; i < 3; i++)
        {
            // don't invert after t_compensate
            // inverting here doesn't break centering

            if (m(i+3).opts.invert)
                rot(i) = -rot(i);
            if (m(i).opts.invert)
                pos(i) = -pos(i);
        }

        t_compensate(real_rotation.camera.t(), pos, pos, false, false, false);

        for (int i = 0; i < 3; i++)
        {
            value(i) = pos(i);
            value(i+3) = rot(i);
        }
    }

    logger.write_pose(value); // "corrected" - after various transformations to account for camera position

    nanp |= is_nan(value);

    {
        {
            Pose tmp(value);

            // nan/inf values will corrupt filter internal state
            if (!nanp && libs.pFilter)
                libs.pFilter->filter(tmp, value);

            logger.write_pose(value); // "filtered"
        }
    }

    nanp |= is_nan(value);

    {
#if 0
        euler_t neck, rel;

        if (false)
        {
            double nz = -s.neck_z;

            if (nz != 0)
            {
                const rmat R = euler_to_rmat(
                       euler_t(value(Yaw)   * d2r,
                               value(Pitch) * d2r,
                               value(Roll)  * d2r));
                euler_t xyz(0, 0, nz);
                t_compensate(R, xyz, xyz, false, false, false);
                neck(TX) = xyz(TX);
                neck(TY) = xyz(TY);
                neck(TZ) = xyz(TZ) - nz;
            }
        }

        // CAVEAT rotation only, due to tcomp
        for (int i = 3; i < 6; i++)
            value(i) = map(value(i), m(i));

        if (s.tcomp_p)
        {
            const double tcomp_c[] =
            {
                double(!s.tcomp_disable_src_yaw),
                double(!s.tcomp_disable_src_pitch),
                double(!s.tcomp_disable_src_roll),
            };
            const rmat R = euler_to_rmat(
                       euler_t(value(Yaw)   * d2r * tcomp_c[0],
                               value(Pitch) * d2r * tcomp_c[1],
                               value(Roll)  * d2r * tcomp_c[2]));
            euler_t ret;
            t_compensate(R,
                         euler_t(value(TX), value(TY), value(TZ)),
                         ret,
                         s.tcomp_disable_tx,
                         s.tcomp_disable_ty,
                         s.tcomp_disable_tz);

            for (int i = 0; i < 3; i++)
                rel(i) = ret(i) - value(i);
        }

        // don't t_compensate existing compensated values
        for (int i = 0; i < 3; i++)
            value(i) += neck(i) + rel(i);

        nanp |= is_nan(neck) | is_nan(rel) | is_nan(value);
#endif
    }

    // CAVEAT translation only, due to tcomp
    for (int i = 0; i < 3; i++)
        value(i) = map(value(i), m(i));

    if (nanp)
    {
        QMutexLocker foo(&mtx);

        value = output_pose;
        raw = raw_6dof;

        // for widget last value display
        for (int i = 0; i < 6; i++)
            (void) map(raw_6dof(i), m(i));
    }

    if (get(f_zero))
        for (int i = 0; i < 6; i++)
            value(i) = 0;

#if 0
    // custom zero position
    for (int i = 0; i < 6; i++)
        value(i) += m(i).opts.zero * (m(i).opts.invert ? -1 : 1);
#endif

    if (!nanp)
        libs.pProtocol->pose(value);

    QMutexLocker foo(&mtx);
    output_pose = value;
    raw_6dof = raw;

    logger.write_pose(value); // "mapped"

    logger.reset_dt();
    logger.next_line();
}

void Tracker::run()
{
    setPriority(QThread::HighPriority);

    timer_resolution res(1);

    {
        static constexpr const char* posechannels[6] = { "TX", "TY", "TZ", "Yaw", "Pitch", "Roll" };
        static constexpr const char* datachannels[5] = { "dt", "raw", "corrected", "filtered", "mapped" };
        logger.write(datachannels[0]);
        char buffer[128];
        for (unsigned j = 1; j < 5; ++j)
        {
            for (unsigned i = 0; i < 6; ++i)
            {
                std::sprintf(buffer, "%s%s", datachannels[j], posechannels[i]);
                logger.write(buffer);
            }
        }
        logger.next_line();
    }

    logger.reset_dt();

    t.start();

    while (!get(f_should_quit))
    {
        logic();

        static constexpr ns const_sleep_ms(time_cast<ns>(ms(4)));
        const ns elapsed_nsecs = prog1(t.elapsed<ns>(), t.start());

        backlog_time += ns(elapsed_nsecs - const_sleep_ms);

        if (backlog_time > secs_(3) || backlog_time < secs_(-3))
        {
            qDebug() << "tracker: backlog interval overflow"
                     << time_cast<ms>(backlog_time).count() << "ms";
            backlog_time = backlog_time.zero();
        }

        const int sleep_time_ms = iround(time_cast<ms>(clamp(const_sleep_ms - backlog_time,
                                                             ns(0), ms(50)))
                                         .count());

        portable::sleep(sleep_time_ms);
    }

    {
        // filter may inhibit exact origin
        Pose p;
        libs.pProtocol->pose(p);
    }

    for (int i = 0; i < 6; i++)
    {
        m(i).spline_main.set_tracking_active(false);
        m(i).spline_alt.set_tracking_active(false);
    }
}

void Tracker::raw_and_mapped_pose(double* mapped, double* raw) const
{
    QMutexLocker foo(&const_cast<Tracker&>(*this).mtx);

    for (int i = 0; i < 6; i++)
    {
        raw[i] = raw_6dof(i);
        mapped[i] = output_pose(i);
    }
}

void Tracker::center() { set(f_center, true); }

void Tracker::set_toggle(bool value) { set(f_enabled_h, value); }
void Tracker::set_zero(bool value) { set(f_zero, value); }

void Tracker::zero() { negate(f_zero); }
void Tracker::toggle_enabled() { negate(f_enabled_p); }


void bits::set(bits::flags flag_, bool val_)
{
    const unsigned flag = unsigned(flag_);
    const unsigned val = unsigned(val_);

    for (;;)
    {
        unsigned b_(b);
        if (b.compare_exchange_weak(b_,
                                    unsigned((b_ & ~flag) | (flag * val)),
                                    std::memory_order_seq_cst,
                                    std::memory_order_seq_cst))
            break;
    }
}

void bits::negate(bits::flags flag_)
{
    const unsigned flag = unsigned(flag_);

    for (;;)
    {
        unsigned b_(b);

        if (b.compare_exchange_weak(b_,
                                    b_ ^ flag,
                                    std::memory_order_seq_cst,
                                    std::memory_order_seq_cst))
            break;
    }
}

bool bits::get(bits::flags flag)
{
    return !!(b & flag);
}

bits::bits() : b(0u)
{
    set(f_center, true);
    set(f_enabled_p, true);
    set(f_enabled_h, true);
    set(f_zero, false);
    set(f_should_quit, false);
}
