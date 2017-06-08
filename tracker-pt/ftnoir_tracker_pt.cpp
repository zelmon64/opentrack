/* Copyright (c) 2012 Patrick Ruoff
 * Copyright (c) 2014-2016 Stanislaw Halik <sthalik@misaki.pl>
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 */

#include "ftnoir_tracker_pt.h"
#include "compat/camera-names.hpp"
#include <QHBoxLayout>
#include <cmath>
#include <QDebug>
#include <QFile>
#include <QCoreApplication>
#include <functional>

//#define PT_PERF_LOG	//log performance

//-----------------------------------------------------------------------------
Tracker_PT::Tracker_PT() :
      point_count(0),
      commands(0),
      ever_success(false)
{
    connect(s.b.get(), SIGNAL(saving()), this, SLOT(maybe_reopen_camera()), Qt::DirectConnection);
#if 0
    connect(&s.fov, SIGNAL(valueChanged(int)), this, SLOT(set_fov(int)), Qt::DirectConnection);
    set_fov(s.fov);
#endif

    static constexpr int fov = 75;
    set_fov(fov);
}

Tracker_PT::~Tracker_PT()
{
    set_command(ABORT);
    wait();

    QMutexLocker l(&camera_mtx);
    camera.stop();
}

void Tracker_PT::set_command(Command command)
{
    //QMutexLocker lock(&mutex);
    commands |= command;
}

void Tracker_PT::reset_command(Command command)
{
    //QMutexLocker lock(&mutex);
    commands &= ~command;
}

void Tracker_PT::run()
{
    cv::setNumThreads(0);

#ifdef PT_PERF_LOG
    QFile log_file(QCoreApplication::applicationDirPath() + "/PointTrackerPerformance.txt");
    if (!log_file.open(QIODevice::WriteOnly | QIODevice::Text)) return;
    QTextStream log_stream(&log_file);
#endif

    maybe_reopen_camera();

    while((commands & ABORT) == 0)
    {
        CamInfo cam_info;
        bool new_frame;

        {
            QMutexLocker l(&camera_mtx);
            std::tie(new_frame, cam_info) = camera.get_frame(frame);
        }

        if (new_frame)
        {
            cv::resize(frame, preview_frame, cv::Size(preview_size.width(), preview_size.height()), 0, 0, cv::INTER_NEAREST);

            point_extractor.extract_points(frame, preview_frame, points);
            point_count = points.size();

            f fx;
            cam_info.get_focal_length(fx);

            const bool success = points.size() >= PointModel::N_POINTS;

            if (success)
            {
                const bool dynamic_pose = s.active_model_panel == settings_pt::model::cap;
                static constexpr int init_phase_timeout = 1500;

                point_tracker.track(points,
                                    PointModel(s),
                                    cam_info,
                                    dynamic_pose ? init_phase_timeout : 0);
                ever_success = true;
            }

            {
                Affine X_CM;
                {
                    QMutexLocker l(&data_mtx);
                    X_CM = point_tracker.pose();
                }

                Affine X_MH(mat33::eye(), vec3(s.t_MH_x, s.t_MH_y, s.t_MH_z)); // just copy pasted these lines from below
                Affine X_GH = X_CM * X_MH;
                vec3 p = X_GH.t; // head (center?) position in global space
                vec2 p_(p[0] / p[2] * fx, p[1] / p[2] * fx);  // projected to screen

                static constexpr int len = 9;

                cv::Point p2(iround(p_[0] * preview_frame.cols + preview_frame.cols/2),
                             iround(-p_[1] * preview_frame.cols + preview_frame.rows/2));
                static const cv::Scalar color(0, 255, 255);
                cv::line(preview_frame,
                         cv::Point(p2.x - len, p2.y),
                         cv::Point(p2.x + len, p2.y),
                         color,
                         1);
                cv::line(preview_frame,
                         cv::Point(p2.x, p2.y - len),
                         cv::Point(p2.x, p2.y + len),
                         color,
                         1);
            }

            video_widget->update_image(preview_frame);
        }
    }
    qDebug() << "pt: thread stopped";
}

void Tracker_PT::maybe_reopen_camera()
{
    QMutexLocker l(&camera_mtx);

    static constexpr int cam_fps = 60, cam_res_x = 640, cam_res_y = 480;
    static const QString camera_name = "PS3Eye Camera";

    Camera::open_status status = camera.start(camera_name_to_index(camera_name), cam_fps, cam_res_x, cam_res_y);

    switch (status)
    {
    case Camera::open_error:
        qDebug() << "can't start camera" << camera_name;
        break;
    case Camera::open_ok_change:
        frame = cv::Mat();
        break;
    case Camera::open_ok_no_change:
        break;
    }
}

void Tracker_PT::set_fov(int value)
{
    QMutexLocker l(&camera_mtx);
    camera.set_fov(value);
}

void Tracker_PT::start_tracker(QFrame* video_frame)
{
    //video_frame->setAttribute(Qt::WA_NativeWindow);
    preview_size = video_frame->size();

    preview_frame = cv::Mat(video_frame->height(), video_frame->width(), CV_8UC3);
    preview_frame.setTo(cv::Scalar(0, 0, 0));

    video_widget = qptr<cv_video_widget>(video_frame);
    layout = qptr<QHBoxLayout>(video_frame);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(video_widget.data());
    video_frame->setLayout(layout.data());
    //video_widget->resize(video_frame->width(), video_frame->height());
    video_frame->show();
    start();
}

void Tracker_PT::data(double *data)
{
    if (ever_success)
    {
        Affine X_CM = pose();

        Affine X_MH(mat33::eye(), vec3(s.t_MH_x, s.t_MH_y, s.t_MH_z));
        Affine X_GH = X_CM * X_MH;

        // translate rotation matrix from opengl (G) to roll-pitch-yaw (E) frame
        // -z -> x, y -> z, x -> -y
        mat33 R_EG(0, 0,-1,
                   -1, 0, 0,
                   0, 1, 0);
        mat33 R = R_EG *  X_GH.R * R_EG.t();

        using std::atan2;
        using std::sqrt;
        using std::atan;
        using std::fabs;
        using std::copysign;

        // get translation(s)
        const vec3& t = X_GH.t;

        // extract rotation angles
        {
            f alpha, beta, gamma;
            beta  = atan2( -R(2,0), sqrt(R(2,1)*R(2,1) + R(2,2)*R(2,2)) );
            alpha = atan2( R(1,0), R(0,0));
            gamma = atan2( R(2,1), R(2,2));

#if 0
            if (t[2] > 1e-4)
            {
                alpha += copysign(atan(t[0] / t[2]), t[0]);
                // pitch is skewed anyway due to only one focal length value
                //beta -= copysign(atan(t[1] / t[2]), t[1]);
            }
#endif

            data[Yaw] = rad2deg * alpha;
            data[Pitch] = -rad2deg * beta;
            data[Roll] = rad2deg * gamma;
        }

        // convert to cm
        data[TX] = t[0] / 10;
        data[TY] = t[1] / 10;
        data[TZ] = t[2] / 10;
    }
}

Affine Tracker_PT::pose()
{
    QMutexLocker l(&data_mtx);

    return point_tracker.pose();
}

int Tracker_PT::get_n_points()
{
    return int(point_count);
}

bool Tracker_PT::get_cam_info(CamInfo* info)
{
    QMutexLocker lock(&camera_mtx);
    bool ret;

    std::tie(ret, *info) = camera.get_info();
    return ret;
}

#include "ftnoir_tracker_pt_dialog.h"
OPENTRACK_DECLARE_TRACKER(Tracker_PT, TrackerDialog_PT, PT_metadata)

