//
// Created by zhuoyang on 2018-12-29.
//

#ifndef PLANNING_FRENET_FRAME_H
#define PLANNING_FRENET_FRAME_H

#include <iostream>
#include <vector>

#include "../common/static_obstacles.h"
#include "vehicle_state.pb.h"
#include "environment_conf.pb.h"
#include "../common/reference_route.h"
#include "../path_planner/image_proc.h"

#include "path_planner_conf.pb.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

namespace  planning {
class FrenetFrame {
public:
    FrenetFrame() = default;

    FrenetFrame(const ReferenceRoute& reference_route,
                const VehicleState& vehicle_state,
                const StaticObstacles& obstacles,
                const FrenetFrameConf& frenet_frame_conf);

    void FromFrenetToImage(double s, double d,
                           double* row, double* col) const {
        *row = frenet_frame_conf_.image_row() - (d - d0_) / frenet_frame_conf_.dd()-1;
        *col = (s - s0_) / frenet_frame_conf_.ds();
    }

    cv::Point FromFrenetToImage(double s, double d) const {
        double row = frenet_frame_conf_.image_row() - (d - d0_) / frenet_frame_conf_.dd()-1;
        double col = (s - s0_) / frenet_frame_conf_.ds();
        return cv::Point(col, row);
    }
    void FromImageToFrenet(double row, double col,
                           double* s, double* d) const {
        *s = col * frenet_frame_conf_.ds() + s0_;
        *d = (frenet_frame_conf_.image_row() - row) * frenet_frame_conf_.dd() + d0_;
    }

    cv::Mat GetFrenetSpace() const {return image_frenet_; }

    void FromXYToSD(double x, double y, double* s, double* d) const {
        reference_route_.FromXYToSD(x, y, s, d);
    }

    void FromSDToXY(double s, double d, double* x, double* y) const {
        reference_route_.FromSDToXY(s, d, x, y);
    }

    cv::Point FromXYToImage(double x, double y) const {
        double s, d;
        FromXYToSD(x, y, &s, &d);
        return FromFrenetToImage(s, d);
    }

    planning::ReferenceRoute ReferenceRoute() const {return reference_route_; }

    vector<vector<cv::Point>> obstacle_polygons() const {
        return obstacle_polygons_;
    }

private:
    void DrawSDMap();

    planning::ReferenceRoute reference_route_;
    VehicleState vehicle_state_;
    vector<StaticObstacle> obstacles_;
    FrenetFrameConf frenet_frame_conf_;

    double d0_ = 0.0;
    double s0_ = 0.0;

    cv::Mat image_frenet_;
    vector<vector<cv::Point>> obstacle_polygons_;
};
}
#endif //PLANNING_FRENET_FRAME_H
