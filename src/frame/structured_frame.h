//
// Created by zhuoyang on 2019-02-20.
//

#ifndef THESIS_STRUCTURED_FRAME_H
#define THESIS_STRUCTURED_FRAME_H

#include "frame.h"
#include "frenet_frame.h"
#include "../common/reference_route.h"
#include "../common/obstacles.h"
#include "path_planner_conf.pb.h"
#include "vehicle_state.pb.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace planning {
class StructuredFrame : public Frame {
public:
    StructuredFrame() = default;

    void Init(const PlanningConf& planning_conf);

    cv::Point PixelGoal() const {return pixel_goal_;};

    void GetSearchingGridMap(
                    cv::Mat* image,
                    SearchingImageBorder* image_border) const;

    cv::Point PixelVehicleState() const { return pixel_current_; };

    void FromImageToGlobalCoord(double row, double col,
                                double*x, double* y) const;

private:
    PathPlannerConf path_planner_conf_;
    VehicleState vehicle_state_;

    planning::ReferenceRoute reference_route_;
    planning::Obstacles obstacles_;
    planning::FrenetFrame frenet_frame_;

    cv::Point pixel_goal_;
    cv::Point pixel_current_;
    SearchingImageBorder image_border_;
};
}
#endif //THESIS_STRUCTURED_FRAME_H
