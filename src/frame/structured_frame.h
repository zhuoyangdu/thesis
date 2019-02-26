//
// Created by zhuoyang on 2019-02-20.
//

#ifndef THESIS_STRUCTURED_FRAME_H
#define THESIS_STRUCTURED_FRAME_H

#include "frame.h"
#include "frenet_frame.h"
#include "../common/reference_route.h"
#include "../common/static_obstacles.h"
#include "path_planner_conf.pb.h"
#include "vehicle_state.pb.h"
#include "../common/reference_path.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace planning {
class StructuredFrame : public Frame {
public:
    StructuredFrame() = default;

    void Init(const PlanningConf& planning_conf);

    VehicleState InitVehicleState(const VehicleState& ego_car);

    void InitPathPlannerEnv();

    void InitSpeedPlannerEnv();

    void InitObstacles();

    cv::Point PixelGoal() const {return pixel_goal_;};

    void GetSearchingGridMap(
                    cv::Mat* image,
                    SearchingImageBorder* image_border) const;

    cv::Point PixelVehicleState() const { return pixel_current_; };

    void FromImageToGlobalCoord(double row, double col,
                                double*x, double* y) const;

    VehicleState vehicle_state() const {return vehicle_state_;}

    StaticObstacles static_obstacles() const {return static_obstacles_;}

    PredictionObstacles dynamic_obstacles() const {return dynamic_obstacles_;}

    ReferencePath reference_path() const { return reference_path_;}

    vector<vector<cv::Point>> obstacle_polygons() const {
        return frenet_frame_.obstacle_polygons();
    }

    void SetReferencePath(
                const std::vector<double>& reference_path_x,
                const std::vector<double>& reference_path_y);

private:
    PlanningConf planning_conf_;
    // PathPlannerConf path_planner_conf_;
    VehicleState vehicle_state_;

    planning::ReferenceRoute reference_route_;
    planning::StaticObstacles static_obstacles_;
    planning::FrenetFrame frenet_frame_;

    cv::Point pixel_goal_;
    cv::Point pixel_current_;
    SearchingImageBorder image_border_;

    planning::ReferencePath reference_path_;
    planning::PredictionObstacles dynamic_obstacles_;

};
}
#endif //THESIS_STRUCTURED_FRAME_H
