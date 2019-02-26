//
// Created by zhuoyang on 2019-02-20.
//

#ifndef THESIS_FRAME_H
#define THESIS_FRAME_H

#include "vehicle_state.pb.h"
#include "../common/reference_path.h"
#include "prediction_obstacles.pb.h"
#include "../common/static_obstacles.h"

#include "planning_conf.pb.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "trajectory.pb.h"
#include "vehicle_state.pb.h"

namespace planning {
class Frame {
    public:
        Frame() = default;

        virtual void Init(const PlanningConf& planning_conf) = 0;

        virtual cv::Point PixelGoal() const = 0;

        virtual cv::Point PixelVehicleState() const = 0;

        virtual void GetSearchingGridMap(
                    cv::Mat* image,
                    SearchingImageBorder* image_border) const = 0;

        virtual void FromImageToGlobalCoord(double row, double col,
                                            double*x, double* y) const = 0;

        virtual ReferencePath reference_path() const = 0;

        virtual VehicleState vehicle_state() const = 0;

        virtual StaticObstacles static_obstacles() const = 0;

        virtual PredictionObstacles dynamic_obstacles() const = 0;

        virtual vector<vector<cv::Point>> obstacle_polygons() const = 0;

        virtual void SetReferencePath(
                const std::vector<double>& reference_path_x,
                const std::vector<double>& reference_path_y) = 0;

        virtual void UpdateState(double time_period) = 0;

        void UpdateTrajectory(const Trajectory& trajectory) {
            last_trajectory_ = trajectory;
        }

 protected:
    Trajectory last_trajectory_;
};
}
#endif //THESIS_FRAME_H
