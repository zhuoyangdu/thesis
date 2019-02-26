#ifndef THESIS_SEARCHING_FRAME_H
#define THESIS_SEARCHING_FRAME_H

#include <memory>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "node.h"
#include "image_proc.h"
#include "../frame/frame.h"
#include "../common/static_obstacles.h"

#include "path_planner_conf.pb.h"



namespace planning {
namespace path_planner {
class SearchingFrame
{
public:
    SearchingFrame() = default;

    SearchingFrame(const PathPlannerConf& path_planner_conf,
                   const std::unique_ptr<Frame>& frame);

    void GetPixelCoord(double x, double y,
                       double* row, double* col) const;

    void GetWorldCoord(double row, double col,
                       double* x, double* y) const;

    bool CheckCollisionByPixelCoord(double row, double col) const;

    bool CheckCollisionByPixelCoord(const cv::Point& point) const;

    bool CheckCollisionByWorldCoord(double x, double y);

    bool CollisionCheckByEdge(const Node& a, const Node& b);

    cv::Mat DynamicMap() const { return grid_map_; }

    cv::Mat AttractiveMap() const {return attractive_map_; }

    cv::Mat TargetAttractiveMap() const {return goal_prob_map_;}

    cv::Mat VoronoiAttractiveMap() const {return voronoi_prob_map_;}

    cv::Mat RepulsiveX() const { return repulsive_filed_x_; }

    cv::Mat RepulsiveY() const { return repulsive_filed_y_; }

    cv::Point PixelGoal() const { return goal_point_; }

    cv::Point PixelCurrentState() const { return current_point_; }

    void FromImageToXY(double row, double col, double*x, double* y) const;

    std::vector<std::vector<cv::Point>> obstacle_polygons() const {
        return frame_->obstacle_polygons();
    }

    VehicleState vehicle_state() const {
        return frame_->vehicle_state();
    }

    StaticObstacles static_obstacles() const {
        return frame_->static_obstacles();
    }

    void SetResult(const std::vector<double>& global_x,
                   const std::vector<double>& global_y) {
        frame_->SetReferencePath(global_x, global_y);
    }

private:
    void GenerateAttractiveProbMap();

    PathPlannerConf path_planner_conf_;

    cv::Point goal_point_;
    cv::Point current_point_;

    cv::Mat grid_map_;
    SearchingImageBorder image_border_;

    cv::Mat dilate_map_;
    cv::Mat attractive_map_;
    cv::Mat goal_prob_map_;
    cv::Mat voronoi_prob_map_;
    cv::Mat repulsive_filed_x_;
    cv::Mat repulsive_filed_y_;

    Frame* frame_;
};
}
}

#endif // THESIS_SEARCHING_FRAME_H
