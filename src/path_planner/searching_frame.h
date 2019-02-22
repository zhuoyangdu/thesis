#ifndef THESIS_SEARCHING_FRAME_H
#define THESIS_SEARCHING_FRAME_H

#include <memory>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "node.h"
#include "image_proc.h"
#include "../frame/frame.h"

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
