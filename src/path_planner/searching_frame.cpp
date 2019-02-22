#include "searching_frame.h"

namespace planning {
namespace path_planner {

SearchingFrame::SearchingFrame(const PathPlannerConf& path_planner_conf,
                               const std::unique_ptr<Frame>& frame) :
        path_planner_conf_(path_planner_conf) {
    goal_point_ = frame->PixelGoal();
    current_point_ = frame->PixelVehicleState();
    frame->GetSearchingGridMap(&grid_map_, &image_border_);
    GenerateAttractiveProbMap();
    std::cout << "[SearchingFrame] Generated attractive prob map." << std::endl;

    ImageProc::GetObstacleRepulsiveField(grid_map_,
                                         &repulsive_filed_x_,
                                         &repulsive_filed_y_);
    std::cout << "[SearchingFrame] Generated obstacle repulsive field." << std::endl;

    Mat element = cv::getStructuringElement(MORPH_RECT, Size(10, 10));
    erode(grid_map_, dilate_map_, element);
    std::cout << "[SearchingFrame] Generate dilate map." << std::endl;

    frame_ = frame.get();
}

void SearchingFrame::GetPixelCoord(double x, double y,
                                double *row, double *col) const {
    *row = (image_border_.row1() - y)
            / (image_border_.row1() - image_border_.row0())
            * image_border_.resolution_row();
    *col = (x - image_border_.col0()) * image_border_.resolution_col()
           / (image_border_.col1() - image_border_.col0());
}

void SearchingFrame::GetWorldCoord(double row, double col,
                                double *x, double *y) const {
    *y = image_border_.row1() - row / image_border_.resolution_row()
         * (image_border_.row1() - image_border_.row0());
    *x = image_border_.col0() + col / image_border_.resolution_col()
         * (image_border_.col1() - image_border_.col0());
}

void SearchingFrame::GenerateAttractiveProbMap() {
    ImageProc::GetAttractiveProbMap(grid_map_, goal_point_,
                                    path_planner_conf_.rrt_conf().k_voronoi(),
                                    path_planner_conf_.rrt_conf().k_goal(),
                                    &goal_prob_map_,
                                    &voronoi_prob_map_,
                                    &attractive_map_);
}

bool SearchingFrame::CheckCollisionByPixelCoord(double row, double col) const {
    int value = static_cast<int>(dilate_map_.at<uchar>(static_cast<int>(row),
                                                       static_cast<int>(col)));
    if (value == 0) {
        // Is collided.
        return true;
    } else {
        return false;
    }
}

bool SearchingFrame::CheckCollisionByPixelCoord(const cv::Point& point) const {
    return CheckCollisionByPixelCoord(point.x, point.y);
}

bool SearchingFrame::CheckCollisionByWorldCoord(double x, double y) {
    double row, col;
    GetPixelCoord(x, y, &row, &col);
    return CheckCollisionByPixelCoord(static_cast<int>(row),
                                      static_cast<int>(col));
}

void SearchingFrame::FromImageToXY(
        double row, double col,
        double*x, double* y) const {
    frame_->FromImageToGlobalCoord(row, col, x, y);
}


}
}
