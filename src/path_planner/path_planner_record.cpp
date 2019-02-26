#include "path_planner_record.h"

using namespace std;

namespace planning {
namespace path_planner {
PathPlannerRecord::PathPlannerRecord() {
    time_t t = std::time(0);
    struct tm * now = std::localtime(&t);

    //the name of bag file is better to be determined by the system time
    time_string_ = utils::StringUtils::int2string(now->tm_year + 1900)+
             '-' + utils::StringUtils::int2string(now->tm_mon + 1)+
             '-' + utils::StringUtils::int2string(now->tm_mday)+
             '-' + utils::StringUtils::int2string(now->tm_hour)+
             '-' + utils::StringUtils::int2string(now->tm_min)+
             '-' + utils::StringUtils::int2string(now->tm_sec);

}

void PathPlannerRecord::RecordTree(const std::vector<Node>& tree) {
    debug::Tree debug_tree;
    for (Node node : tree) {
        debug::ImagePoint point;
        point.set_index(node.index());
        point.set_parent_index(node.parent_index());
        point.set_row(node.row());
        point.set_col(node.col());
        debug_tree.add_nodes()->CopyFrom(point);
    }
    path_planner_debug_.mutable_tree()->CopyFrom(debug_tree);
}

void PathPlannerRecord::RecordPath(const std::vector<Node>& path) {
    debug::Path debug_path;
    for (Node node : path) {
        debug::ImagePoint point;
        point.set_row(node.row());
        point.set_col(node.col());
        debug_path.add_nodes()->CopyFrom(point);
    }
    path_planner_debug_.mutable_path()->CopyFrom(debug_path);
}

void PathPlannerRecord::RecordSplinePath(
        const std::vector<Node>& path) {
    debug::Path debug_path;
    for (Node node : path) {
        debug::ImagePoint point;
        point.set_row(node.row());
        point.set_col(node.col());
        debug_path.add_nodes()->CopyFrom(point);
    }
    path_planner_debug_.mutable_spline_path()->CopyFrom(debug_path);
}

void PathPlannerRecord::RecordObstaclePolygons(
        const vector<vector<cv::Point>>& obstacle_polygons) {
    debug::ObstacleImagePolygons debug_polys;
    for (std::vector<cv::Point> poly : obstacle_polygons) {
        debug::ObstacleImagePolygon debug_poly;
        for (cv::Point point : poly) {
            debug::ImagePoint debug_point;
            debug_point.set_row(point.x);
            debug_point.set_col(point.y);
            debug_poly.add_vertexes()->CopyFrom(debug_point);
        }
        debug_polys.add_obstacles()->CopyFrom(debug_poly);
    }
    path_planner_debug_.mutable_obstacle_image_polygons()
        ->CopyFrom(debug_polys);
}

void PathPlannerRecord::RecordVehicleState(
        const VehicleState& state) {
    debug::GlobalPoint debug_vehicle;
    debug_vehicle.set_x(state.x());
    debug_vehicle.set_y(state.y());
    debug_vehicle.set_theta(state.theta());
    path_planner_debug_.mutable_vehicle_state()->CopyFrom(debug_vehicle);
}

void PathPlannerRecord::RecordStaticObstacles(
        const StaticObstacles& static_obstacles) {
    for (StaticObstacle obs : static_obstacles.obstacles()) {
        debug::GlobalPoint debug_obs;
        debug_obs.set_x(obs.x());
        debug_obs.set_y(obs.y());
        debug_obs.set_theta(obs.theta());
        path_planner_debug_.add_obstacles()->CopyFrom(debug_obs);
    }
}

void PathPlannerRecord::RecordGlobalPath(
        const std::vector<double> global_x,
        const std::vector<double> global_y) {
    debug::GlobalPath path;
    for (int i = 0; i < global_x.size(); ++i) {
        debug::GlobalPoint p;
        p.set_x(global_x[i]);
        p.set_y(global_y[i]);
        path.add_points()->CopyFrom(p);
    }
    path_planner_debug_.mutable_global_path()->CopyFrom(path);
}

void PathPlannerRecord::PrintToFile(const std::string& folder) {
    std::string file_name = folder + "path_" + ".txt";
    std::cout << "[PathPlannerRecord] record file name:" << file_name << std::endl;
    utils::SetProtoToASCIIFile(path_planner_debug_, file_name);
}

}
}
