//
// Created by zhuoyang on 2019-02-20.
//

#include "structured_frame.h"

namespace planning {

void StructuredFrame::Init(const PlanningConf& planning_conf) {
    planning_conf_ = planning_conf;
    EnvironmentConf environment_conf = planning_conf.environment_conf();

    reference_route_ = ReferenceRoute(environment_conf.reference_route());
    vehicle_state_ = InitVehicleState(environment_conf.ego_car());

    InitObstacles();

    InitPathPlannerEnv();

    InitSpeedPlannerEnv();
}

void StructuredFrame::InitObstacles() {
    // Init obstacles.
    EnvironmentConf environment_conf = planning_conf_.environment_conf();
    static_obstacles_ = StaticObstacles(reference_route_,
                                        environment_conf.static_obs());

    dynamic_obstacles_.clear_obstacle();
    for (int i = 0; i < environment_conf.dynamic_obs().obstacle_size(); ++i) {
        PredictionObstacle obs = environment_conf.dynamic_obs().obstacle(i);
        obs.set_theta(M_PI / 2 - obs.theta());
        dynamic_obstacles_.add_obstacle()->CopyFrom(obs);
    }
    for (int i = 0; i < environment_conf.static_obs().obstacle_size(); ++i) {
        PredictionObstacle obs = environment_conf.static_obs().obstacle(i);
        obs.set_theta(M_PI / 2 - obs.theta());
        dynamic_obstacles_.add_obstacle()->CopyFrom(obs);
    }
}

void StructuredFrame::InitPathPlannerEnv() {
    // Init frenet frame.
    FrenetFrameConf frenet_conf =
        planning_conf_.path_planner_conf().rrt_conf().frenet_conf();
    frenet_frame_= FrenetFrame(reference_route_, vehicle_state_,
                               static_obstacles_, frenet_conf);

    // Init pixel goal and current state.
    auto goal = planning_conf_.environment_conf().goal();
    pixel_goal_ = frenet_frame_.FromFrenetToImage(goal.delta_s() + vehicle_state_.s(),
                                                  goal.d());
    pixel_current_ = frenet_frame_.FromFrenetToImage(vehicle_state_.s(),
                                                     vehicle_state_.d());

    // Get Image border of path planner.
    image_border_.set_delta_col(frenet_conf.ds());
    image_border_.set_delta_row(frenet_conf.dd());
    image_border_.set_col0(vehicle_state_.s());
    image_border_.set_col1(vehicle_state_.s() +
                           frenet_conf.image_col() * frenet_conf.ds());
    image_border_.set_row0(frenet_conf.lane_width() * frenet_conf.min_lane());
    image_border_.set_row1(image_border_.row0() +
                           frenet_conf.image_row() * frenet_conf.dd());
    image_border_.set_resolution_col(frenet_conf.image_col());
    image_border_.set_resolution_row(frenet_conf.image_row());
}

void StructuredFrame::InitSpeedPlannerEnv() {
    std::string road_file = planning_conf_.speed_profile_conf().road_file();
    if (!planning_conf_.run_path()) {
        reference_path_ = ReferencePath(road_file);
    }
}

void StructuredFrame::SetReferencePath(
        const std::vector<double>& reference_path_x,
        const std::vector<double>& reference_path_y) {
    reference_path_ = ReferencePath(reference_path_x, reference_path_y);
}

VehicleState StructuredFrame::InitVehicleState(const VehicleState& ego_car) {
    auto vehicle_state = ego_car;
    vehicle_state.set_theta(M_PI / 2 - vehicle_state.theta());
    double s0, d0;
    reference_route_.FromXYToSD(vehicle_state.x(), vehicle_state.y(), &s0, &d0);
    vehicle_state.set_s(s0);
    vehicle_state.set_d(d0);

    std::cout << "[StructuredFrame] Init vehicle state: x:" << vehicle_state.x()
                << ", y:" << vehicle_state.y()
                << ", theta:" << vehicle_state.theta()
                << ", s:" << vehicle_state.s()
                << ", d:" << vehicle_state.d() << std::endl;
    double nx, ny;
    reference_route_.FromSDToXY(s0, d0, &nx, &ny);
    if (nx != vehicle_state.x() || ny != vehicle_state.y()) {
        std::cout << "[StructuredFrame] warning: wrong in reference route." << std::endl;
    }
    return vehicle_state;
}

void StructuredFrame::GetSearchingGridMap(
                    cv::Mat* image,
                    SearchingImageBorder* image_border) const {
    *image = frenet_frame_.GetFrenetSpace();
    *image_border = image_border_;
}

void StructuredFrame::FromImageToGlobalCoord(
        double row, double col,
        double*x, double* y) const {
    double s, d;
    frenet_frame_.FromImageToFrenet(row, col, &s, &d);
    frenet_frame_.FromSDToXY(s, d, x, y);
}

}
