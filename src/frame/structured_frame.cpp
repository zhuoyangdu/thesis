//
// Created by zhuoyang on 2019-02-20.
//

#include "structured_frame.h"

namespace planning {

void StructuredFrame::Init(const PlanningConf& planning_conf) {
    path_planner_conf_ = planning_conf.path_planner_conf();
    EnvironmentConf environment_conf = planning_conf.environment_conf();
    reference_route_ = ReferenceRoute(environment_conf.reference_route());
    vehicle_state_.set_x(environment_conf.ego_car().x());
    vehicle_state_.set_y(environment_conf.ego_car().y());
    vehicle_state_.set_theta(environment_conf.ego_car().theta());
    double s0, d0;
    reference_route_.FromXYToSD(vehicle_state_.x(), vehicle_state_.y(), &s0, &d0);
    vehicle_state_.set_s(s0);
    vehicle_state_.set_d(d0);
    environment_conf.mutable_ego_car()->CopyFrom(vehicle_state_);
    path_planner_conf_.mutable_environment_conf()->CopyFrom(environment_conf);

    std::cout << "[StructuredFrame] Init vehicle state: x:" << vehicle_state_.x()
                << ", y:" << vehicle_state_.y()
                << ", theta:" << vehicle_state_.theta()
                << ", s:" << vehicle_state_.s()
                << ", d:" << vehicle_state_.d() << std::endl;
    double nx, ny;
    reference_route_.FromSDToXY(s0, d0, &nx, &ny);
    if (nx != vehicle_state_.x() || ny != vehicle_state_.y()) {
        std::cout << "[planner] warning: wrong in reference route." << std::endl;
    }

    obstacles_ = Obstacles(reference_route_, environment_conf.static_obs());

    frenet_frame_= FrenetFrame(reference_route_, vehicle_state_, obstacles_,
                        path_planner_conf_.rrt_conf().frenet_conf());

    pixel_goal_ = frenet_frame_.FromFrenetToImage(
        path_planner_conf_.environment_conf().goal().delta_s()
        + vehicle_state_.s(), path_planner_conf_.environment_conf().goal().d());

    pixel_current_ = frenet_frame_.FromFrenetToImage(vehicle_state_.s(),
                                                     vehicle_state_.d());
    // Get Image border.
    FrenetFrameConf frenet_conf = path_planner_conf_.rrt_conf().frenet_conf();
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
