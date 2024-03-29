#include "planner.h"
using namespace std;

namespace planning {
Planner::Planner() {
    ParamConfig();
    RegisterPlanner();
    InitFrame();
}

void Planner::ParamConfig() {
    // Get configuration file path.
    std::string conf_path = "/Users/zhuoyang/workspace/thesis/conf/planning_conf.pb.txt";
    utils::GetProtoFromASCIIFile(conf_path, &planning_conf_);

    utils::GetProtoFromASCIIFile(planning_conf_.path_planner_conf_path(),
                                 &path_planner_conf_);

    utils::GetProtoFromASCIIFile(planning_conf_.environment_conf_path(),
                                       &environment_conf_);

    utils::GetProtoFromASCIIFile(planning_conf_.speed_profile_conf_path(),
                                 &speed_profile_conf_);

    planning_conf_.mutable_environment_conf()->CopyFrom(environment_conf_);
    planning_conf_.mutable_path_planner_conf()->CopyFrom(path_planner_conf_);
    planning_conf_.mutable_speed_profile_conf()->CopyFrom(speed_profile_conf_);
}

void Planner::InitFrame() {
    if (environment_conf_.type() ==
        planning::EnvironmentConf::STRUCTURED_ENVIRONMENT) {
        frame_.reset(new StructuredFrame());
        frame_->Init(planning_conf_);
    }
}

void Planner::Run() {
    RunPathPlanner();
}

void Planner::Replan() {
    PlanningStatus status = path_planner_->MultiThreadSolve(frame_);
    if (status.ok()) {
        std::cout << "[Planner] path plan finished." << std::endl;
    }
    Trajectory trajectory = speed_profile_planner_->Solve(frame_);
    frame_->UpdateTrajectory(trajectory);
    frame_->UpdateState(planning_conf_.time_period());
}

void Planner::RegisterPlanner() {
    path_planner_.reset(new path_planner::HeuristicRRT(path_planner_conf_));
    speed_profile_planner_.reset(new speed_profile::RRT(speed_profile_conf_));
}

void Planner::RunPathPlanner() {
    PlanningStatus status = path_planner_->MultiThreadSolve(frame_);
    if (status.ok()) {

    }
}

void Planner::RunSpeedPlanner() {
    Trajectory trajectory = speed_profile_planner_->Solve(frame_);
}

}

