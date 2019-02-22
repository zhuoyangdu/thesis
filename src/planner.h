#ifndef PLANNING_SRC_PLANNER_H_
#define PLANNING_SRC_PLANNER_H_

#include <fcntl.h>
#include <iostream>
#include <memory>
#include <zconf.h>
#include <google/protobuf/text_format.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

#include "utils/file_config.h"
#include "frame/frame.h"
#include "frame/structured_frame.h"
#include "frame/unstructured_frame.h"
#include "path_planner/heuristic_rrt.h"
#include "speed_planner/rrt.h"

#include "planning_conf.pb.h"
#include "speed_profile_conf.pb.h"
#include "path_planner_conf.pb.h"

namespace planning {
class Planner
{
 public:
    Planner();

    void Run();

 private:
    void ParamConfig();

    void InitFrame();

    void RegisterPlanner();

    void RunSpeedPlanner();

    void RunPathPlanner();

    PlanningConf planning_conf_;
    PathPlannerConf path_planner_conf_;
    planning::EnvironmentConf environment_conf_;

    std::unique_ptr<Frame> frame_;
    std::unique_ptr<planning::path_planner::HeuristicRRT> path_planner_;

    speed_profile::SpeedProfileConf speed_profile_conf_;
    std::unique_ptr<speed_profile::RRT> speed_profile_planner_;


};
}

#endif // PLANNING_SRC_PLANNER_H_
