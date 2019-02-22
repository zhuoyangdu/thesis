//
// Created by zhuoyang on 2019-02-20.
//

#ifndef THESIS_PATH_PLANNER_H
#define THESIS_PATH_PLANNER_H

#include <fcntl.h>
#include <iostream>
#include <memory>

#include <google/protobuf/text_format.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

#include "../frame/frame.h"
#include "../frame/structured_frame.h"
#include "../frame/unstructured_frame.h"
#include "searching_frame.h"
#include "heuristic_rrt.h"

namespace planning {
namespace path_planner {

class PathPlanner {
public:
    PathPlanner();

    void Run();

private:
    void ParamConfig();

    void InitFrame();

    void RegisterPlanner();

    PathPlannerConf path_planner_conf_;

    std::unique_ptr<Frame> frame_;
    std::unique_ptr<HeuristicRRT> rrt_planner_;

};

} // namespace path_planner
} // namespace planning

#endif //THESIS_PATH_PLANNER_H
