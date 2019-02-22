//
// Created by zhuoyang on 2019-02-20.
//

#include <zconf.h>
#include "path_planner.h"

using namespace std;

namespace planning {
namespace path_planner {

PathPlanner::PathPlanner() {
    ParamConfig();
    RegisterPlanner();
    InitFrame();
}

void PathPlanner::ParamConfig() {
    // Get configuration file path.
    std::string conf_path = "/home/zy/thesis/conf/path_planner_conf.pb.txt";

    // Parse the text file into protobuf.
    using google::protobuf::TextFormat;
    using google::protobuf::io::FileInputStream;
    using google::protobuf::io::ZeroCopyInputStream;
    std::cout << "[PathPlanner] path planner config path:"
              << conf_path.c_str() << std::endl;
    int file_descriptor = open(conf_path.c_str(), O_RDONLY);
    if (file_descriptor < 0) {
        std::cout << "[PathPlanner] Invalid file descriptor." << std::endl;
        return;
    }
    ZeroCopyInputStream *input = new FileInputStream(file_descriptor);
    if (!TextFormat::Parse(input, &path_planner_conf_)) {
        std::cout << "[PathPlanner] Failed to parse file." << std::endl;
    }
    delete input;
    close(file_descriptor);

    planning::EnvironmentConf environment_conf;
    std::cout << "[PathPlanner] environment config path:"
              << path_planner_conf_.environment_conf_path() << endl;
    std::string env_conf_path = path_planner_conf_.environment_conf_path();
    file_descriptor = open(env_conf_path.c_str(), O_RDONLY);
    if (file_descriptor < 0) {
        std::cout << "[PathPlanner] Invalid file descriptor." << std::endl;
        return;
    }
    ZeroCopyInputStream *input_env = new FileInputStream(file_descriptor);
    if (!TextFormat::Parse(input_env, &environment_conf)) {
        std::cout << "[PathPlanner] Failed to parse file." << std::endl;
    }
    delete input_env;
    close(file_descriptor);

    path_planner_conf_.mutable_environment_conf()->CopyFrom(environment_conf);
}

void PathPlanner::InitFrame() {
    if (path_planner_conf_.environment_conf().type() ==
        planning::EnvironmentConf::STRUCTURED_ENVIRONMENT) {
        frame_.reset(new StructuredFrame());
        frame_->Init(path_planner_conf_);
    }
}

void PathPlanner::Run() {
    SearchingFrame searching_frame(path_planner_conf_, frame_);
    rrt_planner_->MultiThreadSolve(&searching_frame);

}

void PathPlanner::RegisterPlanner() {
    rrt_planner_.reset(new HeuristicRRT(path_planner_conf_));
}

}
}
