#include "path_planner_record.h"

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

void PathPlannerRecord::PrintToFile(const std::string& folder) {
    std::string file_name = folder + "path_" + ".txt";
    std::cout << "[PathPlannerRecord] record file name:" << file_name << std::endl;
    utils::SetProtoToASCIIFile(path_planner_debug_, file_name);
}

}
}
