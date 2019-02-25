#ifndef SRC_PLANNING_SRC_PLANNING_DEBUG_H_
#define SRC_PLANNING_SRC_PLANNING_DEBUG_H_

#include <iostream>
#include <fstream>
#include <vector>
#include <ctime>
#include <stdlib.h>
#include "node.h"
#include "../utils/file_config.h"
#include "../utils/string.h"
#include "path_planner_debug.pb.h"

namespace planning {
namespace path_planner {
class PathPlannerRecord {
public:
    PathPlannerRecord();

    void RecordTree(const std::vector<Node>& tree);

    void PrintToFile(const std::string& folder);

private:
    planning::debug::PathPlannerDebug path_planner_debug_;
    std::string time_string_;
};

}
}

#endif // SRC_PLANNING_SRC_PLANNING_DEBUG_H_
