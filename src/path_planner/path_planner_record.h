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
#include "../common/static_obstacles.h"
#include "path_planner_debug.pb.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

namespace planning {
namespace path_planner {
class PathPlannerRecord {
public:
    PathPlannerRecord();

    void RecordTree(const std::vector<Node>& tree);

    void PrintToFile(const std::string& folder);

    void RecordPath(const std::vector<Node>& path);

    void RecordSplinePath(const std::vector<Node>& path);

    void RecordObstaclePolygons(
        const vector<vector<cv::Point>>& obstacle_polygons);

    void RecordVehicleState(const VehicleState& state);

    void RecordStaticObstacles(const StaticObstacles& static_obstacles);

    void RecordGlobalPath(const std::vector<double> global_x,
                          const std::vector<double> global_y);

private:
    planning::debug::PathPlannerDebug path_planner_debug_;
    std::string time_string_;

    int num_ = 0;

};

}
}

#endif // SRC_PLANNING_SRC_PLANNING_DEBUG_H_
