#ifndef SRC_SPEED_PLANNER_SPEED_PROFILE_RECORD_H_
#define SRC_SPEED_PLANNER_SPEED_PROFILE_RECORD_H_

#include <iostream>
#include <fstream>
#include <vector>
#include <deque>
#include <ctime>
#include <stdlib.h>
#include "node.h"
#include "../utils/file_config.h"
#include "../utils/string.h"
#include "../utils/spline.h"
#include "speed_profile_debug.pb.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "prediction_obstacles.pb.h"
#include "speed_profile_conf.pb.h"

using namespace std;

namespace planning {
namespace speed_profile {
class SpeedProfileRecord {
public:
    SpeedProfileRecord();

    void RecordTree(const std::vector<Node>& tree);

    void PrintToFile(const std::string& folder);

    void RecordPath(const RRTConfig& rrt_conf,
                    const std::deque<Node>& path,
                    const utils::Spline& curve_x,
                    const utils::Spline& curve_y);

    void RecordDistanceMap(
        const std::vector<double>& distance_t,
        const std::vector<double>& distance_s);

    void RecordObstacles(
        const std::vector<PredictionObstacle>& obstacles);

private:
    planning::debug::speed::SpeedProfileDebug speed_profile_debug_;
    std::string time_string_;
};

}
}

#endif // SRC_SPEED_PLANNER_SPEED_PROFILE_RECORD_H_
