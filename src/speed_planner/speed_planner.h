#ifndef PLANNING_SRC_RRT_PLANNING_NODE_H_
#define PLANNING_SRC_RRT_PLANNING_NODE_H_

#include <vector>
#include <memory>
#include <fstream>
#include <speed_profile_conf.pb.h>
#include <environment_conf.pb.h>

#include "rrt.h"
#include "../common/spline.h"
#include "../common/string.h"
#include "../common/file_config.h"
#include "ReferencePath.h"

#include "prediction_obstacles.pb.h"
#include "vehicle_state.pb.h"


namespace planning {
namespace spoeed_profile {

    class SpeedPlanner {
    public:
        SpeedPlanner();

        void Start();

    private:
        //void GetGeometryPath();
        void ParamConfig();

    private:
        planning::VehicleState vehicle_state_;
        planning::PredictionObstacles obstacle_map_;

        std::unique_ptr<RRT> rrt_ptr_;

        std::string planning_path_;

        ReferencePath reference_path_;

        planning::speed_profile::SpeedProfileConf speed_profile_conf_;
        planning::EnvironmentConf env_conf_;

    };
}
}

#endif //PLANNING_SRC_RRT_PLANNING_NODE_H_
