#include <string>
#include <trajectory.pb.h>
#include "planning_node.h"
using namespace std;

namespace planning {
    namespace speed_profile{
        SpeedPlanner::SpeedPlanner() {
            // Read the planning parameters and environment settings.
            ParamConfig();

            // Initialize the rrt planner.
            rrt_ptr_ = std::move(std::unique_ptr<RRT>(new RRT(speed_profile_conf_)));
        }

        void SpeedPlanner::Start() {
            // There are two types of simulation: single test for debug and replanning test
            // for real-time traffic. The information of single test is given by the
            // config file, and the replanning test subscribes information from the
            // simulation environment SUMO.

            // The single test mode only generates a trajectory for one period.
            planning::Trajectory trajectory;
            bool is_path = rrt_ptr_->GenerateTrajectory(vehicle_state_, obstacle_map_,
                                                        reference_path_, &trajectory);
        }

        void SpeedPlanner::ParamConfig() {
            std::string planning_path = "/Users/zhuoyang/workspace/planning/";
            std::string file_name = planning_path + "conf/speed_profile_config.pb.txt";

            if (!common::GetProtoFromASCIIFile(file_name, &speed_profile_conf_)) {
                std::cout << "Error read config!" << std::endl;
            }

            reference_path_ = ReferencePath(speed_profile_conf_);
            std::string file_name1 = speed_profile_conf_.test_config_file();
            if (!common::GetProtoFromASCIIFile(file_name1, &env_conf_)) {
                std::cout << "Error read config!" << std::endl;
            }

            // Read params.
            vehicle_state_ = env_conf_.ego_car();
            vehicle_state_.set_theta(M_PI / 2 - vehicle_state_.theta());
            obstacle_map_.clear_obstacle();
            for (int i = 0; i < env_conf_.dynamic_obs().obstacle_size(); ++i) {
                PredictionObstacle obs = env_conf_.dynamic_obs().obstacle(i);
                obs.set_theta(M_PI / 2 - obs.theta());
                obstacle_map_.add_obstacle()->CopyFrom(obs);
            }
            for (int i = 0; i < env_conf_.static_obs().obstacle_size(); ++i) {
                PredictionObstacle obs = env_conf_.static_obs().obstacle(i);
                obs.set_theta(M_PI / 2 - obs.theta());
                obstacle_map_.add_obstacle()->CopyFrom(obs);
            }
        }
    }

}  // namespace planning

int main(int argc, char** argv) {
    planning::speed_profile::PlanningNode planning_node;
    planning_node.Start();
    return 0;
}
