#include <deque>
#include <fstream>
#include <cmath>
#include "node.h"
#include "trajectory.pb.h"
#include "prediction_obstacles.pb.h"
#include "vehicle_state.pb.h"
#include "../utils/file_config.h"
#include "../utils/spline.h"
#include "planning_debug.pb.h"
#include "speed_profile_conf.pb.h"

namespace planning {
    namespace speed_profile {
        class DynamicObstacles {
        public:
            DynamicObstacles() = default;

            DynamicObstacles(const SpeedProfileConf& speed_profile_conf);

            void SetObstacles(const planning::PredictionObstacles &obstacle_map);

            bool CollisionFree(const Node &parent_node,
                               const Node &child_node,
                               const utils::Spline &curve_x,
                               const utils::Spline &curve_y);


            double RiskAssessment(const std::deque<Node> &path,
                                  const utils::Spline &curve_x,
                                  const utils::Spline &curve_y);

            void InitializeDistanceMap(const VehicleState &vehicle_state,
                                       const utils::Spline &curve_x,
                                       const utils::Spline &curve_y,
                                       double s0);

            double ReadDistanceMap(const Node &node);

            double ComputeTTC(double node_time, double node_distance,
                              double node_vel,
                              const utils::Spline &curve_x,
                              const utils::Spline &curve_y);

            std::vector<std::vector<double>> ComputeTTCForFixedVel(
                    double current_path_length,
                    double node_vel,
                    const utils::Spline &curve_x,
                    const utils::Spline &curve_y);

            void ComputeTTCMap(double current_path_length,
                               const utils::Spline &curve_x,
                               const utils::Spline &curve_y);

            std::vector<planning::PredictionObstacle> GetObstacles() {
                return obstacles_;
            }

        private:
            std::vector<PredictionObstacle> obstacles_;
            std::vector<std::vector<double>> distance_map_;

            double NonlinearRisk(double input);

            double init_vehicle_path_length_;
            double epsilon_ = 1e3;
            std::string planning_path_;
            double kDeltaT = 0.1;
            double kDeltaS = 1.0;

            void recordDistanceMap();

            bool recordDistanceMapProto();

            double EuclideanDisToObs(double x, double y, double t);

            planning::speed_profile::SpeedProfileConf speed_profile_conf_;
            planning::speed_profile::RRTConfig rrt_conf_;

        };
    }
} // namespace planning
