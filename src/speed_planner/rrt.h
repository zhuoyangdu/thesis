#ifndef PLANNING_SRC_RRT_H_
#define PLANNING_SRC_RRT_H_

#include <random>
#include <cmath>
#include <deque>
#include <fstream>
#include <iostream>
#include <algorithm>

#include <time.h>
#include <map>

#include "common.h"
#include "../common/spline.h"
#include "obstacles.h"
#include "ReferencePath.h"

#include "vehicle_state.pb.h"
#include "trajectory.pb.h"
#include "prediction_obstacles.pb.h"
#include "speed_profile_conf.pb.h"

namespace planning {
    namespace speed_profile {

        class RRT {
        public:
            RRT(const SpeedProfileConf& speed_profile_conf);

            bool GenerateTrajectory(const VehicleState &vehicle_state,
                                    const PredictionObstacles &obstacle_map,
                                    const ReferencePath& reference_path,
                                    Trajectory *trajectory);

        private:
            double GetGeometryPathLength(double x, double y);

            Node RandomSample(double s0);

            void Extend(Node &sample, Node *new_node, bool *node_valid);

            void GetNearestNode(const Node &sample, Node *nearest_node,
                                bool *node_valid);

            void Steer(const Node &sample, const Node &nearest_node, Node *new_node);

            double ComputeVelocity(const Node &n1, const Node &n2);

            double ComputeAcceleration(const Node &n1, const Node &n2);

            bool VertexFeasible(const Node &parent_node, const Node &child_node);


            std::vector<double> GetNodeCost(const Node &parent_node,
                                            const Node &child_node);

            std::vector<double> GetSingleNodeCost(const Node &node);

            std::deque<Node> GetParentPath(const Node &node);

            std::vector<Node> GetLowerRegion(const Node &node);

            std::vector<Node> GetUpperRegion(const Node &node);

            double WeightingCost(std::vector<double> &cost);

            bool ReachingGoal(const Node &node);

            std::vector<double> GetPathCost(const std::deque<Node> &path);

            double GetPathSmoothness(const std::deque<Node> &path);

            double GetPathVelError(const std::deque<Node> &path);

            void newFile();

            void SendVisualization(const std::deque<Node> &final_path,
                                   const Spline &curve_x,
                                   const Spline &curve_y);

            void ChooseParent(const Node &nearest_node, Node *new_node);

            void Rewire(Node &new_node);

            std::string int2string(int value);

            std::deque<Node> PostProcessing(std::deque<Node> &path);

            void PrintTree();

            void PrintPath(const std::vector<Node> &path);

            void CopyFile(const std::string&source, const std::string& new_file);

        private:

            Obstacles obstacles_;
            std::vector<Node> tree_;
            double max_tree_t_ = 0;

            ReferencePath reference_path_;
            Spline curve_x_;
            Spline curve_y_;
            int is_rand_ = 0;

            std::string planning_path_;
            std::string file_name_;
            std::ofstream out_file_;

            planning::speed_profile::SpeedProfileConf speed_profile_conf_;
            planning::speed_profile::RRTConfig rrt_conf_;

        };
    }
}

#endif //PLANNING_SRC_RRT_H_
