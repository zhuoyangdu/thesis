//
//  heuristic_rrt.hpp
//  rrt
//
//  Created by zhuoyang on 2018/10/24.
//  Copyright © 2018年 zhuoyang. All rights reserved.
//

// Copyright [2018] <Zhuoyang Du>

#ifndef SRC_PLANNING_SRC_RRT_HEURISTIC_RRT_H_
#define SRC_PLANNING_SRC_RRT_HEURISTIC_RRT_H_

#include "path_planner_conf.pb.h"

#include <iostream>
#include <vector>
#include <memory>
#include <thread>
#include <mutex>

#include "image_proc.h"
#include "searching_frame.h"
#include "node.h"
#include "probablistic_map.h"
#include "gnat.h"
#include "path_planner_record.h"

#include "../common/planning_status.h"
#include "../frame/frame.h"

#include "vehicle_state.pb.h"


using namespace std;

namespace planning {

    namespace  path_planner {
class HeuristicRRT {
public:
    HeuristicRRT() = default;

    explicit HeuristicRRT(const PathPlannerConf& path_planner_conf);

    PlanningStatus Solve(const SearchingFrame* searching_frame);
    PlanningStatus MultiThreadSolve(const SearchingFrame* searching_frame);

    PlanningStatus MultiThreadSolve(const std::unique_ptr<Frame>& frame);

private:
    void Init(const SearchingFrame* searching_frame);

    void Extend(const SearchingFrame* searching_frame);

    void Plot(const SearchingFrame* environment);

    void PlotStructured(const SearchingFrame* environment,
                        const std::vector<Node>& spline_path);

    struct Compare {
        Compare(Node sample) {this->sample = sample;}
        bool operator() (Node& a, Node& b) {
            int dist1 = (a.col() - sample.col()) * (a.col() - sample.col())
            + (a.row() - sample.row()) * (a.row() - sample.row());
            int dist2 = (b.col() - sample.col()) * (b.col() - sample.col())
            + (b.row() - sample.row()) * (b.row() - sample.row());
            return dist1 > dist2;
        }
        Node sample;
    };

    bool GetNearestNode(const Node& sample,
                        Node* nearest_node);

    bool GetNearestNodes(const Node& sample,
                         vector<Node>* nearest_nodes);

    bool CheckCollision(const Node& a, const Node& b, const SearchingFrame& env);

    bool Steer(const Node& sample, const Node& nearest, Node* new_node);

    bool CheckTarget(const Node& node, const Node& goal);

    vector<Node> GetPath(const std::vector<Node>& tree, const Node& new_node);

    double PathLength(const std::vector<Node>& path);

    double FrenetPathLength(const std::vector<Node>& path);

    // double FrenetNodeDistance(const Node& a, const Node& b);

    std::vector<Node> PostProcessing(const std::vector<Node>& path,
                                     const SearchingFrame* env);

    void Record(const std::vector<Node>& tree,
                const std::vector<Node>& spline_path,
                const std::vector<Node>& path);

    void RecordStructured(const std::vector<Node>& tree,
                        const std::vector<Node>& spline_path,
                        const std::vector<Node>& path,
                        const vector<double>& path_x,
                        const vector<double>& path_y);

    Node UniformSample(const SearchingFrame* environment);

    void GetGlobalPath(const std::vector<Node>&spline_path,
                       const SearchingFrame* searching_frame,
                       vector<double>* global_x,
                       vector<double>* global_y);

    void CopyFile(const std::string&source, const std::string& new_file);

    bool is_init_ = false;
    PathPlannerConf path_planner_conf_;
    RRTConf rrt_conf_;
    bool show_image_ = false;
    double shortest_path_length_ = 0;
    double shortest_spath_length_ = 0;
    std::vector<Node> min_path_;

    ProbablisticMap probablistic_map_;
    std::vector<Node> tree_;
    Node init_node_;
    Node goal_node_;
    GNAT gnat_;

    std::mutex rrt_mutex;
    PathPlannerRecord path_planner_record_;

};

    }
}  // namespace planning

#endif  // SRC_PLANNING_SRC_RRT_HEURISTIC_RRT_H_
