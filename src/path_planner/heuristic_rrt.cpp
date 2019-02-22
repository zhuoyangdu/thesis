//
//  heuristic_rrt.cpp
//  rrt
//
//  Created by zhuoyang on 2018/10/24.
//  Copyright © 2018年 zhuoyang. All rights reserved.
//

#include "heuristic_rrt.h"

#include <queue>
#include <functional>
#include <chrono>
#include "float.h"
#include <fstream>

using namespace std;

namespace planning {
namespace  path_planner {
HeuristicRRT::HeuristicRRT(const PathPlannerConf& path_planner_conf)
    : rrt_conf_(path_planner_conf.rrt_conf()), is_init_(true),
      path_planner_conf_(path_planner_conf) {
    show_image_ = rrt_conf_.show_image();
    tree_.reserve(2000);
}

void HeuristicRRT::Init(const SearchingFrame* searching_frame) {
    cv::Mat attractive_prob = searching_frame->AttractiveMap();
    probablistic_map_ = ProbablisticMap(attractive_prob);
    cout << "[HeuristicRRT] Generate Probablistic Map." << endl;

    cv::Point init = searching_frame->PixelCurrentState();
    cv::Point goal = searching_frame->PixelGoal();
    cout << "[HeuristicRRT] goal:" << goal.x << "," << goal.y << endl;
    cout << "[HeuristicRRT] init" << init.x << "," << init.y << endl;


    init_node_ = Node(int(init.y), int(init.x));
    init_node_.SetIndex(0);
    init_node_.SetParent(-1);
    tree_ = {init_node_};


    goal_node_ = Node(int(goal.y), int(goal.x));
    std::cout << "[HeuristicRRT] init pixel: col" << init_node_.col()
        << ", row:" << init_node_.row() << std::endl;
    std::cout << "[HeuristicRRT] goal pixel: col: " << goal_node_.col()
        << ", row:" << goal_node_.row() << std::endl;

    gnat_ = GNAT(rrt_conf_.pivots_k(), cv::Size(512,512));
    gnat_.add(init_node_);

}

PlanningStatus HeuristicRRT::Solve(const SearchingFrame* searching_frame) {
    utils::Timer t1;
    srand(time(0));

    Init(searching_frame);
    std::cout << "[HeuristicRRT] Init time:" << t1.duration() << std::endl;

    utils::Timer t2;
    Extend(searching_frame);

    std::cout << "[HeuristicRRT] shortest_path_length:" << shortest_path_length_
    << ", spline: " << shortest_spath_length_ << std::endl;
    std::cout << "[HeuristicRRT] expand elapsed seconds:" << t2.duration() << "s\n";

    if (min_path_.size()!=0) {
        std::vector<Node> spline_path = PostProcessing(min_path_, searching_frame);
        // shortest_spath_length_ = FrenetPathLength(spline_path);
        vector<double> global_x, global_y;
        GetGlobalPath(spline_path, searching_frame, &global_x, &global_y);

        if (rrt_conf_.record())
            RecordStructured(tree_, spline_path, min_path_, global_x, global_y);

        if (show_image_) {
            PlotStructured(searching_frame, spline_path);
        }

    } else {
        if (rrt_conf_.record())
            RecordStructured(tree_, std::vector<Node>{}, min_path_,
                    vector<double>{} , vector<double>{});

    }

    std::cout << "heuristic rrt elapsed time: " << t1.duration() << endl;
    return PlanningStatus::OK();
}


PlanningStatus HeuristicRRT::MultiThreadSolve(const SearchingFrame* searching_frame) {
    utils::Timer t1;
    srand(time(0));

    Init(searching_frame);
    std::cout << "[HeuristicRRT] Init time:" << t1.duration() << std::endl;

    utils::Timer t2;
    rrt_conf_.set_max_attemp(rrt_conf_.max_attemp() / 4);
    std::thread thread1(&HeuristicRRT::Extend, this, searching_frame);
    std::thread thread2(&HeuristicRRT::Extend, this, searching_frame);
    std::thread thread3(&HeuristicRRT::Extend, this, searching_frame);
    std::thread thread4(&HeuristicRRT::Extend, this, searching_frame);
    // Extend(searching_frame);
    thread1.join();
    thread2.join();
    thread3.join();
    thread4.join();

    std::cout << "[HeuristicRRT] shortest_path_length:" << shortest_path_length_
    << ", spline: " << shortest_spath_length_ << std::endl;
    std::cout << "[HeuristicRRT] expand elapsed seconds:" << t2.duration() << "s\n";

    if (min_path_.size()!=0) {
        std::vector<Node> spline_path = PostProcessing(min_path_, searching_frame);
        // shortest_spath_length_ = FrenetPathLength(spline_path);
        vector<double> global_x, global_y;
        GetGlobalPath(spline_path, searching_frame, &global_x, &global_y);

        if (rrt_conf_.record())
            RecordStructured(tree_, spline_path, min_path_, global_x, global_y);

        if (show_image_) {
            PlotStructured(searching_frame, spline_path);
        }

    } else {
        if (rrt_conf_.record())
            RecordStructured(tree_, std::vector<Node>{}, min_path_,
                    vector<double>{} , vector<double>{});

    }

    std::cout << "heuristic rrt elapsed time: " << t1.duration() << endl;
    return PlanningStatus::OK();
}

void HeuristicRRT::GetGlobalPath(const std::vector<Node>&spline_path,
                                 const SearchingFrame* searching_frame,
                                 vector<double>* global_x,
                                 vector<double>* global_y) {
    for (Node node : spline_path) {
        double s, d, x, y;
        searching_frame->FromImageToXY(node.row(), node.col(), &x, &y);
        global_x->push_back(x);
        global_y->push_back(y);
    }
}

void HeuristicRRT::Plot(const SearchingFrame* environment) {
    cv::Mat img_env;
    cvtColor(environment->DynamicMap(), img_env, COLOR_GRAY2BGR);
    for (Node node : tree_) {
        if (node.parent_index() >= 0) {
            Node parent_node = tree_[node.parent_index()];
            ImageProc::PlotLine(img_env, node, parent_node,
                                Scalar(0,255,0), 1);
        }
    }
    if (min_path_.size() > 0) {
        ImageProc::PlotPath(img_env, min_path_, Scalar(0,0,255), 2);
    }

    cv:imshow("result", img_env);
    cv::waitKey(0);
}

void HeuristicRRT::PlotStructured(const SearchingFrame* environment,
                                  const std::vector<Node>& spline_path) {
    cv::Mat img_env;
    cvtColor(environment->DynamicMap(), img_env, COLOR_GRAY2BGR);
    for (Node node : tree_) {
        if (node.parent_index() >= 0) {
            Node parent_node = tree_[node.parent_index()];
            ImageProc::PlotLine(img_env, node, parent_node,
                                Scalar(0,255,0), 1);
        }
    }
    if (min_path_.size() > 0) {
        ImageProc::PlotPath(img_env, min_path_, Scalar(0,0,255), 2);
        ImageProc::PlotPath(img_env, spline_path, Scalar(255,0,255), 2);
    }

    imshow("result", img_env);
    cv::waitKey(1);
}

void HeuristicRRT::Extend(const SearchingFrame* searching_frame) {
    int i = 0;
    while (i <= rrt_conf_.max_attemp()) {
        // if (i % 100 == 0) cout << "[HeuristicRRT] sample time:" << i << endl;
        // Heuristic sample.
        Node sample;
        if (rrt_conf_.uniform_sample()) {
            sample = UniformSample(searching_frame);
        } else {
            sample = probablistic_map_.Sampling();
        }

        // Path prior.
        bool turn_on_prior = rrt_conf_.turn_on_prior();
        if (turn_on_prior) {
            double max_dist_sample = sqrt(Node::SquareDistance(sample, init_node_)
                                     + Node::SquareDistance(sample, goal_node_));
            if (shortest_path_length_!=0 && max_dist_sample > shortest_path_length_) {
                continue;
            }
        }

        vector<Node> nearest_nodes;
        if (!GetNearestNodes(sample, &nearest_nodes)) {
            continue;
        }

        Node new_node;
        Node nearest_node;
        bool success = false;
        for (Node candidate_node : nearest_nodes) {
            if (! Steer(sample, candidate_node, &new_node)) {
                continue;
            }
            if (CheckCollision(candidate_node, new_node, *searching_frame)) {
                // Collide.
                continue;
            }
            nearest_node = candidate_node;
            success = true;
            break;
        }

        if (!success) {
            continue;
        }

        // Add to tree.
        rrt_mutex.lock();
        i++;
        new_node.SetIndex(int(tree_.size()));
        new_node.SetParent(nearest_node.index());
        tree_.push_back(new_node);
        gnat_.add(new_node);
        if (CheckTarget(new_node, goal_node_)) {
            vector<Node> path = GetPath(tree_, new_node);
            double path_length;
            path_length = PathLength(path);
            if (shortest_path_length_==0 || shortest_path_length_ > path_length) {
                std::cout << "A shorter path found!" << std::endl;
                shortest_path_length_ = path_length;
                min_path_ = path;
            }
        }
        rrt_mutex.unlock();
    }
}

Node HeuristicRRT::UniformSample(const SearchingFrame* frame) {
    // srand(time(0));
    int rand_row = int((double) rand() / RAND_MAX * 511);
    int rand_col = int((double) rand() / RAND_MAX * 511);
    if (frame->CheckCollisionByPixelCoord(rand_row, rand_col)) {
        return UniformSample(frame);
    } else {
        return Node(rand_row, rand_col);
    }
}

bool HeuristicRRT::GetNearestNode(const Node& sample,
                                  Node* nearest_node) {

    vector<Node> k_nearest = gnat_.kNearestPoints(sample, 5);
    bool success = false;
    double dtheta = 0.0;
    for (int i = k_nearest.size()-1; i >=0; --i) {
        Node tmp = k_nearest[i];
        Node parent_tmp;
        if (tmp.parent_index() != -1) {
            parent_tmp = tree_[tmp.parent_index()];
            dtheta = Node::GetDeltaTheta(parent_tmp, tmp, sample);
            if (dtheta < M_PI / 3) {
                continue;
            } else {
                *nearest_node = tmp;
                success = true;
                break;
            }
        } else {
            *nearest_node = tmp;
            success = true;
            break;
        }
    }
    return success;
}

bool HeuristicRRT::GetNearestNodes(const Node& sample,
                                  vector<Node>* nearest_nodes) {
    vector<Node> k_nearest = gnat_.kNearestPoints(sample, 5);
    nearest_nodes->clear();
    double dtheta = 0.0;
    for (int i = k_nearest.size()-1; i >=0; --i) {
        // TODO(zhuoyang add constraint)
        //if (is_structured_) {
            if (k_nearest[i].col() > sample.col()) {
                continue;
            }
        //}

        Node tmp = k_nearest[i];
        Node parent_tmp;
        if (tmp.parent_index() != -1) {
            parent_tmp = tree_[tmp.parent_index()];
            dtheta = Node::GetDeltaTheta(parent_tmp, tmp, sample);
            if (dtheta < M_PI / 2) {
                continue;
            } else {
                nearest_nodes->push_back(k_nearest[i]);
            }
        } else {
            nearest_nodes->push_back(k_nearest[i]);
        }

    }
    if (nearest_nodes->size() > 0) return true;
    else return false;
}

bool HeuristicRRT::Steer(const Node& sample, const Node& nearest,
                         Node* new_node) {
    double theta = atan2(sample.row() - nearest.row(), sample.col() - nearest.col());
    new_node->SetTheta(theta);
    new_node->SetRow(nearest.row() + rrt_conf_.step_size() * sin(theta));
    new_node->SetCol(nearest.col() + rrt_conf_.step_size() * cos(theta));
    if (new_node->row() < rrt_conf_.frenet_conf().image_row() &&
        new_node->col() < rrt_conf_.frenet_conf().image_col() &&
        new_node->row() >= 0 && new_node->col() >= 0) {
        return true;
    } else {
        return false;
    }
}

// If collide, return true.
bool HeuristicRRT::CheckCollision(const Node &a, const Node &b,
                                  const SearchingFrame& env) {
    double dist = sqrt(Node::SquareDistance(a, b));
    double theta = atan2(a.row() - b.row(), a.col() - b.col());
    for (int i = 0; i <= dist; i = i + 2) {
        double row = b.row() + i * sin(theta);
        double col = b.col() + i * cos(theta);
        if (env.CheckCollisionByPixelCoord(row, col)) {
            return true;
        }
    }

    return false;
}

bool HeuristicRRT::CheckTarget(const Node& node, const Node& goal) {
    if (Node::SquareDistance(node, goal) < 400) {
        return true;
    }
    return false;
}

vector<Node> HeuristicRRT::GetPath(const std::vector<Node>& tree,
                                   const Node& new_node) {
    std::vector<Node> path = {new_node};
    int parent_index = new_node.parent_index();
    while (parent_index != -1) {
        path.insert(path.begin(), tree[parent_index]);
        parent_index = tree[parent_index].parent_index();
    }
    return path;
}

double HeuristicRRT::PathLength(const std::vector<Node>& path) {
    double length = 0;
    for (int i = 0; i < path.size()-1; ++i) {
        length += sqrt(Node::SquareDistance(path[i], path[i+1]));
    }
    return length;
}

/*
double HeuristicRRT::FrenetNodeDistance(const Node& a, const Node& b) {
    double dd = (a.row() - b.row()) * path_planner_conf_.rrt_conf().frenet_conf().dd();
    double ds = (a.col() - b.col()) * path_planner_conf_.rrt_conf().frenet_conf().ds();
    return sqrt(dd * dd + ds * ds);
}
*/

double HeuristicRRT::FrenetPathLength(const std::vector<Node>& path) {
    double length = 0;
    for (int i = 0; i < path.size()-1; ++i) {
        double dd = (path[i+1].row() - path[i].row()) *
                    path_planner_conf_.rrt_conf().frenet_conf().dd();
        double ds = (path[i+1].col() - path[i].col()) *
                    path_planner_conf_.rrt_conf().frenet_conf().ds();
        length += sqrt(dd * dd + ds * ds);
    }
    return length;
}

std::vector<Node> HeuristicRRT::PostProcessing(const std::vector<Node>& path,
                                               const SearchingFrame* env) {
    cv::Mat img_env;
    cvtColor(env->DynamicMap(), img_env, COLOR_GRAY2BGR);

    cv::Mat repulsive_col = env->RepulsiveX();
    cv::Mat repulsive_row = env->RepulsiveY();
    std::vector<double> x;
    std::vector<double> y;
    for (Node node : path) {
        y.push_back(node.row());
        x.push_back(node.col());
    }

    double init_a = path[0].theta();
    double y1, y0, x1, x0;

    if (init_a <= M_PI/4 && init_a >=-M_PI/4) {
        double step = rrt_conf_.step_size();
        double angle = 0;

        y0 = y[0] + step * sin(angle);
        y1 = y[0] + step * sin(angle);
        x0 = x[0] - step * cos(angle);
        x1 = x[0] - 2 * step * cos(angle);

    } else if (init_a >= 3*M_PI/4 || init_a <= -3*M_PI/4) {
        y1 = y[0] + 40;
        y0 = (5*y[0]-y1)/4;
        x1 = -2*((0.5* y[0] - 0.5*y1) * tan(init_a) - 0.5*x[0]);
        x0 = (5*x[0] - x1)/4;
    } else if (init_a>M_PI/4 && init_a<= 3*M_PI/4) {
        x1 = x[0] - 40;
        x0 = (5*x[0] - x1)/4;
        y1 = -2*((0.5* x[0]  - 0.5*x1)/ tan(init_a) - 0.5*y[0]);
        y0 = (5*y[0]-y1)/4;
    } else {
        x1 = x[0] + 40;
        x0 = (5*x[0] - x1)/4;
        y1 = -2*((0.5* x[0] - 0.5*x1)/ tan(init_a) - 0.5*y[0]);
        y0 = (5*y[0]-y1)/4;
    }

    x.insert(x.begin(), {x1, x0});
    y.insert(y.begin(), {y1, y0});
    int size = x.size();
    // x.push_back(2*x[size-1] - x[size-2]);
    // y.push_back(2*y[size-1] - y[size-2]);
    x.push_back(x[size-1] + 15);
    x.push_back(x[size-1] + 50);
    y.push_back(y[size-1]);
    y.push_back(y[size-1]);
    size = x.size();

    for (int n = 0; n < rrt_conf_.post_iteration(); ++n) {
        std::vector<double> l_phi_dx, l_phi_dy, p_phi_x, p_phi_y;
        for (int i = 0; i < size; ++i) {
            l_phi_dx.push_back(0);
            l_phi_dy.push_back(0);
            p_phi_x.push_back(0);
            p_phi_y.push_back(0);
        }
        for (int i = 2; i < size -2; ++i) {
            for (int t = 0; t <20; ++t) {
                double u  = t * 1.0 / 19.0;
                double b0 = pow(1.0-u,3)/6.0;
                double b1 = (3.0*pow(u,3)-6.0*pow(u,2)+4)/6.0;
                double b2 = (-3.0*pow(u,3)+3.0*pow(u,2)+3.0*u+1.0)/6.0;
                double b3 = pow(u,3)/6.0;

                double b0_dot = -pow(1.0-u,2)/2.0;
                double b1_dot = 1.5*pow(u,2)-2.0*u;
                double b2_dot = -1.5*pow(u,2)+u+0.5;
                double b3_dot = 0.5*pow(u,2);

                double xd = x[i-2] * b0_dot + x[i-1] * b1_dot +
                x[i] * b2_dot + x[i+1] * b3_dot;
                double yd = y[i-2] * b0_dot + y[i-1] * b1_dot +
                y[i] * b2_dot + y[i+1] * b3_dot;
                double xk = x[i-2] * b0 + x[i-1] * b1 +
                x[i] * b2 + x[i+1] * b3;
                double yk = y[i-2] * b0 + y[i-1] * b1 +
                y[i] * b2 + y[i+1] * b3;

                l_phi_dx[i-2] += 2.0 * xd * b0_dot;
                l_phi_dx[i-1] += 2.0 * xd * b1_dot;
                l_phi_dx[i]   += 2.0 * xd * b2_dot;
                l_phi_dx[i+1] += 2.0 * xd * b3_dot;
                l_phi_dy[i-2] += 2.0 * yd * b0_dot;
                l_phi_dy[i-1] += 2.0 * yd * b1_dot;
                l_phi_dy[i]   += 2.0 * yd * b2_dot;
                l_phi_dy[i+1] += 2.0 * yd * b3_dot;

                xk = xk > 511 ? 511 : xk;
                xk = xk < 0 ? 0 : xk;
                yk = yk > 511 ? 511 : yk;
                yk = yk < 0 ? 0 : yk;

                double py = repulsive_row.at<double>(yk, xk);
                double px = repulsive_col.at<double>(yk, xk);
                p_phi_x[i-2] += px * b0;
                p_phi_x[i-1] += px * b1;
                p_phi_x[i]   += px * b2;
                p_phi_x[i+1] += px * b3;
                p_phi_y[i-2] += py * b0;
                p_phi_y[i-1] += py * b1;
                p_phi_y[i]   += py * b2;
                p_phi_y[i+1] += py * b3;
            }
        }

        double s_error = 0.0;
        double p_error = 0.0;
        for (int t = 2; t < size-2; ++t) {
            s_error += fabs(l_phi_dx[t]) + fabs(l_phi_dy[t]);
            p_error += fabs(p_phi_x[t]) + fabs(p_phi_y[t]);
        }
        double lambda = rrt_conf_.k_repulsive();
        for (int t = 3; t < size; ++t) {
            x[t] -= 0.01 * (1.2 * l_phi_dx[t] / 512 * 20 - lambda * p_phi_x[t]);
            y[t] -= 0.01 * (1.2 * l_phi_dy[t] / 512 * 20 - lambda * p_phi_y[t]);
        }
        //ImageProc::PlotPath(img_env, y, x, Scalar(0,0,0),1);
    }

    std::vector<Node> spline_path;
    for (int i = 2 ; i < x.size()-2; ++i) {
        spline_path.push_back(Node(y[i], x[i]));
    }
    return spline_path;
}


string int2string(int value)
{
    stringstream ss;
    ss<<value;
    return ss.str();
}

void HeuristicRRT::Record(const std::vector<Node>& tree,
                          const std::vector<Node>& spline_path,
                          const std::vector<Node>& path) {
    time_t t = std::time(0);
    struct tm * now = std::localtime( & t );
    string time_s;
    //the name of bag file is better to be determined by the system time
    time_s = int2string(now->tm_year + 1900)+
        '-'+int2string(now->tm_mon + 1)+
        '-'+int2string(now->tm_mday)+
        '-'+int2string(now->tm_hour)+
        '-'+int2string(now->tm_min)+
        '-'+int2string(now->tm_sec);

    std::string file_name = rrt_conf_.record_path()
                            + "/tree-" + time_s;
    if (rrt_conf_.uniform_sample()) {
        file_name += "-uniform-cost_" + std::to_string(shortest_path_length_/512*20) + ".txt";
    } else {
        file_name += "heuristic-cost_" + std::to_string(shortest_path_length_/512*20)
                    + "-scost_" + std::to_string(shortest_spath_length_/512*20) + ".txt";
    }
    std::cout << "record file name:" << file_name << std::endl;
    std::ofstream out_file(file_name.c_str());
    if (!out_file) {
        std::cout << "no file!" << std::endl;
    }
    for (Node node : tree) {
        out_file << node.index() << "\t" << node.row()
        << "\t" << node.col() << "\t" << node.parent_index()
        << "\n";
    }
    out_file.close();

    file_name = rrt_conf_.record_path()
    + "/path-" + time_s + ".txt";
    std::ofstream out_path_file(file_name.c_str());
    for (Node node : path) {
        out_path_file << node.row() << "\t" << node.col() << "\n";
    }
    out_path_file.close();

    if (!rrt_conf_.uniform_sample()) {
        file_name = rrt_conf_.record_path()
        + "/splinepath-" + time_s + ".txt";
        std::ofstream out_spath_file(file_name.c_str());
        for (Node node : spline_path) {
            out_spath_file << node.row() << "\t" << node.col() << "\n";
        }
        out_spath_file.close();
    }

}

void HeuristicRRT::CopyFile(const std::string&source, const std::string& new_file) {
    std::ifstream in(source.c_str(), ios::binary);
    std::ofstream out_file(new_file.c_str());
    if (!in) {
        std::cout << "no input file!" << std::endl;
    }
    if (!out_file) {
        std::cout << "no output file!" << std::endl;
    }
    out_file << in.rdbuf();
    out_file.close();
    in.close();
}

void HeuristicRRT::RecordStructured(const std::vector<Node>& tree,
                          const std::vector<Node>& spline_path,
                          const std::vector<Node>& path,
                          const vector<double>& path_x,
                          const vector<double>& path_y) {
    time_t t = std::time(0);
    struct tm * now = std::localtime( & t );
    string time_s;
    //the name of bag file is better to be determined by the system time
    time_s = int2string(now->tm_year + 1900)+
             '-'+int2string(now->tm_mon + 1)+
             '-'+int2string(now->tm_mday)+
             '-'+int2string(now->tm_hour)+
             '-'+int2string(now->tm_min)+
             '-'+int2string(now->tm_sec);

    double frenet_length = FrenetPathLength(spline_path);
    std::string file_name = rrt_conf_.record_path()
                            + "/tree-" + time_s;
    if (rrt_conf_.uniform_sample()) {
        file_name += "-uniform-cost_" + std::to_string(shortest_path_length_) +
                "frenet_length_" + std::to_string(frenet_length) + ".txt";
    } else {
        file_name += "heuristic-cost_" + std::to_string(shortest_path_length_) +
                "frenet_length_" + std::to_string(frenet_length) + ".txt";
    }
    std::cout << "record file name:" << file_name << std::endl;
    std::ofstream out_file(file_name.c_str());
    if (!out_file) {
        std::cout << "no file!" << std::endl;
    }
    for (Node node : tree) {
        out_file << node.index() << "\t" << node.row()
                 << "\t" << node.col() << "\t" << node.parent_index()
                 << "\n";
    }
    out_file.close();
    std::string copy_file = rrt_conf_.record_path() + "/tree.txt";
    CopyFile(file_name, copy_file);

    file_name = rrt_conf_.record_path()
                + "/path-" + time_s + ".txt";
    std::ofstream out_path_file(file_name.c_str());
    for (Node node : path) {
        out_path_file << node.row() << "\t" << node.col() << "\n";
    }
    out_path_file.close();
    copy_file = rrt_conf_.record_path() + "/path.txt";
    CopyFile(file_name, copy_file);


    if (!rrt_conf_.uniform_sample()) {
        file_name = rrt_conf_.record_path()
                    + "/splinepath-" + time_s + ".txt";
        std::ofstream out_spath_file(file_name.c_str());
        for (Node node : spline_path) {
            out_spath_file << node.row() << "\t" << node.col() << "\n";
        }
        out_spath_file.close();
    }
    copy_file = rrt_conf_.record_path() + "/splinepath.txt";
    CopyFile(file_name, copy_file);

    file_name = rrt_conf_.record_path()
                + "/globalpath-" + time_s + ".txt";
    std::ofstream out_global_file(file_name.c_str());
    for (int i = 0; i < path_x.size(); ++i ) {
        out_global_file << path_x[i] << "\t" << path_y[i] << "\n";
    }
    out_global_file.close();
    copy_file = rrt_conf_.record_path() + "/globalpath.txt";
    CopyFile(file_name, copy_file);

/*
    file_name = rrt_conf_.record_path()
                + "/obstacle_polygons-" + time_s + ".txt";
    std::ofstream out_obs_file(file_name.c_str());
    vector<vector<cv::Point>> obstacle_polygons = frenet_frame.obstacle_polygons();
    for (vector<cv::Point> obstacle_polygon : obstacle_polygons) {
        for (cv::Point p : obstacle_polygon) {
            out_obs_file << p.x << "\t" << p.y << "\n";
        }
        out_obs_file << "-1\t-1\n";
    }
    out_obs_file.close();
    copy_file = rrt_conf_.record_path() + "/obstacle_polygons.txt";
    CopyFile(file_name, copy_file);
*/
    /*
    file_name = rrt_conf_.record_path()
                + "/vehicle_obs-" + time_s + ".txt";
    std::ofstream out_veh_file(file_name.c_str());
    out_veh_file << vehicle_state_.x() << "\t" << vehicle_state_.y()
        << "\t" << vehicle_state_.theta() << endl;
    PredictionObstacles obstacles = path_planner_conf_.environment_conf().static_obs();
    for (int i = 0; i < obstacles.obstacle_size(); ++i) {
        out_veh_file << obstacles.obstacle(i).x() << "\t"
            << obstacles.obstacle(i).y() << "\t"
            << obstacles.obstacle(i).theta() << "\n";
    }
    out_veh_file.close();
    copy_file = rrt_conf_.record_path() + "/vehicle_obs.txt";
    CopyFile(file_name, copy_file);
*/
}

}

}  // namespace planning

