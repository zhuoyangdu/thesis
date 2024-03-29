#include "dynamic_obstacles.h"
using namespace std;
namespace planning {
namespace speed_profile {
DynamicObstacles::DynamicObstacles(const SpeedProfileConf &speed_profile_conf) :
        speed_profile_conf_(speed_profile_conf) {
    rrt_conf_ = speed_profile_conf_.rrt();
}

void DynamicObstacles::SetObstacles(const planning::PredictionObstacles &obstacle_map) {
    for (int i = 0; i < obstacle_map.obstacle_size(); ++i) {
        obstacles_.push_back(obstacle_map.obstacle(i));
    }
}

bool DynamicObstacles::CollisionFree(const planning::speed_profile::Node &parent_node,
                              const planning::speed_profile::Node &child_node,
                              const utils::Spline &curve_x,
                              const utils::Spline &curve_y) {
    for (int i = 0; i < obstacles_.size(); i++) {
        PredictionObstacle obs = obstacles_[i];
        double t = parent_node.time;
        while (t <= child_node.time) {
            double obs_pos_x = obs.x() + obs.velocity() * t * sin(obs.theta());
            double obs_pos_y = obs.y() + obs.velocity() * t * cos(obs.theta());
            double vel = (parent_node.distance - child_node.distance) /
                         (parent_node.time - child_node.time);
            double s = parent_node.distance + vel * (t - parent_node.time);
            double vehicle_x = curve_x(s);
            double vehicle_y = curve_y(s);
            double dist = pow(pow(obs_pos_x - vehicle_x, 2) + pow(obs_pos_y - vehicle_y,
                                                                  2), 0.5);
            t = t + 0.1;
            if (dist < rrt_conf_.collision_distance()) {
                return false;
            }
        }
    }
    return true;
}

double DynamicObstacles::NonlinearRisk(double input) {
    if (input > rrt_conf_.safe_distance()) return 0;

    return rrt_conf_.k_risk() / (input - rrt_conf_.danger_distance());
}

double DynamicObstacles::RiskAssessment(
        const std::deque<Node> &path,
        const utils::Spline &curve_x,
        const utils::Spline &curve_y) {
    if (obstacles_.empty()) {
        return 0;
    }
    double max_ttc = rrt_conf_.safe_ttc();
    for (int i = 0; i < path.size(); i++) {
        double node_ttc = ComputeTTC(path[i].time, path[i].distance,
                                     path[i].velocity, curve_x, curve_y);
        if (node_ttc >= max_ttc) {
            continue;
        } else {
            max_ttc = node_ttc;
        }
    }
    double risk;
    if (max_ttc < rrt_conf_.safe_ttc()) {
        risk = 1 / max_ttc;
    } else {
        risk = 0;
    }

    return risk;
}

void DynamicObstacles::InitializeDistanceMap(
        const VehicleState &vehicle_state,
        const utils::Spline &curve_x,
        const utils::Spline &curve_y,
        double s0) {
    init_vehicle_path_length_ = s0;
    int nt = static_cast<int>(rrt_conf_.t_max() / kDeltaT) + 1;
    int ns = static_cast<int>(rrt_conf_.s_max() / kDeltaS) + 1;
    // Init distance map.
    distance_map_.clear();
    for (int i = 0; i <= nt; i++) {
        std::vector<double> dd;
        for (int j = 0; j <= ns; j++) {
            dd.push_back(0);
        }
        distance_map_.push_back(dd);
    }

    if (obstacles_.size() == 0) {
        return;
    }

    for (int k = 0; k < obstacles_.size(); k++) {
        PredictionObstacle obs = obstacles_[k];
    }

    for (int i = 0; i <= nt; i++) {
        for (int j = 0; j <= ns; j++) {
            double t = i * kDeltaT;
            double s = j * kDeltaS;
            double vehicle_x = curve_x(s + s0);
            double vehicle_y = curve_y(s + s0);
            double ss = rrt_conf_.safe_distance();
            for (int k = 0; k < obstacles_.size(); k++) {
                PredictionObstacle obs = obstacles_[k];
                double obs_pos_x = obs.x() + obs.velocity() * t * sin(obs.theta());
                double obs_pos_y = obs.y() + obs.velocity() * t * cos(obs.theta());
                double dis = pow(pow(obs_pos_x - vehicle_x, 2) + pow(obs_pos_y - vehicle_y,
                                                                     2), 0.5);
                if (ss > dis) ss = dis;
            }
            distance_map_[i][j] = ss;
        }
    }
    recordDistanceMap();
    if (!recordDistanceMapProto()) {
        std::cout << "Error in distance map." << std::endl;
    }

    cout <<  ReadDistanceMap(Node(4.5, 30, -1)) << endl;
    return;
}

double DynamicObstacles::EuclideanDisToObs(double x, double y, double t) {
    double dis = 1000;
    for (int k = 0; k < obstacles_.size(); k++) {
        double obs_x = obstacles_[k].x() + obstacles_[k].velocity() * t * sin(
                obstacles_[k].theta());
        double obs_y = obstacles_[k].y() + obstacles_[k].velocity() * t * cos(
                obstacles_[k].theta());
        double od = pow(pow(x - obs_x, 2) + pow(y - obs_y,
                                                2), 0.5);
        if (dis > od) dis = od;
    }
    return dis;
}

double DynamicObstacles::ComputeTTC(
        double node_time, double node_distance,
        double node_vel,
        const utils::Spline &curve_x,
        const utils::Spline &curve_y) {
    double ttc = 10;
    if (obstacles_.size() == 0) {
        return ttc;
    }

    for (int tt = 0; tt < 5; tt++) {
        double t = tt * 0.1;
        double veh_x = curve_x(node_distance + t * node_vel);
        double veh_y = curve_y(node_distance + t * node_vel);
        double distance = EuclideanDisToObs(veh_x, veh_y, t + node_time);
        if (distance < rrt_conf_.collision_distance()) {
            ttc = t;
            break;
        }
    }
    return ttc;
}

std::vector<std::vector<double>> DynamicObstacles::ComputeTTCForFixedVel(
        double current_path_length,
        double node_vel,
        const utils::Spline &curve_x,
        const utils::Spline &curve_y) {
    std::string file_name_ = speed_profile_conf_.record_path() + "ttc_map.txt";
    std::ofstream out_file_(file_name_.c_str(), std::ios::in | std::ios::app);
    out_file_ << "velocity\t" << node_vel << "\n";

    std::vector<std::vector<double>> ttc_for_vel;
    //t s v
    for (int tt = 0; tt < 10 * rrt_conf_.t_max(); tt++) {
        double t = tt * 0.1;
        std::vector<double> ttc_t;
        for (int s = 0; s < rrt_conf_.s_max(); s++) {
            // Compute ttc of a node. (t, s, v)
            double ttc = ComputeTTC(t, current_path_length + s, node_vel,
                                    curve_x, curve_y);
            ttc_t.push_back(ttc);
            out_file_ << ttc << "\t";
        }
        ttc_for_vel.push_back(ttc_t);
        out_file_ << "\n";
    }
    // out_file_ << "end_vel\n";
    out_file_.close();
    return ttc_for_vel;
}

void DynamicObstacles::ComputeTTCMap(
        double current_path_length,
        const utils::Spline &curve_x,
        const utils::Spline &curve_y) {
    for (int i = 0; i < 20; i++) {
            std::vector<std::vector<double> > ttc_vel;;
            ttc_vel = ComputeTTCForFixedVel(current_path_length, i,
                                            curve_x, curve_y);
        }
}

double DynamicObstacles::ReadDistanceMap(const Node &node) {
    if (obstacles_.size() < 1) {
        return rrt_conf_.safe_distance();
    }
    int index_t = static_cast<int>(node.time / kDeltaT);
    int index_s = static_cast<int>((node.distance - init_vehicle_path_length_) /
                                   kDeltaS);
    double dist = distance_map_[index_t][index_s];

    return dist;
}

bool DynamicObstacles::recordDistanceMapProto() {
    std::string file_name = speed_profile_conf_.record_path() + "distance_map_dbg.pb.txt";
    planning::speed_profile::ObstacleMapDebug obs_debug;
    obs_debug.set_init_path_length(init_vehicle_path_length_);
    obs_debug.set_delta_t(kDeltaT);
    obs_debug.set_delta_s(kDeltaS);
    obs_debug.set_t_goal(rrt_conf_.t_goal());
    obs_debug.set_max_vel(rrt_conf_.max_vel());
    obs_debug.set_danger_distance(rrt_conf_.danger_distance());
    return utils::SetProtoToASCIIFile(obs_debug, file_name);
}

void DynamicObstacles::recordDistanceMap() {
    int nt = static_cast<int>(rrt_conf_.t_max() / kDeltaT) + 1;
    int ns = static_cast<int>(rrt_conf_.s_max() / kDeltaS) + 1;
    std::string file_name_ = speed_profile_conf_.record_path() + "distance_map.txt";
    std::ofstream out_file_(file_name_.c_str());
    // out_file_ << init_vehicle_path_length_ << "\t";
    // for (int j = 1; j <= ns; j++) {
    //     out_file_ << 0 << "\t";
    // }
    // out_file_ << "\n";

    for (int i = 0; i <= nt; i++) {
        for (int j = 0; j <= ns; j++) {
            double t = i * kDeltaT;
            double s = j * kDeltaS;
            double dist = distance_map_[i][j];
            out_file_ << dist << "\t";
        }
        out_file_ << "\n";
    }
    out_file_.close();
}

void DynamicObstacles::DistancePoint(
    std::vector<double>* distance_t,
    std::vector<double>* distance_s) {
    if (obstacles_.size() == 0) {
        return;
    }
    int nt = static_cast<int>(rrt_conf_.t_max() / kDeltaT) + 1;
    int ns = static_cast<int>(rrt_conf_.s_max() / kDeltaS) + 1;
    for (int i = 0; i <= nt; i++) {
        for (int j = 0; j <= ns; j++) {
            double t = i * kDeltaT;
            double s = j * kDeltaS;
            if (distance_map_[i][j] < rrt_conf_.danger_distance()) {
                distance_t->push_back(i * kDeltaT);
                distance_s->push_back(j * kDeltaS);
            }
        }
    }
}
}

}
