#include "speed_profile_record.h"

using namespace std;

namespace planning {
namespace speed_profile {
SpeedProfileRecord::SpeedProfileRecord() {
    time_t t = std::time(0);
    struct tm * now = std::localtime(&t);

    //the name of bag file is better to be determined by the system time
    time_string_ = utils::StringUtils::int2string(now->tm_year + 1900)+
             '-' + utils::StringUtils::int2string(now->tm_mon + 1)+
             '-' + utils::StringUtils::int2string(now->tm_mday)+
             '-' + utils::StringUtils::int2string(now->tm_hour)+
             '-' + utils::StringUtils::int2string(now->tm_min)+
             '-' + utils::StringUtils::int2string(now->tm_sec);

}

void SpeedProfileRecord::RecordTree(const std::vector<Node>& tree) {
    for (Node node : tree) {
        debug::speed::stState point;
        point.set_t(node.time);
        point.set_index(node.self_id);
        point.set_parent_index(node.parent_id);
        point.set_s(node.distance);
        point.set_v(node.velocity);
        speed_profile_debug_.add_tree()->CopyFrom(point);
    }
}

void SpeedProfileRecord::RecordPath(
        const RRTConfig& rrt_conf,
        const std::deque<Node>& path,
        const utils::Spline& curve_x,
        const utils::Spline& curve_y) {
    for (Node node : path) {
        debug::speed::stState point;
        point.set_t(node.time);
        point.set_s(node.distance);
        point.set_v(node.velocity);
        speed_profile_debug_.add_st_path()->CopyFrom(point);
    }

    debug::speed::xyStates states;
    for (int i = 0; i < rrt_conf.t_goal(); i++) {
        debug::speed::xyState state;
        double angle = 0;
        int k = i / rrt_conf.dt();
        state.set_x(curve_x(path[k].distance));
        state.set_y(curve_y(path[k].distance));
        state.set_theta(atan2(curve_x.deriv1(path[k].distance),
                              curve_y.deriv1(path[k].distance)));
        states.add_states()->CopyFrom(state);
    }
    speed_profile_debug_.mutable_prediction_veh()->CopyFrom(states);
}


void SpeedProfileRecord::PrintToFile(const std::string& folder) {
    std::string file_name = folder + "speed_" + ".txt";
    std::cout << "[SpeedProfileRecord] record file name:" << file_name << std::endl;
    utils::SetProtoToASCIIFile(speed_profile_debug_, file_name);
}

void SpeedProfileRecord::RecordDistanceMap(
        const std::vector<double>& distance_t,
        const std::vector<double>& distance_s) {
    debug::speed::DistanceMap map;
    for (int i = 0; i < distance_t.size(); ++i) {
        debug::speed::stState point;
        point.set_t(distance_t[i]);
        point.set_s(distance_s[i]);
        map.add_points()->CopyFrom(point);
    }
    speed_profile_debug_.mutable_distance_map()->CopyFrom(map);
}

void SpeedProfileRecord::RecordObstacles(
        const std::vector<PredictionObstacle>& obstacles) {
    for (PredictionObstacle obs : obstacles) {
        debug::speed::xyStates states;
        for (int i = 0; i < 5; i++) {
            debug::speed::xyState state;
            state.set_x(obs.x() + obs.velocity() * i * sin(obs.theta()));
            state.set_y(obs.y() + obs.velocity() * i * cos(obs.theta()));
            state.set_theta(obs.theta());
            states.add_states()->CopyFrom(state);
        }
        speed_profile_debug_.add_prediction_obs()->CopyFrom(states);
    }
}

}
}
