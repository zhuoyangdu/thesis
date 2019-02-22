//
// Created by zhuoyang on 2018-12-30.
//

#include "static_obstacles.h"

namespace planning {

StaticObstacle::StaticObstacle(const planning::PredictionObstacle& prediction_obstacle)
    : obstacle_(prediction_obstacle) {
}

void StaticObstacle::GetVertexes(std::vector<double> *vertexes_x,
                           std::vector<double> *vertexes_y) const {
    double angle1 = atan2(width_, length_) + obstacle_.theta();
    double angle2 = obstacle_.theta() - atan2(width_, length_);

    double edge = sqrt(width_ * width_ + length_ * length_) / 2.0;
    double x1 = obstacle_.x() + edge * cos(angle1);
    double y1 = obstacle_.y() + edge * sin(angle1);
    double x3 = obstacle_.x() - edge * cos(angle1);
    double y3 = obstacle_.y() - edge * sin(angle1);
    double x2 = obstacle_.x() + edge * cos(angle2);
    double y2 = obstacle_.y() + edge * sin(angle2);
    double x4 = obstacle_.x() - edge * cos(angle2);
    double y4 = obstacle_.y() - edge * sin(angle2);
    *vertexes_x = {x1, x2, x3, x4};
    *vertexes_y = {y1, y2, y3, y4};
}

StaticObstacles::StaticObstacles(const ReferenceRoute& reference_route,
                     const planning::PredictionObstacles &prediction_obstacles) {
    for (int i = 0; i < prediction_obstacles.obstacle_size(); ++i) {
        PredictionObstacle pre_obs = prediction_obstacles.obstacle(i);
        double os, od;
        reference_route.FromXYToSD(pre_obs.x(), pre_obs.y(), &os, &od);
        pre_obs.set_s(os);
        pre_obs.set_d(od);
        StaticObstacle obstacle(pre_obs);
        obstacles_.push_back(obstacle);
    }
}

}
