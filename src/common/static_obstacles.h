//
// Created by zhuoyang on 2018-12-30.
//

#ifndef PLANNING_OBSTACLES_H
#define PLANNING_OBSTACLES_H

#include <iostream>
#include <vector>

#include "prediction_obstacles.pb.h"
#include "reference_route.h"

using namespace std;

namespace  planning {
class StaticObstacle {
public:
    StaticObstacle(const PredictionObstacle& prediction_obstacle);

    void GetVertexes(std::vector<double>* vertexes_x, std::vector<double>* vertexes_y) const;

    double x() { return obstacle_.x(); }

    double y() {return obstacle_.y(); }

    double theta() {return obstacle_.theta();}

private:
    PredictionObstacle obstacle_;

    double width_ = 4;
    double length_ = 5;
};

class StaticObstacles {
public:
    StaticObstacles() = default;

    StaticObstacles(const ReferenceRoute& reference_route,
              const PredictionObstacles& prediction_obstacles);

    vector<StaticObstacle> obstacles() const {return obstacles_;}

private:
    std::vector<StaticObstacle> obstacles_;
};
}
#endif //PLANNING_OBSTACLES_H
