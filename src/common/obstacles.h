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
class Obstacle {
public:
    Obstacle(const PredictionObstacle& prediction_obstacle);

    void GetVertexes(std::vector<double>* vertexes_x, std::vector<double>* vertexes_y) const;

private:
    PredictionObstacle obstacle_;

    double width_ = 4;
    double length_ = 5;
};

class Obstacles {
public:
    Obstacles() = default;

    Obstacles(const ReferenceRoute& reference_route,
              const PredictionObstacles& prediction_obstacles);

    vector<Obstacle> obstacles() const {return obstacles_;}

private:
    std::vector<Obstacle> obstacles_;

};
}
#endif //PLANNING_OBSTACLES_H
