#ifndef PLANNING_SRC_ROUTE_H_
#define PLANNING_SRC_ROUTE_H_

#include <iostream>

#include "../utils/spline.h"
#include "../utils/file_config.h"
#include "../utils/string.h"

namespace planning {
class ReferencePath {
 public:
    ReferencePath() = default;

    ReferencePath(const std::string& road_file);

    ReferencePath(
        const std::vector<double> path_x,
        const std::vector<double> path_y);

    utils::Spline get_x() { return curve_x_; }

    utils::Spline get_y() { return curve_y_; }

    double GetCurvature(double s);

    double x(double s) { return curve_x_(s); }

    double y(double s) { return curve_y_(s); }

    double theta(double s) { return atan2(curve_x_.deriv1(s), curve_y_.deriv1(s)); }

    std::vector<double> path_x() {return path_x_; }

    std::vector<double> path_y() {return path_y_; }

 private:
    void GetGeometryPath();

    void GetGeometryPath(
        const std::vector<double> xs,
        const std::vector<double> ys);

    std::string planning_path_;
    std::string road_file_;

    utils::Spline curve_x_;
    utils::Spline curve_y_;

    std::vector<double> path_x_;
    std::vector<double> path_y_;
};

} // namespace planning

#endif // PLANNING_SRC_ROUTE_H_
