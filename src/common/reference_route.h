//
// Created by zhuoyang on 2018-12-29.
//

#ifndef PLANNING_REFERENCE_ROUTE_H
#define PLANNING_REFERENCE_ROUTE_H

#include <iostream>
#include <vector>

#include "../utils/spline.h"
#include "environment_conf.pb.h"

namespace planning {

class ReferenceRoute {
public:
    ReferenceRoute() = default;

    ReferenceRoute(const planning::Route& reference_route);

    utils::Spline get_x() const {return curve_x_;}

    utils::Spline get_y() const {return curve_y_;}

    double GetCurvature(double s);

    double x(double s) const {return curve_x_(s);}

    double y(double s) const {return curve_y_(s);}

    double theta(double s) const {return atan2(curve_y_.deriv1(s), curve_x_.deriv1(s));}

    void FromXYToSD(double x, double y, double* s, double* d) const;

    void FromSDToXY(double s, double d, double* x, double* y) const;

private:

    void RouteSpline(const std::vector<double>& xs,
                     const std::vector<double>& ys);

    utils::Spline curve_x_;
    utils::Spline curve_y_;


};

}

#endif //PLANNING_REFERENCE_ROUTE_H
