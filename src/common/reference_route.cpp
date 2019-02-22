//
// Created by zhuoyang on 2018-12-29.
//

#include "reference_route.h"

namespace planning {
ReferenceRoute::ReferenceRoute(const planning::Route &reference_route) {
    std::vector<double> xs, ys;
    for (int i = 0; i < reference_route.route_points_size(); ++i) {
        xs.push_back(reference_route.route_points(i).x());
        ys.push_back(reference_route.route_points(i).y());
    }
    RouteSpline(xs, ys);
}

void ReferenceRoute::RouteSpline(const std::vector<double>& xs,
                                 const std::vector<double>& ys) {
    double path_length;
    Spline::fitCurve(xs, ys, &curve_x_, &curve_y_, &path_length);
    std::cout << "path length:" << path_length << std::endl;
}

double ReferenceRoute::GetCurvature(double s) {
    double dx = curve_x_.deriv1(s);
    double ddx = curve_x_.deriv2(s);
    double dy = curve_y_.deriv1(s);
    double ddy = curve_y_.deriv2(s);
    double curvature = fabs(dx * ddy - ddx * dy) / pow(dx*dx+dy*dy, 1.5);
    return curvature;
}

void ReferenceRoute::FromXYToSD(double x, double y,
                                double* s, double* d) const {
    Spline::getClosestPointOnCurve(
            curve_x_, curve_y_, x, y, s, d);

    double x0 = curve_x_(*s);
    double y0 = curve_y_(*s);
    double theta0 = theta(*s);

    double theta = atan2(y - y0, x - x0);
    double d_theta = theta - theta0;
    if (d_theta < -M_PI || (d_theta > 0 && d_theta < M_PI)) {
        *d = *d;
    } else {
        *d = -*d;
    }
}

void ReferenceRoute::FromSDToXY(double s, double d,
                                double *x, double *y) const {
    double x0 = curve_x_(s);
    double y0 = curve_y_(s);
    double theta0 = theta(s);

    *x = x0 - d * sin(theta0);
    *y = y0 + d * cos(theta0);

}

}
