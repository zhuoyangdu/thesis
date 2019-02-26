#include "reference_path.h"

using namespace std;

namespace planning {

ReferencePath::ReferencePath(const std::string& road_file)
    : road_file_(road_file) {
    GetGeometryPath();
}

ReferencePath::ReferencePath(
        const std::vector<double> path_x,
        const std::vector<double> path_y) {
    GetGeometryPath(path_x, path_y);
}

void ReferencePath::GetGeometryPath(
        const std::vector<double> xs,
        const std::vector<double> ys) {
    double path_length;
    path_x_ = xs;
    path_y_ = ys;
    utils::Spline::fitCurve(xs, ys, &curve_x_, &curve_y_, &path_length);
}

void ReferencePath::GetGeometryPath() {
    std::vector<double> xs, ys;
    std::string line;
    std::ifstream file(road_file_);
    if (file.is_open()) {
        int i;
        while (getline(file, line)) {
            std::vector<std::string> ps;
            utils::StringUtils string_utils;
            string_utils.SplitString(line, "\t", &ps);
            double x, y;
            x = atof(ps[0].c_str());
            y = atof(ps[1].c_str());
             //x = atof(ps[1].c_str());
             // y = 511 -  atof(ps[0].c_str());
            xs.push_back(x);
            ys.push_back(y);
        }
        file.close();
    } else {
        std::cout << "cannot open path config file." << std::endl;
    }
    path_x_ = xs;
    path_y_ = ys;
    double path_length;
    utils::Spline::fitCurve(xs, ys, &curve_x_, &curve_y_, &path_length);
}

double ReferencePath::GetCurvature(double s) {
    double dx = curve_x_.deriv1(s);
    double ddx = curve_x_.deriv2(s);
    double dy = curve_y_.deriv1(s);
    double ddy = curve_y_.deriv2(s);
    double curvature = fabs(dx * ddy - ddx * dy) / pow(dx * dx + dy * dy, 1.5);
    return curvature;
}

}
