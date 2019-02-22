#ifndef PLANNING_SRC_ROUTE_H_
#define PLANNING_SRC_ROUTE_H_

#include <iostream>
#include "../common/spline.h"
#include "../common/file_config.h"
#include "../common/string.h"
#include "speed_profile_conf.pb.h"

namespace planning {
    namespace speed_profile {

        class ReferencePath {
        public:
            ReferencePath() = default;

            ReferencePath(const SpeedProfileConf& speed_profile_conf);

            Spline get_x() { return curve_x_; }

            Spline get_y() { return curve_y_; }

            double GetCurvature(double s);

            double x(double s) { return curve_x_(s); }

            double y(double s) { return curve_y_(s); }

            double theta(double s) { return atan2(curve_x_.deriv1(s), curve_y_.deriv1(s)); }

        private:

            void GetGeometryPath();

            std::string planning_path_;
            std::string road_file_;

            Spline curve_x_;
            Spline curve_y_;

        };
    }
} // namespace planning

#endif // PLANNING_SRC_ROUTE_H_
