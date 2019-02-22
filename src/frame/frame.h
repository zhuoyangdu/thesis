//
// Created by zhuoyang on 2019-02-20.
//

#ifndef THESIS_FRAME_H
#define THESIS_FRAME_H

#include "planning_conf.pb.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace planning {
class Frame {
    public:
        Frame() = default;

        virtual void Init(const PlanningConf& planning_conf) = 0;

        virtual cv::Point PixelGoal() const = 0;

        virtual cv::Point PixelVehicleState() const = 0;

        virtual void GetSearchingGridMap(
                    cv::Mat* image,
                    SearchingImageBorder* image_border) const = 0;

        virtual void FromImageToGlobalCoord(double row, double col,
                                            double*x, double* y) const = 0;
};
}
#endif //THESIS_FRAME_H
