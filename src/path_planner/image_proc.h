//
//  image_proc.hpp
//  rrt
//
//  Created by zhuoyang on 2018/10/24.
//  Copyright © 2018年 zhuoyang. All rights reserved.
//

#ifndef SRC_PLANNING_SRC_COMMON_IMAGE_PROC_H_
#define SRC_PLANNING_SRC_COMMON_IMAGE_PROC_H_

#include <iostream>
#include <string>
#include <cstdlib>
#include <vector>
#include <set>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "node.h"
#include "../utils/timer.h"
#include "../common/reference_route.h"
#include "path_planner_conf.pb.h"

using namespace cv;
using namespace std;

namespace planning {
    namespace  path_planner {
    class ImageProc {
    public:
        ImageProc() = delete;

        static std::vector<Point> GetVertex(const cv::Mat& image);

        static cv::Mat GetVoronoiProbMap(const cv::Mat& image);

        static cv::Mat GetTargetAttractiveMap(
                                              const cv::Mat& image, const cv::Point& goal);

        static cv::Mat GetAttractiveProbMap(
                                            const cv::Mat& image, const cv::Point& goal,
                                            double k_voronoi, double k_goal);

        static void GetAttractiveProbMap(
                                         const cv::Mat& image, const cv::Point& goal,
                                         double k_voronoi, double k_goal,
                                         cv::Mat* goal_prob_map,
                                         cv::Mat* voronoi_prob_map,
                                         cv::Mat* attractive_prob_map);

        static void GetObstacleRepulsiveField(const cv::Mat& image,
                                              cv::Mat* repulsive_filed_x,
                                              cv::Mat* repulsive_filed_y);

        static void PlotPoint(const cv::Mat& image,
                              const Point& point,
                              const cv::Scalar& scalar,
                              double thickness);

        static void PlotPoint(const cv::Mat& image,
                              const Node& node,
                              const cv::Scalar& scalar,
                              double thickness);

        static void PlotLine(const cv::Mat& image,
                             const Node& a,
                             const Node& b,
                             const cv::Scalar& scalar,
                             double thickness);

        static void PlotPath(const cv::Mat& image,
                             const std::vector<Node> path,
                             const cv::Scalar& scalar,
                             double thickness);

        static void PlotPath(const cv::Mat& image,
                             const std::vector<double> x,
                             const std::vector<double> y,
                             const cv::Scalar& scalar,
                             double thickness);

        static void DrawPoly(const cv::Mat& image,
                             const vector<vector<cv::Point>>& polygons,
                             const cv::Scalar& scalar);

        static void DrawDelaunay(Mat& img, Subdiv2D& subdiv, Scalar delaunay_color);

        //Draw voronoi diagram
        static void DrawVoronoi(Mat* img, Subdiv2D& subdiv )
        {
            vector<vector<Point2f> > facets;
            vector<Point2f> centers;
            subdiv.getVoronoiFacetList(vector<int>(), facets, centers);

            vector<Point> ifacet;
            vector<vector<Point> > ifacets(1);

            for( size_t i = 0; i < facets.size(); i++ )
            {
                ifacet.resize(facets[i].size());
                for( size_t j = 0; j < facets[i].size(); j++ )
                    ifacet[j] = facets[i][j];

                Scalar color(i);
                fillConvexPoly(*img, ifacet, color, 8, 0);
            }
        }

        static std::vector<Point> PaintVertex(const cv::Mat &origin_image,
                                              cv::Mat* image);

        static void RestrictImagePoint(Point2f *p) {
            if (p->x < 0) p->x = 0;
            if (p->x > 511) p->x = 511;
            if (p->y < 0) p->y = 0;
            if (p->y > 511) p->y = 511;

        }
    };
    }
}  // namespace planning
#endif  // SRC_PLANNING_SRC_COMMON_IMAGE_PROC_H_
