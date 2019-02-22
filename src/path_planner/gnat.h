//
//  gnat.h
//  rrt
//
//  Created by zhuoyang on 2018/10/24.
//  Copyright © 2018年 zhuoyang. All rights reserved.
//

//  Created by zhuoyang on 2018/10/22.
//  Copyright © 2018年 zhuoyang. All rights reserved.
//

#ifndef SRC_PLANNING_SRC_RRT_GNAT_H_
#define SRC_PLANNING_SRC_RRT_GNAT_H_

#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <functional>
#include <chrono>
#include <map>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "node.h"
#include "image_proc.h"

using namespace std;
using namespace cv;

namespace planning {
    namespace path_planner {
    class GNAT {
    public:
        GNAT(){}

        GNAT(int pivots_k, const cv::Size& size);

        vector<int> linspace(int num);

        void GetRange(int pivot, int region, int* min, int* max) const;

        int add(const Node& node);

        vector<vector<Point2f>> Voronoi();

        cv::Scalar color(int i);

        int FindPivot(const Node& node) const;

        struct Compare {
            Compare(Node sample) {this->sample = sample;}
            bool operator() (Node& a, Node& b) {
                int dist1 = (a.row() - sample.row()) * (a.row() - sample.row())  +
                (a.col() - sample.col()) * (a.col() - sample.col()) ;
                int dist2 = (b.row() - sample.row()) * (b.row() - sample.row())  +
                (b.col() - sample.col()) * (b.col() - sample.col()) ;
                return dist1 < dist2;
            }
            Node sample;
        };

        int SqureDistance(const Node& p1, const Node& p2) const;

        int SqureDistance(const cv::Point& p1, const Node& p2) const;

        vector<Node> kNearestPoints(const Node& sample, int k) const;

    private:
        vector<pair<int, int>> pivots_;
        cv::Size size_;
        vector<vector<Point2f> > facets_;
        vector<vector<pair<int, int>>> distance_range_table_; // min, max.

        vector<vector<Node>> nodes_;
        vector<cv::Scalar> colors_;

        int point_count_ = 0;

        std::map<int,int> pivot_region_map_;
        cv::Mat pivot_image_;

    };
    }
} // namespace

#endif // SRC_PLANNING_SRC_RRT_GNAT_H_
