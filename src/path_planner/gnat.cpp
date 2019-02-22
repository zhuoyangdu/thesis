//
//  gnat.cpp
//  rrt
//
//  Created by zhuoyang on 2018/11/13.
//  Copyright © 2018年 zhuoyang. All rights reserved.
//

#include "gnat.h"
using namespace std;
using namespace cv;

namespace planning {
    namespace  path_planner {
    GNAT::GNAT(int pivots_k, const cv::Size& size) : size_(size) {
        vector<int> px = linspace(pivots_k);
        vector<int> py = px;
        pivots_.clear();
        pivots_.reserve(px.size() * py.size());
        for (int i = 0; i < px.size(); ++i) {
            for (int j = 0; j < py.size(); ++j) {
                pivots_.push_back(pair<int,int>(px[i], py[j]));
            }
        }

        cv::Mat image(size, CV_8UC1,Scalar(255));
        cv::Rect rect(0, 0, size_.width, size_.height);
        cv::Subdiv2D subdiv(rect);
        for (pair<int, int> point : pivots_) {
            subdiv.insert(cv::Point(point.first, point.second));
        }
        // ImageProc::DrawDelaunay(image, subdiv, Scalar(100));
        ImageProc::DrawVoronoi(&image, subdiv);

        vector<Point2f> centers;
        subdiv.getVoronoiFacetList(vector<int>(), facets_, centers);


        for (int i = 0; i < centers.size(); ++i) {
            // ImageProc::PlotPoint(image, centers[i], Scalar(0), 2);
            vector<pair<int, int>> range;
            for (int j = 0; j < centers.size(); ++j) {
                if (i != j) {
                    int min = INT_MAX;
                    int max = INT_MIN;
                    for (Point2f p : facets_[j]) {
                        int dist = (p.x - centers[i].x)*(p.x - centers[i].x) +
                        (p.y - centers[i].y)*(p.y - centers[i].y);
                        if (dist > max) max = dist;
                        if (dist < min) min = dist;
                    }
                    range.push_back(pair<int, int>(min, max));
                } else {
                    range.push_back(pair<int, int>(INT_MIN, INT_MAX));
                }
            }
            distance_range_table_.push_back(range);
        }

        for (int i = 0; i < pivots_.size(); ++i) {
            cv::Scalar color(int((double)rand()/RAND_MAX * 255),
                             int((double)rand()/RAND_MAX * 255),
                             int((double)rand()/RAND_MAX * 255));
            colors_.push_back(color);
        }

        for (int i = 0; i < pivots_.size(); ++i) {
            nodes_.push_back({});
        }


        // cout << "pivots size:" << pivots_.size() << endl;
        for(int i = 0; i < pivots_.size(); ++i) {
            int pixel = image.at<uchar>(pivots_[i].first, pivots_[i].second);
         //    cout << "pixel:" << pivots_[i].first << "," << pivots_[i].second <<
         //        "," << pixel << endl;
            pivot_region_map_.insert(pair<int ,int>(pixel, i));
            // pivot_region_map_[pixel] = i;
        }
        cout << pivot_region_map_.size() << endl;
        pivot_image_ = image;
        //imshow("contour", image);
        //cv::waitKey(1);
    }

    vector<int> GNAT::linspace(int num) {
        vector<int> lins;
        int dist = 512/num/2;

        while (dist < 512) {
            lins.push_back(dist);
            dist += 512/num;
        }
        return lins;
    }

    void GNAT::GetRange(int pivot, int region, int* min, int* max) const {
        if (pivot == region) {
            std::cout << "You cant use the same pivot and region!!!!!" << std::endl;
        }
        *min = distance_range_table_[pivot][region].first;
        *max = distance_range_table_[pivot][region].second;
    }

    int GNAT::add(const Node& node) {
        int min_pivots = -1;
        int min_dist = INT_MAX;
        for (int i = 0; i < pivots_.size(); ++i) {
            pair<int, int> pivot = pivots_[i];
            int dist = (node.row() - pivot.first)*(node.row() - pivot.first) +
                       (node.col() - pivot.second)*(node.col() - pivot.second);
            if (dist < min_dist) {
                min_dist = dist;
                min_pivots = i;
            }
        }
        nodes_[min_pivots].push_back(node);
        point_count_++;
        return min_pivots;
    }

    vector<vector<Point2f>> GNAT::Voronoi() {
        return facets_;
    }

    cv::Scalar GNAT::color(int i) {
        return colors_[i];
    }

    int GNAT::FindPivot(const Node& node) const {
        int pixel = pivot_image_.at<uchar>(node.row(), node.col());
        //if (pixel ==255) pixel = 254;
        // cout << "picel" << pixel << endl;
        //cout << "size:" << pivots_.size() << endl;
        auto iter = pivot_region_map_.find(pixel);
        if (iter != pivot_region_map_.end()) {
            return (*iter).second;
        } else {
            std::cout << "node.row:" << node.row() << ", " << node.col() << endl;
            std::cout << "pixel:" << pixel << endl;
            return  0;
        }
    }


    int GNAT::SqureDistance(const Node& p1, const Node& p2) const {
        return (p1.row() - p2.row()) * (p1.row() - p2.row()) +
        (p1.col() - p2.col()) * (p1.col() - p2.col());
    }

    int GNAT::SqureDistance(const cv::Point& p1, const Node& p2) const {
        return (p1.x - p2.row()) * (p1.x - p2.row()) +
        (p1.y - p2.col()) * (p1.y - p2.col());
    }

    vector<Node> GNAT::kNearestPoints(const Node& sample, int k) const {
        int pivot_index = FindPivot(sample);
        while (point_count_ == 0) {
            return vector<Node>{};
        }
        while (nodes_[pivot_index].size() == 0) {
            pivot_index = int((double)rand()/RAND_MAX * pivots_.size());
        }
        Compare cmp(sample);
        std::priority_queue<Node, vector<Node>, decltype(cmp)> candidates(cmp);
        for (Node point : nodes_[pivot_index]) {
            candidates.push(point);
        }
        while (candidates.size() > k) {
            candidates.pop();
        }
        double threshold;
        if (candidates.size() == k) {
            threshold = sqrt(SqureDistance(candidates.top(), sample));
        } else {
            threshold = INT_MAX;
        }

        cv::Point pivot_p(pivots_[pivot_index].first, pivots_[pivot_index].second);
        double d1 = sqrt(SqureDistance(pivot_p, sample));
        threshold = (d1 + threshold) * (d1 + threshold);

        int max, min;
        for (int i = 0; i < pivots_.size(); ++i) {
            if (i == pivot_index || nodes_[i].size() == 0) {
                continue;
            }
            GetRange(pivot_index, i, &min, &max);
            if (min > threshold) {
                continue;
            } else {
                for (Node node : nodes_[i]) {
                    if (SqureDistance(node, sample) < threshold) {
                        candidates.push(node);
                        if (candidates.size() > k) {
                            candidates.pop();
                        }
                        double t1 = sqrt(SqureDistance(candidates.top(), sample));
                        threshold = (t1 + d1) * (d1 + t1);
                    }
                }
            }
        }

        vector<Node> k_nearest_points;
        while (!candidates.empty()) {
            k_nearest_points.push_back(candidates.top());
            candidates.pop();
        }

        return k_nearest_points;
    }
    }
}
