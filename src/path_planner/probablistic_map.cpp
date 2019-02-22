//
//  probablistic_map.cpp
//  rrt
//
//  Created by zhuoyang on 2018/10/24.
//  Copyright © 2018年 zhuoyang. All rights reserved.
//

#include "probablistic_map.h"
#include <random>
#include "../utils/timer.h"

using namespace std;

namespace planning {
    namespace  path_planner {
   ProbablisticMap::ProbablisticMap(const cv::Mat& attractive_prob) {
        attractive_prob_ = attractive_prob.clone();
        InitMap();
   }

    void ProbablisticMap::InitMap() {
        vector<int> prob_vector;
        prob_vector.reserve(attractive_prob_.rows * attractive_prob_.cols);
        for (int i = 0; i < attractive_prob_.rows; ++i) {
            for (int j = 0; j < attractive_prob_.cols; ++j) {
                prob_vector.push_back(attractive_prob_.at<uchar>(i, j));
            }
        }
        vector<int> prob_cumsum;
        prob_cumsum.reserve(prob_vector.size());
        int sum = 0;
        for (int prob : prob_vector) {
            sum += prob;
            prob_cumsum.push_back(sum);
        }
        prob_sum_ = sum;
        prob_cumsum_ = prob_cumsum;
    }

    Node ProbablisticMap::Sampling() const {
        int rand_sample = int((double) rand() / RAND_MAX * prob_sum_);
        if (rand_sample <= prob_cumsum_[0]) return Node(0, 0);
        int index = FindRandSection(rand_sample, 0, prob_cumsum_.size()-1);

        int row = (index) / attractive_prob_.cols;
        int col = (index) - row * attractive_prob_.cols;
        if (attractive_prob_.at<uchar>(row, col) == 0) {
            return Sampling();
        }
        return Node(row, col);
    }


    int ProbablisticMap::FindRandSection(int num, int lower, int upper) const {
        if (lower == upper) return upper;
        if (lower + 1 == upper) return upper;
        int mid = int((lower + upper) / 2);
        if (num <= prob_cumsum_[mid]) {
            return FindRandSection(num, lower, mid);
        } else {
            return FindRandSection(num, mid+1, upper);
        }
    }

    }
}

