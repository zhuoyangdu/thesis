// Copyright [2018] <Zhuoyang Du>

#ifndef SRC_PLANNING_SRC_RRT_NODE_H_
#define SRC_PLANNING_SRC_RRT_NODE_H_

#include <vector>
#include <cmath>
#include <iostream>

namespace planning {
    namespace path_planner {

    class Node {
    public:
        Node() {}

        Node(int row, int col, double theta) {
            row_ = row;
            col_ = col;
            theta_ = theta;
        }

        Node(int row, int col) {
            row_ = row;
            col_ = col;
        }

        void SetTheta(double theta) { theta_ = theta; }

        void SetRow(double row) { row_ = row; }

        void SetCol(double col) { col_ = col; }

        void SetIndex(int index) { index_ = index; }

        void SetParent(int index) { parent_index_ = index; }

        int row() const { return row_; }

        int col() const { return col_; }

        double theta() const { return theta_; }

        int index() const { return index_; }

        int parent_index() const { return parent_index_; }

        void print() const { std::cout << "row:" << row_ << ", col:" << col_ << std::endl;}
        static double SquareDistance(const Node& a, const Node& b) {
            return (a.col() - b.col()) * (a.col() - b.col())
            + (a.row() - b.row()) * (a.row() - b.row());
        }

        // 沿x轴， 顺时针为正
        static double GetDeltaTheta(const Node& parent_parent,
                                    const Node& parent,
                                    const Node& child) {
            int v1x = parent_parent.row() - parent.row();
            int v1y = parent_parent.col() - parent.col();
            int v2x = child.row() - parent.row();
            int v2y = child.col() - child.col();
            double cos_theta = (v1x * v2x + v1y * v2y) /
            ((sqrt(v1x*v1x+v1y*v1y)) * (sqrt(v2x*v2x+v2y*v2y)));

            return acos(cos_theta);
        }

        static double ComputeTheta(const Node& parent, const Node& child) {
            return atan2(child.row() - parent.row(), child.col() - parent.col());
        }

    private:
        int row_;
        int col_;
        double theta_;
        int index_;
        int parent_index_;
        std::vector<Node *> children_;

        /// number of child nodes.
        unsigned int degree_;
    };

    }  // namespace path_plan
}  // namespace planning

#endif  // SRC_PLANNING_SRC_RRT_NODE_H_

