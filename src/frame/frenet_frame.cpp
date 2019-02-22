//
// Created by zhuoyang on 2018-12-29.
//

#include "frenet_frame.h"

namespace planning {

FrenetFrame::FrenetFrame(const planning::ReferenceRoute &reference_route,
                         const planning::VehicleState &vehicle_state,
                         const StaticObstacles &obstacles,
                         const FrenetFrameConf& frenet_frame_conf)
     : reference_route_(reference_route),
       vehicle_state_(vehicle_state),
       obstacles_(obstacles.obstacles()),
       frenet_frame_conf_(frenet_frame_conf) {

    d0_ = frenet_frame_conf_.lane_width() * frenet_frame_conf_.min_lane();
    s0_ = vehicle_state_.s();

    DrawSDMap();
}

void FrenetFrame::DrawSDMap() {
    std::cout << "frenet frame conf : dd: " << frenet_frame_conf_.dd() << ","
        << "ds:" << frenet_frame_conf_.ds()
        << "," << "row:" << frenet_frame_conf_.image_row()
        << ", col:" << frenet_frame_conf_.image_col() << std::endl;
    std::cout << "range d:" << frenet_frame_conf_.lane_width() * frenet_frame_conf_.min_lane() << ","
        << frenet_frame_conf_.lane_width() * frenet_frame_conf_.max_lane() << std::endl;


    cv::Mat img_frenet = cv::Mat::ones(frenet_frame_conf_.image_row(),
                                        frenet_frame_conf_.image_col(),
                                        CV_8UC1) * 512;

    // Get Obstacle space.
    vector<vector<cv::Point>> obstacle_polygons;
    for (int i = 0; i < obstacles_.size(); ++i) {
        vector<double> ox, oy;
        obstacles_[i].GetVertexes(&ox, &oy);
        ox.push_back(ox[0]);
        oy.push_back(oy[0]);
        vector<double> edge_x, edge_y;
        vector<double> edge_s, edge_d;
        vector<cv::Point> obstacle_poly;
        for (int j = 0; j < ox.size()-1; ++j) {
            // cout << "v1: " << ox[j] << "," << oy[j] << endl;
            // cout << "v2: " << ox[j+1] << "," << oy[j+1] << endl;
            double edge_l = sqrt((ox[j+1] - ox[j]) * (ox[j+1] - ox[j])
                                 + (oy[j+1] - oy[j]) * (oy[j+1] - oy[j]));
            double t = 0;
            while (t < edge_l) {
                double tx = t / edge_l * ox[j+1] + (1 - t / edge_l) * ox[j];
                double ty = t / edge_l * oy[j+1] + (1 - t / edge_l) * oy[j];
                edge_x.push_back(tx);
                edge_y.push_back(ty);
                t = t + 0.2;

                double es, ed;
                reference_route_.FromXYToSD(tx, ty, &es, &ed);
                cv::Point ep = FromFrenetToImage(es, ed);
                obstacle_poly.push_back(ep);
            }
        }
        obstacle_polygons.push_back(obstacle_poly);
    }
    obstacle_polygons_ = obstacle_polygons;

    path_planner::ImageProc::DrawPoly(img_frenet, obstacle_polygons, cv::Scalar(0));
    image_frenet_ = img_frenet;

}


}
