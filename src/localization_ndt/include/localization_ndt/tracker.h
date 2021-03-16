//
// Created by eagleflag on 2020/8/31.
//

#ifndef CATKIN_WS_NDT_TRACKER_H
#define CATKIN_WS_NDT_TRACKER_H

#include "frame.h"
#include "object_det.h"
#include "Hungarian.h"

class tracker
{
public:
    std::vector<object_det::ptr> unmatched_detections_;
    std::vector<object_det::ptr> all_tracks_;
    std::vector<object_det::ptr> unmatched_tracks_;
//    std::pair<object_det::ptr, object_det::ptr> matched_pair_; // first track, second detection
    std::vector<std::vector<double>> costMatrix_;
    std::vector<int> assignment_;

public:
    tracker();
    ~tracker();
    void initialize(const std::vector<object_det::ptr>& ini_dets);
    void update(const std::vector<object_det::ptr>& new_dets_raw);
    inline double affinity_iou(const cv::Rect& track, const cv::Rect& det);
    double affinity_appearance(const cv::Mat& track_img, const cv::Mat& det_img);
    double compute_affinity(const object_det::ptr track, const object_det::ptr det);
    void draw_tracks(const frame::ptr frame);

private:
    const double IOU_THRESHOLD = 0.3;
    const double IOU_EXTREME = 10.0;
    const double APPEARANCE_THRESHOLD = 0.3;
    const double APPEARANCE = 10.0;

};

#endif //CATKIN_WS_NDT_TRACKER_H
