//
// Created by eagleflag on 2020/8/31.
//

#include "tracker.h"
#include <numeric>
#include <opencv2/imgproc.hpp>


tracker::tracker()
{

}

tracker::~tracker()
{

}

void tracker::initialize(const std::vector<object_det::ptr>& ini_dets)
{
    unmatched_tracks_ = ini_dets;
    unmatched_detections_ = ini_dets;
    all_tracks_ = ini_dets;
    costMatrix_ = {};
}

inline double tracker::affinity_iou(const cv::Rect& track, const cv::Rect& det)
{
    //calculat the IoU
    cv::Rect or_rect = track | det;
    cv::Rect and_rect = track & det;
//    std::cout << "and area is " <<  and_rect.area() << std::endl;
//    std::cout << "or area is " <<  or_rect.area() << std::endl;
    double iou_metric = 1.0 - and_rect.area()*1.0/or_rect.area(); // because the hungarian algorithm try to minimize the cost-> 越小越好
    // Apply threshhold 0.7 to the  intersection area
    if (iou_metric  > 1.0 - IOU_THRESHOLD) //当交叉面积过小
        iou_metric = IOU_EXTREME; //metric置很大的值

    return iou_metric;
}

double tracker::affinity_appearance(const cv::Mat& track_img, const cv::Mat& det_img)
{
    // Scale the det_img to the size of track_img
    cv::Mat det_img_resized = cv::Mat();
    cv::Size track_img_size = cv::Size(track_img.cols, track_img.rows);
    cv::resize(det_img, det_img_resized, track_img_size, 0, 0, cv::INTER_AREA);

    // Calculate the metric -> 归一化的相关性系数(-1, +1), 越大越好
    cv::Mat match_result;
    cv::matchTemplate(track_img, det_img_resized, match_result, cv::TM_CCOEFF_NORMED);
    float match_metric = (match_result.at<float>(0, 0) + 1.0)*0.5; //(-1, +1) -> (0, 1) 越大越好
    double appearance_metric = 1.0 - match_metric; // (0, 1) 越小越好
    return appearance_metric;
}

double tracker::compute_affinity(const object_det::ptr track, const object_det::ptr det)
{
//    double appearance_metric = affinity_appearance(track->bbox_img_, det->bbox_img_);
    double appearance_metric = 0;
    double iou_metric = affinity_iou(track->rect_, det->rect_);
    return appearance_metric + iou_metric;
}

void tracker::update(const std::vector<object_det::ptr>& new_dets_raw)
{
    std::cout << "Start Tracker Update "<< std::endl;

    //step1-> Compute Affinity /////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Construct the matrix of all_tracks_.size() X new_dets_raw.size()
    std::vector<std::vector<double>> costMatrix_raw;

    std::cout << "size of the track is " << all_tracks_.size() << std::endl;
    std::cout << "size of the raw new det is " << new_dets_raw.size() << std::endl;
    for(int i = 0; i < all_tracks_.size(); i++) // all_tracks_.size->rows->each existing track, reference at hungarian maintest.cpp
    {
        std::vector<double> costRows;
        for(int j = 0; j < new_dets_raw.size(); j++) // col: each new detection
        {
            costRows.push_back(compute_affinity(all_tracks_[i], new_dets_raw[j]));
//            std::cout << "cost is " << costRows.back() << std::endl;
        }
//        std::cout << "new row" << std::endl;

        costMatrix_raw.push_back(costRows); // Get the cost of one row
        costRows.clear();
    }

    //Find the invalid column in the cost matrix, whose matching score is invalid (greater than threshold) for all dets in its column
    //低于Threshold的new det将不会被匹配算法考虑
    std::vector<bool> isvalid = {};
    double eps = 1e-6;
    for(int j = 0; j < new_dets_raw.size(); j++) //针对每一列
    {
        std::vector<double> cost_col = {};
        for(int i = 0; i < all_tracks_.size(); i++) // col consists of all new detections for one track
        {
            cost_col.push_back(costMatrix_raw[i][j]);
        }
        double sum_cost_col = std::accumulate(cost_col.begin(), cost_col.end(), 0.0);
        if(fabs(sum_cost_col - IOU_EXTREME*cost_col.size()) < eps) // 如果cost_col里面所有的值都等于IOU_EXTREME
        {
            isvalid.push_back(false);//匹配注定无效，剔除出匹配算法
        }
        else
        {
            isvalid.push_back(true);//匹配可能效，保留在匹配算法
        }
        cost_col.clear();
    }

    //从所有的原始new det中根据isvalid抽出用来匹配的new det和不用来匹配的new det
    std::vector<object_det::ptr> new_dets = {};
    std::vector<object_det::ptr> new_dets_invalid = {};
    for(int j = 0; j < new_dets_raw.size(); j++) //遍历raw new det的每一列，判断是否需要放入匹配算法
    {
        if(isvalid[j])
        {
            new_dets.push_back(new_dets_raw[j]);
        }
        else
        {
            new_dets_invalid.push_back(new_dets_raw[j]);
        }
    }
//    std::cout << "size of the new det is after cutting " << new_dets.size() << std::endl;

    // Construct the matrix of all_tracks_.size() X (new_dets_raw.size() - num_invalid_dets)
    // 为所有的现有track和valid的det计算cost矩阵
    costMatrix_.clear();
    for(int i = 0; i < all_tracks_.size(); i++) // all_tracks_.size->rows->each existing track, reference at hungarian maintest.cpp
    {
        std::vector<double> costRows;
        for(int j = 0; j < new_dets_raw.size(); j++) // col: each new detection
        {
            if(isvalid[j])
            {
                costRows.push_back(costMatrix_raw[i][j]);
//                std::cout << "cost is " << costRows.back() << std::endl;
            }

        }
//        std::cout << "new row" << std::endl;
        costMatrix_.push_back(costRows); // Get the cost of one row
        costRows.clear();
    }

    //step2-> Data Association //////////////////////////////////////////////////////////////////////////////////////////////////////
    HungarianAlgorithm HungAlgo; //
    double cost = HungAlgo.Solve(costMatrix_, assignment_);

    for (unsigned int x = 0; x < costMatrix_.size(); x++) //对每一个track
    {
        std::cout << x << "," << assignment_[x] << "\t"; //输出他的det的编号
        new_dets; //删除new dets里的相应元素,留下剩下的元素
    }
    std::cout << "\ncost: " << cost << std::endl;

    //step3 -> update the track list, det list

    // 更新track的状态，具体策略：
    // 遍历all_tracks
    for (int i = 0; i < all_tracks_.size(); i++)
    {
        if(assignment_[i] >= 0) //当发现被assign了新的detection，则更新coundown=2，并更新rect
        {
            int assigned_det_id = assignment_[i];
            all_tracks_[i]->rect_ = new_dets[assigned_det_id]->rect_;
            all_tracks_[i]->CountDown_ = 2;
        }
        else //当发现没有被匹配到新的detection时，则coundown--
        {
            all_tracks_[i]->CountDown_ = all_tracks_[i]->CountDown_ - 1;
        }
    }

    //删除长久没有detection的track，具体策略：
    // 遍历all_tracks, 当发现CountDown == 0 便剔除
    for (auto iter = all_tracks_.begin(); iter != all_tracks_.end();)
    {
        if((*iter)->CountDown_ <= 0)
        {
//            std::cout << "track with label id " << (*iter)->id_ << " is deleted" << std::endl;
            iter = all_tracks_.erase(iter);

        }
        else
        {
            iter++;
        }
    }
//    std::cout << "size of the track after deletion is " << all_tracks_.size() << std::endl;
    //增加新的潜在track，具体策略：
    //对每一个new detections，如果不在assignement里面，便加入all_tracks_
    for(int i = 0; i < new_dets.size(); i++)
    {
        //如果无法在assignement里面找到i这个编号
        auto iter = std::find(assignment_.begin(), assignment_.end(), i); //在assignment里面找i
        if(iter == assignment_.end())// 如果没找到，说明这个detections里面没有与现在的track匹配成功
        {
//            std::cout << "track with label id " << new_dets[i]->id_ << " is added" << std::endl;
            all_tracks_.push_back(new_dets[i]); //放入track里面，作为新的潜在track
        }
    }

    //每一个invalid new detections都加入all_tracks_，不然track的数量会迅速降至0
    for(int i = 0; i < new_dets_invalid.size(); i++)
    {
//        std::cout << "invalid new det with label id " << new_dets_invalid[i]->id_ << " is added" << std::endl;
        all_tracks_.push_back(new_dets_invalid[i]); //放入track里面，作为新的潜在track
    }
//    std::cout << "size of the track is after adding " << all_tracks_.size() << std::endl;

    std::cout << "Finish Tracker Update "<< std::endl;

}

void tracker::draw_tracks(const frame::ptr frame_ptr)
{
    std::cout << "Start Draw Track "<< std::endl;
    cv::Mat img_track = frame_ptr->image_;
    for(int i = 0; i < all_tracks_.size(); i++)
    {
        cv::rectangle(img_track, all_tracks_[i]->rect_, cv::Scalar(123, 1, 122), 2);
        std::string id_label = std::to_string(all_tracks_[i]->id_);
        int font_face = cv::FONT_HERSHEY_COMPLEX;
        double font_scale = 1;
        int thickness = 2;
        cv::Point origin;
        origin.x = all_tracks_[i]->rect_.x + (all_tracks_[i]->rect_.width)*0.5;
        origin.y = all_tracks_[i]->rect_.y + 1;
        cv::putText(img_track, id_label, origin, font_face, font_scale, cv::Scalar(0, 255, 255), thickness, 8, 0);
    }

    // Store the images ////////////////////////////////////////////////////////////////////////////////
    std::string image_file1("/work/catkin_ws_ndt/src/localization_ndt/img/img_frame_with_track_");
    std::stringstream str_stream;
    str_stream << frame_ptr->id_;
    std::string image_file2 = str_stream.str();
    std::string image_file3(".png");
    std::string image_file = image_file1 + image_file2 + image_file3;
    cv::imwrite(image_file, img_track);
    std::cout << "Finish Draw Track "<< std::endl;
}
