//
// Created by eagleflag on 2021/3/21.
//

#include "cam_frame.h"
#include <iostream>

cam_frame::cam_frame()
{
    std::cout << "camera frame without id " << "is constructed" << std::endl;
}

cam_frame::cam_frame(
        unsigned long id,
        double time_stamp,
        cv::Mat image
):
        id_(id),
        time_stamp_(time_stamp),
        image_(image)// Implementation of the constructor
{
    std::cout << "camera frame with id " << id_ << " is constructed" << std::endl;
}

cam_frame::~cam_frame()
{
    std::cout << "camera frame with id " << id_ << " is destructed" << std::endl;
}
cam_frame::ptr cam_frame::create_frame()
{
    static unsigned long cam_frame_id = 0;
    return cam_frame::ptr( new cam_frame(cam_frame_id++) ); // ptr here is the shared pointer defined with typedef
}

//void cam_frame::detect_yolo_dark(std::vector<object_det::ptr>& new_objects)
//{
//    std::cout << "Detection by Dark Started" << std::endl;
//    std::string  names_file = CocoNamesFile;
//    std::string  cfg_file = YoloConfigFile;
//    std::string  weights_file = YoloWeightFile;
//    float const thresh = 0.2;
//    auto obj_names = objects_names_from_file(names_file);
//
//    Detector detector(cfg_file, weights_file);
//    std::vector<bbox_t> result_vec;
//    result_vec = detector.detect(image_, thresh, true); //image_src使用原始图片（不需要用resize过的），result也会调整回原图的比例
////    result_vec = detector.detect(image_); //image_src使用原始图片（不需要用resize过的），result也会调整回原图的比例
//    if(result_vec.empty())
//        std::cout << "Nothing is out" << std::endl;
//
//    bboxes_to_objects(result_vec, new_objects);
//
////    cv::Mat draw_frame = image_.clone();
////    draw_boxes(draw_frame, result_vec, obj_names, 1, 1);
//
//    std::cout << "Detection by Dark Finished" << std::endl;
//}