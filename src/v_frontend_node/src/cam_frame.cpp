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



void cam_frame::bboxes_to_objects(const std::vector<bbox_t>& bboxes_2d, std::vector<object_det::ptr>& new_objects)
{
    if(!bboxes_2d.empty())
    {
        for(int i = 0; i < bboxes_2d.size(); i++)
        {
            object_det::ptr new_obj_ptr = new_obj_ptr->create_object();
            bbox_t bbox = bboxes_2d[i];
            if ((bbox.y + bbox.h) > image_.rows)
            {
                int y_delta = (bbox.y + bbox.h) - image_.rows;
                bbox.h = bbox.h - y_delta;
            }
            if ((bbox.x + bbox.w) > image_.cols)
            {
                int x_delta = (bbox.x + bbox.w) - image_.cols;
                bbox.w = bbox.w - x_delta;
            }
            new_obj_ptr->rect_ = cv::Rect(bbox.x, bbox.y, bbox.w, bbox.h);
//            std::cout << "size of the image is " << "cols: " << image_.cols << " rows: " << image_.rows << std::endl;
//            std::cout << "size of the bbox is " << "x: " << bbox.x << " y: " << bbox.y << " w: " << bbox.w << " h: " << bbox.h << std::endl;
            new_obj_ptr->bbox_img_ = image_(new_obj_ptr->rect_);
            new_objects.push_back(new_obj_ptr);
//            cv::imshow("BBox Image", new_obj_ptr->bbox_img_);
//            cv::waitKey(0);

        }
    }
}

std::vector<std::string> cam_frame::objects_names_from_file(std::string const filename) {
    std::ifstream file(filename);
    std::vector<std::string> file_lines;
    if (!file.is_open()) return file_lines;
    for(std::string line; getline(file, line);) file_lines.push_back(line);
    std::cout << "object names loaded \n";
    return file_lines;
}

void cam_frame::detect_yolo_dark(std::vector<object_det::ptr>& new_objects)
{
    std::cout << "Detection by Dark Started" << std::endl;
    std::string  names_file = CocoNamesFile;
    std::string  cfg_file = YoloConfigFile;
    std::string  weights_file = YoloWeightFile;
    float const thresh = 0.2;
    auto obj_names = objects_names_from_file(names_file);

    Detector detector(cfg_file, weights_file);
    std::vector<bbox_t> result_vec;
    result_vec = detector.detect(image_, thresh, true); //image_src使用原始图片（不需要用resize过的），result也会调整回原图的比例
//    result_vec = detector.detect(image_); //image_src使用原始图片（不需要用resize过的），result也会调整回原图的比例
    if(result_vec.empty())
        std::cout << "Nothing is out" << std::endl;

    bboxes_to_objects(result_vec, new_objects);

//    cv::Mat draw_frame = image_.clone();
//    draw_boxes(draw_frame, result_vec, obj_names, 1, 1);

    std::cout << "Detection by Dark Finished" << std::endl;
}