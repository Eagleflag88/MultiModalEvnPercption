//
// Created by eagleflag on 2021/3/21.
//

#ifndef MULTIMODALEVNPERCPTION_CAM_FRAME_H
#define MULTIMODALEVNPERCPTION_CAM_FRAME_H

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "yolo_v2_class.hpp"
#include "object_det.h"
#include "NvInfer.h"
#include "NvInferRuntimeCommon.h"
#include "cuda_runtime.h"


class cam_frame
{

public:
    typedef std::shared_ptr<cam_frame> ptr;
    unsigned long id_;
    double time_stamp_;
    cv::Mat image_;
    std::vector<cv::KeyPoint> keypoints_;
    cv::Mat descriptors_;

public:
    cam_frame();
    cam_frame(unsigned long id,
          double time_stamp = 0.0,
          cv::Mat image = cv::Mat(),
          std::vector<cv::KeyPoint> keypoints = {},
          cv::Mat descriptors = {}); //constructor with default input, only id is not specified
    ~cam_frame();
    ptr create_frame();
    void detect_feature();
    std::vector<std::string> objects_names_from_file(std::string const filename);
    void bboxes_to_objects(const std::vector<bbox_t>& bboxes_2d, std::vector<object_det::ptr>& new_objects);
    void detect_yolo_dark(std::vector<object_det::ptr>& new_objects);
    void detect_yolo_opencv(std::vector<object_det::ptr>& new_objects);
    void detect_ssd(std::vector<object_det::ptr>& new_objects);
    size_t getSizeByDim(const nvinfer1::Dims& dims);
    void detect_yolo_trt(std::vector<object_det::ptr>& new_objects);

private:
    const std::string tensorflowConfigFile = "/home/eagleflag/Documents/AutoMirroAdjustment/models/opencv_face_detector.pbtxt";
    const std::string tensorflowWeightFile = "/home/eagleflag/Documents/AutoMirroAdjustment/models/opencv_face_detector_uint8.pb";

    const std::string SSDConfigFile = "/work/tools/MobilNet_SSD_opencv/MobileNetSSD_deploy.prototxt";
    const std::string SSDWeightFile = "/work/tools/MobilNet_SSD_opencv/MobileNetSSD_deploy.caffemodel";

    const std::string YoloConfigFile = "/work/catkin_ws_ndt/src/localization_ndt/model/darknet/yolov4-tiny.cfg";
    const std::string YoloWeightFile = "/work/catkin_ws_ndt/src/localization_ndt/model/darknet/yolov4-tiny.weights";
    const int NUM_CLASSES_YOLO = 80;
    const float CONFIDENCE_THRESHOLD_YOLO = 0.24;
    const float NMS_THRESHOLD_YOLO = 0.4;

    const std::string CocoNamesFile = "/work/tools/darknet-master/cfg/coco.names";

};

#endif //MULTIMODALEVNPERCPTION_CAM_FRAME_H
