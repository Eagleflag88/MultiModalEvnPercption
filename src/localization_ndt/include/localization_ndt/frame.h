//
// Created by eagleflag on 2020/8/17.
//

#ifndef CATKIN_WS_NDT_FRAME_H
#define CATKIN_WS_NDT_FRAME_H

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Image.h>
#include "yolo_v2_class.hpp"
#include "object_det.h"
#include "NvInfer.h"
#include "NvInferRuntimeCommon.h"
#include "cuda_runtime.h"
#include "mappoint.h"


class map; // forward declaration：因为需要在不互相include头文件的情况下使用map中的数据类型
class frame
{

public:
    typedef std::shared_ptr<frame> ptr;
    unsigned long id_;
    double time_stamp_;
    Eigen::Isometry3d T_cw_;
    cv::Mat image_;
    std::vector<cv::Point3d> landmark_points_;
    std::vector<cv::KeyPoint> keypoints_;
    std::vector<cv::Point2f> kps2f_;
    cv::Mat descriptors_;
    bool iskeyframe_;
    unsigned int source_;
    std::vector<mappoint::ptr> mappoints_;

public:
    frame();
    frame(unsigned long id,
             double time_stamp = 0.0,
             Eigen::Isometry3d T_cw = Eigen::Isometry3d::Identity(),
             cv::Mat image = cv::Mat(),
             std::vector<cv::Point3d> landmark_points = {},
             std::vector<cv::KeyPoint> keypoints = {},
             std::vector<cv::Point2f> kps2f = {},
             cv::Mat descriptors = {},
             bool iskeyframe = false,
             unsigned int source = 0,
             std::vector<mappoint::ptr> mappoints = {}); //constructor with default input, only id is not specified
    ~frame();
    ptr create_frame();
    void checkifkeyframe(const map* map_ptr); //头文件互相引用的情况下只能采用指针的形式引用
    void transform_landmark(const cv::Mat R, const cv::Mat t);
    void crop_frame(const std::vector<unsigned char> status);
    void draw_and_store_tracking();
    void pub_odom(ros::Publisher odom_pub);
    void detect_tf();
    void detect_ssd();
    void detect_yolo_opencv();
    void detect_yolo_dark(std::vector<object_det::ptr>& new_objects);
    void detect_yolo_trt(std::vector<object_det::ptr>& new_objects);
    void draw_boxes(const cv::Mat& mat_img, const std::vector<bbox_t>& result_vec, const std::vector<std::string>& obj_names,
                           const int current_det_fps, const int current_cap_fps);
    cv::Scalar obj_id_to_color(int obj_id);
    std::vector<std::string> objects_names_from_file(std::string const filename);
    void bboxes_to_objects(const std::vector<bbox_t>& bboxes_2d, std::vector<object_det::ptr>& new_objects);
    size_t getSizeByDim(const nvinfer1::Dims& dims);

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



#endif //CATKIN_WS_NDT_FRAME_H
