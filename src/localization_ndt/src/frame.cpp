//
// Created by eagleflag on 2020/8/18.
//

#include "frame.h"
#include <iostream>
#include "map.h"
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/all_layers.hpp>
#include "yolo_v2_class.hpp"
#include "object_det.h"


#include <memory>
#include <vector>
#include <deque>
#include <algorithm>
#include <chrono>
#include <string>
#include <sstream>
#include <iostream>
#include <cmath>
#include <fstream>

#define CUDA_CHECK(status); \
    do\
    {\
        auto ret = (status);\
        if (ret != 0)\
        {\
            std::cerr << "Cuda failure: " << ret << std::endl;\
            abort();\
        }\
    } while (0)


class Logger : public nvinfer1::ILogger
{
    void log(nvinfer1::ILogger::Severity severity, const char* msg) override
    {
        // suppress info-level messages
        if (severity != nvinfer1::ILogger::Severity::kINFO)
            std::cout << msg << std::endl;
    }
} gLogger;

frame::frame()
{
    std::cout << "image frame without id " << "is constructed" << std::endl;
}

frame::frame(
        unsigned long id,
        double time_stamp,
        Eigen::Isometry3d T_cw,
        cv::Mat image,
        std::vector<cv::Point3d> landmark_points,
        std::vector<cv::KeyPoint> keypoints,
        std::vector<cv::Point2f> kps2f,
        cv::Mat descriptors,
        bool iskeyframe,
        unsigned int source,
        std::vector<mappoint::ptr> mappoints
        ):
        id_(id),
        time_stamp_(time_stamp),
        T_cw_(T_cw),
        image_(image),
        landmark_points_(landmark_points),
        keypoints_(keypoints),
        kps2f_(kps2f),
        descriptors_(descriptors),
        iskeyframe_(iskeyframe),
        source_(source),
        mappoints_(mappoints)// Implementation of the constructor
{
    std::cout << "image frame with id " << id_ << " is constructed" << std::endl;
}

frame::~frame()
{
    std::cout << "image frame with id " << id_ << " is destructed" << std::endl;
}
frame::ptr frame::create_frame()
{
    static unsigned long img_frame_id = 0;
    return frame::ptr( new frame(img_frame_id++) ); // ptr here is the shared pointer defined with typedef
}

void frame::checkifkeyframe(const map* map_ptr)
{
    std::cout << "size of the map is " << map_ptr->keyframes_.size() << std::endl;
}

void frame::transform_landmark(
        const cv::Mat R,
        const cv::Mat t)
{
    // tranform the landmark point into the frame of the second camera, which will be used as reference frame for landmark points in next iteration
    std::cout << "3D Points Transformation Started "<< std::endl;
    std::vector<cv::Point3d>::iterator iter_landmark;
    cv::Point3d point_landmark;
    for(iter_landmark = std::begin(landmark_points_); iter_landmark != std::end(landmark_points_); ++iter_landmark)
    {
        point_landmark = *iter_landmark;
//        std::cout << "Landmark point before transformation is " << point_landmark << std::endl;
        cv::Mat pt_trans = R.inv()*((cv::Mat_<double>(3,1) << point_landmark.x, point_landmark.y, point_landmark.z) - t);
        point_landmark.x = pt_trans.at<double>(0, 0);
        point_landmark.y = pt_trans.at<double>(0, 1);
        point_landmark.z = pt_trans.at<double>(0, 2);
//        std::cout << "Landmark point after transformation is " << point_landmark << std::endl;
        *iter_landmark = point_landmark; //push back to the points_landmark
    }
    std::cout << "3D Points Transformation Finished "<< std::endl;
}

void frame::crop_frame(const std::vector<unsigned char> status)
{
    std::cout << "Size of the keypoints, 2f, landmark points, descriptor before crop are "
              << keypoints_.size()
              << " " << kps2f_.size()
              << " " << landmark_points_.size()
              << " " << descriptors_.rows
              << std::endl;

//    std::vector <cv::KeyPoint> keypoints_tmp = keypoints_;
//    int i = 0;
//    for (auto iter = keypoints_.begin(); iter != keypoints_.end(); i++) {
//        if (status[i] == 0) {
//            iter = keypoints_tmp.erase(iter);
//            continue;
//        }
//        iter++;
//    }
//    keypoints_ = keypoints_tmp;

    std::vector <cv::Point2f> kps2f_tmp = kps2f_;
    int i = 0;
    for (auto iter = kps2f_.begin(); iter != kps2f_.end(); i++) {
        if (status[i] == 0) {
            iter = kps2f_tmp.erase(iter);
            continue;
        }
        iter++;
    }
    kps2f_ = kps2f_tmp;

    std::vector <cv::Point3d> points_landmark_tmp = landmark_points_;
    i = 0;
    for (auto iter = landmark_points_.begin(); iter != landmark_points_.end(); i++) {
        if (status[i] == 0) {
            iter = points_landmark_tmp.erase(iter);
            continue;
        }
        iter++;
    }
    landmark_points_ = points_landmark_tmp;

    //todo: update the descriptors according to status
//    cv::Ptr <cv::DescriptorExtractor> descriptor = cv::ORB::create();
//    cv::Mat descptor_tmp;
//    descriptor->compute(image_, keypoints_, descptor_tmp);
//    descriptors_ = descptor_tmp;

    std::cout<<"Size of the keypoints, 2f, landmark points, descriptor after crop are " << keypoints_.size()
             << " " << kps2f_.size()
             << " " << landmark_points_.size()
             << " " << descriptors_.rows
             << std::endl;
}

void frame::draw_and_store_tracking()
{
    std::cout << "Drawing the tracked feature "<< std::endl;
    // draw the keypoints tracked by optical flow
    cv::Mat img_show = image_.clone();
    for ( auto kp:kps2f_ )
        cv::circle(img_show, kp, 10, cv::Scalar(0, 240, 0), 1);

    std::cout << "Finishing Drawing "<< std::endl;

    std::cout << "Storing the Drawing "<< std::endl;

    // Store the images ////////////////////////////////////////////////////////////////////////////////
    std::string image_file1("/work/catkin_ws_ndt/src/localization_ndt/img/img_frame_with_LK_");
    std::stringstream str_stream;
    str_stream << id_;
    std::string image_file2 = str_stream.str();
    std::string image_file3(".png");
    std::string image_file = image_file1 + image_file2 + image_file3;
    cv::imwrite(image_file, img_show);

    std::cout << "Finishing Storing "<< std::endl;
}

void frame::pub_odom(ros::Publisher odom_pub)
{
    std::cout << "Publishing the odom "<< std::endl;
    // update the position relative to first frame
    double pose_x = T_cw_.matrix()(0, 3);
    double pose_y = T_cw_.matrix()(1, 3);
    double pose_z = T_cw_.matrix()(2, 3);

    // update the rotation matrix relative to first frame
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix(0, 0) = T_cw_.matrix()(0, 0);
    rotation_matrix(0, 1) = T_cw_.matrix()(0, 1);
    rotation_matrix(0, 2) = T_cw_.matrix()(0, 2);
    rotation_matrix(1, 0) = T_cw_.matrix()(1, 0);
    rotation_matrix(1, 1) = T_cw_.matrix()(1, 1);
    rotation_matrix(1, 2) = T_cw_.matrix()(1, 2);
    rotation_matrix(2, 0) = T_cw_.matrix()(2, 0);
    rotation_matrix(2, 1) = T_cw_.matrix()(2, 1);
    rotation_matrix(2, 2) = T_cw_.matrix()(2, 2);

    // Update the Quaternion
    Eigen::Quaterniond q = Eigen::Quaterniond(rotation_matrix);
    double pose_qx = q.coeffs()(0);
    double pose_qy = q.coeffs()(1);
    double pose_qz = q.coeffs()(2);
    double pose_qw = q.coeffs()(3);


    // publish the left camera odometry /////////////////////////////////////////////////////////////////////////////////////////
    nav_msgs::Odometry curr_odom;
//    curr_odom.header.stamp = lcam_img_msg_ptr->header.stamp;
    curr_odom.header.frame_id = "imu_link";
    curr_odom.child_frame_id = "lcam_odom";

    curr_odom.pose.pose.position.x = pose_x;
    curr_odom.pose.pose.position.y = pose_y;
    curr_odom.pose.pose.position.z = pose_z;

    curr_odom.pose.pose.orientation.x = pose_qx;
    curr_odom.pose.pose.orientation.y = pose_qy;
    curr_odom.pose.pose.orientation.z = pose_qz;
    curr_odom.pose.pose.orientation.w = pose_qw;

    odom_pub.publish(curr_odom);

    std::cout << "Finishing Publishing "<< std::endl;
}

void frame::detect_tf()
{
    const size_t inWidth = 512;
    const size_t inHeight = 512;
    const double inScaleFactor = 1.0;
    const float confidenceThreshold = 0.7;
    const cv::Scalar meanVal(104.0, 177.0, 123.0);

    cv::Mat inputBlob = cv::dnn::blobFromImage(image_, inScaleFactor, cv::Size(inWidth, inHeight), meanVal, true, false);
    cv::dnn::Net net = cv::dnn::readNetFromTensorflow(tensorflowConfigFile, tensorflowWeightFile);
    net.setInput(inputBlob, "data");
    cv::Mat detection = net.forward("detection_out");
    std::cout << "size of the detection is " << detection.size << std::endl;

}

void frame::detect_ssd()
{
    const size_t inWidth = 512;
    const size_t inHeight = 512;
    const double inScaleFactor = 1.0;
    const float confidenceThreshold = 0.7;
    const cv::Scalar meanVal(104.0, 177.0, 123.0);

    cv::dnn::Net net = cv::dnn::readNetFromCaffe(SSDConfigFile, SSDWeightFile);
    cv::Mat inputBlob = cv::dnn::blobFromImage(image_, inScaleFactor, cv::Size(inWidth, inHeight), meanVal, true, false);

    net.setInput(inputBlob, "data");
    cv::Mat detection = net.forward("detection_out");
    std::cout << "size of the detection is " << detection.size << std::endl;

}

void frame::detect_yolo_opencv()
{
    std::cout << "Detection by Yolo Started" << std::endl;

    cv::dnn::Net net = cv::dnn::readNetFromDarknet(YoloConfigFile, YoloWeightFile);
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);

//    net.setPreferableBackend(cv::dnn::DNN_BACKEND_DEFAULT);
//    net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    std::vector<std::string> layernames;
    layernames = net.getLayerNames();
    std::vector<std::string> output_layers;
    output_layers = net.getUnconnectedOutLayersNames();
    cv::Mat img = image_.clone();
    cv::Mat blob = cv::dnn::blobFromImage(img,1.0/255.0,{512,512},true,false);
    net.setInput(blob);
    std::vector<cv::Mat> detections;
    auto dnn_start = std::chrono::steady_clock::now();
    net.forward(detections, output_layers);
    auto dnn_end = std::chrono::steady_clock::now();
    float inference_fps = 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(dnn_end - dnn_start).count();
    std::cout << "Inference FPS is " << inference_fps << std::endl;

//    // 遍历所有结果集
//    for (auto& detectionlayer:detections) //对每一个输出层
//    {
//        for(int i = 0;i < detectionlayer.rows;++i) // rows 等于检测出的Candidate BBox的个数
//        {
//            const int probability_index = 5;
//            const int probability_size = detectionlayer.cols - probability_index; // 输出的类的数目
//            float *prob_array_ptr = &detectionlayer.at<float>(i, probability_index); // todo:不应该是4吗
//            size_t objectClass = std::max_element(prob_array_ptr, prob_array_ptr + probability_size) - prob_array_ptr;
//            float confidence = detectionlayer.at<float>(i, (int)objectClass + probability_index);
////            std::cout << "Confidence is " << confidence << std::endl;
//            // 比较置信度并绘制满足条件的置信度
//            if (confidence > CONFIDENCE_THRESHOLD_YOLO)
//            {
//                float x = detectionlayer.at<float>(i, 0);
//                float y = detectionlayer.at<float>(i, 1);
//                float width = detectionlayer.at<float>(i, 2);
//                float height = detectionlayer.at<float>(i, 3);
//
//                int xLeftBottom = static_cast<int>((x - width / 2) * img.cols);
//                int yLeftBottom = static_cast<int>((y - height / 2) * img.rows);
//                int xRightTop = static_cast<int>((x + width / 2) * img.cols);
//                int yRightTop = static_cast<int>((y + height / 2) * img.rows);
//
//                cv::Rect object(xLeftBottom, yLeftBottom,xRightTop - xLeftBottom,yRightTop - yLeftBottom);//x y w h
//                cv::rectangle(img, object, cv::Scalar(0, 0, 255), 2, 8);
//            }
//        }
//    }
//
//    // 显示图片
//    cv::imshow("Darknet",img);
//    cv::waitKey(0);

    std::vector<int> indices[NUM_CLASSES_YOLO];
    std::vector<cv::Rect> boxes[NUM_CLASSES_YOLO];
    std::vector<float> scores[NUM_CLASSES_YOLO];

    for (auto& output : detections) //对每一个输出层
    {
//        std::cout << "Detection is " << output << std::endl;
        const auto num_boxes = output.rows;
        for (int i = 0; i < num_boxes; i++)
        {
            float x = output.at<float>(i, 0);
            float y = output.at<float>(i, 1);
            float width = output.at<float>(i, 2);
            float height = output.at<float>(i, 3);

            int xLeftBottom = static_cast<int>((x - width / 2) * img.cols);
            int yLeftBottom = static_cast<int>((y - height / 2) * img.rows);
            int xRightTop = static_cast<int>((x + width / 2) * img.cols);
            int yRightTop = static_cast<int>((y + height / 2) * img.rows);

            cv::Rect rect(xLeftBottom, yLeftBottom, xRightTop - xLeftBottom, yRightTop - yLeftBottom);//x y w h

            for (int c = 0; c < NUM_CLASSES_YOLO; c++)
            {
                auto confidence = *output.ptr<float>(i, 5 + c);

                if (confidence >= CONFIDENCE_THRESHOLD_YOLO)
                {
                    boxes[c].push_back(rect);
                    scores[c].push_back(confidence);
//                    std::cout << "Confidence is " << confidence << std::endl;
                }
            }
        }
    }

    for (int c = 0; c < NUM_CLASSES_YOLO; c++)
    {
        cv::dnn::NMSBoxes(boxes[c], scores[c], 0.0, NMS_THRESHOLD_YOLO, indices[c]);
    }

    // Drawing the picture with bb
    for (int c= 0; c < NUM_CLASSES_YOLO; c++)
    {
        for (size_t i = 0; i < indices[c].size(); ++i)
        {
            auto idx = indices[c][i];
            const auto& rect = boxes[c][idx];
            cv::rectangle(img, rect, cv::Scalar(0, 0, 255), 2, 8);
        }
    }

    cv::namedWindow("output");
    cv::imshow("output", img);
    cv::waitKey(0);

    std::cout << "Detection by Yolo Finished" << std::endl;
}

void frame::detect_yolo_dark(std::vector<object_det::ptr>& new_objects)
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

void frame::detect_yolo_trt(std::vector<object_det::ptr>& new_objects)
{
    std::cout << "Detection by YOLO TensorRT Started" << std::endl;

    // Read Engine File
    std::fstream file;
    std::string engineFile = "/work/tools/tensorrt/TensorRT-7.1.3.4/bin/yolo-512-fp32.engine";
    file.open(engineFile, std::ios::binary | std::ios::in);
    if(!file.is_open())
    {
        std::cout << "read engine file" << engineFile <<" failed" << std::endl;
        return;
    }
    file.seekg(0, std::ios::end);
    int length = file.tellg();
    file.seekg(0, std::ios::beg);
    std::unique_ptr<char[]> data(new char[length]);
    file.read(data.get(), length);
    file.close();

    // Creating Runtime
    nvinfer1::IRuntime* runtime = nvinfer1::createInferRuntime(gLogger);
    if(!runtime)
    {
        std::cout << "Failed to create runtime" << std::endl;
        return;
    }

    // Create Engine
    nvinfer1::ICudaEngine* engine = runtime->deserializeCudaEngine(data.get(), length, nullptr); // Deserializing the model back into a engine
    if(!engine)
    {
        std::cout << "Failed to deserialize nor create engine" << std::endl;
        return;
    }
    // Creating Context
    nvinfer1::IExecutionContext* context = engine->createExecutionContext();
    if(!context)
    {
        std::cout << "Failed to create context" << std::endl;
        return;
    }
    int index_number = engine->getNbBindings();
    std::cout << "Number of Bindings are " << index_number << std::endl;
    void* buffers[index_number]; // buffers for input and output data
    int batchSize = 1;

    std::vector<nvinfer1::Dims> input_dims;
    std::vector<nvinfer1::Dims> output_dims;

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    // Scale the det_img to the size of track_img
    cv::Mat img_resized = cv::Mat();
    cv::Size yolo_input_size = cv::Size(512, 512);
    cv::resize(image_, img_resized, yolo_input_size, 0, 0, cv::INTER_AREA);
//    size_t single_input_size = img_resized.total()*img_resized.elemSize();
    size_t input_cols = img_resized.cols;
    size_t input_rows = img_resized.rows;
    size_t num_output = 80;

    // 申请heap空间
    std::unique_ptr<float []> img_flat(new float[input_cols*input_rows*3]); //img_flat指向包含input_cols*input_rows*3个未初始化float的数组
    for(size_t ch = 0; ch < 3; ch++)
    {
        for(size_t row = 0; row < input_rows; row++)
        {
            for(size_t col = 0; col < input_cols; col++)
            {
                img_flat[ch*input_cols*input_rows + input_cols*row + col] = img_resized.at<cv::Vec3b>(row, col)[ch]; // BGR
            }
        }
    }
    // todo: convert to the color order of yolo, subtract the mean

    //使用cuda 流来管理并行计算
    // Use CUDA streams to manage the concurrency of copying and executing
    cudaStream_t stream;
    cudaStreamCreate(&stream);


    for (int i = 0; i < index_number; ++i)
    {
        // Create CUDA buffer for Tensor.

        if(engine->bindingIsInput(i))
        {
            size_t single_input_size = getSizeByDim(engine->getBindingDimensions(i))*sizeof(float);
            std::cout << "Input Dim is " << single_input_size << std::endl;
            input_dims.emplace_back(engine->getBindingDimensions(i));
            CUDA_CHECK(cudaMalloc(&buffers[i], batchSize*single_input_size)); // cudaMalloc的第一个参数是指针的地址，类型为void**，此处c++进行了隐式的转换 从&buffers[i](void*)转成(void**)
            //从内存到显存，input是读入内存中的数据；buffers[inputIndex]是显存上的存储区域，用于存放输入数据
            // Copy Input Data to the GPU
            std::cout << "Allocate GPU memory for input " << std::endl;
            CUDA_CHECK(cudaMemcpyAsync(buffers[i], &img_flat[0], // 指向数组的unique pointer只提供[]操作符，这里取img_flat管理的数组的第一个元素的地址
                    batchSize*single_input_size,
                    cudaMemcpyHostToDevice, stream));
        }
        else
        {
            size_t single_output_size = getSizeByDim(engine->getBindingDimensions(i))*sizeof(float);
            std::cout << "output Dim is " << single_output_size << std::endl;
            output_dims.emplace_back(engine->getBindingDimensions(i));
            CUDA_CHECK(cudaMalloc(&buffers[i], batchSize*single_output_size));// cudaMalloc的第一个参数是指针的地址，类型为void**，此处c++进行了隐式的转换 从&buffers[i](void*)转成(void**)
            std::cout << "Allocate GPU memory for output " << std::endl;

        }
    }
    std::cout<<"size of input dim is " << input_dims.size() << std::endl;
    std::cout<<"size of output dim is " << output_dims.size() << std::endl;

    if (input_dims.empty() || output_dims.empty())
    {
        std::cerr << "Expect at least one input and one output for network\n";
        return;
    }

    //启动cuda核计算
    // Launch an instance of the GIE compute kernel
    context->enqueue(batchSize, buffers, stream, nullptr);

    int output0_size = getSizeByDim(output_dims[0]);
    int output1_size = getSizeByDim(output_dims[1]);
//    std::cout<<"size of output0 is " << output0_size << std::endl;
//    std::cout<<"size of output1 is " << output1_size << std::endl;

    std::unique_ptr<float []> output0_array(new float[output0_size]); //output0_array指向包含output0_size个未初始化float的数组
    std::unique_ptr<float []> output1_array(new float[output1_size]); //output1_array指向包含output1_size个未初始化float的数组
    output1_array[0] = 3.0
    //从显存到内存，buffers[outputIndex]是显存中的存储区，存放模型输出；output是内存中的数据
    // Copy Output Data to the Host
    CUDA_CHECK(cudaMemcpyAsync(&output0_array[0], buffers[1], // 指向数组的unique pointer只提供[]操作符，这里取output0_array管理的第一个元素的地址
                    output0_size*sizeof(float),
                    cudaMemcpyDeviceToHost, stream));
    CUDA_CHECK(cudaMemcpyAsync(&output1_array[0], buffers[2],
                    output1_size*sizeof(float),
                    cudaMemcpyDeviceToHost, stream));

    std::cout<<"first element of output1 array is " << output1_array[0] <<std::endl;

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>> ( t2-t1 );
    std::cout<<"Detection by TrT "<<time_used.count() <<" seconds."<<std::endl;

    //todo: delete the cuda stream, buffer
    cudaStreamDestroy(stream);
    CUDA_CHECK(cudaFree(buffers[0]));
    CUDA_CHECK(cudaFree(buffers[1]));
    CUDA_CHECK(cudaFree(buffers[2]));

    // destroy the engine
    context->destroy();
    engine->destroy();

    std::cout << "Detection by YOLO TensorR Finished" << std::endl;
}

void frame::draw_boxes(const cv::Mat& mat_img,
                       const std::vector<bbox_t>& result_vec,
                       const std::vector<std::string>& obj_names,
                       const int current_det_fps = -1,
                       const int current_cap_fps = -1)
{
    int const colors[6][3] = { { 1,0,1 },{ 0,0,1 },{ 0,1,1 },{ 0,1,0 },{ 1,1,0 },{ 1,0,0 } };
    for (auto &i : result_vec) {
        cv::Scalar color = obj_id_to_color(i.obj_id);
        std::cout << "Detection Catagory is " << i.obj_id << std::endl;
        std::cout << "Rectangle is " << i.x << " " << i.y << " " << i.w << " " << i.h << std::endl;
        std::cout << "Prob is " << i.prob << std::endl;
        cv::rectangle(mat_img, cv::Rect(i.x, i.y, i.w, i.h), color, 2);
    }
    cv::imshow ( "Detection", mat_img );
    cv::waitKey(0);
    std::cout << "Detection by Dark Finished" << std::endl;
}

cv::Scalar frame::obj_id_to_color(int obj_id) {
    int const colors[6][3] = { { 1,0,1 },{ 0,0,1 },{ 0,1,1 },{ 0,1,0 },{ 1,1,0 },{ 1,0,0 } };
    int const offset = obj_id * 123457 % 6;
    int const color_scale = 150 + (obj_id * 123457) % 100;
    cv::Scalar color(colors[offset][0], colors[offset][1], colors[offset][2]);
    color *= color_scale;
    return color;
}

std::vector<std::string> frame::objects_names_from_file(std::string const filename) {
    std::ifstream file(filename);
    std::vector<std::string> file_lines;
    if (!file.is_open()) return file_lines;
    for(std::string line; getline(file, line);) file_lines.push_back(line);
    std::cout << "object names loaded \n";
    return file_lines;
}

void frame::bboxes_to_objects(const std::vector<bbox_t>& bboxes_2d, std::vector<object_det::ptr>& new_objects)
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

size_t frame::getSizeByDim(const nvinfer1::Dims& dims)
{
    size_t size = 1;
    for (size_t i = 0; i < dims.nbDims; ++i)
    {
        size *= dims.d[i];
    }
    return size;
}


