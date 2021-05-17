//
// Created by eagleflag on 2021/3/21.
//

#include "cam_frame.h"
#include <iostream>

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

cam_frame::cam_frame()
{
    std::cout << "camera frame without id " << "is constructed" << std::endl;
}

cam_frame::cam_frame(
        unsigned long id,
        double time_stamp,
        cv::Mat image,
        std::vector<cv::KeyPoint> keypoints,
        cv::Mat descriptors
):
        id_(id),
        time_stamp_(time_stamp),
        image_(image),
        keypoints_(keypoints),
        descriptors_(descriptors)// Implementation of the constructor
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

void cam_frame::detect_ssd(std::vector<object_det::ptr>& new_objects)
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

    std::vector<bbox_t> result_vec;
    if(result_vec.empty())
        std::cout << "Nothing is out" << std::endl;

    bboxes_to_objects(result_vec, new_objects);

    std::cout << "size of the detection is " << detection.size << std::endl;

}

void cam_frame::detect_yolo_opencv(std::vector<object_det::ptr>& new_objects)
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

    std::vector<bbox_t> result_vec;
    if(result_vec.empty())
        std::cout << "Nothing is out" << std::endl;

    bboxes_to_objects(result_vec, new_objects);

    std::cout << "Detection by Yolo-OpenCV Finished" << std::endl;
}

size_t cam_frame::getSizeByDim(const nvinfer1::Dims& dims)
{
    size_t size = 1;
    for (size_t i = 0; i < dims.nbDims; ++i)
    {
        size *= dims.d[i];
    }
    return size;
}

void cam_frame::detect_yolo_trt(std::vector<object_det::ptr>& new_objects)
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

void cam_frame::detect_feature()
{
    //-- 初始化
    cv::Mat descriptors_;
    // used in OpenCV3
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();

    //-- 第一步:检测 Oriented FAST 角点位置
    std::vector<cv::KeyPoint> keypoints;
    detector->detect (image_, keypoints_);

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute (image_, keypoints_, descriptors_);
}