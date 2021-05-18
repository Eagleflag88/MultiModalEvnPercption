# MultiModalEvnPercption

A ROS-Based Experimental Software Platform For Multi-Modal Simultaneous Localization And Mapping

For the purpose of studying and research in the field of SLAM, based on the infrastructure of ROS, this Repo provide a software platform 

# Dependencies

## Basic Software Environment
Ubuntu 20.04 \
ROS1 Noetic\
OpenCV/Opencv_Contrib: 4.4.0 (For Compatibility with Cuda 11), Consult the Installation Instruction\
OpenMP:
```
sudo apt install libomp-dev    
```

## Software for DL Based Environment Perception
Cuda: 11: Consult the Installation Instruction\
Cudnn: 8: Consult the Installation Instruction\
TensorRT: Consult the Installation Instruction\
Darknet For Yolo: Consult the Installation Instruction

## SLAM Related
Ceres: Consult the Installation Instruction; SuiteSparse and CXSparse included \
Eigen 3:
```
sudo apt install libeigen3-dev
```
g2o: Comes with the Repo, github commit a48ff8c\
Geographic_Lib: A library dealing with Comes with the Repo

# Building

## Clone the Repo
```
git clone https://github.com/Eagleflag88/MultiModalEvnPercption.git
```
## Build the Ros Package
```
./build.sh
```
# Launch and Run

# Software Architecture

The whole package of software consists of three layers:

## Sensors
Modeling of the sensor driver, which acts as the source of sensor measurements. In the case of simulation, these nodes take the playback of the rosbag or other compliant data format.

## Frontend
Responsible for preprocessing of the measurements, such as filtering and undistortion, feature extraction and motion estimation. Meanwhile, objection detection based on deep learning is implemented for object-level environmental understanding. In addition, a tracker node is exploited to take care of data association, both in object-level and feature-level.

## Backend 
While the frontend focuses on the real-time performance, the nodes of the backend try to fuse the initial guesses from frontend to obtain more accurate estimate of the ego poses and landmarks. At first, a sliding window is maintained to constrain the computational burden and complexity. Within the sliding window, maximum-a-posteriori estimates are calculated based on the inference of a factor graph. Alternatively, an EKF can be applied to the sliding window.


![Software Architecture](https://github.com/Eagleflag88/MultiModalEvnPercption/blob/main/SoftwareArch.jpg)



