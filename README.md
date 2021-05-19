# MultiModalEvnPercption

A ROS-based experimental software platform for Multi-Modal Simultaneous Localization And Mapping.

For the purpose of studying and research in the field of SLAM, based on the infrastructure of ROS, this repo provide a software platform which is highly 
modularized and is able to fuse the information from a number of different modalities to achieve to an accurate and consistent environmental understanding. 

# Software Architecture

The whole package of software consists of three layers, as depicted in following picture:

![Software Architecture](https://github.com/Eagleflag88/MultiModalEvnPercption/blob/main/SoftwareArch.jpg)

## Sensors
Modeling of the sensor driver, which acts as the source of sensor measurements. In the case of simulation, 
these nodes take the playback of rosbag or other compliant data format.

## Frontend
Responsible for preprocessing of the measurements, such as filtering and undistortion. 
As next steps, feature extraction and motion estimation are implemented. 
Meanwhile, objection detection based on deep learning is implemented for object-level environmental understanding. 
In addition, a tracker node is exploited to take care of data association, in feature and object-level.

## Backend 
While the frontend focuses on the real-time performance, the nodes of the backend try to fuse the initial guesses 
from frontend to obtain more accurate estimate of the ego poses and landmarks. 
To this end, a sliding window is maintained to constrain the computational burden and complexity. 
Within the sliding window, maximum-a-posteriori estimates are calculated based on the inference of a factor graph. 
Alternatively, an EKF can be applied.


# Dependencies

## Basic Software Environment
OS: Ubuntu 20.04 \
ROS: Noetic\
OpenCV & Opencv_Contrib: 4.4.0 (For Compatibility with Cuda 11), Consult the Installation Instruction\
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
Ceres: Consult the Installation Instruction; SuiteSparse and CXSparse are included \
Eigen 3:
```
sudo apt install libeigen3-dev
```
g2o: Comes with the repo, github commit a48ff8c\
Geographic_Lib: A library dealing with geographic coordinate conversion, comes with the repo

# Building

## Clone the Repo
```
git clone https://github.com/Eagleflag88/MultiModalEvnPercption.git
```
## Build the Ros Package
```
./build.sh
```

# Todo

0. Refactor the codebase.
1. Implement the keyframe_node, factor_graph_node and ekf_node.
2. Create launch files for different functional modes, for example, camera_only, lidar_only, visual inertial, etc.
3. Add the node for place recognition into the software architecture.
4. Add the HD-Map as a further source of information.
5. Add semantic pipeline





