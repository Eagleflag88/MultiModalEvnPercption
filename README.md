# MultiModalEvnPercption

# Hardware Setup
Ubuntu 20.04 \
CPU: AMD R7 4800h\
GPU: NVIDIA GTX 1650

# Software Dependencies

## Basic Software Environment
ROS: Noetic\
OpenCV: 4.4.0 (For Compatibility with Cuda 11), Consult the Installation Instruction\
Opencv_Contrib：same version as OpenCV\
OpenMP:\
``
sudo apt install libomp-dev
``

## DL Based Environment Perception
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
GeographicLib: A library dealing with Comes with the Repo

# Building the ROS Package

## Clone the Repo
```
git clone https://github.com/Eagleflag88/MultiModalEvnPercption.git
```
## Build the Ros Package
```
./build.sh
```
# Launch and Run
