# MultiModalEvnPercption

A ROS-Based Experimental Platform For Multi Modal Simultaneous Localization And Mapping

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
Geographic_Lib: A library dealing with Comes with the Repo

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
