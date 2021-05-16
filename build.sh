#!/bin/bash

rm -rf build/ devel/

#cd src/localization_ndt/Thirdparty/g2o
#mkdir build
#cd build
#cmake .. -DCMAKE_BUILD_TYPE=Release
#make -j8
#cd ../../../../..

cd src/thirdparty/g2o
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8
cd ../../../..

#cd src/localization_ndt/Thirdparty/GeographicLib
#mkdir build
#cd build
#cmake .. -DCMAKE_BUILD_TYPE=Release
#make -j8
#cd ../../../../..

cd src/localization_ndt/Thirdparty/ndt_gpu
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8
cd ../../../../..

#cd src/localization_ndt/Thirdparty/darknet
#make
#cd ../../../..

cd src/thirdparty/darknet
make
cd ../../..

cd src/thirdparty/GeographicLib
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8
cd ../../../..

#cd src/localization_ndt/Thirdparty/DBoW2
#mkdir build
#cd build
#cmake .. -DCMAKE_BUILD_TYPE=Release
#make -j8
#cd ../../../../..

cd src/thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8
cd ../../../..

catkin_make

