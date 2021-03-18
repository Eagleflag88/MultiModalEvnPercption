#!/bin/bash

rm -rf build/ devel/

cd src/localization_ndt/Thirdparty/g2o
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8

cd ../../../../..

catkin_make

