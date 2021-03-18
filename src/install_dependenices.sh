#!/bin/bash

cd localization_ndt/Thirdparty/g2o
git checkout 
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j2
cd ../../../..

