cmake_minimum_required(VERSION 3.0.2)
project(DBoW2)

#set (dbow2_ROOR_DIR "${PROJECT_SOURCE_DIR}/orb_slam2/Thirdparty/DBoW2")
set (dbow2_ROOR_DIR "/work/tools/MultiModalEvnPercption/src/localization_ndt/Thirdparty/DBoW2")


set(HDRS_DBOW2
  ${dbow2_ROOR_DIR}/DBoW2/BowVector.h
  ${dbow2_ROOR_DIR}/DBoW2/FORB.h
  ${dbow2_ROOR_DIR}/DBoW2/FClass.h
  ${dbow2_ROOR_DIR}/DBoW2/FeatureVector.h
  ${dbow2_ROOR_DIR}/DBoW2/ScoringObject.h
  ${dbow2_ROOR_DIR}/DBoW2/TemplatedVocabulary.h)
set(SRCS_DBOW2
  ${dbow2_ROOR_DIR}/DBoW2/BowVector.cpp
  ${dbow2_ROOR_DIR}/DBoW2/FORB.cpp
  ${dbow2_ROOR_DIR}/DBoW2/FeatureVector.cpp
  ${dbow2_ROOR_DIR}/DBoW2/ScoringObject.cpp)

set(HDRS_DUTILS
  ${dbow2_ROOR_DIR}/DUtils/Random.h
  ${dbow2_ROOR_DIR}/DUtils/Timestamp.h)
set(SRCS_DUTILS
  ${dbow2_ROOR_DIR}/DUtils/Random.cpp
  ${dbow2_ROOR_DIR}/DUtils/Timestamp.cpp)

set(OpenCV_DIR /work/opencv/build)
find_package(OpenCV REQUIRED PATHS "/work/opencv/build")

include_directories(
        ${OpenCV_INCLUDE_DIRS})

#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${dbow2_ROOR_DIR}/lib)

add_library(DBoW2 SHARED ${SRCS_DBOW2} ${SRCS_DUTILS})
target_link_libraries(DBoW2 ${OpenCV_LIBS})
