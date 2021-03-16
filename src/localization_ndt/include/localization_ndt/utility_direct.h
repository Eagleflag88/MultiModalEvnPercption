//
// Created by eagleflag on 2020/8/14.
//
#include <iostream>
#include <opencv2/core/core.hpp>
#include <cmath>
#include <climits>

#ifndef CATKIN_WS_NDT_UTILITY_DIRECT_H
#define CATKIN_WS_NDT_UTILITY_DIRECT_H

inline void getIntensity( const cv::Mat* img, const float x, const float y, float i)
{
    uchar* data = & img->data[ int ( y ) * img->step + int ( x ) ];
    float xx = x - floor ( x );
    float yy = y - floor ( y );
    i =     ( 1-xx ) * ( 1-yy ) * data[0] +
            xx* ( 1-yy ) * data[1] +
            ( 1-xx ) *yy*data[ img->step ] +
            xx*yy*data[img->step+1];
}

//// Back Project the pixel point 3d point (in current frame)
//inline void project2Dto3D( const float x, const float y, const float d, std::vector<float> pts_3d)
//{
//    float zz = d;
//    float xx = zz* ( x-cx ) /fx;
//    float yy = zz* ( y-cy ) /fy;
////        float pts_3d[3];
//    pts_3d.push_back(xx);
//    pts_3d.push_back(yy);
//    pts_3d.push_back(zz);
////        pts_3d[1] = yy;
////        pts_3d[2] = zz;
//    return;
//}

// Project the pixel point 3d point (in current frame)
inline void project3Dto2D( const float x, const float y, const float z, std::vector<float> pts_2d)
{
    float u1 = fx*x/z+cx;
    float v1 = fy*y/z+cy;
    pts_2d.push_back(u1);
    pts_2d.push_back(v1);
//        float pts_2d[2];
//        pts_2d[0] = u1;
//        pts_2d[1] = v1;
}

#endif //CATKIN_WS_NDT_UTILITY_DIRECT_H
