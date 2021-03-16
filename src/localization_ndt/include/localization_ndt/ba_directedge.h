//
// Created by eagleflag on 2020/8/14.
//

#ifndef CATKIN_WS_NDT_BA_DIRECTEDGE_H
#define CATKIN_WS_NDT_BA_DIRECTEDGE_H

#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <chrono>

class BA_DIRECT_COST {

public:
    BA_DIRECT_COST(double x, double y, cv::Mat& img0, cv::Mat& img1):
            u0(x), v0(y), img_pre(img0), img_curr(img1){}

//    template <typename T>
    bool operator()(const double* const pose_cam, // pose_cam[0,1,2] should come in the form of angle-axis rotation, because ceres deals with this form of representation
                    const double* const pos_landmark,
                    double* residual) const
    {

        // Computation of the intensity error //////////////////////////////////////////////////////////////////////////////////////////

        // Step 1: Get the intensity of u0, v0
        double i0;
        getIntensity(&img_pre, u0, v0, i0);

        // Step 2: Back Project the pixel point in current frame into 3d point (in the first frame)
        double pos_landmark_pre[3];
        project2Dto3D(u0, v0, pos_landmark[2], pos_landmark_pre);

        // Step 3: Transform the 3d point in the first frame into 3d point in the second frame
        double pos_landmark_curr[3];
        // rotate the landmark point using the pose camera
        ceres::AngleAxisRotatePoint(pose_cam, pos_landmark_pre, pos_landmark_curr);
        // add the translation
        pos_landmark_curr[0] += pose_cam[3];
        pos_landmark_curr[1] += pose_cam[4];
        pos_landmark_curr[2] += pose_cam[5];
//        std::cout << "pos_landmark_converted in camera frame " << pos_landmark_converted[0] << pos_landmark_converted[1] << pos_landmark_converted[2] << std::endl;

        // Step 4: Project the 3d point in the second frame into the pixel point in second camera
        double pts2d_curr_img[2];
        project3Dto2D(pos_landmark_curr[0], pos_landmark_curr[1], pos_landmark_curr[2], pts2d_curr_img);

        // Step 5: Calculate the intensity and photometric error
        double i1;
        getIntensity(&img_curr, pts2d_curr_img[0], pts2d_curr_img[1], i1);
        residual[0] = i1 - i0;

        return true;
    }

//    static ceres::CostFunction* Create(const double u0, const double v0, const cv::Mat& img_pre, const cv::Mat& img_curr){
//        return (new ceres::AutoDiffCostFunction<BA_DIRECT_COST,1,6,3>(
//                new BA_DIRECT_COST(u0, v0, img_pre, img_curr)));
//    }
//    template <typename T>
//    inline T BilinearInterpolation(T q11, T q12, T q21, T q22, T x1, T x2, T y1, T y2, T x, T y)
//    {
//        T x2x1, y2y1, x2x, y2y, yy1, xx1;
//        x2x1 = x2 - x1;
//        y2y1 = y2 - y1;
//        x2x = x2 - x;
//        y2y = y2 - y;
//        yy1 = y - y1;
//        xx1 = x - x1;
//        return 1.0 / (x2x1 * y2y1) * (
//                q11 * x2x * y2y +
//                q21 * xx1 * y2y +
//                q12 * x2x * yy1 +
//                q22 * xx1 * yy1
//        );
//    }

    // get a the intensity scale value from the image (bilinear interpolated)
//    template <typename T>
    inline void project2Dto3D( const double& x, const double& y, const double& d, double* pts_3d) const
    {
        //todo: check the computation
        //todo: why is there the const describer
        double zz = d;
        double xx = zz* ( x-cx ) /fx;
        double yy = zz* ( y-cy ) /fy;
        *pts_3d = xx;
        *(pts_3d + 1) = yy;
        *(pts_3d + 2) = zz;
        return;
    }

    // Project the pixel point 3d point (in current frame)
//    template <typename T>
    inline void project3Dto2D( const double& x, const double& y, const double& z, double* pts_2d) const
    {
        //todo: check the computation
        // todo: check the bound of the projected pixel
        double u1 = fx*x/z+cx;
        double v1 = fy*y/z+cy;
        *pts_2d = u1;
        *(pts_2d + 1) = v1;
        return;
    }

    inline void getIntensity( const cv::Mat* img, const double& x, const double& y, double i) const
    {
        //todo: check the computation
        uchar* data = & img->data[ int ( y ) * img->step + int ( x ) ];
        double xx = x - floor ( x );
        double yy = y - floor ( y );
        i =     ( 1-xx ) * ( 1-yy ) * data[0] +
                xx* ( 1-yy ) * data[1] +
                ( 1-xx ) *yy*data[ img->step ] +
                xx*yy*data[img->step+1];
    }

private:
    const double u0, v0;
    const cv::Mat img_pre, img_curr;
    const double fx = 520.9;
    const double cx = 325.1;
    const double fy = 521.0;
    const double cy = 249.7;

};

#endif //CATKIN_WS_NDT_BA_DIRECTEDGE_H
