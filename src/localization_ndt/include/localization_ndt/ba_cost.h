//
// Created by eagleflag on 2020/7/28.
//

#ifndef CATKIN_WS_NDT_BA_COST_H
#define CATKIN_WS_NDT_BA_COST_H

#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <chrono>

class BA_3D2D_COST {

public:
    BA_3D2D_COST(double x, double y): observed_x(x), observed_y(y){}

    template <typename T>
    bool operator()(const T* const pose_cam,
                    const T* const pos_landmark,
                    T* residual) const
    {
        // pose_cam[0,1,2] are the angle-axis rotation.
        T pos_landmark_converted[3];
        // todo: check what is calculated and how
        // rotate the landmark point using the pose camera
        ceres::AngleAxisRotatePoint(pose_cam, pos_landmark, pos_landmark_converted);
//        std::cout << "pos_landmark_converted in camera frame " << pos_landmark_converted[0] << pos_landmark_converted[1] << pos_landmark_converted[2] << std::endl;
        // add the translation
        pos_landmark_converted[0] += pose_cam[3];
        pos_landmark_converted[1] += pose_cam[4];
        pos_landmark_converted[2] += pose_cam[5];

        // Normalization to the plane where z = 1
        const T xp = pos_landmark_converted[0]/pos_landmark_converted[2];
        const T yp = pos_landmark_converted[1]/pos_landmark_converted[2];

//        const T xp = pos_landmark_converted[0];
//        const T yp = pos_landmark_converted[1];
//        std::cout << "x in pixel " << xp << std::endl;

//        static cv::Mat K_lcam = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

        // calculate the pixel point
        const T predicted_x = fx*xp + cx;
        const T predicted_y = fy*yp + cy;

        residual[0] = observed_x - predicted_x;
        residual[1] = observed_y - predicted_y;
        return true;
    }

//    // Factory to hide the construction of the CostFunction object from
//    // the client code.
//    static ceres::CostFunction* Create(const double observed_x,
//                                       const double observed_y) {
//
//        return (new ceres::AutoDiffCostFunction<BA_3D2D_COST, 2, 6, 3>(
//                new BA_3D2D_COST(observed_x, observed_y)));
//    }

private:
    const double observed_x, observed_y;
    const double fx = 520.9;
    const double cx = 325.1;
    const double fy = 521.0;
    const double cy = 249.7;

};

#endif //CATKIN_WS_NDT_BA_COST_H
