//
// Created by eagleflag on 2020/8/19.
//

#ifndef CATKIN_WS_NDT_UTILITY_H
#define CATKIN_WS_NDT_UTILITY_H

#include <opencv2/core/core.hpp>
#include <Eigen/Geometry>
#include <Eigen/Core>

inline cv::Point2f pixel2cam ( const cv::Point2f& p, const cv::Mat& K )
{
    return cv::Point2f
            (
                    ( p.x - K.at<double>(0,2) ) / K.at<double>(0,0),
                    ( p.y - K.at<double>(1,2) ) / K.at<double>(1,1)
            );
}

Eigen::Vector3d pixel2cam ( const cv::Point2f& p, const double d, const cv::Mat& K )
{
    return Eigen::Vector3d
            (
                    ( p.x - K.at<double>(0,2) ) / K.at<double>(0,0),
                    ( p.y - K.at<double>(1,2) ) / K.at<double>(1,1),
                    d
            );
}

inline bool isRotationMatrix(const cv::Mat& R)
{
    cv::Mat Rt;
    cv::transpose(R, Rt);
    cv::Mat ShouldBeIdentity = Rt*R;
    cv::Mat I = cv::Mat::eye(3,3, ShouldBeIdentity.type());
    return  cv::norm(I, ShouldBeIdentity) < 1e-6;
}

void convert_cv2eigen(const cv::Mat& R,
                      const cv::Mat& t,
                      Eigen::Isometry3d& T)
{
    Eigen::Matrix3d rotation_matrix;
    Eigen::Vector3d translate_vec;
    cv2eigen(R, rotation_matrix);
    cv2eigen(t, translate_vec);
    Eigen::Quaterniond q = Eigen::Quaterniond(rotation_matrix);

// Update the pose with Isometry
    //todo: check rotate or prerotate
    T.rotate(rotation_matrix);
    T.pretranslate(translate_vec);
}







#endif //CATKIN_WS_NDT_UTILITY_H
