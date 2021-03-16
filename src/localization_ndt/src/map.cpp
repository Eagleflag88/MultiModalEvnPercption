//
// Created by eagleflag on 2020/8/18.
//

#include <iostream>
#include <assert.h>
#include "map.h"
#include "frame.h"
#include "mappoint.h"
#include "utility.h"

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include "g2o_directedge.h"
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <fast_pcl/ndt_gpu/NormalDistributionsTransform.h>


map::map()
{
    std::cout << "instance of map is constructed" << std::endl;
};

map::~map()
{
    std::cout << "instance of map is destructed" << std::endl;
};

void map::add_keyimgframe(const frame::ptr newframe)
{

    std::cout << "size of the key frame before insertion are " << keyframes_.size() << std::endl;
    if (keyframes_.empty()) // 当map为空
    {
        keyframes_.insert(std::make_pair(newframe->id_, newframe));//插入新的一帧
        curr_frame_ = newframe;// 把curr frame赋值为最新的一帧
        std::cout << "size of the key frame after insertion are " << keyframes_.size() << std::endl;
        return;
    }
    //当map不为空时
    // Update the curr and pre frame of the map instance
    auto riter = keyframes_.rbegin(); //指向最后一个元素
    pre_frame_ = riter->second; // 赋值pre_frame
    curr_frame_ = newframe; //赋值new_frame
    std::cout << "key point and kps2f of the pre frame is " << pre_frame_->keypoints_.size() << " " << pre_frame_->kps2f_.size() << std::endl;

    keyframes_.insert(std::make_pair(newframe->id_, newframe)); // insert the pair
    std::cout << "size of the key frame after insertion are " << keyframes_.size() << std::endl;
}


void map::pose_estimation_2d2d(const frame::ptr pre_frame,
                               const frame::ptr curr_frame,
                               cv::Mat& R_opt_ini,
                               cv::Mat& t_opt_ini)
{
    std::cout << "2D2D Estimation Started"<< std::endl;
    //-- 计算本质矩阵, default is RANSAC, the t vector is already normalized to 1
    cv::Mat essential_matrix;
    essential_matrix = cv::findEssentialMat(pre_frame->kps2f_, curr_frame->kps2f_, K);
    std::cout << "essential_matrix is "<< std::endl << essential_matrix << std::endl;

    //-- 从本质矩阵中恢复旋转和平移信息.
    // todo: check if normalization of t is done
    cv::recoverPose(essential_matrix, pre_frame->kps2f_, curr_frame->kps2f_, K, R_opt_ini, t_opt_ini);
    std::cout << "R is " << std::endl << R_opt_ini << std::endl;
    std::cout << "t is " << std::endl << t_opt_ini << std::endl;
    std::cout << "mode of t is " << cv::norm(t_opt_ini) << std::endl;
    std::cout << "2D2D Estimation Finished"<< std::endl;
}

void map::triangulation(
        const cv::Mat& R,
        const cv::Mat& t)
{
    std::cout << "Triangulation Started"<< std::endl;
    cv::Mat P1 = (cv::Mat_<double> (3,4) <<
                                         1,0,0,0,
            0,1,0,0,
            0,0,1,0);

    cv::Mat P2 = (cv::Mat_<double> (3,4) <<
                                         R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0,0),
            R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1,0),
            R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2,0));

    std::cout << "R is " << std::endl << R << std::endl;
    std::cout << "t is " << std::endl << t << std::endl;

    std::vector<cv::Point2f> kps2f_pre, kps2f_curr;
    std::vector<cv::Point2f> pts_1, pts_2;

    kps2f_pre = pre_frame_->kps2f_;
    kps2f_curr = curr_frame_->kps2f_;

    for (int i = 0; i < kps2f_pre.size(); i++)
    {
        // 将像素坐标转换至相机坐标/实际上是归一化平面上的坐标
        pts_1.push_back ( pixel2cam( kps2f_pre[i], K) );
        pts_2.push_back ( pixel2cam( kps2f_curr[i], K) );
    }

    cv::Mat pts_4d; // pts_4d是在P1坐标系下的landmark的非齐次坐标
    cv::triangulatePoints(P1, P2, pts_1, pts_2, pts_4d);

    // 转换成非齐次坐标
    // todo: 为什么要改成齐次
    std::vector<unsigned char> status;
    std::vector<cv::Point3d> points_landmark;
    for ( int i=0; i<pts_4d.cols; i++ )
    {
        cv::Mat x = pts_4d.col(i);
        x /= x.at<float>(3,0); // 归一化
        cv::Point3d p (
                x.at<float>(0,0),
                x.at<float>(1,0),
                x.at<float>(2,0)
        );
//        if(p.z < 0) // when the depth is negative
//        {
//            status.push_back(0); // set the status to 0
//        }
//        else
//        {
//            status.push_back(1); // set the status to 1
//        }
        points_landmark.push_back( p );
    }

    std::cout << "points_landmark size is " << points_landmark.size() << std::endl;
    pre_frame_->landmark_points_ = points_landmark; // landmark points are in the frame of the first camera
    curr_frame_->landmark_points_ = points_landmark;
    curr_frame_->transform_landmark(R, t); // Update the landmark points into the frame of the second camera

    std::cout << "triangulated feature points are "<< std::endl << curr_frame_->landmark_points_[0]
              << std::endl << curr_frame_->landmark_points_[10]
              << std::endl << curr_frame_->landmark_points_[30] << std::endl;
    // todo: how to update landmark point, keypoint and descriptors according to the status
    std::cout << "Triangulation Finished"<< std::endl;
}

void map::ba_3d2d_optimization(const frame::ptr pre_frame,
                               const frame::ptr curr_frame,
                               const cv::Mat& R_ini,
                               const cv::Mat& t_ini,
                               cv::Mat& R_opt,
                               cv::Mat& t_opt,
                               std::vector<cv::Point3d>& points_3d_opt)
{
    std::cout << "ba_g2o_3D2D_optimization started"<< std::endl;
    // 初始化g2o
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // pose 维度为 6, landmark 维度为 3
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>(); // 线性方程求解器
    Block* solver_ptr = new Block( std::unique_ptr<Block::LinearSolverType>(linearSolver) );// 矩阵块求解器// 矩阵块求解器
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<Block>(solver_ptr) );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );

    // vertex
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
    Eigen::Matrix3d R_mat;
    R_mat <<
            R_ini.at<double> ( 0,0 ), R_ini.at<double> ( 0,1 ), R_ini.at<double> ( 0,2 ),
            R_ini.at<double> ( 1,0 ), R_ini.at<double> ( 1,1 ), R_ini.at<double> ( 1,2 ),
            R_ini.at<double> ( 2,0 ), R_ini.at<double> ( 2,1 ), R_ini.at<double> ( 2,2 );

    pose->setId ( 0 );
    pose->setEstimate ( g2o::SE3Quat (
            R_mat,
            Eigen::Vector3d ( t_ini.at<double> ( 0,0 ), t_ini.at<double> ( 1,0 ), t_ini.at<double> ( 2,0 ) )
    ) );
    optimizer.addVertex (pose);

    std::vector<cv::Point3d> points_landmark_pre = pre_frame->landmark_points_;
    int index = 1;
    for ( const cv::Point3d p:points_landmark_pre)   // landmarks
    {
        g2o::VertexSBAPointXYZ* point = new g2o::VertexSBAPointXYZ();
        point->setId (index++);
        point->setEstimate ( Eigen::Vector3d (p.x, p.y, p.z) );
        point->setMarginalized ( true ); // g2o 中必须设置 marg 参见第十讲内容
        optimizer.addVertex (point);
    }

    // parameter: camera intrinsics
    g2o::CameraParameters* camera = new g2o::CameraParameters (
            K.at<double> ( 0,0 ), Eigen::Vector2d ( K.at<double> ( 0,2 ), K.at<double> ( 1,2 ) ), 0
    );
    camera->setId ( 0 );
    optimizer.addParameter (camera);

    std::vector<cv::Point2f> points_2f_curr = curr_frame->kps2f_;
    index = 1;
    for ( const cv::Point2f p:points_2f_curr )
    {
        g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
        edge->setId ( index );
        edge->setVertex ( 0, dynamic_cast<g2o::VertexSBAPointXYZ*> ( optimizer.vertex ( index ) ) );
        edge->setVertex ( 1, pose );
        edge->setMeasurement ( Eigen::Vector2d ( p.x, p.y ) );
        edge->setParameterId ( 0,0 );
        edge->setInformation ( Eigen::Matrix2d::Identity() );
        optimizer.addEdge ( edge );
        index++;
    }

    optimizer.setVerbose ( true );
    optimizer.initializeOptimization();
    optimizer.optimize ( 100 );

    std::cout<<std::endl<<"after optimization using g2o:"<<std::endl;
    std::cout<<"T="<<std::endl<<Eigen::Isometry3d ( pose->estimate() ).matrix() <<std::endl;
    Eigen::Matrix4d T_mat = Eigen::Isometry3d ( pose->estimate() ).matrix();
    R_opt = (cv::Mat_<double> (3, 3) <<
                                     T_mat(0, 0), T_mat(0, 1), T_mat(0, 2),
            T_mat(1, 0), T_mat(1, 1), T_mat(1, 2),
            T_mat(2, 0), T_mat(2, 1), T_mat(2, 2));
    t_opt = (cv::Mat_<double> (3, 1) << T_mat(0, 3), T_mat(1, 3), T_mat(2, 3));

    Eigen::Vector3d point_3d_opt;
    cv::Point3d point_3d_opt_cv;
    index = 1;
    for ( const cv::Point3d p:points_landmark_pre )   // landmarks
    {
        point_3d_opt = dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(index))->estimate();
        point_3d_opt_cv.x = point_3d_opt(0);
        point_3d_opt_cv.y = point_3d_opt(1);
        point_3d_opt_cv.z = point_3d_opt(2);
        points_3d_opt.push_back(point_3d_opt_cv);
        index++;
    }

    //todo: update the landmark in frame or map?
    std::cout << "ba_g2o_3D2D_optimization finished"<< std::endl;
}

void map::pose_estimation_pnp(const frame::ptr pre_frame,
                              const frame::ptr curr_frame,
                              cv::Mat& R,
                              cv::Mat& t)
{
    std::cout << "pose_estimation_pnp started "<< std::endl;
    std::vector<cv::Point3d> points_landmark_pre = pre_frame->landmark_points_;
    std::vector<cv::Point2f> kps2f_curr = curr_frame->kps2f_;
    cv::Mat r;

    std::cout <<"size of kps2f and landmark points are "
              << std::endl << points_landmark_pre.size()
              << std::endl << kps2f_curr.size() << std::endl;

    // points_3d is the landmark point in the first camera frame
    // todo: use ransac pnp
    cv::solvePnP(points_landmark_pre, kps2f_curr, K, cv::Mat(), r, t, false ); // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法
    cv::Rodrigues ( r, R ); // r为旋转向量形式，用Rodrigues公式转换为矩阵

    std::cout<<"R_pnp="<<std::endl<<R<<std::endl;
    std::cout<<"t_pnp="<<std::endl<<t<<std::endl;
    std::cout << "pose_estimation_pnp finished "<< std::endl;
}

void map::feature_matches_2d2d()
{
    std::cout << "feature_matches_2d2d started "<< std::endl;
    cv::Mat img_1, img_2;
    img_1 = pre_frame_->image_;
    img_2 = curr_frame_->image_;
//    std::cout << "pre frame " << img_1 << std::endl;
//    std::cout << "current frame " << img_2 << std::endl;

    //-- 初始化
    cv::Mat descriptors_1, descriptors_2;
    // used in OpenCV3
    // todo: Consider using SIFT Features
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();

    cv::Ptr<cv::DescriptorMatcher> matcher  = cv::DescriptorMatcher::create ( "BruteForce-Hamming" );
    //-- 第一步:检测 Oriented FAST 角点位置
    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    std::vector<cv::DMatch> match;
    //BFMatcher matcher ( NORM_HAMMING );
    matcher->match ( descriptors_1, descriptors_2, match );

    // Draw the image with matches
    cv::Mat img_match;
    std::cout<<"Size of the keypoints, pre and curr are " << keypoints_1.size() <<" "<< keypoints_2.size() << " " << match.size() << std::endl;
    cv::drawMatches(img_1, keypoints_1, img_2, keypoints_2, match, img_match);
//    cv::imshow ( "匹配点对", img_match );
//    cv::waitKey(0);

    //-- 第四步:匹配点对筛选
    double min_dist=10000, max_dist=0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = match[i].distance;
//        std::cout<<"distance of the matches is " << dist << std::endl;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
//        std::cout<<"min distance of the matches is " << min_dist << std::endl;
//        std::cout<<"max distance of the matches is " << max_dist << std::endl;
    }

    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );


    std::vector<cv::Point2f> kps2f_pre, kps2f_curr;
    std::vector<cv::KeyPoint> keypoints_pre, keypoints_curr;
    std::vector<cv::DMatch> good_matches;
    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    // 同时更新匹配和特征点
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( match[i].distance <= std::max ( 2*min_dist, 30.0 ) )
        {
            good_matches.push_back ( match[i] );
            keypoints_pre.push_back (keypoints_1[match[i].queryIdx]); // pixel level->cv::Keypoint
            keypoints_curr.push_back (keypoints_2[match[i].trainIdx]);
            kps2f_pre.push_back (keypoints_1[match[i].queryIdx].pt); // also pixel level, but cv::Point2f
            kps2f_curr.push_back (keypoints_2[match[i].trainIdx].pt);
        }
    }

    cv::drawMatches(img_1, keypoints_1, img_2, keypoints_2, good_matches, img_match);
//    img_store(img_match, 0);
    // 重新根据筛选后的角点位置计算 BRIEF 描述子
    cv::Mat descriptors_pre, descriptors_curr;
    descriptor->compute ( img_1, keypoints_pre, descriptors_pre );
    descriptor->compute ( img_2, keypoints_curr, descriptors_curr );

    std::cout << "number of keypoints and descriptors are " << kps2f_pre.size() << " " << descriptors_pre.rows << std::endl;
//    cv::imshow ( "优化后匹配点对", img_match );
//    cv::waitKey(0);

    // Update the relevant frame
    pre_frame_->kps2f_ = kps2f_pre;
    curr_frame_->kps2f_ = kps2f_curr;
    pre_frame_->keypoints_ = keypoints_pre;
    curr_frame_->keypoints_ = keypoints_curr;
    pre_frame_->descriptors_ = descriptors_pre;
    curr_frame_->descriptors_ = descriptors_curr;

    std::cout<<"一共找到了"<< pre_frame_->keypoints_.size() <<"组匹配点"<<std::endl;
    std::cout << "feature_matches_2d2d finished "<< std::endl;

}

void map::feature_matches_2d2d(frame::ptr& left_frame,
                               frame::ptr& right_frame)
{
    std::cout << "feature_matches_2d2d started "<< std::endl;
    cv::Mat img_1, img_2;
    img_1 = left_frame->image_;
    img_2 = right_frame->image_;

    //-- 初始化
    cv::Mat descriptors_1, descriptors_2;
    // used in OpenCV3
    // todo: Consider using SIFT Features
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();

    cv::Ptr<cv::DescriptorMatcher> matcher  = cv::DescriptorMatcher::create ( "BruteForce-Hamming" );
    //-- 第一步:检测 Oriented FAST 角点位置
    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    std::vector<cv::DMatch> match;
    //BFMatcher matcher ( NORM_HAMMING );
    matcher->match ( descriptors_1, descriptors_2, match );

    //-- 第四步:匹配点对筛选
    double min_dist=10000, max_dist=0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = match[i].distance;
//        std::cout<<"distance of the matches is " << dist << std::endl;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
//        std::cout<<"min distance of the matches is " << min_dist << std::endl;
//        std::cout<<"max distance of the matches is " << max_dist << std::endl;
    }

    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );

    std::vector<cv::Point2f> kps2f_left, kps2f_right;
    std::vector<cv::KeyPoint> keypoints_left, keypoints_right;
    std::vector<cv::DMatch> good_matches;
    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    // 同时更新匹配和特征点
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( match[i].distance <= std::max ( 2*min_dist, 30.0 ) )
        {
            good_matches.push_back ( match[i] );
            keypoints_left.push_back (keypoints_1[match[i].queryIdx]); // pixel level->cv::Keypoint
            keypoints_right.push_back (keypoints_2[match[i].trainIdx]);
            kps2f_left.push_back (keypoints_1[match[i].queryIdx].pt); // also pixel level, but cv::Point2f
            kps2f_right.push_back (keypoints_2[match[i].trainIdx].pt);
        }
    }

    // 重新根据筛选后的角点位置计算 BRIEF 描述子
    cv::Mat descriptors_left, descriptors_right;
    descriptor->compute(img_1, keypoints_left, descriptors_left);
    descriptor->compute(img_2, keypoints_right, descriptors_right);

//// Draw the image with matches
//    cv::Mat img_match;
//    std::cout<<"Size of the keypoints, left and right are " << keypoints_1.size() <<" "<< keypoints_2.size() << " " << good_matches.size() << std::endl;
//    cv::drawMatches(img_1, keypoints_1, img_2, keypoints_2, good_matches, img_match);
//    cv::imshow ( "匹配点对", img_match );
//    cv::waitKey(0);

    std::cout << "number of keypoints and descriptors are " << kps2f_left.size() << " " << descriptors_left.rows << std::endl;

    // Update the key point of the relevant frame
    left_frame->kps2f_ = kps2f_left;
    right_frame->kps2f_ = kps2f_right;
    left_frame->descriptors_ = descriptors_left;
    right_frame->descriptors_ = descriptors_right;
    left_frame->keypoints_ = keypoints_left;
    right_frame->keypoints_ = keypoints_right;

    std::cout<<"一共找到了"<< left_frame->kps2f_.size() <<"组匹配点"<<std::endl;

    std::vector<double> depths;
    for(int i = 0; i < left_frame->kps2f_.size(); i++)
    {
        // 发现所有的disparity都是正的，有可能是因为kitti的图片已经整备过(Stereo Rectification)的原因
        double disparity = left_frame->kps2f_[i].x - right_frame->kps2f_[i].x; // Disparity, 14讲 5.16
//        std::cout << "Disparity is " << disparity << std::endl;
        // todo: check the unit of depth, m or pixel?
        // todo: 深度在stereo的相机setting下对两个相机来说应该是一样大的吗？
        double depth = fx*b/disparity;
        depths.push_back(depth); // Depth, 14讲 5.16
        mappoint::ptr mappoint_ptr = mappoint_ptr->create();
        //todo：这样创建一个三维的点合理吗，查看函数具体实现
        mappoint_ptr->pos_ = pixel2cam(left_frame->kps2f_[i], depth, K); // In the coordinate of the first camera
        mappoint_ptr->num_observed_ = 2;
        mappoint_ptr->kp2f_ = left_frame->kps2f_[i];
        mappoint_ptr->descriptor_ = left_frame->descriptors_.row(i);
        mappoint_ptr->keypoint_ = left_frame->keypoints_[i];
        left_frame->mappoints_.push_back(mappoint_ptr);
    }
    std::cout << "Size of the map point in left_frame is " << left_frame->mappoints_.size() << std::endl;

    std::cout << "feature_matches_2d2d finished "<< std::endl;
}

void map::lk_track(std::vector<unsigned char>& status)
{
    std::cout << "lk_track started "<< std::endl;
    std::vector<float> error;
    std::vector <cv::Point2f> kps2f_curr;
    cv::calcOpticalFlowPyrLK(pre_frame_->image_, curr_frame_->image_, pre_frame_->kps2f_, kps2f_curr, status, error);
    curr_frame_->kps2f_ = kps2f_curr;
    std::cout << "size of kps2f and landmark points in pre frame are " << pre_frame_->kps2f_.size() << " " << pre_frame_->landmark_points_.size() << std::endl;
    std::cout << "size of kps2f and landmark points in curr frame are " << curr_frame_->kps2f_.size() << " " << curr_frame_->landmark_points_.size() << std::endl;

    std::cout << "lk_track finished "<< std::endl;
}

void map::update_lcampose(
        const cv::Mat& R,
        const cv::Mat& t)
{

    std::cout << "pose before update is " << std::endl << (pre_frame_->T_cw_).matrix() << std::endl;
    // update the pose of the camera //////////////////////////////////////////////////////////////////////////////////////////
    Eigen::Isometry3d pose_diff = Eigen::Isometry3d::Identity();
    convert_cv2eigen(R, t, pose_diff);
    // Update the pose
    curr_frame_->T_cw_ = pre_frame_->T_cw_*pose_diff;
    std::cout << "pose after update is " << std::endl << (curr_frame_->T_cw_).matrix() << std::endl;

}

void map::show_eulerangles(const cv::Mat& R)
{
    assert(isRotationMatrix(R));
    float sy = sqrt(R.at<double>(0, 0)*R.at<double>(0, 0) + R.at<double>(1, 0)*R.at<double>(1, 0));
    bool singular = sy < 1e-6;
    float x, y, z; // yaw, pitch, roll
    if(!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    std::cout << "yaw is " << 180.0*x/3.1415 << std::endl
              << "pitch is " << 180.0*y/3.1415 << std::endl
              << "roll is " << 180.0*z/3.1415 << std::endl;
    return;
}

void map::update_lastimgframe()
{
    // 因为经过了特征点抽取，消减，位姿计算，frame的相关成员变量跟初始化时已经不同，所以需要更新把curr pre frame里面的内容更新到keyframe的队列里面
    auto riter = keyframes_.rbegin();
    riter->second = curr_frame_;
    std::cout << "size of the last kps2f are "<< riter->second->kps2f_.size() << std::endl;
    std::cout << "last frame updated "<< std::endl;
}

void map::update_lastimgpair()
{
    // 因为经过了特征点抽取，消减，位姿计算，frame的相关成员变量跟初始化时已经不同，所以需要更新把curr pre frame里面的内容更新到keyframe的队列里面
    auto riter = keyframes_.rbegin();
    riter->second = curr_frame_;
    std::cout << "size of the last kps2f are "<< riter->second->kps2f_.size() << std::endl;
    ++riter;
    riter->second = pre_frame_;
    std::cout << "size of the last - 1 kps2f are "<< riter->second->kps2f_.size() << std::endl;
    std::cout << "last pair updated "<< std::endl;
}

void map::crop_pairframe(const std::vector<unsigned char>& status)
{
    std::cout << "start map cropping "<< std::endl;
    // For Curr Frame, crop the kps2f according to the status
    std::cout << "Size of the kps2f and landmark point in curr frame before crop are "
              << " " << curr_frame_->kps2f_.size()
              << " " << curr_frame_->landmark_points_.size()
              << std::endl;

    std::vector <cv::Point2f> kps2f_curr_tmp = {};
    for (int i = 0; i < status.size(); i++)
    {
        if (status[i] == 1) {
            kps2f_curr_tmp.push_back(curr_frame_->kps2f_[i]);
        }
    }
    curr_frame_->kps2f_.swap(kps2f_curr_tmp);

    std::cout << "Size of the kps2f and landmark point in curr frame after crop are "
              << " " << curr_frame_->kps2f_.size()
              << " " << curr_frame_->landmark_points_.size()
              << std::endl;

    // For Pre Frame, crop the kps2f and landmark points according to the status
    std::cout << "Size of the kps2f and landmark point in pre frame before crop are "
              << " " << pre_frame_->kps2f_.size()
              << " " << pre_frame_->landmark_points_.size()
              << std::endl;

    std::vector <cv::Point2f> kps2f_pre_tmp = {};
    for (int i = 0; i < status.size(); i++)
    {
        if (status[i] == 1) {
            kps2f_pre_tmp.push_back(pre_frame_->kps2f_[i]);
        }
    }
    pre_frame_->kps2f_.swap(kps2f_pre_tmp);

    std::cout << "Size of the kps2f in pre frame after crop are "
              << " " << pre_frame_->kps2f_.size() << std::endl;

    std::vector <cv::Point3d> points_landmark_pre_tmp = {};
    for (int i = 0; i < status.size(); i++)
    {
        if (status[i] == 1) {
            points_landmark_pre_tmp.push_back(pre_frame_->landmark_points_[i]);
        }
    }
    pre_frame_->landmark_points_.swap(points_landmark_pre_tmp);

    std::cout << "Size of the landmark point in pre frame after crop are "
              << " " << pre_frame_->landmark_points_.size()
              << std::endl;

    std::cout << "map cropping finished "<< std::endl;
}

// For pc_frame
void map::add_keypcframe(const pc_frame::ptr newpcframe)
{
    std::cout << "Start KeyPcFrame Adding " << std::endl;

    std::cout << "size of the key pc frame before insertion are " << keypcframes_.size() << std::endl;
    if (keypcframes_.empty()) // 当keypcframes为空
    {
        keypcframes_.insert(std::make_pair(newpcframe->id_, newpcframe));//插入新的一帧
        curr_pc_frame_ = newpcframe;// 把curr pc_frame赋值为最新的一帧，此时pre_pc_frame_为空
        std::cout << "size of the key pc_frame after insertion are " << keypcframes_.size() << std::endl;
        return;
    }

    //当keypcframes不为空时
    // Update the curr and pre pc_frame of the map instance
    auto riter = keypcframes_.rbegin(); //指向最后一个元素
    std::cout << "id of the last pc frame is " << riter->first << std::endl;
    pre_pc_frame_ = riter->second; // 把最后一个元素赋值到pre_pc_frame
    curr_pc_frame_ = newpcframe; //把新帧赋值到curr_pc_frame

    std::cout << "number of point of the pre pc_frame is " << pre_pc_frame_->scan_->width << std::endl;
    std::cout << "number of point of the curr pc_frame is " << curr_pc_frame_->scan_->width << std::endl;

    keypcframes_.insert(std::make_pair(newpcframe->id_, newpcframe)); // insert the pair
    std::cout << "size of the key pc_frame after insertion are " << keypcframes_.size() << std::endl;

    std::cout << "Finish KeyPcFrame Adding " << std::endl;
}

void map::update_localpcmap()
{
    std::cout << "Start updating local pc frame " << std::endl;

    // Executing the transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud = curr_pc_frame_->scan_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>());

    Eigen::Matrix4f Transf = curr_pc_frame_->T_cw_.matrix().cast<float>(); //必须是float类型
    pcl::transformPointCloud(*source_cloud, *transformed_cloud, Transf.inverse()); // 此时的T_cw_应该是原点的位姿
    // Concanetation of the point cloud
    if(!local_pc_map_) // 如果为空
    {
        std::cout << "empty local map " << std::endl;
//        pcl::copyPointCloud(*transformed_cloud, *local_pc_map_); //using transformed_cloud to update local_pc_map
        local_pc_map_ = transformed_cloud;
    }
    else
    {
        std::cout << "non-empty local map " << std::endl;
        *local_pc_map_ += *(transformed_cloud);
    }

    std::cout << "After Concactation the local pc map has " << local_pc_map_->width << " Point" << std::endl;

    std::cout << "Finish updating local pc frame " << std::endl;
}


void map::register_keypcframes(Eigen::Isometry3d& pose_lidar_diff)
{
    //todo: 用imu计算initial guess
    std::cout << "Start Registration of the Lidar Scans" << std::endl;

    std::cout << "Size of the previous Scan is " << pre_pc_frame_->scan_->width << std::endl;
    std::cout << "Size of the current Scan is " << curr_pc_frame_->scan_->width << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_scan_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

//    // Using Normal NDT
//    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
//    ndt.setTransformationEpsilon(0.01);
//    ndt.setStepSize(0.1);
//    ndt.setResolution(1.0);
//    ndt.setMaximumIterations(35);
//    ndt.setInputCloud(curr_pc_frame_->scan_);
//    ndt.setInputTarget(pre_pc_frame_->scan_);
////    ndt.setInputTarget(local_pc_map_);
//    auto t1 = ros::WallTime::now();
//    ndt.align(*aligned_scan_cloud_ptr);
//    auto t2 = ros::WallTime::now();
//    std::cout << "Duration : " << (t2 - t1).toSec() * 1000 << "[msec]" << std::endl;
//
//    std::cout << "Normal Distributions Transform has converged after " << ndt.getFinalNumIteration() << " iters" << std::endl;
//    std::cout << "The score is " << ndt.getFitnessScore() << "\n" << std::endl;
//    Eigen::Matrix4f Transf = Eigen::Matrix4f::Identity();
//    Transf = ndt.getFinalTransformation();
//    std::cout << "Final Transformation Matrix of the Scan Registration is " << Transf << std::endl;

    //Get transform between two frames Using ndt_omp ////////////////////////////////////////////////////////////////////
    pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
    ndt_omp->setInputSource(curr_pc_frame_->scan_);
//    ndt_omp->setInputTarget(local_pc_map_);
    ndt_omp->setInputTarget(pre_pc_frame_->scan_);
    ndt_omp->setTransformationEpsilon (epsilon);
    ndt_omp->setResolution (res);
    ndt_omp->setStepSize (step);
    ndt_omp->setMaximumIterations(100);

    ndt_omp->setNumThreads(NumThreads);
    ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    // get the transform between two poses as guess
    std::cout << "Preparation for Scan Registration Finished" << std::endl;
    auto t1 = ros::WallTime::now();
    ndt_omp->align(*aligned_scan_cloud_ptr);
    auto t2 = ros::WallTime::now();
    std::cout << "Duration : " << (t2 - t1).toSec() * 1000 << "[msec]" << std::endl;

    std::cout << "Normal Distributions Transform has converged after " << ndt_omp->getFinalNumIteration() << " iters" << std::endl;
    std::cout << "The score is " << ndt_omp->getFitnessScore() << std::endl;

    Eigen::Matrix4f Transf = Eigen::Matrix4f::Identity();
    Transf = ndt_omp->getFinalTransformation();

//    // Using GPU normal//////////////////////////////////////////////////////////////////////////////////////////
//    gpu::GNormalDistributionsTransform g_ndt;
//    g_ndt.setTransformationEpsilon (epsilon);
//    g_ndt.setStepSize (step);
//    g_ndt.setResolution (res);
//    // Setting max number of registration iterations.
//    g_ndt.setMaximumIterations (35);
//
//    g_ndt.setInputSource (curr_pc_frame_->scan_);
//    g_ndt.setInputTarget (pre_pc_frame_->scan_);
//    Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
//    auto t1 = ros::WallTime::now();
//    g_ndt.align(init_guess);
//    auto t2 = ros::WallTime::now();
//    std::cout << "Duration : " << (t2 - t1).toSec() * 1000 << "[msec]" << std::endl;
//
//    Eigen::Matrix4f Transf = Eigen::Matrix4f::Identity();
//    Transf = g_ndt.getFinalTransformation();
//    bool converged = g_ndt.hasConverged();
//    double fitness_score = g_ndt.getFitnessScore();

    // Output ////////////////////////////////////////////////////////////

    std::cout << "Final Transformation Matrix of the Scan Registation is " << std::endl << Transf << std::endl;
    cv::Mat t = (cv::Mat_<double>(3,1) << Transf(0, 3), Transf(1, 3), Transf(2, 3));
    std::cout << "Mode of the translation is " << std::endl << cv::norm(t) << std::endl;
    pose_lidar_diff.matrix() = Transf.cast<double>();

    std::cout << "Finish Registration of the Lidar Scans" << std::endl;
}

void map::update_lidarpose(const Eigen::Isometry3d pose_lidar_diff)
{
    std::cout << "lidar pose before update is " << std::endl << (pre_pc_frame_->T_cw_).matrix() << std::endl;
    // update the pose of the lidar
    //右乘！！！！！！！！！！！！！！！！
    curr_pc_frame_->T_cw_ = (pre_pc_frame_->T_cw_)*pose_lidar_diff;
//    curr_pc_frame_->T_cw_ = pose_lidar_diff;
    std::cout << "lidar pose after update is " << std::endl << (curr_pc_frame_->T_cw_).matrix() << std::endl;
}

void map::update_lastpcframe()
{
    // 因为经过了位姿计算，pcframe的相关成员变量跟初始化时已经不同，所以需要更新把curr pre frame里面的内容更新到keyframe的队列里面
    auto riter = keypcframes_.rbegin();
    riter->second = curr_pc_frame_;
    std::cout << "lidar pose of last frame is "<< std::endl << riter->second->T_cw_.matrix() << std::endl;
    std::cout << "number of the point in lidar scan is "<< riter->second->scan_->width << std::endl;
    std::cout << "last lidar frame updated "<< std::endl;
}

Eigen::Vector3d map::get_vel()
{
    Eigen::Vector3d vel = Eigen::Vector3d(0.0, 0.0, 0.0);

    auto riter_last = keypcframes_.rbegin(); //指向最后一个元素
    unsigned long curr_id = riter_last->first;
    std::cout << "id of the current pc frame is " << curr_id << std::endl;

    if(curr_id >= 2) // 从第三帧起,根据倒数第二帧和倒数第三帧计算速度
    {
        auto riter_last1 = std::next(riter_last); //倒数第二帧
        auto riter_last2 = std::next(riter_last1); //倒数第三帧
        std::cout << "id of the frames for calculation are " << riter_last->first << " and " << riter_last1->first << " and " << riter_last2->first << std::endl;
        vel = (riter_last1->second->T_cw_.translation() - riter_last2->second->T_cw_.translation())*10;
        std::cout << "velocity in 3d are x " << vel(0) << " y is " << vel(1) << " z is " << vel(2) << std::endl;
//        std::cout << "curr transform is " << std::endl <<
//        riter_last1->second->T_cw_.matrix() << std::endl <<
//        " previous transform is  " << std::endl <<
//        riter_last2->second->T_cw_.matrix() << std::endl;
    }
    return vel;
}

void map::update_imupose() //在插入最新的imu_frame之后调用
{
    auto curr_iter = imu_frames_.begin(); //指向第一个元素
    unsigned long curr_id = (*curr_iter)->id_;
    std::cout << "id of the current imu frame is " << curr_id << std::endl;

    if(curr_id >= 1 && imu_initialized) //至少从第二帧起,而且必须已经利用GNSS的测量值初始化过之后// 根据最新帧和次新帧计算
    {
        auto pre_iter = std::next(curr_iter); //次新帧
        std::cout << "id of the frames for calculation are " << (*pre_iter)->id_ << " and " << (*curr_iter)->id_ << std::endl;

        double delta_time = (*curr_iter)->frame_.time - (*pre_iter)->frame_.time;
        std::cout << "delta timestamp is " << std::endl << delta_time << std::endl;

        // Using Cosine Direct Matrix (rotation matrix) Update -- According to An introduction to inertial navigation (eq. 35)

        // Get the sigma of the rotation: An introduction to inertial navigation (between eq. 37 and Eq. 38)
        Eigen::Vector3d pre_omega_body; // measurement
        pre_omega_body << (*pre_iter)->frame_.angular_velocity_x, (*pre_iter)->frame_.angular_velocity_y, (*pre_iter)->frame_.angular_velocity_z;
        Eigen::Vector3d curr_omega_body; // measurement
        curr_omega_body << (*curr_iter)->frame_.angular_velocity_x, (*curr_iter)->frame_.angular_velocity_y, (*curr_iter)->frame_.angular_velocity_z;
        Eigen::Vector3d delta_rotation = (pre_omega_body + curr_omega_body)*0.5*delta_time;
        double sigma_rotation = delta_rotation.norm();
//        std::cout << "Angular Velocity Measurement of pre frame are " << std::endl << pre_omega_body << std::endl;

        // Get the skew symmetric matrix of rotation difference during one sample period: An introduction to inertial navigation (eq. 37)
        double B00 = 0.0;
        double B01 = -delta_rotation(2); // -theta_z
        double B02 = delta_rotation(1); // theta_y

        double B10 = -B01;
        double B11 = 0.0;
        double B12 = -delta_rotation(0); // -theta_x

        double B20 = -B02;
        double B21 = -B12;
        double B22 = 0;

        Eigen::Matrix3d B_matrix; // An introduction to inertial navigation (eq. 32)
        B_matrix << B00, B01, B02, B10, B11, B12, B20, B21, B22;
//        std::cout << "skew matrix is " << std::endl << B_matrix << std::endl;

        // Update the rotation matrix Eq 41
        Eigen::Matrix3d pre_rotation_matrix, curr_rotation_matrix, diff_rotation_matrix;
        pre_rotation_matrix = (*pre_iter)->T_cw_.rotation();
        diff_rotation_matrix = Eigen::Matrix3d::Identity() + ((sin(sigma_rotation)/sigma_rotation)*B_matrix) + (((1.0 - cos(sigma_rotation))/(sigma_rotation*sigma_rotation))*B_matrix*B_matrix);
        // todo: 左乘还是右乘
        curr_rotation_matrix = pre_rotation_matrix*diff_rotation_matrix;

//        std::cout << "pre rotation matrix of imu_odom is " << std::endl << pre_rotation_matrix << std::endl
//                  << "current rotation matrix of imu_odom is " << std::endl << curr_rotation_matrix << std::endl;

        Eigen::Matrix3d mag_pre_rotation_matrix = Eigen::Quaterniond((*pre_iter)->frame_.orientation_w, (*pre_iter)->frame_.orientation_x, (*pre_iter)->frame_.orientation_y, (*pre_iter)->frame_.orientation_z).toRotationMatrix();
        Eigen::Matrix3d mag_curr_rotation_matrix = Eigen::Quaterniond((*curr_iter)->frame_.orientation_w, (*curr_iter)->frame_.orientation_x, (*curr_iter)->frame_.orientation_y, (*curr_iter)->frame_.orientation_z).toRotationMatrix();
//        std::cout << "mag pre rotation matrix of imu_odom is " << std::endl << mag_pre_rotation_matrix << std::endl
//                  << "mag current rotation matrix of imu_odom is " << std::endl << mag_curr_rotation_matrix << std::endl;

        // Update the velocity
        Eigen::Vector3d curr_accel_body; // measurement
        curr_accel_body << (*curr_iter)->frame_.linear_acceleration_x, (*curr_iter)->frame_.linear_acceleration_y, (*curr_iter)->frame_.linear_acceleration_z;
        Eigen::Vector3d curr_accel_global = curr_rotation_matrix*curr_accel_body;
        Eigen::Vector3d pre_accel_body; // measurement
        pre_accel_body << (*pre_iter)->frame_.linear_acceleration_x, (*pre_iter)->frame_.linear_acceleration_y, (*pre_iter)->frame_.linear_acceleration_z;
        Eigen::Vector3d pre_accel_global = pre_rotation_matrix*pre_accel_body;

        Eigen::Vector3d curr_vel_global = (*pre_iter)->vel_global_ + (curr_accel_global + accel_gravity + pre_accel_global + accel_gravity)*0.5*delta_time; // Eq45
//        std::cout << "current global accleration is " << std::endl << curr_accel_global << std::endl
//                  << "current global velocity is " << std::endl << curr_vel_global << std::endl;
        Eigen::Vector3d pre_vel_global = (*pre_iter)->vel_global_;

        // Update the position
        Eigen::Vector3d diff_position = (curr_vel_global + pre_vel_global)*0.5*delta_time;
        Eigen::Vector3d pre_position = (*pre_iter)->T_cw_.translation();
        Eigen::Vector3d curr_posistion = pre_position + diff_position; //Eq 46
//        std::cout << "pre position is " << std::endl << pre_position << std::endl
//                  << "current position is " << std::endl << curr_posistion << std::endl;

        // Update the current frame/////////////////////////////////////////////////////////////////////////////

//        double pose_qx = imu_frames_.front()->frame_.orientation_x;
//        double pose_qy = imu_frames_.front()->frame_.orientation_y;
//        double pose_qz = imu_frames_.front()->frame_.orientation_z;
//        double pose_qw = imu_frames_.front()->frame_.orientation_w;
//
//        Eigen::Quaterniond q = Eigen::Quaterniond(pose_qw, pose_qx, pose_qy, pose_qz); // Oder: w, x, y, z
//        q.normalize();
//        (*curr_iter)->T_cw_.prerotate(q);
//        (*curr_iter)->T_cw_.pretranslate(Eigen::Vector3d(gnss_frames_.front()->frame_.local_E, gnss_frames_.front()->frame_.local_N, gnss_frames_.front()->frame_.local_U));

        // rotation and translation

        std::cout << "Transform before update is "<< std::endl << (*curr_iter)->T_cw_.matrix() << std::endl;
        (*curr_iter)->rotation_matrix_ = curr_rotation_matrix;
        (*curr_iter)->T_cw_.rotate(curr_rotation_matrix).pretranslate(curr_posistion);
        // acceleration and velocity
        (*curr_iter)->vel_global_ = curr_vel_global;
        (*curr_iter)->accel_global_ = curr_accel_global;

        // Test for pretranslate and translate --> there is a difference but don't know why

//        Eigen::Isometry3d T_ar1 = (*curr_iter)->T_cw_.rotate(curr_rotation_matrix);
//        Eigen::Isometry3d T_ar2 = T_ar1;
//        std::cout << "Transform before translation after rotation is "<< std::endl << T_ar1.matrix() << " And " << std::endl << T_ar2.matrix() << std::endl;
//        Eigen::Isometry3d T_ar11 = T_ar1.pretranslate(curr_posistion);
//        Eigen::Isometry3d T_ar21 = T_ar2.translate(curr_posistion);
//        std::cout << "Transform after translation and after rotation is "<< std::endl << T_ar11.matrix() << " And " << std::endl
//                  << T_ar21.matrix() << std::endl;
//        std::cout << "Transform after update is "<< std::endl << (*curr_iter)->T_cw_.matrix() << std::endl;



//        // Test for transformation calculation using eigen library /////////////////////////////////////////////////
//
//        // Translation
//        Eigen::Isometry3d T_1 = Eigen::Isometry3d::Identity();
//        Eigen::Isometry3d T_2 = Eigen::Isometry3d::Identity();
//        std::cout << "Transform before translation is "<< std::endl << T_1.matrix() << std::endl;
//        Eigen::Vector3d delta_position = Eigen::Vector3d(1.0, 3.0, 2.0);
//        Eigen::Isometry3d T_tl1 = T_1.translate(delta_position);
//        Eigen::Isometry3d T_tl2 = T_2.pretranslate(delta_position);
//
//        std::cout << "Transform after translation 1 is "<< std::endl << T_tl1.matrix() << std::endl;
//        std::cout << "Transform after translation 2 is "<< std::endl << T_tl2.matrix() << std::endl;
//
//        T_1 = Eigen::Isometry3d::Identity();
//        T_2 = Eigen::Isometry3d::Identity();
//        std::cout << "Transform before rotation is "<< std::endl << T_1.matrix() << std::endl;
//        Eigen::Quaterniond delta_q = Eigen::Quaterniond(0.1, 0.2, 0.2, 0.15);
//        delta_q.normalize();
//        Eigen::Matrix3d delta_rotation_t = delta_q.toRotationMatrix();
//        Eigen::Isometry3d T_r1 = T_1.rotate(delta_rotation_t);
//        Eigen::Isometry3d T_r11 = T_1.rotate(delta_rotation_t);
//        Eigen::Isometry3d T_r2 = T_2.prerotate(delta_rotation_t);
//        Eigen::Isometry3d T_r22 = T_2.prerotate(delta_rotation_t);
//
//        std::cout << "Rotation is "<< std::endl << delta_rotation.matrix() << std::endl;
//        std::cout << "Transform after first rotation 1 is "<< std::endl << T_r1.matrix() << std::endl;
//        std::cout << "Transform after second rotation 1 is "<< std::endl << T_r11.matrix() << std::endl;
//        std::cout << "Transform after first rotation 2 is "<< std::endl << T_r2.matrix() << std::endl;
//        std::cout << "Transform after second rotation 2 is "<< std::endl << T_r22.matrix() << std::endl;

    }
}

void map::set_imu_origin(const gnss_frame::ptr gnss_frame_ptr) //
{
    auto curr_iter = imu_frames_.begin(); //指向最新的元素
    unsigned long curr_id = (*curr_iter)->id_;
    id_imu_ini_ = (*curr_iter)->id_;
    std::cout << "id of the imu frame at initialization is " << id_imu_ini_ << std::endl;
    imu_initialized = true;

    (*curr_iter)->T_cw_ = gnss_frame_ptr->T_cw_;
    (*curr_iter)->T_cw_.pretranslate(Eigen::Vector3d(0.0, 0.0, 0.0));
    std::cout << "imu pose at initialization is " << std::endl
              << (*curr_iter)->T_cw_.matrix() << std::endl;

}


// For gnss_frame
void map::add_keygnssframe(const gnss_frame::ptr new_gnssframe)
{
    std::cout << "size of the key gnss frame before insertion are " << gnss_frames_.size() << std::endl;
    gnss_frames_.push_front(new_gnssframe);
    std::cout << "size of the key gnss_frame after insertion are " << gnss_frames_.size() << std::endl;
}

// For Binocular Image Processing
bool map::stereoframesready()
{
    bool stereoframesready = false;
    if(stereo_frames_.size() <= 1)
    {
        return stereoframesready;
    }
    //获得最新帧和次新帧的iterator
    auto curr_iter = stereo_frames_.rbegin(); //最新帧
    auto pre_iter = std::next(curr_iter); //次新帧
//    std::cout << "Stereo Pair Organization " << std::endl;
//    std::cout << "ID of the curr and pre frame is " << (*curr_iter)->id_ << " " << (*pre_iter)->id_ << std::endl;
    double time_diff = (*curr_iter)->time_stamp_ - (*pre_iter)->time_stamp_;//查看两帧之间的时间差

    if((*curr_iter)->source_ == 0 && (*pre_iter)->source_ == 1) // 如果最新一帧是left image,而且次新帧为right image
    {
        if(time_diff < time_diff_binocular)//如果时间差小于某个阈值
        {
            stereo_pairs_.push_back((*pre_iter)); //先左后右，保证最后排布先左后右
            stereo_pairs_.push_back((*curr_iter));
            stereoframesready = true;
        }// 如果大于某个阈值，return false
    }

    if((*curr_iter)->source_ == 1 && (*pre_iter)->source_ == 0) // 如果最新一帧是right image,而且次新帧为left image
    {
        if(time_diff < time_diff_binocular)//如果时间差小于某个阈值
        {
            stereo_pairs_.push_back((*curr_iter)); //先右后左，保证最后排布先左后右
            stereo_pairs_.push_back((*pre_iter));
            stereoframesready = true;
        }// 如果大于某个阈值，return false
    }
    return stereoframesready;
}

void map::process_binoimg()
{
    std::cout << "Binocular Image Processing Started !!!!!!!!!!!!!!!!!!!!!!!!!!! " << std::endl;

    if(stereoframesready()) // 如果image pair都已经存在
    {
        //left_iter和rigth_iter组成一组stereo pair（左图和右图）
        auto left_iter = stereo_pairs_.rbegin(); //最新帧
        auto right_iter = std::next(left_iter); //次新帧
        std::cout << "ID of the curr and pre frame is " << (*left_iter)->id_ << " " << (*right_iter)->id_ << std::endl;

        // Initialization //////////////////////////////////////////////////////////////////////////////////
        if(!first_binmatch_finished_) //如果是第一次match
        {
            std::cout << "Initialization part of the binocular image processing" << std::endl;
            // Find the matches of point feature between stereo pair
            feature_matches_2d2d(*left_iter, *right_iter);
            // calulate the depths of the feature points and create the map point
            std::cout << "Size of the map point in left_frame is " << (*left_iter)->mappoints_.size() << std::endl;
            for(int i = 0; i < (*left_iter)->mappoints_.size(); i++)
            {
                mappoints_.push_back((*left_iter)->mappoints_[i]);
            }
            std::cout << "Size of the map point in map is " << mappoints_.size() << std::endl;
            (*left_iter)->T_cw_ = (gnss_frames_.front())->T_cw_;
            (*right_iter)->T_cw_ = (*left_iter)->T_cw_;
            first_binmatch_finished_ = true;

            return;
            // Then do nothing but to wait for next pair for inter-pair matching
        }

        // Update /////////////////////////////////////////////////////////////////////////////////////
        std::cout << "Updating part of the binocular image processing" << std::endl;
        // 来了新的一帧stereo pair，产生新的feature, create new map point and insert into the vector
        feature_matches_2d2d(*left_iter, *right_iter);
        // 找到新的stereo pair的feature points和旧的map points的对应关系->比较旧map点集和新pair的feature points的descriptor
        auto pre_left_iter = std::next(right_iter);
//        find_3d2dmatches(*left_iter, *right_iter);
//        find_3d2dmatches(*pre_left_iter, *left_iter);
        find_3d3dmatches(*pre_left_iter, *left_iter);
        (*left_iter)->pub_odom(bicam_odom_pub);
    }
    else// 如果没有准备好的话
    {
        std::cout << "Binocular Image Processing won't be executed " << std::endl;
    }

    std::cout << "Binocular Image Processing Finished !!!!!!!!!!!!!!!!!!!!!!!!!!! " << std::endl;
    return;
}

int map::compute_descriptordist(const cv::Mat &a, const cv::Mat &b)
{
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist=0;

    for(int i=0; i<8; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}



//void map::find_3d2dmatches(frame::ptr& left_frame, frame::ptr& right_frame)
//{
//    std::cout << "Find 3d2d Matches Started"<< std::endl;
//    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
//    int min_dist=10000, max_dist=0;
//    std::cout << "Size of the old map point and new map point are " << mappoints_.size() << " and " << left_frame->keypoints_.size() << std::endl;
//    std::vector<int> min_dists = {}; //每一个map点对应的最小的距离值
//    for(auto iter = mappoints_.begin(); iter != mappoints_.end(); ++iter)
//    {
//        //针对每一个旧map点，遍历新的stereo pair中的keypoints
//        std::vector<int> dists = {};
//        for(int i = 0; i < left_frame->keypoints_.size(); i++)
//        {
//            // 比较新stereo pair和map点的keypoint的描述子
//            cv::Mat mappointdescriptor = (*iter)->descriptor_;
//            cv::Mat newlimgdescriptor = left_frame->descriptors_.row(i);
//            int dist = compute_descriptordist(mappointdescriptor, newlimgdescriptor);
//            if ( dist < min_dist ) min_dist = dist;
//            if ( dist > max_dist ) max_dist = dist;
//            dists.push_back(dist); //每一个map点对应的的距离值的集合
//        }
//        min_dists.push_back(*(std::min_element(dists.begin(), dists.end()))); // std::element 返回的是iterator
//    }
//    std::cout << "min and max dist are " << min_dist << " " << max_dist << std::endl;
//
//    //遍历所有的map点
//    //针对每一个map点，在新的feature points中找匹配
//    std::vector<mappoint::ptr> pre_matched_mappoints, // 旧的map点中被匹配到的点集
//                               pre_unmatched_mappoints, // 旧的map点中未被匹配到的点集
//                               curr_matched_mappoints, // 新帧中的map点中被匹配到的点集
//                               curr_unmatched_mappoints; // 新帧中的map点中未被匹配到的点集
//    std::vector<cv::Point2f> curr_matched_kps2f; // 新帧中的特征点中被匹配到的点集
//
//    std::vector<int> index_matched_curr_mappoints = {}; //新帧中被匹配的map点的index集
//    for(auto iter = mappoints_.begin(); iter != mappoints_.end(); ++iter) //遍历现存的旧map点
//    {
//        // todo:在一个map点在多个image被观察到的情况下，需要实现一个计算对这个map点最适合的特征点的函数(在现有的老的keypoints中寻找)
//        //针对每一个旧map点，遍历新的stereo pair中的keypoints
//        bool mappoint_macthed = false; // 先定义此map点还未被match
//        for(int i = 0; i < left_frame->keypoints_.size(); i++)
//        {
//            // 比较旧map点和新的stereo pair（左图）的keypoint的描述子-->计算距离
//            cv::Mat mappointdescriptor = (*iter)->descriptor_;
//            cv::Mat newlimgdescriptor = left_frame->descriptors_.row(i);
//            int dist = compute_descriptordist(mappointdescriptor, newlimgdescriptor);
//            int index = std::distance(mappoints_.begin(), iter); //本iter对应的下标
//
//            std::vector<int>::iterator iter_index;
//            iter_index = find(index_matched_curr_mappoints.begin(), index_matched_curr_mappoints.end(), i);
//
//            if((dist < std::max(2*min_dist, descriptor_dist_thres) && //当距离小于某个阈值
//                (dist == min_dists[index])) && // 而且这个距离等于本行距离最小值时
//               (iter_index == index_matched_curr_mappoints.end())) //而且i不属于index_matched_curr_mappoints
//            {
//                //把匹配中的旧map点放入匹配的旧map点点集中
//                pre_matched_mappoints.push_back(*iter);
//                //把新pair中匹配的特征点放入匹配的特征点的点集中
//                curr_matched_kps2f.push_back(left_frame->kps2f_[i]);
//                //把匹配中的新帧的map点放入匹配的新map点点集中
//                curr_matched_mappoints.push_back((left_frame->mappoints_)[i]);
//                //把本index放入新帧中被匹配的map点的index集，因为在新pair中特征点的index和map点的index顺序应该一样的
//                index_matched_curr_mappoints.push_back(i);
//                // 把本map点标记为已被匹配
//                mappoint_macthed = true;
//            }
//            continue; //如果发现了一个匹配点，则不在寻找其他的匹配点
//        }
//        if(!mappoint_macthed) // 当此旧map点还未被match
//        {
//            //把此未匹配中的旧map点放入未匹配中的旧map点点集中
//            pre_unmatched_mappoints.push_back(*iter);
//        }
//    }
//    for(int i = 0; i < index_matched_curr_mappoints.size(); i++)
//    {
//        std::cout << "index of the current matched mappoint is " << index_matched_curr_mappoints[i] << std::endl;
//    }
//    // 把新帧中未匹配的map点放入相关点集中，即构建curr_unmatched_mappoints
//    for(int i = 0; i < (left_frame->mappoints_).size(); i++) //遍历新帧中所有的map点
//    {
//        //查看此map点的index是否属于新帧中被匹配的map点的index集
//        std::vector<int>::iterator iter;
//        iter = find(index_matched_curr_mappoints.begin(), index_matched_curr_mappoints.end(), i);
//        if(iter == index_matched_curr_mappoints.end()) //当不属于被匹配的index集时
//        {
//            //把未匹配中的新帧map点放入未匹配中的新帧map点点集中
//            curr_unmatched_mappoints.push_back((left_frame->mappoints_)[i]);
//        }
//    }
//    std::cout <<"size of old map points: matched and unmatched are "
//              << std::endl << pre_matched_mappoints.size()
//              << std::endl << pre_unmatched_mappoints.size() << std::endl;
//
//    std::cout <<"size of new map points: matched and unmatched are "
//              << std::endl << curr_matched_mappoints.size()
//              << std::endl << curr_unmatched_mappoints.size() << std::endl;
//
//    // 进行pnp匹配
//    std::cout <<"size of old map points and matched keypoints for pnp are "
//              << std::endl << pre_matched_mappoints.size()
//              << std::endl << curr_matched_kps2f.size() << std::endl;
//
//    cv::Mat R_pnp, t_pnp;
//    pose_estimation_pnp(pre_matched_mappoints, curr_matched_kps2f, R_pnp, t_pnp);
//
//    // 新帧中的map点中未被匹配到的点需要根据估计出来的坐标变换R，t来更新点的坐标
//    Eigen::Isometry3d T_pnp = Eigen::Isometry3d::Identity();
//    convert_cv2eigen(R_pnp, t_pnp, T_pnp);
//    for(int i = 0; i < curr_unmatched_mappoints.size(); i++)
//    {
//        //对坐标点来说，左乘
//        (curr_unmatched_mappoints[i])->pos_ = (T_pnp.inverse())*((curr_unmatched_mappoints[i])->pos_);
//    }
//    left_frame->T_cw_ = left_frame->T_cw_*T_pnp;
//    right_frame->T_cw_ = left_frame->T_cw_;
//
//    //更新本map的map点，由旧的map点中被匹配到的点集和新帧中的map点中未被匹配到的点集组合而成
//
//    // todo:查看旧map点中未被匹配的点的可见性，并删除不可见的map点
//    //todo: 新帧的坐标需要确定，所有的map点需要转换
//    mappoints_.clear();
//    std::cout << "Size of the map point in map before insertion is " << mappoints_.size() << std::endl;
//    mappoints_.insert(mappoints_.end(), pre_matched_mappoints.begin(), pre_matched_mappoints.end());//将pre_matched_mappoints压入
//    mappoints_.insert(mappoints_.end(), curr_unmatched_mappoints.begin(), curr_unmatched_mappoints.end());//继续将curr_unmatched_mappoints压入
//
//    std::cout << "Size of the map point in map after insertion is " << mappoints_.size() << std::endl;
//    std::cout << "Find 3d2d Matches Finished"<< std::endl;
//}

void map::find_3d2dmatches(frame::ptr& pre_left_frame, frame::ptr& left_frame)
{
    std::cout << "Find 3d2d Matches Started"<< std::endl;

    cv::Ptr<cv::DescriptorMatcher> matcher  = cv::DescriptorMatcher::create ( "BruteForce-Hamming" );
    std::vector<cv::DMatch> match;
    matcher->match(pre_left_frame->descriptors_, left_frame->descriptors_, match);
    //todo: pre_matched_keypoint update
    double min_dist=10000, max_dist=0;
    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for ( int i = 0; i < (pre_left_frame->descriptors_).rows; i++ )
    {
        double dist = match[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }
    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );
    std::vector<cv::DMatch> good_matches;
    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    std::vector<cv::Point2f> pre_matched_kps2f, // 旧帧中的特征点中被匹配到的点集
                             curr_matched_kps2f; // 新帧中的特征点中被匹配到的点集
    std::vector<mappoint::ptr> pre_matched_mappoints; // 旧的map点中被匹配到的点集
    std::vector<int> pre_matchness, index_curr_match;
    for ( int i = 0; i < (pre_left_frame->descriptors_).rows; i++ )
    {
        if (match[i].distance <= std::max (2*min_dist, 30.0))
        {
            std::cout << "index: " << i << std::endl;
            good_matches.push_back(match[i]);
            pre_matched_kps2f.push_back((pre_left_frame->keypoints_)[match[i].queryIdx].pt);
            curr_matched_kps2f.push_back((left_frame->keypoints_)[match[i].trainIdx].pt);
            index_curr_match.push_back(match[i].trainIdx);
            pre_matchness.push_back(1);
            continue;
        }
        pre_matchness.push_back(0);
    }
    std::cout << "size of pre_mactchness is " << pre_matchness.size() << std::endl;
    for(int i = 0; i < pre_matchness.size(); i++)
    {
        if(pre_matchness[i] == 1)
        {
//            std::cout << "size of pre_mactchness is " << i << std::endl;
            pre_matched_mappoints.push_back((pre_left_frame->mappoints_)[i]);
        }
    }
    std::cout << "reach line 1248" << std::endl;
    for(int i = 0; i < index_curr_match.size(); i++)
    {
        std::cout << "index of the current matched mappoint is " << index_curr_match[i] << std::endl;
    }
//    // 把新帧中未匹配的map点放入相关点集中，即构建curr_unmatched_mappoints
//    for(int i = 0; i < (left_frame->mappoints_).size(); i++) //遍历新帧中所有的map点
//    {
//        //查看此map点的index是否属于新帧中被匹配的map点的index集
//        std::vector<int>::iterator iter;
//        iter = find(index_curr_match.begin(), index_curr_match.end(), i);
//        if(iter == index_curr_match.end()) //当不属于被匹配的index集时
//        {
//            //把未匹配中的新帧map点放入未匹配中的新帧map点点集中
//            curr_unmatched_mappoints.push_back((left_frame->mappoints_)[i]);
//        }
//    }

    // 进行pnp匹配
    std::cout <<"size of old map points and matched keypoints for pnp are "
              << std::endl << pre_matched_mappoints.size()
              << std::endl << curr_matched_kps2f.size() << std::endl;

    cv::Mat R_pnp, t_pnp;
    pose_estimation_pnp(pre_matched_mappoints, curr_matched_kps2f, R_pnp, t_pnp);

    std::cout << "Find 3d2d Matches Finished"<< std::endl;
}

void map::pose_estimation_pnp(const std::vector<mappoint::ptr>& mappoints_pre,
                              const std::vector<cv::Point2f>& kps2f_curr,
                              cv::Mat& R,
                              cv::Mat& t)
{
    std::cout << "pose_estimation_pnp started "<< std::endl;
    std::vector<cv::Point3d> points_landmark_pre;
    for(int i = 0; i < mappoints_pre.size(); i++)
    {
        points_landmark_pre.push_back(cv::Point3d(mappoints_pre[i]->pos_(0), mappoints_pre[i]->pos_(1), mappoints_pre[i]->pos_(2)));
    }
    std::cout <<"size of kps2f and landmark points are "
              << std::endl << points_landmark_pre.size()
              << std::endl << kps2f_curr.size() << std::endl;

    cv::Mat r;
    // points_3d is the landmark point in the first camera frame
    // todo: use ransac pnp
    cv::solvePnP(points_landmark_pre, kps2f_curr, K, cv::Mat(), r, t, false ); // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法
    cv::Rodrigues (r, R); // r为旋转向量形式，用Rodrigues公式转换为矩阵

    std::cout<<"R_pnp="<<std::endl<<R<<std::endl;
    std::cout<<"t_pnp="<<std::endl<<t<<std::endl;
    std::cout << "pose_estimation_pnp finished "<< std::endl;
}

void map::find_3d3dmatches(frame::ptr& pre_left_frame, frame::ptr& left_frame)
{
    std::cout << "Find 3d3d Matches Started"<< std::endl;
    std::cout << "source of the frame are " << pre_left_frame->source_ << " and " << left_frame->source_ << std::endl;

//    //找到2d匹配点
//    cv::Ptr<cv::DescriptorMatcher> matcher  = cv::DescriptorMatcher::create ( "BruteForce-Hamming" );
//    std::vector<cv::DMatch> match;
//    matcher->match(pre_left_frame->descriptors_, left_frame->descriptors_, match);
//    std::cout << "Number of initial matches is " << match.size() << std::endl;
//
//    double min_dist=10000, max_dist=0;
//    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
//    for ( int i = 0; i < (pre_left_frame->descriptors_).rows; i++ )
//    {
//        double dist = match[i].distance;
//        if ( dist < min_dist ) min_dist = dist;
//        if ( dist > max_dist ) max_dist = dist;
//    }
//    printf ( "-- Max dist : %f \n", max_dist );
//    printf ( "-- Min dist : %f \n", min_dist );
//    std::vector<cv::DMatch> good_matches;
//
//    std::vector<cv::Point2f> pre_matched_kps2f, // 旧帧中的特征点中被匹配到的点集
//    curr_matched_kps2f; // 新帧中的特征点中被匹配到的点集
//    std::vector<mappoint::ptr> pre_matched_mappoints, // 旧的map点中被匹配到的点集
//    curr_matched_mappoints,  // 新的map点中被匹配到的点集
//    curr_unmatched_mappoints;  // 新的map点中未被匹配到的点集
//    std::vector<int> pre_matchness, index_curr_match;
//
//    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
//    for ( int i = 0; i < (pre_left_frame->descriptors_).rows; i++ )
//    {
//        if (match[i].distance <= std::max (2*min_dist, 30.0))
//        {
//            std::cout << "index: " << i << std::endl;
//            good_matches.push_back(match[i]);
//            pre_matched_kps2f.push_back((pre_left_frame->keypoints_)[match[i].queryIdx].pt);
//            curr_matched_kps2f.push_back((left_frame->keypoints_)[match[i].trainIdx].pt);
//            index_curr_match.push_back(match[i].trainIdx);
//            pre_matchness.push_back(1);
//            continue;
//        }
//        pre_matchness.push_back(0);
//    }
//
//    //找到2d匹配相应的3d匹配点
//    std::cout << "size of pre_mactchness is " << pre_matchness.size() << std::endl;
//    for(int i = 0; i < pre_matchness.size(); i++)
//    {
//        if(pre_matchness[i] == 1)
//        {
////            std::cout << "size of pre_mactchness is " << i << std::endl;
//            pre_matched_mappoints.push_back((pre_left_frame->mappoints_)[i]);
//        }
//    }
////    for(int i = 0; i < index_curr_match.size(); i++)
////    {
////        std::cout << "index of the current matched mappoint is " << index_curr_match[i] << std::endl;
////    }
//    // 把新帧中未匹配的map点放入相关点集中，即构建curr_unmatched_mappoints
//    for(int i = 0; i < (left_frame->mappoints_).size(); i++) //遍历新帧中所有的map点
//    {
//        //查看此map点的index是否属于新帧中被匹配的map点的index集
//        std::vector<int>::iterator iter;
//        iter = find(index_curr_match.begin(), index_curr_match.end(), i);
//        if(iter == index_curr_match.end()) //当不属于被匹配的index集时
//        {
//            //把未匹配中的新帧map点放入未匹配中的新帧map点点集中
//            curr_unmatched_mappoints.push_back((left_frame->mappoints_)[i]);
//        }
//        else
//        {
//            //把匹配中的新帧map点放入匹配中的新帧map点点集中
//            curr_matched_mappoints.push_back((left_frame->mappoints_)[i]);
//        }
//    }

    // 进行3d-3d匹配
    std::vector<mappoint::ptr> pre_matched_mappoints = pre_left_frame->mappoints_;
    std::vector<mappoint::ptr> curr_matched_mappoints = left_frame->mappoints_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pre_left_pcframe_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr left_pcframe_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_scan_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    for(int i = 0; i < pre_matched_mappoints.size(); i++)
    {
        pre_left_pcframe_ptr->push_back(pcl::PointXYZ(pre_matched_mappoints[i]->pos_[0], pre_matched_mappoints[i]->pos_[1], pre_matched_mappoints[i]->pos_(2)));
    }
    for(int i = 0; i < curr_matched_mappoints.size(); i++)
    {
        left_pcframe_ptr->push_back(pcl::PointXYZ(curr_matched_mappoints[i]->pos_[0], curr_matched_mappoints[i]->pos_[1], curr_matched_mappoints[i]->pos_(2)));
    }
    //Get transform between two frames Using ndt_omp ////////////////////////////////////////////////////////////////////
    pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
    ndt_omp->setInputSource(left_pcframe_ptr);
    ndt_omp->setInputTarget(pre_left_pcframe_ptr);
    ndt_omp->setTransformationEpsilon (epsilon);
    ndt_omp->setResolution (res);
    ndt_omp->setStepSize (step);
    ndt_omp->setMaximumIterations(100);

    ndt_omp->setNumThreads(NumThreads);
    ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    // get the transform between two poses as guess
    std::cout << "Preparation for Scan Registration Finished" << std::endl;
    auto t1 = ros::WallTime::now();
    ndt_omp->align(*aligned_scan_cloud_ptr);
    auto t2 = ros::WallTime::now();
    std::cout << "Duration : " << (t2 - t1).toSec() * 1000 << "[msec]" << std::endl;

    std::cout << "Normal Distributions Transform has converged after " << ndt_omp->getFinalNumIteration() << " iters" << std::endl;
    std::cout << "The score is " << ndt_omp->getFitnessScore() << std::endl;

    Eigen::Matrix4f Transf = Eigen::Matrix4f::Identity();
    Transf = ndt_omp->getFinalTransformation();

    std::cout << "Final Transformation Matrix of the Scan Registation is " << std::endl << Transf << std::endl;
    cv::Mat t = (cv::Mat_<double>(3,1) << Transf(0, 3), Transf(1, 3), Transf(2, 3));
    std::cout << "Mode of the translation is " << std::endl << cv::norm(t) << std::endl;
    Eigen::Isometry3d pose_diff = Eigen::Isometry3d::Identity();
    pose_diff.matrix() = Transf.cast<double>();

//    std::vector<cv::Point3d> pts1, pts2;
//    for(int i = 0; i < pre_matched_mappoints.size(); i++)
//    {
//        pts1.push_back(cv::Point3d(pre_matched_mappoints[i]->pos_[0], pre_matched_mappoints[i]->pos_[1], pre_matched_mappoints[i]->pos_(2)));
//    }
//    for(int i = 0; i < curr_matched_mappoints.size(); i++)
//    {
//        pts2.push_back(cv::Point3d(curr_matched_mappoints[i]->pos_[0], curr_matched_mappoints[i]->pos_[1], curr_matched_mappoints[i]->pos_(2)));
//    }
//
//    Eigen::Isometry3d pose_diff = Eigen::Isometry3d::Identity();
//
//    cv::Mat R_3d, t_3d;
//    pose_estimation_3d3d(pts1, pts2, R_3d, t_3d);
//    convert_cv2eigen(R_3d, t_3d, pose_diff);

//    ba_3d3d(pts1, pts2, pose_diff);

    //Update the pose
    left_frame->T_cw_ = pre_left_frame->T_cw_*pose_diff;
    std::cout << "pose after update is " << std::endl << (left_frame->T_cw_).matrix() << std::endl;

    std::cout << "Find 3d2d Matches Finished"<< std::endl;
}

void map::pose_estimation_3d3d (
        const std::vector<cv::Point3d>& pts1,
        const std::vector<cv::Point3d>& pts2,
        cv::Mat& R,
        cv::Mat& t)
{
    cv::Point3d p1, p2;     // center of mass
    int N = pts1.size();
    for ( int i=0; i<N; i++ )
    {
        p1 += pts1[i];
        p2 += pts2[i];
    }
    p1 = cv::Point3d( cv::Vec3d(p1) /  N);
    p2 = cv::Point3d( cv::Vec3d(p2) / N);
    std::vector<cv::Point3d>     q1 ( N ), q2 ( N ); // remove the center
    for ( int i=0; i<N; i++ )
    {
        q1[i] = pts1[i] - p1;
        q2[i] = pts2[i] - p2;
    }

    // compute q1*q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for ( int i=0; i<N; i++ )
    {
        W += Eigen::Vector3d ( q1[i].x, q1[i].y, q1[i].z ) * Eigen::Vector3d ( q2[i].x, q2[i].y, q2[i].z ).transpose();
    }
//    std::cout<<"W="<<W<<std::endl;

    // SVD on W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd ( W, Eigen::ComputeFullU|Eigen::ComputeFullV );
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    if (U.determinant() * V.determinant() < 0)
    {
        for (int x = 0; x < 3; ++x)
        {
            U(x, 2) *= -1;
        }
    }

//    std::cout<<"U="<<U<<std::endl;
//    std::cout<<"V="<<V<<std::endl;

    Eigen::Matrix3d R_ = U* ( V.transpose() );
    Eigen::Vector3d t_ = Eigen::Vector3d ( p1.x, p1.y, p1.z ) - R_ * Eigen::Vector3d ( p2.x, p2.y, p2.z );

    // convert to cv::Mat
    R = ( cv::Mat_<double> ( 3,3 ) <<
                               R_ ( 0,0 ), R_ ( 0,1 ), R_ ( 0,2 ),
            R_ ( 1,0 ), R_ ( 1,1 ), R_ ( 1,2 ),
            R_ ( 2,0 ), R_ ( 2,1 ), R_ ( 2,2 )
    );
    t = ( cv::Mat_<double> ( 3,1 ) << t_ ( 0,0 ), t_ ( 1,0 ), t_ ( 2,0 ) );
    std::cout<<"R="<<std::endl<<R<<std::endl;
    std::cout<<"t="<<std::endl<<t<<std::endl;
}

void map::ba_3d3d(
        const std::vector< cv::Point3d >& pts1,
        const std::vector< cv::Point3d >& pts2,
        Eigen::Isometry3d& T)
{
    // 初始化g2o
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // pose维度为 6, landmark 维度为 3
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<Block::PoseMatrixType>(); // 线性方程求解器
//    Block* solver_ptr = new Block( linearSolver );      // 矩阵块求解器
    Block* solver_ptr = new Block( std::unique_ptr<Block::LinearSolverType>(linearSolver) );// 矩阵块求解器
    g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(std::unique_ptr<Block>(solver_ptr));
//    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<Block>(solver_ptr) );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm( solver );

    // vertex
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
    pose->setId(0);
    pose->setEstimate( g2o::SE3Quat(
            Eigen::Matrix3d::Identity(),
            Eigen::Vector3d( 0,0,0 )
    ) );
    optimizer.addVertex( pose );

    // edges
    int index = 1;
    std::vector<EdgeProjectXYZRGBDPoseOnly*> edges;
    for ( size_t i=0; i<pts1.size(); i++ )
    {
        EdgeProjectXYZRGBDPoseOnly* edge = new EdgeProjectXYZRGBDPoseOnly(
                Eigen::Vector3d(pts2[i].x, pts2[i].y, pts2[i].z) );
        edge->setId( index );
        edge->setVertex( 0, dynamic_cast<g2o::VertexSE3Expmap*> (pose) );
        edge->setMeasurement( Eigen::Vector3d(
                pts1[i].x, pts1[i].y, pts1[i].z) );
        edge->setInformation( Eigen::Matrix3d::Identity()*1e4 );
        optimizer.addEdge(edge);
        index++;
        edges.push_back(edge);
    }

    optimizer.setVerbose( true );
    optimizer.initializeOptimization();
    optimizer.optimize(10);

    T = Eigen::Isometry3d(pose->estimate());

    std::cout<<std::endl<<"after optimization:"<<std::endl;
    std::cout<<"T="<<std::endl<<Eigen::Isometry3d( pose->estimate() ).matrix()<<std::endl;

}


