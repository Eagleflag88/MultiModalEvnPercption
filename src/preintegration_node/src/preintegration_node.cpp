//
// Created by eagleflag on 2021/3/22.
//

#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "imu_frame.h"
#include "integration_base.h"

#include <Eigen/Geometry>
#include <Eigen/Core>

// Declaration of Publishers

static ros::Publisher chatter_pub;
static ros::Publisher imu_pub;

// Declaration of Subscribers
static ros::Subscriber imu_sub;

//窗口中的[P,V,R,Ba,Bg]
Eigen::Vector3d Ps[(WINDOW_SIZE + 1)];
Eigen::Vector3d Vs[(WINDOW_SIZE + 1)];
Eigen::Matrix3d Rs[(WINDOW_SIZE + 1)];
Eigen::Vector3d Bas[(WINDOW_SIZE + 1)];
Eigen::Vector3d Bgs[(WINDOW_SIZE + 1)];

//窗口中的dt,a,v
std::vector<double> dt_buf[(WINDOW_SIZE + 1)];
std::vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
std::vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];

bool first_imu = false;
double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
double dt = 0.01;
Eigen::Vector3d g;

//IMU项[P,Q,B,Ba,Bg,a,g]
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;

int frame_count = 0; //滑动窗口中图像数据的个数

// 预积分instance
IntegrationBase* pre_integrations[(WINDOW_SIZE + 1)];
IntegrationBase* tmp_pre_integration;

//todo: 查看所有容器的使用，复制还是引用
//todo: lock for data race

static void IMU_Callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr)
{
    imu_frame::ptr imu_frame_ptr = imu_frame_ptr->create_frame();
    imu_frame_ptr->update_frame(imu_msg_ptr);

    dx = imu_msg_ptr->linear_acceleration.x;
    dy = imu_msg_ptr->linear_acceleration.y;
    dz = imu_msg_ptr->linear_acceleration.z;
    rx = imu_msg_ptr->angular_velocity.x;
    ry = imu_msg_ptr->angular_velocity.y;
    rz = imu_msg_ptr->angular_velocity.z;

    // 加速度测量值
    Eigen::Vector3d linear_acceleration{dx, dy, dz};
    // 角速度测量值
    Eigen::Vector3d angular_velocity{rx, ry, rz};


    //1.判断是不是第一个imu消息，如果是第一个imu消息，则将当前传入的线加速度和角速度作为初始的加速度和角速度
    if (!first_imu)
    {
        // 当frame_count==0的时候表示滑动窗口中还没有图像帧数据，所以不需要进行预积分，只进行线加速度和角速度初始值的更新
        first_imu = true;
        acc_0 = linear_acceleration; //记录线加速度值
        gyr_0 = angular_velocity; //记录角速度值
    }

    /**
     * 2.创建预积分对象
     * 首先，pre_integrations是一个数组，里面存放了(WINDOW_SIZE + 1)个指针，指针指向的类型是IntegrationBase的对象
    */
    if (!pre_integrations[frame_count])
    {
        // 每新到一个图像帧，就会创建一个IntegrationBase对象存入pre_integrations数组当中
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }

    // 当frame_count!=0的时候，说明滑动窗口中已经有图像帧的数据了，此时就可以对该图像帧对应的imu进行预积分
    if (frame_count != 0)
    {
        //3.进行预计分
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
        //if(solver_flag != NON_LINEAR)
        tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        // 先把线性加速度和角速度和时间间隔放入buffer
        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        int j = frame_count;
        /**
         * 4.更新Rs、Ps、Vs三个向量数组。
         * Rs为旋转向量数组，Ps为位置向量数组，Vs为速度向量数组，数组的大小为WINDOW_SIZE + 1
         * 那么，这三个向量数组中每个元素都对应的是每一个window中的图像帧
        */

        // 这里是对应https://blog.csdn.net/weixin_44580210/article/details/93377806 公式（2
        // 但是这里的Ps[j]、 Vs[j]、Rs[j]是未经后端优化之前的位置、速度和位姿
        //采用的是中值积分的传播方式
        //计算上一时刻的加速度，前边乘一个旋转矩阵Rs[j]的作用是进行坐标系转换
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g; // 世界坐标下的加速度

        //根据上一时刻陀螺仪的角速度和当前时刻的角速度求出平均角速度
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        //计算当前时刻陀螺仪的姿态（旋转）矩阵。这里其实是在上一时刻的旋转矩阵基础上和当前时刻的旋转增量相乘得到的
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        //求当前时刻的加速度
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        //求上一时刻和当前时刻的平均加速度
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        //位移（位置）更新，位置是在之前的基础上加上当前的位移量，使用的是位移公式：s = v*t + 1/2*a*t^2
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        //速度更新，使用的是速度公式：v = a * t a是加速度，t是时间
        Vs[j] += dt * un_acc;
    }
    //更新acc_0和gyr_0的值，本次的线加速度和角速度作为下一个IMU消息的前一个状态值
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "preintegration_node");
    ros::NodeHandle nh;
    // Register the Subscriber

    chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);


    int count = 0;
    std_msgs::String msg;
    while(ros::ok())
    {
        std::stringstream status_msg;
        status_msg << "preintegration_node working fine " << count;
        msg.data = status_msg.str();
        ROS_INFO("%s", msg.data.c_str());
        chatter_pub.publish(msg);
        ros::spin();
        count++;
    }

    return 0;
}