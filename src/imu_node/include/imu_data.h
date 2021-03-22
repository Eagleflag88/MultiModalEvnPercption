//
// Created by eagleflag on 2021/3/22.
//

#ifndef MULTIMODALEVNPERCPTION_IMU_DATA_H
#define MULTIMODALEVNPERCPTION_IMU_DATA_H

struct imu_data
{
    double time = 0.0;
    double linear_acceleration_x = 0.0;
    double linear_acceleration_y = 0.0;
    double linear_acceleration_z = 0.0;

    double angular_velocity_x = 0.0;
    double angular_velocity_y = 0.0;
    double angular_velocity_z = 0.0;

    double orientation_x = 0.0;
    double orientation_y = 0.0;
    double orientation_z = 0.0;
    double orientation_w = 1.0;
};

#endif //MULTIMODALEVNPERCPTION_IMU_DATA_H
