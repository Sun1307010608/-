#include "imu_gps_localizer/initializer.h"

#include <Eigen/Dense>
#include <glog/logging.h>

#include "imu_gps_localizer/utils.h"

namespace ImuGpsLocalization {

Initializer::Initializer(const Eigen::Vector3d& init_I_p_Gps)  // 3， Initializer 构造函数；
    : init_I_p_Gps_(init_I_p_Gps) { }

void Initializer::AddImuData(const ImuDataPtr imu_data_ptr) {  // 3.1 添加IMU数据，保留最新的 kImuDataBufferLength 个IMU数据；
    imu_buffer_.push_back(imu_data_ptr);

    if (imu_buffer_.size() > kImuDataBufferLength) {  // 3.1.1 kImuDataBufferLength 默认值100；
        imu_buffer_.pop_front(); 
    }
}

bool Initializer::AddGpsPositionData(const GpsPositionDataPtr gps_data_ptr, State* state) { // 3.2 添加GPS数据，并初始化系统状态（尤其是根据加速度估算的初始姿态）；
    if (imu_buffer_.size() < kImuDataBufferLength) {  // 如果初始化用的IMU数据不够，不会操作GPS的数据；
        LOG(WARNING) << "[AddGpsPositionData]: No enought imu data!";
        return false;
    }

    const ImuDataPtr last_imu_ptr = imu_buffer_.back();
    // TODO: synchronize all sensors.
    if (std::abs(gps_data_ptr->timestamp - last_imu_ptr->timestamp) > 0.5) {  // 3.2.1 判断最新的IMU和GPS数据的在时间上是一致的；
        LOG(ERROR) << "[AddGpsPositionData]: Gps and imu timestamps are not synchronized!";
        return false;
    }

    // Set timestamp and imu date.  // 下面是状态 state_ 的初始化；
    state->timestamp = last_imu_ptr->timestamp;
    state->imu_data_ptr = last_imu_ptr;

    // Set initial mean.
    state->G_p_I.setZero();

    // We have no information to set initial velocity. 
    // So, just set it to zero and given big covariance.
    state->G_v_I.setZero();

    // We can use the direction of gravity to set roll and pitch. 
    // But, we cannot set the yaw. 
    // So, we set yaw to zero and give it a big covariance.
    if (!ComputeG_R_IFromImuData(&state->G_R_I)) {  // 3.2.2 当最新的IMU队列中数据足够，计算加速度的均值，从而估算处初始的IMU姿态（注意，在初始化的这段时间内，载体必须静止）；
        LOG(WARNING) << "[AddGpsPositionData]: Failed to compute G_R_I!";
        return false;
    }

    // Set bias to zero.
    state->acc_bias.setZero();
    state->gyro_bias.setZero();

    // Set covariance.
    state->cov.setZero();
    state->cov.block<3, 3>(0, 0) = 100. * Eigen::Matrix3d::Identity(); // position std: 10 m
    state->cov.block<3, 3>(3, 3) = 100. * Eigen::Matrix3d::Identity(); // velocity std: 10 m/s
    // roll pitch std 10 degree.
    state->cov.block<2, 2>(6, 6) = 10. * kDegreeToRadian * 10. * kDegreeToRadian * Eigen::Matrix2d::Identity();
    state->cov(8, 8)             = 100. * kDegreeToRadian * 100. * kDegreeToRadian; // yaw std: 100 degree.
    // Acc bias.
    state->cov.block<3, 3>(9, 9) = 0.0004 * Eigen::Matrix3d::Identity();
    // Gyro bias.
    state->cov.block<3, 3>(12, 12) = 0.0004 * Eigen::Matrix3d::Identity();

    return true;
}

bool Initializer::ComputeG_R_IFromImuData(Eigen::Matrix3d* G_R_I) {  // 3.2.2
    // Compute mean and std of the imu buffer.
    Eigen::Vector3d sum_acc(0., 0., 0.);
    for (const auto imu_data : imu_buffer_) {
        sum_acc += imu_data->acc;
    }
    const Eigen::Vector3d mean_acc = sum_acc / (double)imu_buffer_.size();

    Eigen::Vector3d sum_err2(0., 0., 0.);
    for (const auto imu_data : imu_buffer_) {
        sum_err2 += (imu_data->acc - mean_acc).cwiseAbs2();
    }
    const Eigen::Vector3d std_acc = (sum_err2 / (double)imu_buffer_.size()).cwiseSqrt();

    if (std_acc.maxCoeff() > kAccStdLimit) {  // 3.2.2.1 如果初始化这段时间的载体没有静止，那么这段时间测量出的加速度的标准差会比较大，此时初始化会失败，预测初始化时一定要载体静止；
        LOG(WARNING) << "[ComputeG_R_IFromImuData]: Too big acc std: " << std_acc.transpose();
        return false;
    }

    // Compute rotation.
    // Please refer to 
    // https://github.com/rpng/open_vins/blob/master/ov_core/src/init/InertialInitializer.cpp
    
    // Three axises of the ENU frame in the IMU frame.
    // z-axis.
    const Eigen::Vector3d& z_axis = mean_acc.normalized();   // 3.2.2.2 这一段是重点，根据初始化过程中计算的IMU平均加速度，推出IMU的初始姿态；

    // x-axis. 
    Eigen::Vector3d x_axis = 
        Eigen::Vector3d::UnitX() - z_axis * z_axis.transpose() * Eigen::Vector3d::UnitX();
    x_axis.normalize();

    // y-axis.
    Eigen::Vector3d y_axis = z_axis.cross(x_axis);
    y_axis.normalize();

    Eigen::Matrix3d I_R_G;
    I_R_G.block<3, 1>(0, 0) = x_axis;
    I_R_G.block<3, 1>(0, 1) = y_axis;
    I_R_G.block<3, 1>(0, 2) = z_axis;

    *G_R_I = I_R_G.transpose();

    return true;
}

}  // namespace ImuGpsLocalization