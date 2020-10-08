#include "imu_gps_localizer/imu_gps_localizer.h"

#include <glog/logging.h>

#include "imu_gps_localizer/utils.h"

namespace ImuGpsLocalization {

ImuGpsLocalizer::ImuGpsLocalizer(const double acc_noise, const double gyro_noise,
                                 const double acc_bias_noise, const double gyro_bias_noise,
                                 const Eigen::Vector3d& I_p_Gps) // 3，ImuGpsLocalizer 构造函数；
    : initialized_(false){  
    initializer_ = std::make_unique<Initializer>(I_p_Gps);  // 3.1，初始化; ==> 4, initializer.cpp
    imu_processor_ = std::make_unique<ImuProcessor>(acc_noise, gyro_noise, 
                                                    acc_bias_noise, gyro_bias_noise,
                                                    Eigen::Vector3d(0., 0., -9.81007));  // 3.2，创建 ImuProcessor 对象， 用于处理IMU数据；
    gps_processor_ = std::make_unique<GpsProcessor>(I_p_Gps);  // 3.3，创建 GpsProcessor 对象用于处理GPS数据；
}

bool ImuGpsLocalizer::ProcessImuData(const ImuDataPtr imu_data_ptr, State* fused_state) { // 3.2 在 ProcessImuData 中；
    if (!initialized_) {
        initializer_->AddImuData(imu_data_ptr);  // 将IMU数据添加到队列中（在未初始化成功之前，永远在队列中保持最新的固定个数的IMU数据）；
        return false;
    }
    
    // Predict.
    imu_processor_->Predict(state_.imu_data_ptr, imu_data_ptr, &state_);  // 3.2 然后执行 Predict 预测步骤；==> 5，imu_processor.cpp, ImuProcessor

    // Convert ENU state to lla.
    ConvertENUToLLA(init_lla_, state_.G_p_I, &(state_.lla));  // 将预测的状态转换成经纬度坐标系；G_p_I, 全局坐标系下IMU的位置；
    *fused_state = state_;
    return true;
}

bool ImuGpsLocalizer::ProcessGpsPositionData(const GpsPositionDataPtr gps_data_ptr) { // 3.3 
    if (!initialized_) {
        if (!initializer_->AddGpsPositionData(gps_data_ptr, &state_)) {  // 添加GPS数据到系统状态中（IMU队列中的数据足够之后，添加GPS数据才会成功，此时还会初始化全局状态 state_）；
            return false;
        }

        // Initialize the initial gps point used to convert lla to ENU.
        init_lla_ = gps_data_ptr->lla;  // 记录初始化成功后的经纬度作为GPS初始参考点；
        
        initialized_ = true;

        LOG(INFO) << "[ProcessGpsPositionData]: System initialized!";
        return true;
    }

    // Update.
    gps_processor_->UpdateStateByGpsPosition(init_lla_, gps_data_ptr, &state_);  // 3.3 然后执行更新程序；==> 6, gps_processer.cpp, GpsProcessor

    return true;
}

}  // namespace ImuGpsLocalization