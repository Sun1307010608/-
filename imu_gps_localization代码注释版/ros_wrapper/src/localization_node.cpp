#include <memory>

#include <glog/logging.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include "localization_wrapper.h"

int main (int argc, char** argv) {
    // Set glog.
    FLAGS_colorlogtostderr = true;

    // Initialize ros.
    ros::init(argc, argv, "imu_gps_localization");
    ros::NodeHandle nh;
    
    // Initialize localizer.
    LocalizationWrapper localizer(nh);  // 1, 类创建对象，自动执行构造函数；==> 2, localization_wrapper.cpp, LocalizationWrapper

    ros::spin();
    return 1;
}