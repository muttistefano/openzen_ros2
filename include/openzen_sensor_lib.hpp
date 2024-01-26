//===========================================================================//
//
// Copyright (C) 2020 LP-Research Inc.
//
// This file is part of OpenZenRos driver, under the MIT License.
// See the LICENSE file in the top-most folder for details
// SPDX-License-Identifier: MIT
//
//===========================================================================//


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"


#include "ManagedThread.h"
#include <OpenZen.h>

#include <memory>
#include <string>
#include <map>

class OpenZenSensor : public rclcpp::Node
{
public:


    explicit OpenZenSensor();

    // Publisher
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr autocalibration_status_pub;


    // Service
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr autocalibration_serv;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr gyrocalibration_serv;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr resetHeading_serv;
    
    // Parameters
    std::string m_sensorName;
    std::string m_sensorInterface;
    std::string frame_id;
    std::string frame_id_gnss;
    int m_baudrate = 0;


    bool run(void);


    ///////////////////////////////////////////////////
    // Service Callbacks
    ///////////////////////////////////////////////////

    void publishIsAutocalibrationActive();

    bool setAutocalibration (std_srvs::srv::SetBool::Request &req, std_srvs::srv::SetBool::Response &res);

    bool resetHeading (std_srvs::srv::Trigger::Request &req, std_srvs::srv::Trigger::Response &res);

    bool calibrateGyroscope (std_srvs::srv::Trigger::Request &req, std_srvs::srv::Trigger::Response &res);



 private:

    std::unique_ptr<zen::ZenClient> m_zenClient;
    std::unique_ptr<zen::ZenSensor> m_zenSensor;
    std::unique_ptr<zen::ZenSensorComponent> m_zenImu;
    std::unique_ptr<zen::ZenSensorComponent> m_zenGnss;

    bool m_openzenVerbose;
    bool m_useLpmsAccelerationConvention;

    struct SensorThreadParams
    {
        zen::ZenClient * zenClient;
        std::string frame_id;
        std::string frame_id_gnss;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
        rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_pub;
        bool useLpmsAccelerationConvention;
        ZenComponentHandle_t zen_imu_component;
        ZenComponentHandle_t zen_gnss_component;
    };

    enum DefaultGyro {
        Gyro1AsDefault = 1,
        Gyro2AsDefault
    };

    DefaultGyro m_defaultGyroIdx;

    ManagedThread<SensorThreadParams> m_sensorThread;
};

