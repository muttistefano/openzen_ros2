//===========================================================================//
//
// Copyright (C) 2020 LP-Research Inc.
//
// This file is part of OpenZenRos driver, under the MIT License.
// See the LICENSE file in the top-most folder for details
// SPDX-License-Identifier: MIT
//
//===========================================================================//

#ifndef ROBOTIQ_FT_SENSOR__HARDWARE_INTERFACE_HPP_
#define ROBOTIQ_FT_SENSOR__HARDWARE_INTERFACE_HPP_

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "ManagedThread.h"
#include <OpenZen.h>

#include <memory>
#include <string>
#include <map>

class OpenZenSensor : public rclcpp::Node
{
public:

    explicit OpenZenSensor();

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

    std::mutex data_lock_;

    std::array<float, 4> orient_  {{0.0, 0.0, 0.0, 0.0}};
    std::array<float, 3> lin_acc_ {{0.0, 0.0, 0.0}};
    std::array<float, 3> ang_vel_ {{0.0, 0.0, 0.0}};

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



namespace imu_sensor_hardware {
class IMUSensorHardware : public hardware_interface::SensorInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(IMUSensorHardware);

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:

  std::vector<double> hw_sensor_states_;
  std::shared_ptr<OpenZenSensor> nodeState    = std::make_shared<OpenZenSensor>();
};

} // namespace imu_sensor_hardware

#endif // ROBOTIQ_FT_SENSOR__HARDWARE_INTERFACE_HPP_

