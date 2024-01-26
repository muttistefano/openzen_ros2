/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2014, Robotiq, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * * Neither the name of Robotiq, Inc. nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Copyright (c) 2014, Robotiq, Inc
 */

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

#include <openzen_sensor_lib.hpp>

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
