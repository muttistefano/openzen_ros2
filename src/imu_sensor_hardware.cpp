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

#include "imu_sensor_hardware/imu_sensor_lib_hw.hpp"

namespace imu_sensor_hardware {
hardware_interface::CallbackReturn IMUSensorHardware::on_init(const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SensorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_sensor_states_.resize(info_.sensors[0].state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());

  // parameters
  // max_retries_ = std::stoi(info_.hardware_parameters["max_retries"]);


  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> IMUSensorHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  RCLCPP_INFO(rclcpp::get_logger("UrRobotHW"), "Exporting State Interfaces");
  for (auto &sensor : info_.sensors) {
    for (uint j = 0; j < sensor.state_interfaces.size(); ++j) {
      RCLCPP_INFO(rclcpp::get_logger("UrRobotHW"), "Sensor %s state %s", sensor.name.c_str(), sensor.state_interfaces[j].name.c_str());
      state_interfaces.emplace_back(hardware_interface::StateInterface(sensor.name, sensor.state_interfaces[j].name, &hw_sensor_states_[j]));
    }
  }
  return state_interfaces;
}

hardware_interface::CallbackReturn IMUSensorHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("UrRobotHW"), "Activating ...please wait...");

    if (!nodeState->run())
    {
      return hardware_interface::CallbackReturn::FAILURE;
    }

  RCLCPP_INFO(rclcpp::get_logger("UrRobotHW"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IMUSensorHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("UrRobotHW"), "Deactivating ...please wait...");
  delete nodeState.get();
  RCLCPP_INFO(rclcpp::get_logger("UrRobotHW"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type IMUSensorHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {

  // hw_sensor_states_[0] = rq_state_get_received_data(0);
  // hw_sensor_states_[1] = rq_state_get_received_data(1);
  // hw_sensor_states_[2] = rq_state_get_received_data(2);
  // hw_sensor_states_[3] = rq_state_get_received_data(3);
  // hw_sensor_states_[4] = rq_state_get_received_data(4);
  // hw_sensor_states_[5] = rq_state_get_received_data(5);

  {
    std::scoped_lock lock{nodeState->data_lock_};

    hw_sensor_states_[0] = nodeState->orient_[0];
    hw_sensor_states_[1] = nodeState->orient_[1];
    hw_sensor_states_[2] = nodeState->orient_[2];
    hw_sensor_states_[3] = nodeState->orient_[3];

    hw_sensor_states_[4] = nodeState->ang_vel_[0];
    hw_sensor_states_[5] = nodeState->ang_vel_[1];
    hw_sensor_states_[6] = nodeState->ang_vel_[2];

    hw_sensor_states_[7] = nodeState->lin_acc_[0];
    hw_sensor_states_[8] = nodeState->lin_acc_[1];
    hw_sensor_states_[9] = nodeState->lin_acc_[2];
  }

  return hardware_interface::return_type::OK;
}

} // namespace imu_sensor_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(imu_sensor_hardware::IMUSensorHardware, hardware_interface::SensorInterface)
