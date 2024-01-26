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

#include <openzen_sensor_lib.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto nodeState    = std::make_shared<OpenZenSensor>();
    executor.add_node(nodeState->get_node_base_interface());
    if (!nodeState->run())
    {
        rclcpp::shutdown();
        return 1;
    }
    executor.spin();
    rclcpp::shutdown();

    return 0;
}
