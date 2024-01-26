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

#include "ManagedThread.h"
#include <OpenZen.h>

#include <memory>
#include <string>
#include <map>

#include <imu_sensor_hardware/imu_sensor_lib_hw.hpp>


OpenZenSensor::OpenZenSensor(): 
    Node("OpenZenSensor"), 
    m_sensorThread( [this](SensorThreadParams const& param) -> bool {

        const float cDegToRad = 3.1415926f/180.0f;
        const float cEarthG = 9.81f;
        const float cMicroToTelsa = 1e-6f;

        auto event_value = param.zenClient->waitForNextEvent();

        if (!event_value->component.handle)
        {
            // not an event from a component
            switch (event_value->eventType)
            {
                case ZenEventType_SensorDisconnected:
                    RCLCPP_INFO(this->get_logger(),"OpenZen sensor disconnected");
                    return false;
            }
        }

        if (event_value->component == param.zen_imu_component)
        {
            if (event_value->eventType == ZenEventType_ImuData)
            {
                // IMU
                auto const& d = event_value->data.imuData;

                {
                    std::scoped_lock lock{data_lock_};
                    orient_[0] = d.q[0];
                    orient_[1] = -d.q[1];
                    orient_[2] = -d.q[2];
                    orient_[3] = -d.q[3];
                }

                switch (m_defaultGyroIdx) {
                    case Gyro1AsDefault:
                        {
                            std::scoped_lock lock{data_lock_};
                            ang_vel_[0] = d.g1[0] * cDegToRad;
                            ang_vel_[1] = d.g1[1] * cDegToRad;
                            ang_vel_[2] = d.g1[2] * cDegToRad;
                        }
                        break;
                    case Gyro2AsDefault:
                        {
                            std::scoped_lock lock{data_lock_};
                            ang_vel_[0] = d.g2[0] * cDegToRad;
                            ang_vel_[1] = d.g2[1] * cDegToRad;
                            ang_vel_[2] = d.g2[2] * cDegToRad;
                        }
                        break;
                }

                // Fill linear acceleration data
                const float rosConversion = -1.0 * (!param.useLpmsAccelerationConvention) +
                    1.0 * param.useLpmsAccelerationConvention;

                {
                    std::scoped_lock lock{data_lock_};
                    lin_acc_[0] = rosConversion * d.a[0] * cEarthG;
                    lin_acc_[1] = rosConversion * d.a[1] * cEarthG;
                    lin_acc_[2] = rosConversion * d.a[2] * cEarthG;
                }
            }
        }
          
        return true;
    })
{

    auto clientPair = zen::make_client();
    m_zenClient = std::unique_ptr<zen::ZenClient>(new zen::ZenClient(std::move(clientPair.second)));

    if (clientPair.first != ZenError_None)
    {
        RCLCPP_FATAL(this->get_logger(),"Cannot start OpenZen");
        return;
    }


    ZenSetLogLevel(ZenLogLevel_Off);


    // no sensor name given, auto-discovery
    if (m_sensorName.size() == 0) 
    {
        RCLCPP_INFO(this->get_logger(),"OpenZen sensors will be listed");
        ZenError listError = m_zenClient->listSensorsAsync();

        if (listError != ZenError_None)
        {
            RCLCPP_FATAL(this->get_logger(),"Cannot list sensors");
            return;
        }
        
        bool listingDone = false;
        bool firstSensorFound = false;
        ZenSensorDesc foundSens;

        while (listingDone == false)
        {
            auto event = m_zenClient->waitForNextEvent();
            // const bool success = pair.first;
            // auto& event = pair.second;
            // if (!success)
                // break;

            if (!event->component.handle)
            {
                switch (event->eventType)
                {
                case ZenEventType_SensorFound:
                    if (!firstSensorFound)
                    {
                        foundSens = event->data.sensorFound;
                        firstSensorFound = true;
                    }
                    RCLCPP_INFO_STREAM(this->get_logger(),"OpenZen sensor with name " << event->data.sensorFound.serialNumber << " on IO system " <<
                        event->data.sensorFound.ioType << " found");
                    break;

                case ZenEventType_SensorListingProgress:
                    if (event->data.sensorListingProgress.progress == 1.0f)
                    {
                        listingDone = true;
                    }
                        
                    break;
                }
            }
        }

        if (!firstSensorFound)
        {
            RCLCPP_FATAL(this->get_logger(),"No OpenZen sensors found");
            return;
        }

        RCLCPP_INFO_STREAM(this->get_logger(),"Connecting to found sensor " << foundSens.serialNumber << " on IO system " << foundSens.ioType);
        // if a baudRate has been set, override the default given by OpenZen listing
        if (m_baudrate > 0) {
            foundSens.baudRate = m_baudrate;
        }

        auto sensorObtainPair = m_zenClient->obtainSensor(foundSens);

        if (sensorObtainPair.first != ZenSensorInitError_None)
        {
            RCLCPP_FATAL(this->get_logger(),"Cannot connect to sensor found with discovery. Make sure you have the user rights to access serial devices.");
            return;
        }
        m_zenSensor = std::unique_ptr<zen::ZenSensor>( new zen::ZenSensor(std::move(sensorObtainPair.second)));
    } 
    else
    {
        // directly connect to sensor
        RCLCPP_INFO_STREAM(this->get_logger(),"Connecting directly to sensor " << m_sensorName << " over interface " << m_sensorInterface);
        auto sensorObtainPair = m_zenClient->obtainSensorByName(m_sensorInterface, m_sensorName, m_baudrate);

        if (sensorObtainPair.first != ZenSensorInitError_None)
        {
            RCLCPP_FATAL(this->get_logger(),"Cannot connect directly to sensor.  Make sure you have the user rights to access serial devices.");
            return;
        }
        m_zenSensor = std::unique_ptr<zen::ZenSensor>( new zen::ZenSensor(std::move(sensorObtainPair.second)));
    }

    {
        std::string deviceName = m_zenSensor->deviceName();
        RCLCPP_INFO_STREAM(this->get_logger(),"Sensor name is " << deviceName);
        
        std::map<std::string, DefaultGyro> mapDeviceToDefaultGyro = {
            // NAV series
            {"LPMS-NAV3-CAN", Gyro1AsDefault}, {"LPMS-NAV3-RS232", Gyro1AsDefault}, {"LPMS-NAV3-RS485", Gyro1AsDefault}, {"LPMS-NAV3-TTL", Gyro1AsDefault},

            // CURS and AL series share the same names
            {"LPMS-CURS3-TTL", Gyro2AsDefault}, {"LPMS-CURS3-RS232", Gyro2AsDefault}, {"LPMS-CURS3-CAN", Gyro2AsDefault},

            // U series
            {"LPMS-CU3", Gyro2AsDefault}, {"LPMS-URS3", Gyro2AsDefault}, {"LPMS-UTTL3", Gyro2AsDefault},

            // IG1 series
            {"LPMS-IG1-CAN", Gyro1AsDefault}, {"LPMS-IG1-RS232", Gyro1AsDefault}, {"LPMS-IG1-RS485", Gyro1AsDefault},

            // IG1 (with GPS)
            {"LPMS-IG1P-CAN", Gyro1AsDefault}, {"LPMS-IG1P-RS232", Gyro1AsDefault}, {"LPMS-IG1P-RS485", Gyro1AsDefault},

            // BE series
            {"LPMS-BE1", Gyro2AsDefault}, {"LPMS-BE2", Gyro2AsDefault}, 
        };
        
        auto it = mapDeviceToDefaultGyro.find(deviceName);
        if (it != mapDeviceToDefaultGyro.end()) 
            m_defaultGyroIdx = it->second;
        else
            // match every other legacy sensor
            m_defaultGyroIdx = Gyro1AsDefault;
    }
}

bool OpenZenSensor::run(void)
{
    if (!m_zenClient)
    {
        RCLCPP_FATAL(this->get_logger(),"OpenZen could not be started");
        return false;
    }

    if (!m_zenSensor)
    {
        RCLCPP_FATAL(this->get_logger(),"OpenZen sensor could not be connected");
        return false;
    }

    ZenComponentHandle_t zen_imu_component = {0};
    ZenComponentHandle_t zen_gnss_component = {0};

    if (m_sensorInterface == "TestSensor") {
        zen_imu_component.handle = 1;
    }

    auto imuPair = m_zenSensor->getAnyComponentOfType(g_zenSensorType_Imu);

    RCLCPP_INFO(this->get_logger(),"IMU component found");
    zen_imu_component.handle = imuPair->component().handle;

    m_sensorThread.start( SensorThreadParams{
        m_zenClient.get(),
        frame_id,
        frame_id_gnss,
        m_useLpmsAccelerationConvention,
        zen_imu_component,
        zen_gnss_component
    } );

    RCLCPP_INFO(this->get_logger(),"Data streaming from sensor started");

    return true;
}

