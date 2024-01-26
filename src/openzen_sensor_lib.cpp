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

                sensor_msgs::msg::Imu imu_msg;
                sensor_msgs::msg::MagneticField mag_msg;

                imu_msg.header.stamp = this->now();
                imu_msg.header.frame_id = param.frame_id;

                // Fill orientation quaternion
                imu_msg.orientation.w = d.q[0];
                imu_msg.orientation.x = -d.q[1];
                imu_msg.orientation.y = -d.q[2];
                imu_msg.orientation.z = -d.q[3];

                // Fill angular velocity data
                // - scale from deg/s to rad/s
                switch (m_defaultGyroIdx) {
                    case Gyro1AsDefault:
                        imu_msg.angular_velocity.x = d.g1[0] * cDegToRad;
                        imu_msg.angular_velocity.y = d.g1[1] * cDegToRad;
                        imu_msg.angular_velocity.z = d.g1[2] * cDegToRad;
                        break;
                    case Gyro2AsDefault:
                        imu_msg.angular_velocity.x = d.g2[0] * cDegToRad;
                        imu_msg.angular_velocity.y = d.g2[1] * cDegToRad;
                        imu_msg.angular_velocity.z = d.g2[2] * cDegToRad;
                        break;
                }

                // Fill linear acceleration data
                const float rosConversion = -1.0 * (!param.useLpmsAccelerationConvention) +
                    1.0 * param.useLpmsAccelerationConvention;

                imu_msg.linear_acceleration.x = rosConversion * d.a[0] * cEarthG;
                imu_msg.linear_acceleration.y = rosConversion * d.a[1] * cEarthG;
                imu_msg.linear_acceleration.z = rosConversion * d.a[2] * cEarthG;

                //TODO covariance

                mag_msg.header.stamp = imu_msg.header.stamp;
                mag_msg.header.frame_id = param.frame_id;

                // Units are microTesla in the LPMS library, Tesla in ROS.
                mag_msg.magnetic_field.x = d.b[0] * cMicroToTelsa;
                mag_msg.magnetic_field.y = d.b[1] * cMicroToTelsa;
                mag_msg.magnetic_field.z = d.b[2] * cMicroToTelsa;

                // Publish the messages
                imu_pub->publish(imu_msg);
                mag_pub->publish(mag_msg);
            }
        } else if (event_value->component == param.zen_gnss_component) {
            if (event_value->eventType == ZenEventType_GnssData) {
                // Global navigation satellite system
                auto const& d = event_value->data.gnssData;

                sensor_msgs::msg::NavSatFix nav_msg;
                sensor_msgs::msg::NavSatStatus nav_status;
                nav_status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;

                if (d.fixType == ZenGnssFixType_2dFix ||
                    d.fixType == ZenGnssFixType_3dFix ||
                    d.fixType == ZenGnssFixType_GnssAndDeadReckoning){
                        nav_status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
                    }

                // even better, do we have an RTK navigation solution ?
                if (d.carrierPhaseSolution == ZenGnssFixCarrierPhaseSolution_FloatAmbiguities ||
                    d.carrierPhaseSolution == ZenGnssFixCarrierPhaseSolution_FixedAmbiguities) {
                        nav_status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
                }

                // OpenZen does not output the exact satellite service so assume its
                // only GPS for now
                nav_status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

                nav_msg.status = nav_status;
                nav_msg.latitude = d.latitude;
                nav_msg.longitude = d.longitude;
                nav_msg.altitude = d.height;

                // initialize all members to zero
                nav_msg.position_covariance = {0};
                // OpenZen provides accuracy at 1-sigma in meters
                // here we need covariance entries with m^2
                nav_msg.position_covariance[0] = std::pow(d.horizontalAccuracy, 2);
                nav_msg.position_covariance[4] = std::pow(d.verticalAccuracy, 2);
                // OpenZen GNNS Sensor does not provide an height estimation. Assume a
                // conservative height estimation of 10 meters accuracy.
                nav_msg.position_covariance[8] = std::pow(10.0, 2);

                nav_msg.position_covariance_type = nav_msg.COVARIANCE_TYPE_APPROXIMATED;

                nav_msg.header.stamp = this->now();
                nav_msg.header.frame_id = param.frame_id_gnss;

                nav_pub->publish(nav_msg);
            }
        }
            
        return true;
    })
{
    // Get node parameters

    this->declare_parameter("sensor_name", "");
    this->declare_parameter("sensor_interface", "LinuxDevice");
    this->declare_parameter("openzen_verbose",false);
    this->declare_parameter("baudrate",0);

    this->declare_parameter("use_lpms_acceleration_convention",false);
    this->declare_parameter("frame_id","imu");
    this->declare_parameter("frame_id_gnss","gnss");
    
    m_sensorName = this->get_parameter("sensor_name").as_string();
    m_sensorInterface = this->get_parameter("sensor_interface").as_string();
    m_openzenVerbose = this->get_parameter("openzen_verbose").as_bool();
    m_baudrate = this->get_parameter("baudrate").as_int();

    m_useLpmsAccelerationConvention = this->get_parameter("use_lpms_acceleration_convention").as_bool();
    frame_id = this->get_parameter("frame_id").as_string();
    frame_id_gnss = this->get_parameter("frame_id_gnss").as_string();


    // Publisher
    imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("data", 1);
    mag_pub = this->create_publisher<sensor_msgs::msg::MagneticField>("mag", 1);
    autocalibration_status_pub = this->create_publisher<std_msgs::msg::Bool>("is_autocalibration_active", 1);

    // Services
    
    // autocalibration_serv = this->create_service<std_srvs::srv::SetBool>("enable_gyro_autocalibration", std::bind(&OpenZenSensor::setAutocalibration , this, std::placeholders::_1,std::placeholders::_2));
    // gyrocalibration_serv = this->create_service<std_srvs::srv::SetBool>("calibrate_gyroscope", std::bind(&OpenZenSensor::calibrateGyroscope , this, std::placeholders::_1,std::placeholders::_2));
    // autocalibration_serv = this->create_service<std_srvs::srv::SetBool>("reset_heading", std::bind(&OpenZenSensor::resetHeading , this, std::placeholders::_1,std::placeholders::_2));


    auto clientPair = zen::make_client();
    m_zenClient = std::unique_ptr<zen::ZenClient>(new zen::ZenClient(std::move(clientPair.second)));

    if (clientPair.first != ZenError_None)
    {
        RCLCPP_FATAL(this->get_logger(),"Cannot start OpenZen");
        return;
    }

    if (m_openzenVerbose)
    {
        ZenSetLogLevel(ZenLogLevel_Debug);
    } 
    else
    {
        ZenSetLogLevel(ZenLogLevel_Off);
    }

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
    publishIsAutocalibrationActive();

    m_sensorThread.start( SensorThreadParams{
        m_zenClient.get(),
        frame_id,
        frame_id_gnss,
        imu_pub,
        mag_pub,
        nav_pub,
        m_useLpmsAccelerationConvention,
        zen_imu_component,
        zen_gnss_component
    } );

    RCLCPP_INFO(this->get_logger(),"Data streaming from sensor started");

    return true;
}


///////////////////////////////////////////////////
// Service Callbacks
///////////////////////////////////////////////////
void OpenZenSensor::publishIsAutocalibrationActive()
{
    std_msgs::msg::Bool msg;

    if (!m_zenImu) {
        RCLCPP_INFO(this->get_logger(),"No IMU compontent available, can't publish autocalibration status");
        return;
    }

    auto resPair = m_zenImu->getBoolProperty(ZenImuProperty_GyrUseAutoCalibration);
    auto error = resPair.first;
    auto useAutoCalibration = resPair.second;
    if (error) 
    {
        RCLCPP_INFO(this->get_logger(),"get autocalibration Error");
    }
    else 
    {
        msg.data = useAutoCalibration;
        autocalibration_status_pub->publish(msg);   
    }
}

bool OpenZenSensor::setAutocalibration (std_srvs::srv::SetBool::Request &req, std_srvs::srv::SetBool::Response &res)
{
    RCLCPP_INFO(this->get_logger(),"set_autocalibration");

    std::string msg;

    if (!m_zenImu) {
        RCLCPP_INFO(this->get_logger(),"No IMU compontent available, can't set autocalibration status");
        return false;
    }

    if (auto error = m_zenImu->setBoolProperty(ZenImuProperty_GyrUseAutoCalibration, req.data))
    {
        RCLCPP_INFO(this->get_logger(),"set autocalibration Error");
        res.success = false; 
        msg.append(std::string("[Failed] current autocalibration status set to: ") + (req.data?"True":"False"));
    
    }
    else
    {
        res.success = true;
        msg.append(std::string("[Success] autocalibration status set to: ") + (req.data?"True":"False"));
    }

    publishIsAutocalibrationActive();        
    res.message = msg;

    return res.success;
}

bool OpenZenSensor::resetHeading (std_srvs::srv::Trigger::Request &req, std_srvs::srv::Trigger::Response &res)
{
    if (!m_zenImu) {
        RCLCPP_INFO(this->get_logger(),"No IMU compontent available, can't reset heading");
        return false;
    }

    RCLCPP_INFO(this->get_logger(),"reset_heading");
    // Offset reset parameters:
    // 0: Object reset
    // 1: Heading reset
    // 2: Alignment reset
    if (auto error = m_zenImu->setInt32Property( ZenImuProperty_OrientationOffsetMode, 1)) 
    {
        RCLCPP_INFO(this->get_logger(),"Error");
        res.success = false;
        res.message = "[Failed] Heading reset";
    } 
    else 
    {
        res.success = true;
        res.message = "[Success] Heading reset";
    }
    return res.success;
}

bool OpenZenSensor::calibrateGyroscope (std_srvs::srv::Trigger::Request &req, std_srvs::srv::Trigger::Response &res)
{
    if (!m_zenImu) {
        RCLCPP_INFO(this->get_logger(),"No IMU compontent available, can't start autocalibration");
        return false;
    }

    RCLCPP_INFO(this->get_logger(),"calibrate_gyroscope: Please make sure the sensor is stationary for 4 seconds");

    if (auto error = m_zenImu->executeProperty(ZenImuProperty_CalibrateGyro))
    {
        RCLCPP_INFO(this->get_logger(),"Error");

        res.success = false;
        res.message = "[Failed] Gyroscope calibration procedure error";
    }
    else
    {
        rclcpp::sleep_for(std::chrono::seconds(4));
        res.success = true;
        res.message = "[Success] Gyroscope calibration procedure completed";
        RCLCPP_INFO(this->get_logger(),"calibrate_gyroscope: Gyroscope calibration procedure completed");

    }
    return res.success;
}

