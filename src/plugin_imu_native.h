#ifndef PLUGIN_ROS_IMU_NATIVE_H
#define PLUGIN_ROS_IMU_NATIVE_H

#include "gazebo/common/Plugin.hh"
#include "gazebo/gazebo.hh"

namespace gazebo
{
    class myImuPlugin : public SensorPlugin
    {
    public:
        myImuPlugin()
        {
            topicName = "/drone/imu";
        }
        virtual ~myImuPlugin() {}

        virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
        virtual void onUpdated();

    protected:
        sensors::ImuSensorPtr imu_;
        event::ConnectionPtr updated_conn_;

        transport::NodePtr node_handle_;
        transport::PublisherPtr imuPub;
        std::string topicName;
    };
} // namespace gazebo

#endif // PLUGIN_ROS_IMU_NATIVE_H
