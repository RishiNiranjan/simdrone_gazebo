#include "plugin_imu_native.h"
#include "gazebo/sensors/ImuSensor.hh"

namespace gazebo
{
    void myImuPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
    {
        if (!_sensor)
            gzerr << "Invalid sensor pointer.\n";

        this->imu_ = std::dynamic_pointer_cast<sensors::ImuSensor>(_sensor);

        if (!this->imu_)
        {
            gzerr << "ImuPlugin equires a ImuSensor.\n";
            return;
        }

        this->node_handle_ = transport::NodePtr(new transport::Node());
        this->pub = node_handle_->Advertise<msgs::IMU>(this->topicName);
        this->updated_conn_ = this->imu_->ConnectUpdated(boost::bind(&myImuPlugin::onUpdated, this));
        gzdbg << topicName ;
        gzdbg << "rishi plugin_imu_native Loaded\n";
    }

    void myImuPlugin::onUpdated()
    {
        msgs::IMU imu_msg = this->imu_->ImuMessage();
        this->pub->Publish(imu_msg);
        //gzdbg << "published called \n"; runnning fastly
    }

    GZ_REGISTER_SENSOR_PLUGIN(myImuPlugin)
} // namespace gazebo