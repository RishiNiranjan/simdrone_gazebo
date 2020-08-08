#include "plugin_imu_native.h"
#include "gazebo/sensors/ImuSensor.hh"

namespace gazebo
{
    void myImuPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
    {
        if (!_sensor)
            gzerr << "Invalid sensor pointer.\n";

        this->imu_ = std::dynamic_pointer_cast<sensors::ImuSensor>(_sensor);
        this->imu_->SetUpdateRate(1.0);
        
        this->node_handle_ = transport::NodePtr(new transport::Node());
        this->node_handle_->Init();
        
        this->imuPub = node_handle_->Advertise<msgs::IMU>("/drone/imu");
        
        this->updated_conn_ = this->imu_->ConnectUpdated(boost::bind(&myImuPlugin::onUpdated, this));
        gzdbg << "rishi plugin_imu_native Loaded\n";
    }

    void myImuPlugin::onUpdated()
    {
        msgs::IMU imu_msg = this->imu_->ImuMessage();
        this->imuPub->WaitForConnection();
        // gzerr <<"----\n";
        // gzdbg << imu_msg.orientation().w();
        // gzdbg << imu_msg.orientation().x();
        // gzdbg << imu_msg.orientation().y();
        // gzdbg << imu_msg.orientation().z();
        // gzdbg << imu_msg.angular_velocity().x();
        // gzerr <<"----\n";
        // gzdbg << imu_msg.angular_velocity().y();
        // gzerr <<"----\n";
        // gzdbg << imu_msg.angular_velocity().z();
        // gzerr <<"----\n";
        this->imuPub->Publish(imu_msg);
    }

    GZ_REGISTER_SENSOR_PLUGIN(myImuPlugin)
} // namespace gazebo