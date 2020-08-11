#include "drone_object_ros.h"

namespace gazebo
{
    bool DroneObjectROS::move(float lr, float fb, float ud, float w) {
        msgs::Set(twist_msg.mutable_linear(), ignition::math::Vector3d(1.0, 1.0, ud));
        msgs::Set(twist_msg.mutable_angular(), ignition::math::Vector3d(lr,fb,w));
        this->pubCmd->Publish(twist_msg);
        return true;
    }

    bool DroneObjectROS::moveTo(float x, float y, float z) {
        msgs::Set(twist_msg.mutable_linear(), ignition::math::Vector3d(x, y, z));
        msgs::Set(twist_msg.mutable_angular(), ignition::math::Vector3d(0,0,0));
        this->pubCmd->Publish(twist_msg);
        return true;
    }

    bool DroneObjectROS::pitch(float speed) {
        msgs::Set(twist_msg.mutable_linear(), ignition::math::Vector3d(1.0, 1.0, 0.0));
        msgs::Set(twist_msg.mutable_angular(), ignition::math::Vector3d(0.0,speed,0.0));
        this->pubCmd->Publish(twist_msg);
        return true;
    }

    bool DroneObjectROS::roll(float speed) {
        msgs::Set(twist_msg.mutable_linear(), ignition::math::Vector3d(1.0, 1.0, 0));
        msgs::Set(twist_msg.mutable_angular(), ignition::math::Vector3d(speed,0.0,0.0));
        this->pubCmd->Publish(twist_msg);
        return true;
    }

    bool DroneObjectROS::rise(float speed) {
        msgs::Set(twist_msg.mutable_linear(), ignition::math::Vector3d(0.0, 0.0, speed));
        msgs::Set(twist_msg.mutable_angular(), ignition::math::Vector3d(0,0,0));
        this->pubCmd->Publish(twist_msg);
        return true;
    }

    bool DroneObjectROS::yaw(float speed) {
        msgs::Set(twist_msg.mutable_linear(), ignition::math::Vector3d(0.0, 0.0, 0.0));
        msgs::Set(twist_msg.mutable_angular(), ignition::math::Vector3d(0.0, 0.0, speed));
        this->pubCmd->Publish(twist_msg);
        return true;
    }

    bool DroneObjectROS::land(){
        msgs::Set(twist_msg.mutable_linear(), ignition::math::Vector3d(0.0, 0.0, 0.0));
        msgs::Set(twist_msg.mutable_angular(), ignition::math::Vector3d(0.0, 0.0, 0.0));
        this->pubCmd->Publish(twist_msg);
        return true;
    }

}