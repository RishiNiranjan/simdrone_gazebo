#ifndef ARDRONE_ROS_H
#define ARDRONE_ROS_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>

namespace gazebo
{
    class DroneObjectROS
    {
    public:
        DroneObjectROS() {
            this->isVelMode = 0;
            this->node = gazebo::transport::NodePtr(new gazebo::transport::Node());
            this->node->Init();
            this->pubCmd = this->node->Advertise<gazebo::msgs::Twist>("/cmd/val");
        }

        bool isVelMode;

        transport::NodePtr node;

        transport::PublisherPtr pubCmd;

        msgs::Twist twist_msg;

        bool move(float v_lr, float v_fb, float v_du, float w_lr);
        bool moveTo(float x, float y, float z);
        bool pitch(float speed = 0.2);
        bool roll(float speed = 0.2);
        bool rise(float speed = 0.1);
        bool yaw(float speed = 0.1);
        bool land();
    };
}
#endif // ARDRONE_ROS_H
