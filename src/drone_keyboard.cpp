#include <iostream>
#include <QtWidgets>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include "drone_object_ros.h"
#include "DialogKeyboard.h"
#if GAZEBO_MAJOR_VERSION < 6
#include <gazebo/gazebo.hh>
#else
#include <gazebo/gazebo_client.hh>
#endif



int main(int argc, char** argv)
{
    #if GAZEBO_MAJOR_VERSION < 6
    gazebo::setupClient(_argc, _argv);
    #else
    gazebo::client::setup(argc, argv);
    #endif

    QApplication app(argc, argv);
    
    gazebo::DroneObjectROS drone;
    
    drone.pubCmd->WaitForConnection();
      
    DialogKeyboard dlg_keyboard;
    dlg_keyboard.setDrone(drone);
    dlg_keyboard.show();
  
    return app.exec();
}
