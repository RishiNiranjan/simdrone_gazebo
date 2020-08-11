cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

##############Plugin for IMU natice sensor
add_library( plugin_imu_native SHARED 
    src/plugin_imu_native.cpp
    src/plugin_imu_native.h
    )
target_link_libraries(plugin_imu_native ${GAZEBO_LIBRARIES} )

#############Plugin for drone model
add_library( plugin_drone SHARED
    src/plugin_drone.cpp
    src/pid_controller.cpp
    src/plugin_drone.h
    src/pid_controller.h
)
target_link_libraries( plugin_drone ${GAZEBO_LIBRARIES} )

#############A program using the keyboard to control the quadrotor #####  
set(CMAKE_AUTOUIC ON)

find_package(Qt5 COMPONENTS Widgets REQUIRED)
set(QT_LIBRARIES Qt5::Widgets)

qt5_wrap_cpp(QT_MOC src/DialogKeyboard.h)
qt5_add_resources(RESOURCES src/drone_keyboard.qrc)

add_executable( drone_keyboard
  src/DialogKeyboard.cpp
  src/drone_object_ros.cpp
  src/drone_keyboard.cpp
  ${RESOURCES}
  ${QT_MOC}
)

target_link_libraries( drone_keyboard
  ${QT_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
