cmake_minimum_required(VERSION 2.8.3)
project(unicorn)

find_package(catkin REQUIRED COMPONENTS 
    rospy
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
)

catkin_python_setup()
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    #add_rostest(tests/localization_tester_node.test)
endif()
