#include <Eigen/Eigen>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <quadrotor_msgs/GoalSet.h>
#include <nav_msgs/Odometry.h>
#include <uav_utils/geometry_utils.h>
#include <coordinate_init/fact_initializer_fsm.h>

int main(int argc, char **argv)
{
    int drone_id = 0;
    for (int i = 0; i < argc; ++i) {
        std::string arg_val = argv[i];
        if (arg_val.find("_drone_id:=") != std::string::npos) {
            std::string id_str = arg_val.substr(arg_val.find(":=") + 2);
            drone_id = std::stoi(id_str);  
            // ROS_ERROR("find drone_id");
            break;
        }
        // ROS_ERROR("find no drone_id");
    }  

    ros::init(argc, argv, "coordinate_init_node");
    ros::NodeHandle nh("~");

    fact_initializer::FACTInitializerFSM LeonInitializer;
    LeonInitializer.init(nh, drone_id);
    
    ros::spin();

    return 0;
}