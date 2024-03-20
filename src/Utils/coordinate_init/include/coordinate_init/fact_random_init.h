#ifndef FACT_RANDOM_INIT_H
#define FACT_RANDOM_INIT_H

#include <ros/ros.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <string>
#include <tf/tf.h>

namespace fact_initializer
{
    class FACTRandomInit 
    {
        public:
            FACTRandomInit() {}
            ~FACTRandomInit() {}
            void InitRun(ros::NodeHandle &nh, int argc_drone_id, int seed = static_cast<int>(std::time(nullptr)));

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        private:
            struct DroneInfo {
                int drone_id;
                Eigen::Vector3d position;
                Eigen::Vector3d goal;
                bool hasReachedGoal;
                double roll, pitch, yaw; // 当前姿态角
                double goal_yaw; // 目标偏航角
                DroneInfo() : drone_id(-1), position(Eigen::Vector3d::Zero()), goal(Eigen::Vector3d::Zero()), hasReachedGoal(false), roll(0), pitch(0), yaw(0), goal_yaw(0) {}
                DroneInfo(int id) : drone_id(id), position(Eigen::Vector3d::Zero()), goal(Eigen::Vector3d::Zero()), hasReachedGoal(false), roll(0), pitch(0), yaw(0), goal_yaw(0) {}
            };   
            // DroneInfo onedrone;    
            Eigen::Vector3d generateRandomPoint();
            double generateRandomYaw();
            bool checkIfReachedGoal(const Eigen::Vector3d& position, const Eigen::Vector3d& goal, double yaw, double goal_yaw);
            void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, DroneInfo &drone);
            bool has_odom;
    };
}

#endif