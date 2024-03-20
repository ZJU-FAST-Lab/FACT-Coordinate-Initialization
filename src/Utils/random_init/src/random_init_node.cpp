#include <ros/ros.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <string>
#include <tf/tf.h>

struct DroneInfo {
    Eigen::Vector3d position;
    Eigen::Vector3d goal;
    bool hasReachedGoal;
    double roll, pitch, yaw; // 当前姿态角
    double goal_yaw; // 目标偏航角
};

Eigen::Vector3d generateRandomPoint() {
    double x = (std::rand() % 8000 - 4000) / 1000.0; // -4 到 4
    double y = (std::rand() % 8000 - 4000) / 1000.0; // -4 到 4
    double z = (std::rand() % 800 + 200) / 1000.0;   // 0.2 到 1
    return Eigen::Vector3d(x, y, z);
}

double generateRandomYaw() {
    return -M_PI + static_cast<double>(std::rand()) / (static_cast<double>(RAND_MAX / (2 * M_PI))); // -π 到 π
}

bool checkIfReachedGoal(const Eigen::Vector3d& position, const Eigen::Vector3d& goal, double yaw, double goal_yaw) {
    const double position_threshold = 0.1; // 到达位置的阈值
    const double yaw_threshold = 0.1; // 到达姿态的阈值
    return (position - goal).norm() < position_threshold && std::abs(yaw - goal_yaw) < yaw_threshold;
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, DroneInfo &drone) {
    drone.position.x() = msg->pose.position.x;
    drone.position.y() = msg->pose.position.y;
    drone.position.z() = msg->pose.position.z;

    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.orientation, q);
    tf::Matrix3x3 m(q);
    m.getRPY(drone.roll, drone.pitch, drone.yaw);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "random_goal_publisher");
    ros::NodeHandle nh;

    int seed;
    nh.param("random_seed", seed, static_cast<int>(std::time(nullptr)));
    std::srand(seed);

    int drone_num;
    nh.param("drone_num", drone_num, -1);

    std::vector<DroneInfo> drones(drone_num);
    std::vector<ros::Publisher> publishers;
    std::vector<ros::Subscriber> pose_subscribers;
    std::vector<ros::Publisher> info_publishers;
    
    for (int i = 0; i < drone_num; ++i) {
        drones[i].goal = generateRandomPoint();
        drones[i].goal_yaw = generateRandomYaw();
        // drones[i].goal_yaw = M_PI*3;        
        drones[i].hasReachedGoal = false;

        std::stringstream topic_name_stream;
        topic_name_stream << "/drone_" << i << "_planning/pos_cmd";
        publishers.push_back(nh.advertise<quadrotor_msgs::PositionCommand>(topic_name_stream.str(), 10));

        std::stringstream pose_topic_stream;
        pose_topic_stream << "/drone_" << i << "_odom_visualization/pose";
        pose_subscribers.push_back(nh.subscribe<geometry_msgs::PoseStamped>(
            pose_topic_stream.str(), 10, [i, &drones](const geometry_msgs::PoseStamped::ConstPtr& msg) { poseCallback(msg, drones[i]); }));
        
        // std::stringstream info_topic_stream;
        // info_topic_stream << "/drone_" << i << "/pose_euler";
        // info_publishers.push_back(nh.advertise<geometry_msgs::PoseStamped>(info_topic_stream.str(), 10));
    }

    //构建参数名并存储位置和姿态到参数服务器
    for (int i = 0; i < drone_num; ++i) {
        std::stringstream param_name_stream;
        param_name_stream << "drone_" << i;
        std::string param_name = param_name_stream.str();
        nh.setParam(param_name + "/x/time0", drones[i].goal.x());
        nh.setParam(param_name + "/y/time0", drones[i].goal.y());
        nh.setParam(param_name + "/z/time0", drones[i].goal.z());
        nh.setParam(param_name + "/yaw/time0", drones[i].goal_yaw);       
    }

    ros::Rate rate(10);
    while (ros::ok()) {
        for (int i = 0; i < drone_num; ++i) {
            // // 创建并填充新的消息
            // geometry_msgs::PoseStamped pose_and_euler_msg;
            // pose_and_euler_msg.header.stamp = ros::Time::now();
            // pose_and_euler_msg.pose.position.x = drones[i].position.x();
            // pose_and_euler_msg.pose.position.y = drones[i].position.y();
            // pose_and_euler_msg.pose.position.z = drones[i].position.z();
            // // 转换欧拉角为四元数
            // tf::Quaternion q = tf::createQuaternionFromRPY(drone[i].roll, drone[i].pitch, drone[i].yaw);
            // tf::quaternionTFToMsg(q, pose_and_euler_msg.pose.orientation);
            // // 发布消息
            // info_publishers[i].publish(pose_and_euler_msg);            

            if (!drones[i].hasReachedGoal) {
                quadrotor_msgs::PositionCommand cmd;
                cmd.position.x = drones[i].goal.x();
                cmd.position.y = drones[i].goal.y();
                cmd.position.z = drones[i].goal.z();
                cmd.yaw = drones[i].goal_yaw;
                publishers[i].publish(cmd);
                // 新增的代码
                ROS_INFO("Drone %d Position - X: %f, Y: %f, Yaw: %f", i, drones[i].position.x(), drones[i].position.y(), drones[i].yaw);
                ROS_INFO("Drone %d Goal     - X: %f, Y: %f, Yaw: %f", i, drones[i].goal.x(), drones[i].goal.y(), drones[i].goal_yaw);
                // ROS_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                if (checkIfReachedGoal(drones[i].position, drones[i].goal, drones[i].yaw, drones[i].goal_yaw)) {
                    drones[i].hasReachedGoal = true;
                    ROS_INFO("Drone %d has reached its goal.", i);
                }
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("All drones have reached their goals.");
    return 0;
}
