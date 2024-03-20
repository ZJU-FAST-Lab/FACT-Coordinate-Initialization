#include <coordinate_init/fact_random_init.h>

namespace fact_initializer
{
    void FACTRandomInit::InitRun(ros::NodeHandle &nh, int argc_drone_id, int seed)
    {
        std::srand(seed+argc_drone_id);
        //修改下列代码，不用控制三架飞机，而是控制argc_drone_id指定的那一架飞机
        DroneInfo onedrone(argc_drone_id);

        has_odom = false;

        // 发布cmd
        std::stringstream topic_name_stream;
        topic_name_stream << "/drone_" << argc_drone_id << "_planning/pos_cmd";
        ros::Publisher pose_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>(topic_name_stream.str(), 10);
        
        // 订阅odom
        std::string odom_topic;
        nh.getParam("odom_topic", odom_topic);
        std::string current_drone_odom_topic = "/drone_" + std::to_string(argc_drone_id) + "_" + "visual_slam/odom";
        ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(current_drone_odom_topic, 10, [this, &onedrone](const nav_msgs::Odometry::ConstPtr& msg) { 
            onedrone.position.x() = msg->pose.pose.position.x;
            // ROS_INFO("Drone_%d Position - X: %f", onedrone.drone_id, onedrone.position.x());
            onedrone.position.y() = msg->pose.pose.position.y;
            onedrone.position.z() = msg->pose.pose.position.z;

            tf::Quaternion q;
            tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
            tf::Matrix3x3 m(q);
            m.getRPY(onedrone.roll, onedrone.pitch, onedrone.yaw);
            has_odom = true;
        });

        // onedrone.goal = generateRandomPoint();
        // 随机生成0到1之间的数

        // double x = onedrone.position.x() + static_cast<double>(rand()) / RAND_MAX * 0.8 - 0.4; // -2.5 到 2.5
        // double y = onedrone.position.y() + static_cast<double>(rand()) / RAND_MAX * 0.8 - 0.4; // -2.5 到 2.5
        sleep(2);
        double x = onedrone.position.x();
        ROS_INFO("Drone_%d Position - X: %f", onedrone.drone_id, onedrone.position.x());
        double y = onedrone.position.y();
        double z = onedrone.position.z();
        onedrone.goal = Eigen::Vector3d(x, y, z);


        onedrone.goal_yaw = generateRandomYaw();
        // onedrone.goal_yaw = M_PI*3;
        onedrone.hasReachedGoal = false;

        ros::Rate rate(10);
        while (ros::ok()) {  
            if (checkIfReachedGoal(onedrone.position, onedrone.goal, onedrone.yaw, onedrone.goal_yaw)) 
            {
                onedrone.hasReachedGoal = true;
            }

            if (!onedrone.hasReachedGoal) 
            {
                quadrotor_msgs::PositionCommand cmd;
                cmd.position.x = onedrone.goal.x();
                cmd.position.y = onedrone.goal.y();
                cmd.position.z = onedrone.goal.z();
                cmd.yaw = onedrone.goal_yaw;
                pose_cmd_pub.publish(cmd);
                // 新增的代码
                // ROS_INFO("Drone %d Position - X: %f, Y: %f, Yaw: %f", i, drones[i].position.x(), drones[i].position.y(), drones[i].yaw);
                // ROS_INFO("Drone %d Goal     - X: %f, Y: %f, Yaw: %f", i, drones[i].goal.x(), drones[i].goal.y(), drones[i].goal_yaw);
                // ROS_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            }
            else 
            {
                ROS_INFO("Drone_%d has reached its goal.", onedrone.drone_id);
                break;
            }
            ros::spinOnce();
            rate.sleep();
        }

        ROS_INFO("Drone_%d has reached their goals.", onedrone.drone_id);
    }

    Eigen::Vector3d FACTRandomInit::generateRandomPoint() 
    {
        // double x = (std::rand() % 8000 - 4000) / 1000.0; // -4 到 4
        // double y = (std::rand() % 8000 - 4000) / 1000.0; // -4 到 4
        // double z = (std::rand() % 800 + 200) / 1000.0;   // 0.2 到 1
        double x = static_cast<double>(rand()) / RAND_MAX * 0.8 - 0.4; // -2.5 到 2.5
        double y = static_cast<double>(rand()) / RAND_MAX * 0.8 - 0.4; // -2.5 到 2.5
        double z = static_cast<double>(rand()) / RAND_MAX * 0.5  + 0.3; // 0.2 到 1
        return Eigen::Vector3d(x, y, z);
    }

    double FACTRandomInit::generateRandomYaw()
    {
        return -M_PI + static_cast<double>(std::rand()) / (static_cast<double>(RAND_MAX / (2 * M_PI))); // -π 到 π
    }

    bool FACTRandomInit::checkIfReachedGoal(const Eigen::Vector3d& position, const Eigen::Vector3d& goal, double yaw, double goal_yaw) 
    {
        const double position_threshold = 0.1; // 到达位置的阈值
        const double yaw_threshold = 0.1; // 到达姿态的阈值
        return (position - goal).norm() < position_threshold && std::abs(yaw - goal_yaw) < yaw_threshold;
    }

    void FACTRandomInit::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, DroneInfo &drone) 
    {
        drone.position.x() = msg->pose.position.x;
        drone.position.y() = msg->pose.position.y;
        drone.position.z() = msg->pose.position.z;

        tf::Quaternion q;
        tf::quaternionMsgToTF(msg->pose.orientation, q);
        tf::Matrix3x3 m(q);
        m.getRPY(drone.roll, drone.pitch, drone.yaw);
    }
}