#include <coordinate_init/fact_initializer_fsm.h>
#include <fstream>
#include <fcntl.h>
#include <ceres/ceres.h>

namespace fact_initializer
{
    void FACTInitializerFSM::init(ros::NodeHandle &nh, int argc_drone_id)
    {
        std::srand(seed+argc_drone_id+1);

        drone_datainfo.drone_id = argc_drone_id;
        exec_type = INIT;
        exec_timer = nh.createTimer(ros::Duration(0.01), &FACTInitializerFSM::execFSMCallback, this);
        heartbeat_pub = nh.advertise<std_msgs::Empty>("planning/heartbeat", 10);
        is_rotated = false;
        is_gotobservation = false;
        have_odom = false;
        SDP_is_solved = false;

        nh.param("observe_num", observe_num,-1);
        nh.param("rotation_speed", rotation_speed, -1.0);

        std::string odom_topic;
        nh.getParam("odom_topic", odom_topic);
        std::string current_drone_odom_topic = "/drone_" + std::to_string(drone_datainfo.drone_id) + "_" + odom_topic;
        odom_sub = nh.subscribe(current_drone_odom_topic, 1, &FACTInitializerFSM::odometryCallback, this);

        nh.getParam("drone_num", drone_num);        
        drone_datainfo_others.resize(drone_num);
        odom_sub_others.resize(drone_num);
        for (int i = 0; i < drone_num; ++i) {
            if(i==drone_datainfo.drone_id)
            {
                continue;
            }
            else
            {
                drone_datainfo_others[i].drone_id = i;
                std::string topic_name = "/drone_" + std::to_string(i) + "_" + odom_topic;
                odom_sub_others.push_back(nh.subscribe<nav_msgs::Odometry>(topic_name, 10, [&, i](const nav_msgs::Odometry::ConstPtr& msg) {drone_datainfo_others[i].update(msg);} ));
            }

        }        
        mandatory_stop_sub = nh.subscribe("mandatory_stop", 1, &FACTInitializerFSM::mandatoryStopCallback, this);
        mandatory_stop_pub_ = nh.advertise<std_msgs::Empty>("/mandatory_stop_to_planner", 10);
        pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/drone_" + std::to_string(drone_datainfo.drone_id) + "_planning/pos_cmd", 10);
        cmd_vis_pub = nh.advertise<visualization_msgs::Marker>("planning/position_cmd_vis", 10);
        // 相机FOV参数初始化
        vis_dist = 3.0;
        // vis_dist = 10;
        hor_angle = 40*M_PI/180;
        vert_angle = 30*M_PI/180;
        hor = vis_dist * tan(hor_angle);  // 40度
        vert = vis_dist * tan(vert_angle); // 30度
        origin = Eigen::Vector3d(0, 0, 0);
        left_up = Eigen::Vector3d(vis_dist, hor, vert);
        left_down = Eigen::Vector3d(vis_dist, hor, -vert);
        right_up = Eigen::Vector3d(vis_dist, -hor, vert);
        right_down = Eigen::Vector3d(vis_dist, -hor, -vert);
        cam_vertices1.push_back(origin);
        cam_vertices2.push_back(left_up);
        cam_vertices1.push_back(origin);
        cam_vertices2.push_back(left_down);
        cam_vertices1.push_back(origin);
        cam_vertices2.push_back(right_up);
        cam_vertices1.push_back(origin);
        cam_vertices2.push_back(right_down);
        cam_vertices1.push_back(left_up);
        cam_vertices2.push_back(right_up);
        cam_vertices1.push_back(right_up);
        cam_vertices2.push_back(right_down);
        cam_vertices1.push_back(right_down);
        cam_vertices2.push_back(left_down);
        cam_vertices1.push_back(left_down);
        cam_vertices2.push_back(left_up);   

        std::string endpoint_pub_topic = "/drone_" + std::to_string(drone_datainfo.drone_id) + "/endpoint";
        marker_pub_endpoint = nh.advertise<visualization_msgs::Marker>(endpoint_pub_topic, 10);

        drone_datainfo_rotationinit = drone_datainfo; 
        observe_data_quat_part = Observe_Data(drone_datainfo.drone_id);
        observe_data_yaw_part = Observe_Data(drone_datainfo.drone_id);

        observe_counter=0;
        least_observe_num = (drone_num - 1 + 1) / 2 * 2; // 加1是因为我们想要实现向上取整
        // ROS_WARN("Drone_%d_:least_observe_num:_%d_",drone_datainfo.drone_id,least_observe_num);

        so_n=2;
        std::string observe_pub_topic;
        observe_pub_topic = "/drone_" + std::to_string(drone_datainfo.drone_id) + "/observe_data";
        pub_observe = nh.advertise<coordinate_init::ObserveData>(observe_pub_topic, 10);

        // 初始化订阅者
        get_observe.clear();
        for (int i = 0; i < drone_num; ++i) 
        {
            if (i != drone_datainfo.drone_id) { // 排除自己的ID
                std::string observe_sub_topic = "/drone_" + std::to_string(i) + "/observe_data";
                ros::Subscriber sub = nh.subscribe<coordinate_init::ObserveData>(observe_sub_topic, 10, boost::bind(&FACTInitializerFSM::observeCallback, this, _1, i));
                get_observe.push_back(sub);
            }
        }

        // motion
        goal_pub_ = nh.advertise<quadrotor_msgs::GoalSet>("/goal_user2brig", 10);
        have_arrived_goal=false;

        nh.param("grid_map/map_size_x", my_map_size_width, 20.0);  
        nh.param("grid_map/map_size_y", my_map_size_depth, 50.0);
        nh.param("grid_map/map_size_z", my_map_size_height, 2.0);

        add_noise = true;
        noise_mean = 0.0;
        // nh.param("noise_rate", noise_rate, 0.01);
        std::string solver_name;
        nh.param("solver_type", solver_name, std::string("SDP"));
        output_data.solver = solver_name;
        if (solver_name == "SDP") {
            solver = solver_type::SDP;
        } else if (solver_name == "LM") {
            solver = solver_type::LM;
        } else if (solver_name == "GaussNewton") {
            solver = solver_type::GaussNewton;
        } else {
            ROS_ERROR("Invalid solver type: %s", solver_name.c_str());
            exit(1);
        }
        nh.param("results_path", results_path, std::string("results/result.txt"));
        ROS_INFO("Results will be saved to: %s", results_path.c_str());
        // 读取noise_rate并判断是否读取成功
        if (nh.hasParam("noise_rate")) 
        {
            nh.getParam("noise_rate", noise_rate);
            ROS_INFO("Drone_%d:noise_rate:_%f_",drone_datainfo.drone_id,noise_rate);
            if (drone_datainfo.drone_id == 0)
            {
                std::ofstream outfile(results_path, std::ios::app);
                outfile << "---------------" << std::endl;
                outfile << "drone_num: " << drone_num << std::endl;
                outfile << "noise_rate: " << noise_rate << std::endl;
                outfile << "solver_type: " << output_data.solver << std::endl;
                outfile.close();
            }
        }
        else 
        {
            ROS_ERROR("Drone_%d:Failed to get noise_rate",drone_datainfo.drone_id);
            exit(1);
        }

        // 读取障碍物参数
        int numObstacles = 16; // Assuming there are 16 obstacles
        obstacles.resize(numObstacles);
        for (int i = 0; i < numObstacles; ++i) {
            std::vector<double> obstacleData;
            nh.getParam("obstacle" + std::to_string(i), obstacleData);
            obstacles[i].position << obstacleData[0], obstacleData[1];
            obstacles[i].radius = obstacleData[2];
            // ROS_INFO("Obstacle %d: Position = (%f, %f), Radius = %f", i, obstacleData[0], obstacleData[1], obstacleData[2]);
        }
        output_data.start_time = std::chrono::steady_clock::now();
    }

    void FACTInitializerFSM::execFSMCallback(const ros::TimerEvent &e)
    {
        exec_timer.stop();
        std_msgs::Empty heartbeat_msg;
        heartbeat_pub.publish(heartbeat_msg);

        static int fsm_num = 0;
        fsm_num++;
        if (fsm_num == 100)
        {
            fsm_num = 0;
            printFSMExecState();
        }

        switch (exec_type)
        {
            case INIT:
            {
                if(!have_odom)
                {
                    // ROS_WARN("Drone%d:Have no odom!!!!!!", drone_datainfo.drone_id);
                    break;
                }
                quadrotor_msgs::PositionCommand init_move;
                init_move.position.x = drone_datainfo.odom_pos.x() + static_cast<double>(rand()) / RAND_MAX * 0.8 - 0.4; // -2.5 到 2.5;
                init_move.position.y = drone_datainfo.odom_pos.y() + static_cast<double>(rand()) / RAND_MAX * 0.8 - 0.4; // -2.5 到 2.5;
                init_move.position.z = drone_datainfo.odom_pos.z();
                Eigen::Vector3d init_goal_move;
                init_goal_move.x() = init_move.position.x;
                init_goal_move.y() = init_move.position.y;
                init_goal_move.z() = init_move.position.z;
                init_move.yaw = -M_PI + static_cast<double>(std::rand()) / (static_cast<double>(RAND_MAX / (2 * M_PI))); // -π 到 π
                ros::Rate rate(500);
                while (ros::ok()) 
                {
                    if(((drone_datainfo.odom_pos- init_goal_move ).norm() < 0.01) && AngleEqual(drone_datainfo.angleRPY[2],init_move.yaw,0.01,true))
                    {
                        ROS_INFO("Drone_%d_:Arrived Goal",drone_datainfo.drone_id);
                        break;
                    } 
                    // ROS_WARN("Drone_%d_:Controlling to Goal",drone_datainfo.drone_id);
                    pos_cmd_pub.publish(init_move);
                    ros::spinOnce();
                    rate.sleep();
                }
                exec_type = OBSERVE;
                break;
            }

            case OBSERVE:
            {
                if(!is_rotated)
                {
                    RotateOneCircle(1, rotation_speed);
                    // ROS_INFO("Drone_%d:Rotation is completed!!!!!!!!!!!!!!!!!!!!!!!!!!!!",drone_datainfo.drone_id); 
                    is_rotated=1;
                }
                else if(!is_gotobservation && is_rotated)
                {
                    drone_datainfo_rotationinit = drone_datainfo; 
                    observe_data_quat_part = Observe_Data(drone_datainfo.drone_id);
                    observe_data_yaw_part = Observe_Data(drone_datainfo.drone_id);                    
                    GetPartObservation();                    
                    is_gotobservation=1;
                }
                else if(is_rotated && is_gotobservation)
                {
                    is_rotated = 0;
                    is_gotobservation = 0;
                    exec_type = SOLVE;
                    observe_counter++;
                    publishObserveData(false); // true: use yaw data

                    if(observe_counter==1)
                    {
                        GetAllObservation();
                        recordInitialYawAngles();
                    }
                    else
                    {
                        updateYawAngleDifferences();
                    }
                    ROS_INFO("Drone%d:observe_counter:_%d_",drone_datainfo.drone_id,observe_counter);
                }
                break;
            }

            case SOLVE:
            {
                // solve using the observation
                if(observe_counter<least_observe_num)
                {
                    ROS_WARN("Drone%d:the num of observation is not enough!", drone_datainfo.drone_id);
                }
                else
                {
                    if(!SDP_is_solved)
                    {
                        //record time
                        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
                        SolveMySDP();
                        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
                        std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
                        ROS_INFO("Drone%d:Rotation solving time: %f seconds", drone_datainfo.drone_id, time_span.count());
                        output_data.sdp_time = time_span;
                    }

                    if(SDP_is_solved)
                    {
                        // todo:求解T
                        std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
                        ROS_INFO("Drone%d:observe_counter:_%d_: SDP is solved, then solve T!",drone_datainfo.drone_id,observe_counter);
                        SolveTbyHungarian();
                        std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
                        std::chrono::duration<double> time_span2 = std::chrono::duration_cast<std::chrono::duration<double>>(t4 - t3);
                        ROS_INFO("Drone%d:Translation solving time: %f seconds", drone_datainfo.drone_id, time_span2.count());
                        exec_type = STOP;
                        break;
                    }
                    if(!output_data.finished && (SDP_is_solved || observe_counter >= drone_num + 10)) {
                        struct flock lock;
                        lock.l_type = F_WRLCK; // Write lock
                        lock.l_whence = SEEK_SET; // Lock the whole file
                        lock.l_start = 0;
                        lock.l_len = 0;
                        int fd = open(results_path.c_str(), O_RDWR);
                        fcntl(fd, F_SETLKW, &lock); // Apply the lock
                        std::ofstream result_file(results_path, std::ios::app);
                        result_file << "%%%%%%%" << std::endl;
                        result_file << "drone_id: " << drone_datainfo.drone_id << std::endl;
                        result_file << "counter: " << observe_counter << std::endl;
                        result_file << "angle_result: " << std::endl;
                        for (int i = 0; i < angle_result.size(); ++i) {
                            result_file << angle_result[i] << " ";
                        }
                        result_file << std::endl;
                        result_file << "angle_result_real: " << std::endl;
                        for (int i = 0; i < output_data.angle_result_real.size(); ++i) {
                            result_file << output_data.angle_result_real[i] << " ";
                        }
                        result_file << std::endl;
                        std::chrono::steady_clock::time_point t5 = std::chrono::steady_clock::now();
                        std::chrono::duration<double> time_span3 = std::chrono::duration_cast<std::chrono::duration<double>>(t5 - output_data.start_time);
                        //record in seconds
                        // result_file << "total_time: " << time_span3.count() << std::endl;
                        result_file << "SDP_time: " << output_data.sdp_time.count() << std::endl;
                        result_file.close();
                        lock.l_type = F_UNLCK; // Unlock
                        fcntl(fd, F_SETLK, &lock);
                        close(fd);
                        output_data.finished = true;
                        ROS_INFO("Drone_%d: finish!", drone_datainfo.drone_id);
                    }
                }
                exec_type = MOTION;
                break;
            }

            case MOTION:
            {
                no_control = false;
                need_rotate = false;
                GenerateGoal();
                GoalIsInObstacle();
                // publishGoal(mygoal);
                // ROS_INFO("Drone_%d:waiting to arrive goal x:%f y:%f z:%f",drone_datainfo.drone_id,mygoal[0],mygoal[1],mygoal[2]);
                // WaitingtoArriveGoal();
                if(no_control)
                {
                    if(need_rotate)
                    {
                        // rotate;
                        // 随机一个yaw角
                        double yaw = (double)rand() / RAND_MAX * M_PI;
                        // 随机一个方向
                        bool clockwise = (rand() % 2 == 0);
                        need_rotate = false;
                        no_control = false;
                        ROS_WARN("Drone_%d:No control! No Rotate_ByYaw",drone_datainfo.drone_id);
                    }
                    else
                    {
                        ROS_ERROR("Drone_%d:No control!",drone_datainfo.drone_id);
                        no_control = false;
                    }
                }
                else
                {
                    // ControltoGoal();
                    publishGoal(mygoal);
                    WaitingtoArriveGoal();
                }
        
                exec_type = OBSERVE;

                break;
            }
            
            case STOP:
            {
                // stop
                break;
            }

        }
        exec_timer.start();
    }

    void FACTInitializerFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
    {
        drone_datainfo.odom_pos(0) = msg->pose.pose.position.x;
        drone_datainfo.odom_pos(1) = msg->pose.pose.position.y;
        drone_datainfo.odom_pos(2) = msg->pose.pose.position.z;

        drone_datainfo.odom_vel(0) = msg->twist.twist.linear.x;
        drone_datainfo.odom_vel(1) = msg->twist.twist.linear.y;
        drone_datainfo.odom_vel(2) = msg->twist.twist.linear.z;

        tf2::fromMsg(msg->pose.pose.orientation, drone_datainfo.quat);
        tf2::Matrix3x3(drone_datainfo.quat).getRPY(drone_datainfo.angleRPY(0), drone_datainfo.angleRPY(1), drone_datainfo.angleRPY(2));

        drone_datainfo.angular_vel = Eigen::Vector3d(msg->twist.twist.angular.x,
                                                    msg->twist.twist.angular.y,
                                                    msg->twist.twist.angular.z);

        have_odom = true;

        // FOV可视化
        vector<Eigen::Vector3d> l1, l2;
        getFOV(l1, l2, drone_datainfo.odom_pos, drone_datainfo.angleRPY(2));
        drawFOV(l1, l2);
        if(exec_type==MOTION)
        {
            // 设置箭头的宽度
            marker_endpoint.scale.x = 0.1; // 箭头轴的宽度
            marker_endpoint.scale.y = 0.2; // 箭头头部的宽度
            marker_endpoint.scale.z = 0.2; // 箭头头部的长度
            // 设置颜色和透明度
            marker_endpoint.color.r = 1.0; // 红色
            marker_endpoint.color.g = 0.0;
            marker_endpoint.color.b = 0.0;
            marker_endpoint.color.a = 1.0; // 不透明

            marker_endpoint.header.frame_id = "world";  // 或者任何适合你的参考系
            marker_endpoint.header.stamp = ros::Time::now();
            marker_endpoint.ns = "trajectory";
            marker_endpoint.id = 0;
            marker_endpoint.type = visualization_msgs::Marker::ARROW;

            marker_endpoint.pose.orientation.x = 0.0;
            marker_endpoint.pose.orientation.y = 0.0;
            marker_endpoint.pose.orientation.z = 0.0;
            marker_endpoint.pose.orientation.w = 1.0;

            // Clean old marker
            marker_endpoint.action = visualization_msgs::Marker::DELETE;
            marker_pub_endpoint.publish(marker_endpoint);

            // get marker info
            // 设置箭头的起点和终点
            geometry_msgs::Point start, end;
            start.x = drone_datainfo.odom_pos(0);
            start.y = drone_datainfo.odom_pos(1);
            start.z = drone_datainfo.odom_pos(2); // 起点
            end.x = mygoal[0];
            end.y = mygoal[1];
            end.z = mygoal[2];       // 轨迹终点

            // 将点添加到箭头
            marker_endpoint.points.clear();
            marker_endpoint.points.push_back(start);
            marker_endpoint.points.push_back(end);

            // Pub new marker
            marker_endpoint.action = visualization_msgs::Marker::ADD;
            marker_pub_endpoint.publish(marker_endpoint);
        }
    }

    void FACTInitializerFSM::mandatoryStopCallback(const std_msgs::Empty &msg)
    {
        exec_type = STOP;
        ROS_ERROR("Received a mandatory stop command!");
    }

    void FACTInitializerFSM::printFSMExecState()
    {
        static std::string state_string[5] = {"INIT","OBSERVE","SOLVE","MOTION","STOP"};
        ROS_INFO("Drone_%d_State:__%s__", drone_datainfo.drone_id, state_string[int(exec_type)].c_str());
    } 

    void FACTInitializerFSM::RotateOneCircle(bool clockwise, double rotation_speed) {
        // The difficulty lies in determining the magnitude of angles when they are normalized within the range of -π to π.
        // init
        // observe_timer.start();
        // drone_datainfo_rotationinit = drone_datainfo;
        double initial_yaw = drone_datainfo.angleRPY[2];
        double target_yaw_change = clockwise ? -2 * M_PI : 2 * M_PI;
        double target_yaw = initial_yaw + target_yaw_change;
        double threshhold = 0.01;
        auto normalizeYaw = [](double yaw) {
            while (yaw > M_PI) yaw -= 2 * M_PI;
            while (yaw < -M_PI) yaw += 2 * M_PI;
            return yaw;
        };

        quadrotor_msgs::PositionCommand cmd;
        if(rotation_speed>M_PI)
        {
            // rotation_speed = (clockwise)? -M_PI*0.97:M_PI*0.97;
            rotation_speed = M_PI*0.97;
        }
        double yaw_increment = clockwise ? -rotation_speed : rotation_speed; // 每次增加的角度
        bool noincrease=false;
        double yaw_increment_cumulate=0;
        if(rotation_speed<threshhold)
        {
            threshhold = rotation_speed/100;
        }
        initial_yaw = normalizeYaw(initial_yaw);
        target_yaw = normalizeYaw(target_yaw);

        double current_target_yaw = initial_yaw + yaw_increment;
        yaw_increment_cumulate+=rotation_speed;
        current_target_yaw = normalizeYaw(current_target_yaw);
        double current_yaw = drone_datainfo.angleRPY[2];
        cmd.position.x = drone_datainfo.odom_pos[0];
        cmd.position.y = drone_datainfo.odom_pos[1];
        cmd.position.z = drone_datainfo.odom_pos[2];
        cmd.yaw = current_target_yaw;

        ros::Rate rate(25);
        while (ros::ok()) {
            current_yaw = drone_datainfo.angleRPY[2];
            if (AngleEqual(current_yaw,current_target_yaw,threshhold,true))
            {
                current_target_yaw += yaw_increment;
                yaw_increment_cumulate+=rotation_speed;
                current_target_yaw = normalizeYaw(current_target_yaw);
                if(yaw_increment_cumulate>2*M_PI)
                {
                    // ROS_INFO("Drone_%d:yaw_increment_cumulate>2*M_PI.",drone_datainfo.drone_id);
                    if(AngleMore(target_yaw,current_target_yaw,rotation_speed) && yaw_increment>0)
                    {
                        current_target_yaw=target_yaw;
                        noincrease=true;
                        // ROS_INFO("Drone_%d:noincrease=true.",drone_datainfo.drone_id);
                    }
                    else if(AngleMore(current_target_yaw,target_yaw,rotation_speed) && yaw_increment<0)
                    {
                        current_target_yaw=target_yaw;
                        noincrease=true;
                        // ROS_INFO("Drone_%d:noincrease=true.",drone_datainfo.drone_id);
                    }
                }
                // cmd.position.z = 0.7;
                cmd.yaw = current_target_yaw;
            }       
            else
            {
                // ROS_ERROR("Drone_%d_:No Update Angle_CMD",drone_datainfo.drone_id);
            }  
            pos_cmd_pub.publish(cmd);
            if (noincrease && AngleEqual(current_yaw,current_target_yaw,threshhold,true))
            {
                // ROS_INFO("Drone_%d_:Break:AngleEqual",drone_datainfo.drone_id);
                break; 
            }
            else if(noincrease && yaw_increment>0 && AngleMore(target_yaw,current_target_yaw,1.2*rotation_speed))// 有时候数值比较特殊，上一次马上到，下次刚好更新了odom，就直接在一开始AngleEqual，转过了
            {
                // ROS_INFO("Drone_%d_:Break:AngleMore1",drone_datainfo.drone_id);
                break;
            }
            else if(noincrease && yaw_increment<0 && AngleMore(current_target_yaw,target_yaw, 1.2*rotation_speed))
            {
                // ROS_INFO("Drone_%d_:Break:AngleMore2",drone_datainfo.drone_id);
                break;
            }

            if(noincrease==true)
            {
                // ROS_INFO("Drone_%d_:noincrease==true, but not break!",drone_datainfo.drone_id);
            }
            // ROS_WARN("Drone_%d_:current_target_yaw:_%f_,target_yaw:_%f_,current_yaw:_%f_,yaw_increment:_%f_",drone_datainfo.drone_id,current_target_yaw,target_yaw,current_yaw,yaw_increment);

            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO("Drone_%d_:Drone has completed rotation.",drone_datainfo.drone_id);
        // observe_timer.stop();
    }

    bool FACTInitializerFSM::AngleEqual(double angle1,double angle2,double threshhold,bool is_abs)
    {
        // Warn: this is not mean that two number equal in math. Becauce the angle is in range of -pi to pi.
        double diff = angle2-angle1;
        if(is_abs)
        {
            diff = abs(diff);
        }
        if(diff<threshhold)
        {
            // ROS_INFO("Drone_%d_:Angle is Equal, diff<threshhold",drone_datainfo.drone_id);
            return true;
        }
        else if(diff>2*M_PI-threshhold)
        {
            // ROS_INFO("Drone_%d_:Angle is Equal, diff>2*M_PI-threshhold",drone_datainfo.drone_id);            
            return true;
        }
        else
        {
            // ROS_INFO("Drone_%d_:Angle is not Equal, diff = %f, threshhold = %f.",drone_datainfo.drone_id, diff, threshhold);
            return false;
        }
    }

    bool FACTInitializerFSM::AngleMore(double angle1, double angle2,double rotation_speed)
    {
        double diff = angle2-angle1;
        if(diff>0 || diff<(-2*M_PI+rotation_speed))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    void FACTInitializerFSM::getFOV(vector<Eigen::Vector3d>& list1, vector<Eigen::Vector3d>& list2, Eigen::Vector3d pos, double yaw) 
    {
        list1.clear();
        list2.clear();

        // Get info for visualizing FOV at (pos, yaw)
        Eigen::Matrix3d Rwb;
        Rwb << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
        for (int i = 0; i < cam_vertices1.size(); ++i) 
        {
            auto p1 = Rwb * cam_vertices1[i] + pos;
            auto p2 = Rwb * cam_vertices2[i] + pos;
            list1.push_back(p1);
            list2.push_back(p2);
        }
    }   

    void FACTInitializerFSM::drawFOV(const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2) 
    {
        visualization_msgs::Marker mk;
        mk.header.frame_id = "world";
        mk.header.stamp = ros::Time::now();
        mk.id = 0;
        mk.ns = "current_pose";
        mk.type = visualization_msgs::Marker::LINE_LIST;
        mk.pose.orientation.x = 0.0;
        mk.pose.orientation.y = 0.0;
        mk.pose.orientation.z = 0.0;
        mk.pose.orientation.w = 1.0;
        mk.color.r = 1.0;
        mk.color.g = 0.0;
        mk.color.b = 0.0;
        mk.color.a = 1.0;
        mk.scale.x = 0.04;
        mk.scale.y = 0.04;
        mk.scale.z = 0.04;

        // Clean old marker
        mk.action = visualization_msgs::Marker::DELETE;
        cmd_vis_pub.publish(mk);

        if (list1.size() == 0) return;

        // Pub new marker
        geometry_msgs::Point pt;
        for (int i = 0; i < int(list1.size()); ++i) {
            pt.x = list1[i](0);
            pt.y = list1[i](1);
            pt.z = list1[i](2);
            mk.points.push_back(pt);

            pt.x = list2[i](0);
            pt.y = list2[i](1);
            pt.z = list2[i](2);
            mk.points.push_back(pt);
        }
        mk.action = visualization_msgs::Marker::ADD;
        cmd_vis_pub.publish(mk);
    } 

    void FACTInitializerFSM::observeFSMCallback(const ros::TimerEvent &e)
    {
        observe_timer.stop();
        observe_timer.start();
    }

    void FACTInitializerFSM::GetAllObservation()
    {
        // observe_data initialization
        observe_data_quat.observer_id = drone_datainfo.drone_id;
        observe_data_yaw.observer_id  = drone_datainfo.drone_id;
        observe_data_quat.observed_num = 0;
        observe_data_yaw.observed_num  = 0;
        observe_data_quat.observed_positions.clear();
        observe_data_yaw.observed_positions.clear();
        // observe_data_quat.observed_positions.resize(drone_num-1);
        // observe_data_yaw.observed_positions.resize(drone_num-1);
        init_all_yaw.clear();
        init_all_yaw.reserve(drone_num);

        // rotation by quaternion
        Eigen::Matrix3d rotation_matrix_quat;
        rotation_matrix_quat = Eigen::Quaterniond(drone_datainfo.quat.w(), 
                                                drone_datainfo.quat.x(), 
                                                drone_datainfo.quat.y(), 
                                                drone_datainfo.quat.z()).toRotationMatrix();

        // rotation by yaw
        double yaw = drone_datainfo.angleRPY[2];
        Eigen::Matrix3d rotation_matrix_yaw;
        rotation_matrix_yaw = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        double yaw_temp=0;
        for (int i = 0; i < drone_num; ++i) 
        {
            if (i == drone_datainfo.drone_id) {
                yaw_temp = drone_datainfo.angleRPY[2] - yaw;
                init_all_yaw.push_back(yaw_temp);
                continue; // 跳过当前无人机自己
            }

            // 计算相对位置
            Eigen::Vector3d relative_position = drone_datainfo_others[i].odom_pos - drone_datainfo.odom_pos;

            // 使用两种不同的旋转矩阵转换位置
            Eigen::Vector3d transformed_position_quat = rotation_matrix_quat.transpose() * relative_position;
            Eigen::Vector3d transformed_position_yaw = rotation_matrix_yaw.transpose() * relative_position;

            yaw_temp = drone_datainfo_others[i].angleRPY[2] - yaw;
            init_all_yaw.push_back(yaw_temp);
            // 更新观测数据
            observe_data_quat.observed_positions.push_back(transformed_position_quat);
            observe_data_yaw.observed_positions.push_back(transformed_position_yaw);
            observe_data_quat.observed_num++;
            observe_data_yaw.observed_num++;
            // ROS_WARN("Drone_%d_:observe_num:_%d_:positionBYquat:X_%f_Y_%f_Z_%f_Yaw:_%f_",drone_datainfo.drone_id,observe_data_quat.observed_num,transformed_position_quat(0),transformed_position_quat(1),transformed_position_quat(2), yaw_temp);
            // ROS_WARN("Drone_%d_:observe_num:_%d_:positionBYyaw :X_%f_Y_%f_Z_%f_Yaw:_%f_",drone_datainfo.drone_id,observe_data_yaw.observed_num,transformed_position_yaw(0),transformed_position_yaw(1),transformed_position_yaw(2), yaw_temp);
        }
    }

    void FACTInitializerFSM::GetPartObservation()
    {
        observe_data_quat_part.observed_num=0;
        observe_data_yaw_part.observed_num=0;
        for(int i=0;i<drone_num;i++)
        {
            if(i==drone_datainfo_rotationinit.drone_id)
            {
                continue;
            }
            else
            {
                if(IsinRange(drone_datainfo_rotationinit.odom_pos, drone_datainfo_others[i].odom_pos))
                {
                    // get observation and transform to rotate init
                    calculateRelativePositionQuat(drone_datainfo_others[i]);
                    calculateRelativePositionYaw(drone_datainfo_others[i]);   
                    observe_data_quat_part.observed_num++;
                    observe_data_yaw_part.observed_num++;
                    
                }
            }
        }
        ROS_WARN("Drone_%d_:observe_num:_%d_",drone_datainfo_rotationinit.drone_id,observe_data_quat_part.observed_num);
        // 把自己的观测数据加入到observe_data_quat_part_vector、observe_data_yaw_part_vector中
        observe_data_quat_part_vector.push_back(observe_data_quat_part);
        observe_data_yaw_part_vector.push_back(observe_data_yaw_part);
    }

    bool FACTInitializerFSM::IsinRange(const Eigen::Vector3d pc, const Eigen::Vector3d pw)
    {
        Eigen::Vector3d dir = pw - pc;
        if (dir.norm() > vis_dist) 
        {
            return false;
        }
        return true;
    }

    void FACTInitializerFSM::calculateRelativePositionQuat(const Drone_Data& other_drone) 
    {
        // 使用四元数转换矩阵
        Eigen::Matrix3d rotation_matrix = Eigen::Quaterniond(drone_datainfo.quat.w(), 
                                                            drone_datainfo.quat.x(), 
                                                            drone_datainfo.quat.y(), 
                                                            drone_datainfo.quat.z()).toRotationMatrix();
        Eigen::Vector3d relative_position = other_drone.odom_pos - drone_datainfo.odom_pos;

        if(add_noise)
        {
            // 高斯噪声参数
            std::default_random_engine generator;
            // double stddev = 0.16; // 标准差
            // double stddev = 0.06; // 标准差
            double distance = relative_position.norm();
            double sigmar = distance * noise_rate / 3.0;
            std::normal_distribution<double> distribution(noise_mean, sigmar);
            relative_position[0] += distribution(generator);
            relative_position[1] += distribution(generator);
            relative_position[2] += distribution(generator);
        }
        Eigen::Vector3d transformed_position = rotation_matrix.transpose() * relative_position;

        // ROS_WARN("Drone_%d_:positionBYquat:X_%f_Y_%f_Z_%f_",drone_datainfo.drone_id,transformed_position(0),transformed_position(1),transformed_position(2));
        observe_data_quat_part.observed_positions.push_back(transformed_position);

        // show
        ROS_INFO("The relative position between Drone%d and Drone%d: X:%f Y:%f Z:%f", drone_datainfo.drone_id, other_drone.drone_id, transformed_position(0), transformed_position(1), transformed_position(2));
    }

    void FACTInitializerFSM::calculateRelativePositionYaw(const Drone_Data& other_drone) {
        // 使用偏航角转换矩阵
        double yaw = drone_datainfo.angleRPY[2];
        Eigen::Matrix3d rotation_matrix = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

        Eigen::Vector3d relative_position = other_drone.odom_pos - drone_datainfo.odom_pos;

        if(add_noise)
        {
            // 高斯噪声参数
            double mean = 0.0; // 均值
            std::default_random_engine generator;
            // double stddev = 0.16; // 标准差
            // double stddev = 0.06; // 标准差
            double distance = relative_position.norm();
            double sigmar = distance * noise_rate / 3.0;
            std::normal_distribution<double> distribution(noise_mean, sigmar);
            relative_position[0] += distribution(generator);
            relative_position[1] += distribution(generator);
            relative_position[2] += distribution(generator);
        }

        Eigen::Vector3d transformed_position = rotation_matrix.transpose() * relative_position;
        // ROS_WARN("Drone_%d_:positionBYyaw :X_%f_Y_%f_Z_%f_",drone_datainfo.drone_id,transformed_position(0),transformed_position(1),transformed_position(2));
        observe_data_yaw_part.observed_positions.push_back(transformed_position);
    }
    
    Eigen::MatrixXd FACTInitializerFSM::GetCMatrix(int i)
    {
        // i must be in [1, drone_num]
        // ROS_INFO("i must be in [0, drone_num-1]");
        if(i < 0 || i >= drone_num) {
            ROS_ERROR("i must be in [0, drone_num-1]");
            return Eigen::MatrixXd::Zero(0, 0);
        }
        else
        {
            Eigen::VectorXd vec = Eigen::VectorXd::Zero(drone_num);
            vec(i) = 1;
            Eigen::MatrixXd I = Eigen::MatrixXd::Identity(so_n, so_n);
            Eigen::MatrixXd CMatrix = Eigen::kroneckerProduct(vec, I);
            return CMatrix;
        }
    }

    template<typename T>
    bool FACTInitializerFSM::isEqual(const T& x, const T& y, double tolerance) const {
        if (std::is_arithmetic<T>::value) 
        {
            return std::abs(x - y) <= tolerance;
        }
        else 
        {
            return x.isApprox(y, tolerance);  // 近似相等
        }
    }
    
    int FACTInitializerFSM::rankOfMatrix(const Eigen::MatrixXd& matrix) const {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
        double tolerance = 1e-10;
        return (svd.singularValues().array() > tolerance).count();
    }

    void FACTInitializerFSM::printCMatrix(const Eigen::MatrixXd& CMatrix) 
    {
        std::stringstream ss;
        ss << "CMatrix:\n";

        for (int i = 0; i < CMatrix.rows(); ++i) {
            for (int j = 0; j < CMatrix.cols(); ++j) {
                ss << CMatrix(i, j) << "\t";
            }
            ss << "\n";
        }

        ROS_INFO("%s", ss.str().c_str());
    }

    void FACTInitializerFSM::recordInitialYawAngles() 
    {
        auto normalizeYaw = [](double yaw) {
            while (yaw > M_PI) yaw -= 2 * M_PI;
            while (yaw < -M_PI) yaw += 2 * M_PI;
            return yaw;
        };

        // Warn: 这是一个全局的角度，不是相对的，使用时要注意
        initial_yaw_angles.clear();
        for (int i=0; i<drone_num; i++)
        {   
            if(i==drone_datainfo.drone_id)
            {
                initial_yaw_angles.push_back(drone_datainfo.angleRPY[2]);
            }
            else
            {
                initial_yaw_angles.push_back(drone_datainfo_others[i].angleRPY[2]);
            }
        }
        // 使用for循环cout打印initial_yaw_angles
        // show result_real
        std::stringstream ssresult_real;
        ssresult_real << "Drone_"<<drone_datainfo.drone_id<<":initial_yaw_angles: ";
        for (int i = 0; i < drone_num; ++i) 
        {
            ssresult_real << normalizeYaw(initial_yaw_angles[i]-initial_yaw_angles[drone_datainfo.drone_id]) << " ";
        }
        ROS_INFO("%s", ssresult_real.str().c_str());
    }

    void FACTInitializerFSM::updateYawAngleDifferences() 
    {
        std::vector<double> current_differences;

        for (int i=0; i<drone_num; i++)
        {   
            if(i==drone_datainfo.drone_id)
            {
                current_differences.push_back(drone_datainfo.angleRPY[2] - initial_yaw_angles[i]);
            }
            else
            {
                current_differences.push_back(drone_datainfo_others[i].angleRPY[2] - initial_yaw_angles[i]);
            }
        }
        yaw_angle_differences.push_back(current_differences);
        // show
        // ROS_INFO("Drone_%d_:yaw_angle_differences:diff:%f init:%f now:%f", drone_datainfo.drone_id, yaw_angle_differences[observe_counter-2][drone_datainfo.drone_id], initial_yaw_angles[drone_datainfo.drone_id], drone_datainfo.angleRPY[2]);

    }

    void FACTInitializerFSM::observeCallback(const coordinate_init::ObserveData::ConstPtr& msg, int drone_id) 
    {
        int msg_drone_id = msg->drone_id;
        int msg_observe_counter = msg->observe_counter;
        // ROS_INFO("Drone_%d: Received observe data from Drone_%d, observe counter is %d, observe_others[msg_drone_id].size() = %ld", drone_datainfo.drone_id, msg_drone_id, msg_observe_counter, observe_others[msg_drone_id].size());
        // ROS_INFO("Drone_%d: Received observe data from Drone_%d", drone_datainfo.drone_id, msg_drone_id);
        if (observe_others[msg_drone_id].size() < msg_observe_counter) 
        {
            // 扩展 observe_others 以包含新的观测时间点
            observe_others[msg_drone_id].resize(msg_observe_counter);

            // 处理新数据
            std::vector<Eigen::Vector3d> current_vectors;
            for (const auto& vec3 : msg->vectors) 
            {
                current_vectors.push_back(Eigen::Vector3d(vec3.x, vec3.y, vec3.z));
                // ROS_INFO("Drone_%d: Current Vector: x = %f, y = %f", drone_datainfo.drone_id, vec3.x, vec3.y);
            }
            // 将新数据添加到对应的观测时间点
            observe_others[msg_drone_id][msg_observe_counter - 1] = current_vectors;
        }

        // ROS_INFO("Drone_%d: observe_others[%d] timesize = %lu", drone_datainfo.drone_id, msg_drone_id, observe_others[msg_drone_id].size());
        // ROS_INFO("Drone_%d: observe_others[%d][%d] currenttimesize = %lu", drone_datainfo.drone_id, msg_drone_id, msg_observe_counter - 1, observe_others[msg_drone_id][msg_observe_counter - 1].size());

    }

    void FACTInitializerFSM::publishObserveData(bool use_yaw_data) {
        if (observe_counter <= 0 || observe_counter > observe_data_yaw_part_vector.size() || observe_counter > observe_data_quat_part_vector.size()) 
        {
            ROS_WARN("Invalid observe_counter value for publishing observe data.");
            return;
        }

        // 创建 ROS 消息
        coordinate_init::ObserveData msg;
        msg.drone_id = drone_datainfo.drone_id;
        msg.observe_counter = observe_counter;
        
        // 选择数据并填充消息
        const auto& selected_data = use_yaw_data ? observe_data_yaw_part_vector[observe_counter - 1] : observe_data_quat_part_vector[observe_counter - 1];
        
        for (const auto& pos : selected_data.observed_positions) {
            geometry_msgs::Vector3 vec_msg;
            vec_msg.x = pos.x();
            vec_msg.y = pos.y();
            vec_msg.z = pos.z();
            // vec_msg.z = 0;
            msg.vectors.push_back(vec_msg);
        }
        // while循环发布msg消息
        while(ros::ok())
        {
            pub_observe.publish(msg);
            if(CheckObserveData())
            {
                // ROS_INFO("Drone_%d_:observe_counter_%d:has received all the ObserveData",drone_datainfo.drone_id,observe_counter);
                break;
            }
            ros::spinOnce();
            ros::Duration(0.01).sleep();
        }

        // 发布消息
        pub_observe.publish(msg);
    }

    bool FACTInitializerFSM::CheckObserveData()
    {
        for(int i=0;i<drone_num;i++)
        {
            if(i!= drone_datainfo.drone_id && observe_others[i].size()<observe_counter)
            {
                // ROS_ERROR("Drone_%d_:observe_counter:_%d_:has not received all the ObserveData, observe_others[%d].size()_%ld is not enough!",drone_datainfo.drone_id,observe_counter, i, observe_others[i].size());
                return false;
            }
        }
        return true;
    }

    void FACTInitializerFSM::GoalIsInObstacle()
    {
        // 将goal的xy生成一个2维向量
        Eigen::Vector2d goal_xy;
        goal_xy << mygoal.x(), mygoal.y();
        for (const auto& obstacle : obstacles) {
            double distance = ( obstacle.position - (goal_xy) ).norm();
            if (distance < obstacle.radius*1.1) {
                ROS_WARN("Goal is inside an obstacle, adjusting position...");
                // goal = adjustGoalPosition(goal, obstacle);

                // 随机生成一个角度
                std::random_device rd;
                std::mt19937 gen(rd());
                std::uniform_real_distribution<> dis(-M_PI, M_PI);
                double angle = dis(gen);


                // Randomly offset the goal position in the XY plane
                Eigen::Vector3d newGoal = mygoal;
                newGoal.x() += obstacle.radius * 2.1 * cos(angle);
                newGoal.y() += obstacle.radius * 2.1 * sin(angle);

                ROS_INFO("New goal position: (%f, %f, %f)", newGoal.x(), newGoal.y(), newGoal.z());
                mygoal = newGoal;
                break; // Assuming only one adjustment is needed
            }
        }

    }

    void FACTInitializerFSM::publishGoal(const Eigen::Vector3d& target_position) 
    {
        quadrotor_msgs::GoalSet msg;
        msg.drone_id = drone_datainfo.drone_id; // 使用当前无人机ID
        msg.goal[0] = target_position[0];
        msg.goal[1] = target_position[1];
        msg.goal[2] = target_position[2];

        goal_pub_.publish(msg);
        ROS_INFO("Published goal for drone %d: [%f, %f, %f]", drone_datainfo.drone_id, target_position.x(), target_position.y(), target_position.z());
    }

    void FACTInitializerFSM::GenerateGoal()
    {
        long int now_observe_number = observe_data_yaw_part_vector[observe_counter-1].observed_num;
        if(now_observe_number==0)
        {
            mygoal = drone_datainfo.odom_pos;
            no_control = true;
            return;
        }

        // 计算observe_data_yaw_part_vector[observe_counter-1].observed_positions的距离和角度,共有now_observe_number个
        std::vector<double> angle;
        std::vector<double> distance;
        for(int i=0;i<now_observe_number;i++)
        {
            angle.push_back(atan2(observe_data_yaw_part_vector[observe_counter-1].observed_positions[i].y(), observe_data_yaw_part_vector[observe_counter-1].observed_positions[i].x()) + drone_datainfo.angleRPY[2]);
            distance.push_back(observe_data_yaw_part_vector[observe_counter-1].observed_positions[i].norm());
        }
        int min_index = std::min_element(distance.begin(), distance.end()) - distance.begin();
        double shortest_distance = distance[min_index];
        Eigen::Vector3d min_point = observe_data_yaw_part_vector[observe_counter-1].observed_positions[min_index];
        int num_0_6 = 0;
        for(int i=0;i<now_observe_number;i++)
        {
            if(distance[i]<0.6)
            {
                num_0_6++;
            }
        }

        // 如果0.6距离以内点的数量大于等于2个，那么就选择原地转yaw
        if(shortest_distance<0.5)
        {
            if(observe_data_yaw_part_vector[observe_counter-1].observed_num==1)
            {
                // 垂直方向走0.6
                double angle_goal = atan2(min_point.y(), min_point.x()) + drone_datainfo.angleRPY[2] + M_PI/2.0;
                mygoal[0] = drone_datainfo.odom_pos[0] + 0.6 * cos(angle_goal);
                mygoal[1] = drone_datainfo.odom_pos[1] + 0.6 * sin(angle_goal);
                mygoal[2] = static_cast<double>(rand()) / RAND_MAX * 0.8 + 0.2; // 0.2 到 1
                return;
            }
            else
            {
                mygoal = drone_datainfo.odom_pos;
                no_control = true;
                need_rotate = true;
                return; 
            }
           
        }
        if(num_0_6>=2)
        {
            mygoal = drone_datainfo.odom_pos;
            no_control = true;
            need_rotate = true;
            return;
        }
        if(shortest_distance<0.65)
        {
            // 最近点的垂直方向走
            double angle_goal = atan2(min_point.y(), min_point.x()) + drone_datainfo.angleRPY[2] + M_PI/2.0;
            mygoal[0] = drone_datainfo.odom_pos[0] + shortest_distance * 0.8 * cos(angle_goal);
            mygoal[1] = drone_datainfo.odom_pos[1] + shortest_distance * 0.8 * sin(angle_goal);
            mygoal[2] = static_cast<double>(rand()) / RAND_MAX * 0.8 + 0.2; // 0.2 到 1    
            return;        
        }

        if(observe_data_yaw_part_vector[observe_counter-1].observed_num==1)
        {
            Eigen::Vector3d goal_1 = observe_data_yaw_part_vector[observe_counter-1].observed_positions[0];
            double angle_rand = atan2(goal_1.y(), goal_1.x()) + drone_datainfo.angleRPY[2];
            mygoal[0] = drone_datainfo.odom_pos[0] + goal_1.norm() / 3.0 * cos(angle_rand);
            mygoal[1] = drone_datainfo.odom_pos[1] + goal_1.norm() / 3.0 * sin(angle_rand);
            mygoal[2] = static_cast<double>(rand()) / RAND_MAX * 0.8 + 0.2; // 0.2 到 1
            return;
        }

        double random_num = static_cast<double>(rand()) / RAND_MAX;
        if(random_num<0.7)
        {
            // 最近点的垂直方向走
            double angle_goal = atan2(min_point.y(), min_point.x()) + drone_datainfo.angleRPY[2] + M_PI/2.0;
            mygoal[0] = drone_datainfo.odom_pos[0] + shortest_distance/2.0 * cos(angle_goal);
            mygoal[1] = drone_datainfo.odom_pos[1] + shortest_distance/2.0 * sin(angle_goal);
            mygoal[2] = static_cast<double>(rand()) / RAND_MAX * 0.8 + 0.2; // 0.2 到 1
        }
        else
        {
            // 随机点的方向走
            int random_index = rand() % observe_data_yaw_part_vector[observe_counter-1].observed_positions.size();
            Eigen::Vector3d rand_goal = observe_data_yaw_part_vector[observe_counter-1].observed_positions[random_index];
            double angle_rand = atan2(rand_goal.y(), rand_goal.x()) + drone_datainfo.angleRPY[2];
            mygoal[0] = drone_datainfo.odom_pos[0] + rand_goal.norm() / 3.0 * cos(angle_rand);
            mygoal[1] = drone_datainfo.odom_pos[1] + rand_goal.norm() / 3.0 * sin(angle_rand);
            mygoal[2] = static_cast<double>(rand()) / RAND_MAX * 0.8 + 0.2; // 0.2 到 1
        }
    }

    bool FACTInitializerFSM::OutofMap(const Eigen::Vector3d& pos)
    {
      if(pos[0]>-my_map_size_width && pos[0]<my_map_size_width && pos[1]>-my_map_size_depth && pos[1]<my_map_size_depth && pos[2]>-0.1 && pos[2]<my_map_size_height)
      {
          return false;
      }
      else
      {
          return true;
      }
    }

    void FACTInitializerFSM::WaitingtoArriveGoal()
    {
        // 定义rate
        ROS_WARN("Drone_%d_:Waiting to arrive goal",drone_datainfo.drone_id);
        ros::Time start = ros::Time::now();
        ros::Duration timeout(7.0);
        ros::Rate rate(50);
        while(ros::ok() && !have_arrived_goal)
        {
            if( ((drone_datainfo.odom_pos-mygoal).norm() < 0.05) && (ros::Time::now() - start > timeout) )
            {
                have_arrived_goal=true;
                break;
                ROS_INFO("Drone_%d_:Arrived Goal",drone_datainfo.drone_id);
            }
            if (ros::Time::now() - start > timeout) {
                ROS_INFO("Drone_%d_:Planning over time",drone_datainfo.drone_id);
                break;  // Exit the loop
            }
            ros::spinOnce();
            rate.sleep();
        }
        have_arrived_goal=false;

        // std_msgs::Empty stop_msg;
        // mandatory_stop_pub_.publish(stop_msg);
    }

    void FACTInitializerFSM::SolveMySDP()
    {
        // GetPMatrix
        GetPMatrix();
        // GetQMatrix
        GetQMatrix();

        const int all_dim = so_n * drone_num;  // 矩阵维度
        Eigen::MatrixXd R(all_dim, so_n);

        if (solver == solver_type::SDP) {
            // ROS_INFO("Using SDP solver");
            Model::t M = new Model("SDP");
            M->acceptedSolutionStatus(AccSolutionStatus::Optimal);
            int all_dim = so_n * drone_num;  // 矩阵维度
            Variable::t X = M->variable("X", Domain::inPSDCone(all_dim));
            for (int i = 0; i < drone_num; i++)
            {
                int index = so_n * i;
                M->constraint(X->index(index, index), Domain::equalsTo(1.00));
                M->constraint(X->index(index + 1, index + 1), Domain::equalsTo(1.00));

                M->constraint(X->index(index, index + 1), Domain::equalsTo(0.00));
                M->constraint(X->index(index + 1, index), Domain::equalsTo(0.00));
            }
            auto Q_data = std::make_shared<monty::ndarray<double, 2>>((double *)(Q_sumall.data()), shape(all_dim, all_dim));
            mosek::fusion::Matrix::t QQ = mosek::fusion::Matrix::dense(Q_data);
            std::vector<double> Z_vec;

            try
            {
                M->objective(ObjectiveSense::Minimize, Expr::sum(Expr::mulElm(QQ, X)));
                M->solve();
                auto Z_level = M->getVariable("X")->level();
                Z_vec = monty::new_vector_from_array_ptr(Z_level);
            }
            catch (const OptimizeError &e)
            {
                std::cout << "Optimization failed. Error: " << e.what() << "\n";
            }
            catch (const SolutionError &e)
            {
            // The solution with at least the expected status was not available.
            // We try to diagnoze why.
                std::cout << "Requested solution was not available.\n";
                auto prosta = M->getProblemStatus();
                auto prista = M->getPrimalSolutionStatus();
                auto duasta = M->getDualSolutionStatus();
                switch (prosta)
                {
                    case ProblemStatus::DualInfeasible:
                        std::cout << "Dual infeasibility certificate found.\n";
                        std::cout << "Primal Status" << prista << std::endl;
                        std::cout << "Dual Status" << duasta << std::endl;
                        break;
                    case ProblemStatus::PrimalInfeasible:
                        std::cout << "Primal infeasibility certificate found.\n";
                        std::cout << "Primal Status" << prista << std::endl;
                        std::cout << "Dual Status" << duasta << std::endl;
                        break;
                    case ProblemStatus::Unknown:
                        // The solutions status is unknown. The termination code
                        // indicates why the optimizer terminated prematurely.
                        std::cout << "The solution status is unknown.\n";
                        char symname[MSK_MAX_STR_LEN];
                        char desc[MSK_MAX_STR_LEN];
                        MSK_getcodedesc((MSKrescodee)(M->getSolverIntInfo("optimizeResponse")), symname, desc);
                        M->writeTask("dump.opf");
                        std::cout << "Termination code: " << symname << " " << desc << "\n";
                        std::cout << "Problem Status" << prosta << std::endl;
                        std::cout << "Primal Status" << prista << std::endl;
                        std::cout << "Dual Status" << duasta << std::endl;
                        break;
                    default:
                        std::cout << "Another unexpected problem status: " << prosta << "\n";
                }
            }
            catch (const std::exception &e)
            {
                std::cerr << "Unexpected error: " << e.what() << "\n";
            }

            // 获得Z_star
            Z_star = Eigen::Map<Eigen::MatrixXd>(Z_vec.data(), all_dim, all_dim);
            // 显示 Z_star
            // std::stringstream ssZ;
            // ssZ << "Z_star:\n";
            // for (int i = 0; i < Z_star.rows(); ++i) {
            //     for (int j = 0; j < Z_star.cols(); ++j) {
            //         ssZ << Z_star(i, j) << "\t";
            //     }
            //     ssZ << "\n";
            // }
            // ROS_INFO("%s", ssZ.str().c_str());
            // 显示Z_star的Rank
            // ROS_ERROR("Drone_%d: Z_star:%d", drone_datainfo.drone_id, rankOfMatrix(Z_star));

            // 求解R矩阵
            // Z_star奇异值分解
            Eigen::JacobiSVD<Eigen::MatrixXd> svd(Z_star, Eigen::ComputeThinU | Eigen::ComputeThinV);
            Eigen::MatrixXd U = svd.matrixU();
            Eigen::MatrixXd S = svd.singularValues().asDiagonal();
            for (int i = 0; i < S.rows(); ++i) {
                S(i, i) = std::sqrt(S(i, i)); // 对角线元素开平方根
            }
            // 构造 R
            R = U * S;
        } else {
            ROS_INFO("Using Ceres solver");
            // the variable to be optimized, angles
            double angle0 = 0.0;
            double angle1 = 0.0;
            double angle2 = 0.0;
            double angle3 = 0.0;
            double angle4 = 0.0;
            double angle5 = 0.0;
            double angle6 = 0.0;
            double angle7 = 0.0;

            ceres::Problem problem;
            // Limiting angle to [-pi, pi]
            problem.AddParameterBlock(&angle0, 1);
            problem.AddParameterBlock(&angle1, 1);
            problem.AddParameterBlock(&angle2, 1);
            problem.AddParameterBlock(&angle3, 1);
            problem.AddParameterBlock(&angle4, 1);
            problem.AddParameterBlock(&angle5, 1);
            problem.AddParameterBlock(&angle6, 1);
            problem.AddParameterBlock(&angle7, 1);

            problem.SetParameterLowerBound(&angle0, 0, -M_PI);
            problem.SetParameterUpperBound(&angle0, 0, M_PI);
            problem.SetParameterLowerBound(&angle1, 0, -M_PI);
            problem.SetParameterUpperBound(&angle1, 0, M_PI);
            problem.SetParameterLowerBound(&angle2, 0, -M_PI);
            problem.SetParameterUpperBound(&angle2, 0, M_PI);
            problem.SetParameterLowerBound(&angle3, 0, -M_PI);
            problem.SetParameterUpperBound(&angle3, 0, M_PI);
            problem.SetParameterLowerBound(&angle4, 0, -M_PI);
            problem.SetParameterUpperBound(&angle4, 0, M_PI);
            problem.SetParameterLowerBound(&angle5, 0, -M_PI);
            problem.SetParameterUpperBound(&angle5, 0, M_PI);
            problem.SetParameterLowerBound(&angle6, 0, -M_PI);
            problem.SetParameterUpperBound(&angle6, 0, M_PI);
            problem.SetParameterLowerBound(&angle7, 0, -M_PI);
            problem.SetParameterUpperBound(&angle7, 0, M_PI);

            ceres::CostFunction* cost_function;

            // Add residual block
            switch (drone_num) {
                case 2: {
                    cost_function = new ceres::AutoDiffCostFunction<CostFunctor, 1, 1, 1>(new CostFunctor(Q_sumall));
                    problem.AddResidualBlock(cost_function, NULL, &angle0, &angle1);
                    break;
                }
                case 3: {
                    cost_function = new ceres::AutoDiffCostFunction<CostFunctor, 1, 1, 1, 1>(new CostFunctor(Q_sumall));
                    problem.AddResidualBlock(cost_function, NULL, &angle0, &angle1, &angle2);
                    break;
                }
                case 4: {
                    cost_function = new ceres::AutoDiffCostFunction<CostFunctor, 1, 1, 1, 1, 1>(new CostFunctor(Q_sumall));
                    problem.AddResidualBlock(cost_function, NULL, &angle0, &angle1, &angle2, &angle3);
                    break;
                }
                case 5: {
                    cost_function = new ceres::AutoDiffCostFunction<CostFunctor, 1, 1, 1, 1, 1, 1>(new CostFunctor(Q_sumall));
                    problem.AddResidualBlock(cost_function, NULL, &angle0, &angle1, &angle2, &angle3, &angle4);
                    break;
                }
                case 6: {
                    cost_function = new ceres::AutoDiffCostFunction<CostFunctor, 1, 1, 1, 1, 1, 1, 1>(new CostFunctor(Q_sumall));
                    problem.AddResidualBlock(cost_function, NULL, &angle0, &angle1, &angle2, &angle3, &angle4, &angle5);
                    break;
                }
                case 7: {
                    cost_function = new ceres::AutoDiffCostFunction<CostFunctor, 1, 1, 1, 1, 1, 1, 1, 1>(new CostFunctor(Q_sumall));
                    problem.AddResidualBlock(cost_function, NULL, &angle0, &angle1, &angle2, &angle3, &angle4, &angle5, &angle6);
                    break;
                }
                case 8: {
                    cost_function= new ceres::AutoDiffCostFunction<CostFunctor, 1, 1, 1, 1, 1, 1, 1, 1, 1>(new CostFunctor(Q_sumall));
                    problem.AddResidualBlock(cost_function, NULL, &angle0, &angle1, &angle2, &angle3, &angle4, &angle5, &angle6, &angle7);
                    break;
                }

                default: {
                    ROS_ERROR("Drone_%d: The number of drones is not supported", drone_datainfo.drone_id);
                    return;
                }
            }

            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.minimizer_type = ceres::TRUST_REGION;
            options.trust_region_strategy_type = solver == solver_type::LM ? ceres::LEVENBERG_MARQUARDT : ceres::DOGLEG;

            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            std::cout << summary.BriefReport() << std::endl;

            Eigen::MatrixXd R0 = Eigen::Rotation2Dd(angle0).toRotationMatrix();
            Eigen::MatrixXd R1 = Eigen::Rotation2Dd(angle1).toRotationMatrix();
            Eigen::MatrixXd R2 = Eigen::Rotation2Dd(angle2).toRotationMatrix();
            Eigen::MatrixXd R3 = Eigen::Rotation2Dd(angle3).toRotationMatrix();
            Eigen::MatrixXd R4 = Eigen::Rotation2Dd(angle4).toRotationMatrix();
            Eigen::MatrixXd R5 = Eigen::Rotation2Dd(angle5).toRotationMatrix();
            Eigen::MatrixXd R6 = Eigen::Rotation2Dd(angle6).toRotationMatrix();
            Eigen::MatrixXd R7 = Eigen::Rotation2Dd(angle7).toRotationMatrix();
            std::vector<Eigen::MatrixXd> R_vector{R0, R1, R2, R3, R4, R5, R6, R7};

            Eigen::MatrixXd R_all(2, 2 * drone_num);
            for (int i = 0; i < drone_num; i++) {
                R_all.block(0, 2 * i, 2, 2) = R_vector[i];
            }
            R = R_all.transpose();
        }
        // std::cout << "R matrix:\n" << R << std::endl;
        // 提取SO(2)矩阵
        int index_start_ = so_n * drone_datainfo.drone_id;
        Eigen::MatrixXd SO2_matrix = R.block(index_start_, 0, so_n, so_n);
        // 计算转置
        Eigen::MatrixXd SO2_transpose = SO2_matrix.transpose();
        // 初始化结果矩阵
        Eigen::MatrixXd result = Eigen::MatrixXd::Zero(so_n * drone_num, so_n);
        // 乘以每个小矩阵
        for (int i = 0; i < drone_num; ++i) 
        {
            int start_col = i * so_n;
            Eigen::MatrixXd small_matrix = R.block(start_col, 0, so_n, so_n);
            result.block(start_col, 0, so_n, so_n) = SO2_transpose * small_matrix;
        }
        // 初始化结果矩阵的块转置
        Eigen::MatrixXd result_blocktranspose = Eigen::MatrixXd::Zero(so_n * drone_num, so_n);  
        // 乘以每个小矩阵
        for (int i = 0; i < drone_num; ++i) 
        {
            int start_col = i * so_n;
            Eigen::MatrixXd small_matrix = result.block(start_col, 0, so_n, so_n);
            result_blocktranspose.block(start_col, 0, so_n, so_n) = small_matrix.transpose();
        }
        // 计算optimal value并进行比较
        double optimalvalue1 = (Q_sumall * result * (result.transpose()) ).trace();
        double optimalvalue2 = (Q_sumall * result_blocktranspose * (result_blocktranspose.transpose()) ).trace();
        result_end = Eigen::MatrixXd::Zero(so_n * drone_num, so_n);  
        if(optimalvalue1 > optimalvalue2)
        {
            result_end = result_blocktranspose;
            ROS_WARN("Drone_%d_:result_blocktranspose", drone_datainfo.drone_id);
        }
        else
        {
            result_end = result;
        }

        // 显示最终结果
        std::stringstream ssresult;
        ssresult << "result:\n";
        for (int i = 0; i < result_end.rows(); ++i) {
            for (int j = 0; j < result_end.cols(); ++j) {
                ssresult << result_end(i, j) << "\t";
            }
            ssresult << "\n";
        }
        ROS_INFO("%s", ssresult.str().c_str());

        // 显示最终结果的optimal value
        double Optimal1 = (Q_sumall * result_end * (result_end.transpose()) ).trace();
        // double Optimal2 = (Q_sumall * Z_star ).trace();
        // ROS_INFO("Drone_%d_:observe_counter_%d:Optimal:%f", drone_datainfo.drone_id, observe_counter, Optimal1);
        // ROS_INFO("Drone_%d_:Optimal:%f", drone_datainfo.drone_id, Optimal2);
        output_data.optimal = Optimal1;
        // output_data.optimal2 = Optimal2;
        // 注意：使用result要取弧度相反数

        auto normalizeYaw = [](double yaw) {
            while (yaw > M_PI) yaw -= 2 * M_PI;
            while (yaw < -M_PI) yaw += 2 * M_PI;
            return yaw;
        };

        // 显示真实结果的旋转矩阵
        Eigen::MatrixXd result_real = Eigen::MatrixXd::Zero(so_n * drone_num, so_n);
        std:vector<double> angle_result_real;
        for (int i = 0; i < drone_num; ++i) {
            double theta = initial_yaw_angles[i] - initial_yaw_angles[drone_datainfo.drone_id]; 
            theta = -theta;
            // 将theta转到-pi到pi之间
            theta = normalizeYaw(theta);
            angle_result_real.push_back(theta);
            Eigen::Matrix2d rotation_matrix;
            rotation_matrix << cos(theta), -sin(theta),
                            sin(theta),  cos(theta);

            // Place the rotation matrix in the larger matrix
            result_real.block<2, 2>(i * so_n, 0) = rotation_matrix;
        }
        output_data.angle_result_real = angle_result_real;
        // show result_real
        // std::stringstream ssresult_real;
        // ssresult_real << "result_real:\n";
        // for (int i = 0; i < result_real.rows(); ++i) {
        //     for (int j = 0; j < result_real.cols(); ++j) {
        //         ssresult_real << result_real(i, j) << "\t";
        //     }
        //     ssresult_real << "\n";
        // }
        // ROS_INFO("%s", ssresult_real.str().c_str());

        // 显示真实结果的optimal value
        double Optimal_real = (Q_sumall * result_real * (result_real.transpose()) ).trace();
        // ROS_INFO("Drone_%d_:Optimal_real:%f", drone_datainfo.drone_id, Optimal_real);
        output_data.optimal_real = Optimal_real;


        // 获取估计结果的弧度
        angle_result.clear();
        for (int i = 0; i < drone_num; ++i) 
        {
            int start_col = i * so_n;
            Eigen::MatrixXd small_matrix = result_end.block(start_col, 0, so_n, so_n);
            // 从small_matrix中提取角度
            double theta = std::atan2(small_matrix(1, 0), small_matrix(0, 0));
            // 将角度转换为度数，如果需要的话
            // double theta_degrees = theta * 180.0 / M_PI;
            angle_result.push_back(theta);
        }
        // 获取真实结果的弧度（已获得）
        // 显示angle_result和angle_result_real
        std::stringstream ssangle_result;
        ssangle_result << "angle_result:\n";
        for (int i = 0; i < angle_result.size(); ++i) {
            ssangle_result << angle_result[i] << "\t";
        }
        ssangle_result << "\n";
        ROS_INFO("%s", ssangle_result.str().c_str());

        std::stringstream ssangle_result_real;
        ssangle_result_real << "angle_result_real:\n";
        for (int i = 0; i < angle_result_real.size(); ++i) {
            ssangle_result_real << angle_result_real[i] << "\t";
        }
        ssangle_result_real << "\n";
        ROS_INFO("%s", ssangle_result_real.str().c_str());

        // 计算angle_result和angle_result_real均方差
        double sum_error = 0;
        for(int i=0;i<drone_num;i++)
        {
            sum_error += pow(angle_result[i] - angle_result_real[i], 2);
        }
        mean_error = sum_error / drone_num;
        // 显示均方差，angle_result，angle_result_real
        ROS_INFO("Drone_%d_:observe_counter_%d:mean_error:%f", drone_datainfo.drone_id, observe_counter, mean_error);

        if(least_observe_num<=observe_counter && mean_error<0.00001)
        {
            SDP_is_solved = true;
            ROS_INFO("Drone_%d_:observe_counter_%d: SDP is solved well!", drone_datainfo.drone_id, observe_counter);
        }
        else if(observe_counter>=(least_observe_num+2) && mean_error<0.03)
        {
            SDP_is_solved = true;
            ROS_INFO("Drone_%d_:observe_counter_%d: SDP is solved!", drone_datainfo.drone_id, observe_counter);
        }
        else
        {
            // SDP_is_solved = true;
            // ROS_ERROR("Drone_%d_:observe_counter_%d: SDP is not solved!", drone_datainfo.drone_id, observe_counter);
        }

        // free(cost_function);
    }

    void FACTInitializerFSM::GetPMatrix()
    {
        // 世界系下验证
        // GetPMatrix
        P.clear();
        P.resize(observe_counter); // 初始化时间维度
        // Ptest.clear();
        Ptest.resize(observe_counter); // 初始化时间维度
        for(int k=0; k<observe_counter;k++) // 时间维度
        {
            P[k].resize(drone_num, Eigen::Vector2d::Zero()); // 初始化观测者维度
            Ptest[k] = Eigen::Vector2d::Zero();
            for(int i=0; i<drone_num;i++) // 观测者维度
            {
                // ld类型变量
                long int observe_size=0;
                int observe_time=0;

                std::vector<Eigen::Vector2d> observe_data_now;
                std::vector<Eigen::Vector2d> observe_data_now_rotate;
                if(i==drone_datainfo.drone_id)
                {
                    // ROS_ERROR("Drone_%d:Get observe_size!", drone_datainfo.drone_id);
                    observe_size = observe_data_yaw_part_vector[k].observed_positions.size();
                    // ROS_ERROR("Drone_%d:time k:%d observer i:%d observe_data_now_size:%ld", drone_datainfo.drone_id, k,i, observe_size);
                    observe_time = observe_data_yaw_part_vector.size();
                    // ROS_ERROR("Drone_%d:time k:%d observer i:%d observe_data_now_size:%ld", drone_datainfo.drone_id, k,i, observe_size);
                    observe_data_now.clear();
                    // ROS_ERROR("Drone_%d:time k:%d observer i:%d observe_data_now_size:%ld", drone_datainfo.drone_id, k,i, observe_size);
                    observe_data_now.resize(observe_size);
                    observe_data_now_rotate.clear();
                    // ROS_ERROR("Drone_%d:time k:%d observer i:%d observe_data_now_size:%ld", drone_datainfo.drone_id, k,i, observe_size);
                    observe_data_now_rotate.resize(observe_size);
                    // observe_data_now = observe_data_yaw_part_vector[k].observed_positions;// 3d to 2d
                    for(int j=0;j<observe_size;j++)
                    {
                        observe_data_now[j].x() = observe_data_yaw_part_vector[k].observed_positions[j].x();
                        observe_data_now[j].y() = observe_data_yaw_part_vector[k].observed_positions[j].y();
                    }
                    // ROS_ERROR("i==drone_datainfo.drone_id");
                    // ROS_INFO("Drone_%d:time k:%d observer i:%d observe_data_now_size:%ld", drone_datainfo.drone_id, k,i,observe_size);
                    // ROS_INFO("Drone_%d:time k:%d observer i:%d observe_data_all_time:%d", drone_datainfo.drone_id, k,i,observe_time);
                    observe_data_yaw_part_vector[k].observed_positions;
                }
                else
                {
                    // ROS_ERROR("Drone_%d:Get observe_size!", drone_datainfo.drone_id);
                    observe_size = observe_others[i][k].size();
                    // ROS_ERROR("Drone_%d:time k:%d observer i:%d observe_data_now_size:%ld", drone_datainfo.drone_id, k,i, observe_size);
                    observe_time = observe_others[i].size();
                    // ROS_ERROR("Drone_%d:time k:%d observer i:%d observe_data_now_size:%ld", drone_datainfo.drone_id, k,i, observe_size);
                    observe_data_now.clear();
                    // ROS_ERROR("Drone_%d:time k:%d observer i:%d observe_data_now_size:%ld", drone_datainfo.drone_id, k,i, observe_size);
                    observe_data_now.resize(observe_size);
                    observe_data_now_rotate.clear();
                    // ROS_ERROR("Drone_%d:time k:%d observer i:%d observe_data_now_size:%ld", drone_datainfo.drone_id, k,i, observe_size);
                    observe_data_now_rotate.resize(observe_size);
                    // observe_data_now = observe_others[i][k];
                    for(int j=0;j<observe_size;j++)
                    {
                        observe_data_now[j].x() = observe_others[i][k][j].x();
                        observe_data_now[j].y() = observe_others[i][k][j].y();
                    }
                    // ROS_ERROR("i!=drone_datainfo.drone_id");
                    // ROS_INFO("Drone_%d:time k:%d observer i:%d observe_data_now_size:%ld", drone_datainfo.drone_id, k,i, observe_size);
                    // ROS_INFO("Drone_%d:time k:%d observer i:%d observe_data_all_time:%d", drone_datainfo.drone_id, k,i, observe_time);
                    observe_others[i][k];
                }

                if(observe_time!=observe_counter)
                {
                    ROS_ERROR("Observe time is not equal to observe_counter.");
                }
                else
                {
                    Eigen::Vector2d sumXY = Eigen::Vector2d::Zero(); // 初始化为零
                    // 检查 observe_data_now是否为空
                    if(observe_data_now.empty())
                    {
                        ROS_ERROR("Drone_%d:time %d observor %d: observe_data is empty.", drone_datainfo.drone_id, k, i);
                    }
                    else
                    {
                        // ROS_ERROR("Drone_%d:time %d observor %d: observe_data is not empty.", drone_datainfo.drone_id, k, i);
                        Eigen::Matrix2d rotationMatrix_now;
                        if(i==drone_datainfo.drone_id)
                        {
                            // ROS_ERROR("Drone_%d:i==drone_datainfo.drone_id done!", drone_datainfo.drone_id);
                            // double theta = (-1) * drone_datainfo.angleRPY[2];
                            double theta = (+1) * drone_datainfo.angleRPY[2];
                            double theta_test=0;
                            if(k==0)
                            {
                                // theta_test = initial_yaw_angles[i];
                                theta_test = 0;
                            }
                            else
                            {
                                // theta_test = initial_yaw_angles[i] + yaw_angle_differences[k-1][i];
                                theta_test = yaw_angle_differences[k-1][i];
                            }
                            
                            // ROS_ERROR("Drone_%d:time k:%d observer i:%d observe_yaw(+):%f", drone_datainfo.drone_id, k, i, theta);
                            // ROS_ERROR("Drone_%d:time k:%d observer i:%d observe_yaw_test(+):%f", drone_datainfo.drone_id, k, i, theta_test);
                            theta = theta_test;
                            rotationMatrix_now << cos(theta), -sin(theta),
                                                sin(theta), cos(theta);
                            // ROS_ERROR("Drone_%d:i==drone_datainfo.drone_id done!", drone_datainfo.drone_id);
                        }
                        else
                        {
                            // ROS_ERROR("Drone_%d:i!=drone_datainfo.drone_id done!", drone_datainfo.drone_id);
                            // double theta = (-1) * drone_datainfo_others[i].angleRPY[2];
                            double theta = (+1) * drone_datainfo_others[i].angleRPY[2];
                            double theta_test=0;
                            if(k==0)
                            {
                                // theta_test = initial_yaw_angles[i];
                                theta_test = 0;
                            }
                            else
                            {
                                theta_test = initial_yaw_angles[i] + yaw_angle_differences[k-1][i];
                                theta_test = yaw_angle_differences[k-1][i];
                            }
                            // ROS_ERROR("Drone_%d:time k:%d observer i:%d observe_yaw(+):%f", drone_datainfo.drone_id, k, i, theta);
                            // ROS_ERROR("Drone_%d:time k:%d observer i:%d observe_yaw_test(+):%f", drone_datainfo.drone_id, k, i, theta_test);
                            theta = theta_test;
                            rotationMatrix_now << cos(theta), -sin(theta),
                                                sin(theta), cos(theta);
                            // ROS_ERROR("Drone_%d:i!=drone_datainfo.drone_id done!", drone_datainfo.drone_id);
                        } 
                        // ROS_ERROR("Drone_%d:observe_counter_%d: rotationMatrix_now done!", drone_datainfo.drone_id, observe_counter);
                        for(int j=0;j<observe_size;j++)
                        {
                            observe_data_now_rotate[j] = rotationMatrix_now * observe_data_now[j];
                            // ROS_ERROR("Drone_%d:time k:%d observer i:%d observe_data_now[%d]:%f %f", drone_datainfo.drone_id, k, i, j, observe_data_now[j].x(), observe_data_now[j].y());
                        }
                        // ROS_ERROR("Drone_%d:observe_counter_%d: observe_data_now_rotate done!", drone_datainfo.drone_id, observe_counter);
                        for(int j=0;j<observe_size;j++)
                        {
                            sumXY.x() += observe_data_now_rotate[j].x(); // 累加 x 分量
                            sumXY.y() += observe_data_now_rotate[j].y(); // 累加 y 分量
                            // ROS_ERROR("Drone_%d:time k:%d observer i:%d observe_data_now_rotate[%d]:%f %f", drone_datainfo.drone_id, k, i, j, observe_data_now_rotate[j].x(), observe_data_now_rotate[j].y());
                        }
                        // ROS_ERROR("Drone_%d:observe_counter_%d: sumXY done!", drone_datainfo.drone_id, observe_counter);
                    }
                    P[k][i] = sumXY;
                    // ROS_ERROR("Drone_%d:observe_counter_%d: P[%d][%d] done!", drone_datainfo.drone_id, observe_counter, k, i);
                }
                Eigen::Matrix2d rotationMatrix_init;
                double theta_init = initial_yaw_angles[i]-initial_yaw_angles[drone_datainfo.drone_id];
                rotationMatrix_init << cos(theta_init), -sin(theta_init),
                                        sin(theta_init), cos(theta_init);

                initial_yaw_angles;
                Ptest[k] = Ptest[k] + rotationMatrix_init * P[k][i];
                // ROS_ERROR("Drone_%d:P[%d][%d] = %f %f",drone_datainfo.drone_id , k, i, P[k][i].x(), P[k][i].y());
            }
            // check that Ptest[k] is equal to 0
            // ROS_ERROR("Drone_%d:Ptest[%d] = %f %f",drone_datainfo.drone_id , k, Ptest[k].x(), Ptest[k].y());
            if(Ptest[k].isZero(1e-2))
            {
                ROS_INFO("Drone_%d:observe_counter_%d: Ptest is equal to 0.", drone_datainfo.drone_id, observe_counter);
            }
            else
            {
                // show
                ROS_ERROR("Drone_%d:observe_counter_%d: Ptest is not equal to 0.", drone_datainfo.drone_id, observe_counter);
            }            
        }
        // ROS_ERROR("Drone_%d: observe_counter:_%d_: P Matrix Done!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!", drone_datainfo.drone_id, observe_counter);        
    }

    void FACTInitializerFSM::GetQMatrix()
    {
        // GetQMatrix
        Q_sum = Eigen::MatrixXd::Zero(so_n * drone_num, so_n * drone_num);
        Q_sumall = Q_sum;
        // std::vector<Eigen::MatrixXd> Q(observe_counter); // 存储每一个时间点的Q矩阵
        Q.clear();
        Q.resize(observe_counter); // 初始化时间维度

        for(int k=0; k<observe_counter;k++)
        {
            for(int m=0; m<drone_num;m++)
            {
                for(int n=0; n<drone_num;n++)
                {
                    Eigen::MatrixXd MatrixCm = GetCMatrix(m);
                    Eigen::MatrixXd MatrixCn = GetCMatrix(n);
                    Q_sum = MatrixCn * P[k][n] * P[k][m].transpose() * MatrixCm.transpose() + Q_sum;
                    // printCMatrix(MatrixCn);
                    // printCMatrix(MatrixCm);
                }
            }
            Q_sumall = Q_sumall + Q_sum;
            Q[k] = Q_sum;  
            // ROS_ERROR("Drone_%d: time k:%d: RANK_Qsum:%d", drone_datainfo.drone_id, k, rankOfMatrix(Q_sum));
            // 显示Q_sum，并且要对齐
            // printEigenMatrix(Q_sum);
            // ROS_INFO("RANK_Qsumall:%d", rankOfMatrix(Q_sumall));
            // show Q_sumall
            // printEigenMatrix(Q_sumall);
            Q_sum.setZero();
        }
        // ROS_INFO("Drone_%d_:observe_counter:_%d_: Q Matrix completed",drone_datainfo.drone_id,observe_counter);
    }

    void FACTInitializerFSM::printEigenMatrix(const Eigen::MatrixXd& matrix) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(4);  // 设置小数点后4位精度

        for (int i = 0; i < matrix.rows(); ++i) {
            for (int j = 0; j < matrix.cols(); ++j) {
                ss << std::setw(10) << matrix(i, j) << " ";  // 设置宽度，保证对齐
            }
            ss << "\n";
        }

        ROS_INFO_STREAM("Matrix:\n" << ss.str());
    }

    void FACTInitializerFSM::ControltoGoal()
    {
        quadrotor_msgs::PositionCommand cmd;
        cmd.position.x = mygoal.x();
        cmd.position.y = mygoal.y();
        cmd.position.z = mygoal.z();
        cmd.yaw = -M_PI + static_cast<double>(std::rand()) / (static_cast<double>(RAND_MAX / (2 * M_PI))); // -π 到 π
        ROS_WARN("Drone_%d_:Controlling to Goal",drone_datainfo.drone_id);
        // cmd.yaw = 0;
        ros::Rate rate(500);
        while (ros::ok()) 
        {
            if((drone_datainfo.odom_pos-mygoal).norm() < 0.05)
            {
                ROS_INFO("Drone_%d_:Arrived Goal",drone_datainfo.drone_id);
                break;
            } 
            // ROS_WARN("Drone_%d_:Controlling to Goal",drone_datainfo.drone_id);
            pos_cmd_pub.publish(cmd);
            ros::spinOnce();
            rate.sleep();
        }
    }

    void FACTInitializerFSM::SolveTbyHungarian()
    // 观测为空的情况需要详细检测
    {
        // 获取观测数据合集的子元素
        solve_data_yaw.resize(drone_num);
        for(int i=0;i<drone_num;i++)
        {
            if(i==drone_datainfo.drone_id)
            {
                solve_data_yaw[i].drone_id = i;
                solve_data_yaw[i].observed_num = observe_data_yaw_part_vector[0].observed_num;
                solve_data_yaw[i].observed_positions = observe_data_yaw_part_vector[0].observed_positions;
                solve_data_yaw[i].angle_yaw = 0;
                solve_data_yaw[i].rotation_matrix = Eigen::Matrix2d::Identity();
            }
            else
            {
                solve_data_yaw[i].drone_id = i;
                solve_data_yaw[i].observed_num = observe_others[i][0].size();
                solve_data_yaw[i].observed_positions = observe_others[i][0]; // todo: observe_others改为3d
                solve_data_yaw[i].angle_yaw = -angle_result[i]; // 从自己到别人的角度
                solve_data_yaw[i].rotation_matrix = result_end.block(i * so_n, 0, so_n, so_n);
                solve_data_yaw[i].rotation_matrix.transposeInPlace();
            }
            // ROS_INFO("Drone_%d_:solve_data_yaw[%d]:    %f",          drone_datainfo.drone_id, i, solve_data_yaw[i].angle_yaw);
            // ROS_INFO("Drone_%d_:solve_data_matrix[%d]: %f %f;%f %f.",drone_datainfo.drone_id, i, solve_data_yaw[i].rotation_matrix(0,0),solve_data_yaw[i].rotation_matrix(0,1),solve_data_yaw[i].rotation_matrix(1,0),solve_data_yaw[i].rotation_matrix(1,1));
        }

        // 初始化观测数据合集
        solve_data_yaw_rotate.resize(drone_num); // 分别从drone_num个飞机的角度去计算相对位置
        for(int i=0;i<drone_num;i++)
        {
            solve_data_yaw_rotate[i] = solve_data_yaw;
            solve_data_yaw_rotate[i];
            double angle_temp_i = angle_result[i];
            for(int j=0;j<drone_num;j++)
            {
                double angle_temp_j = angle_result[j];
                double angle_temp_ij = -angle_temp_i + angle_temp_j;
                angle_temp_ij = (-1) * angle_temp_ij;

                solve_data_yaw_rotate[i][j].angle_yaw = angle_temp_ij;
                Eigen::Matrix2d rotation_matrix_temp;
                rotation_matrix_temp << cos(angle_temp_ij), -sin(angle_temp_ij),
                                        sin(angle_temp_ij),  cos(angle_temp_ij);
                solve_data_yaw_rotate[i][j].rotation_matrix = rotation_matrix_temp;
                // solve_data_yaw_rotate[i][j].rotation_matrix.transposeInPlace();
                for(int k=0;k<solve_data_yaw_rotate[i][j].observed_num;k++)
                {
                    double x = solve_data_yaw_rotate[i][j].observed_positions[k].x();
                    double y = solve_data_yaw_rotate[i][j].observed_positions[k].y();
                    double z = solve_data_yaw_rotate[i][j].observed_positions[k].z();
                    Eigen::Vector2d temp_vec(x, y);
                    // 应用旋转矩阵
                    temp_vec = solve_data_yaw_rotate[i][j].rotation_matrix * temp_vec;
                    solve_data_yaw_rotate[i][j].observed_positions[k] = Eigen::Vector3d(temp_vec.x(), temp_vec.y(), z);
                    // ROS_INFO("Drone_%d:relative to %d :observer2_%d: %d  x:%f y:%f z:%f", drone_datainfo.drone_id,i,j, k,solve_data_yaw_rotate[i][j].observed_positions[k].x(), solve_data_yaw_rotate[i][j].observed_positions[k].y(), solve_data_yaw_rotate[i][j].observed_positions[k].z());
                    // ROS_INFO("Drone_%d:relative to %d :observer1_%d: %d  x:%f y:%f z:%f", drone_datainfo.drone_id,i,j, k,solve_data_yaw[i].observed_positions[k].x(), solve_data_yaw[i].observed_positions[k].y(), solve_data_yaw[i].observed_positions[k].z());
                }
            }
        }
        // 累加每一个observer的观测数量
        int total_observe_position_num = 0;
        std::vector<int> drones_observed_num;
        // drones_observed_num.resize(drone_num);
        int drones_observed_num_temp=0;
        for(int i=0;i<drone_num;i++)
        {
            total_observe_position_num = total_observe_position_num + solve_data_yaw[i].observed_num;
            drones_observed_num.push_back(total_observe_position_num);
            // drones_observed_num_temp=drones_observed_num[i];
            // ROS_INFO("Drone_%d_:drones_observed_num[%d]=%d", drone_datainfo.drone_id, i, drones_observed_num[i]);

        }
        // ROS_INFO("Drone_%d_:total_observe_position_num:%d", drone_datainfo.drone_id, total_observe_position_num);

        // 匈牙利算法求解drone_num次一次得到不完整的drone_num个T的相对位置
        t_result.clear();
        t_result.resize(drone_num);
        for(int i=0;i<drone_num;i++)
        {
            
            t_result[i].resize(drone_num);
            solve_data_yaw_rotate[i];
            if(solve_data_yaw_rotate[i][i].observed_num==0)
            {
                // 没有观测，赋值为空
                for(int j=0;j<drone_num;j++)
                {
                    t_result[i][j].is_known = false;
                    t_result[i][j].t = Eigen::Vector3d::Zero();
                }

            }
            else if(solve_data_yaw_rotate[i][i].angle_yaw!=0)
            {
                ROS_ERROR("Drone_%d_:solve_data_yaw_rotate[%d][%d].angle_yaw!=0", drone_datainfo.drone_id, i, i);
            }
            else
            {
                // 执行匈牙利算法
                // 计算成本矩阵
                std::vector<std::vector<double>> cost_matrix;
                cost_matrix.resize(solve_data_yaw_rotate[i][i].observed_num);
                // for(int j=0;j<solve_data_yaw_rotate[i][i].observed_num;j++)
                // {
                //     cost_matrix[j].resize(total_observe_position_num - solve_data_yaw_rotate[i][i].observed_num);
                // }
                // Eigen::MatrixXd::Zero(solve_data_yaw_rotate[i][i].observed_num, total_observe_position_num - solve_data_yaw_rotate[i][i].observed_num);

                for(int j=0;j<solve_data_yaw_rotate[i][i].observed_num;j++) // 列
                {
                    int rows_index=0;
                    for(int k=0;k<drone_num;k++)// 行大
                    {
                        for(int m=0;m<solve_data_yaw_rotate[i][k].observed_num;m++) //行小
                        {     

                            // cost_matrix(j,rows_index+m) = (solve_data_yaw_rotate[i][i].observed_positions[j] - solve_data_yaw_rotate[i][k].observed_positions[m]).norm();
                            // cost_matrix[j][rows_index+m] = (solve_data_yaw_rotate[i][i].observed_positions[j] - solve_data_yaw_rotate[i][k].observed_positions[m]).norm();
                            cost_matrix[j].push_back((solve_data_yaw_rotate[i][i].observed_positions[j] + solve_data_yaw_rotate[i][k].observed_positions[m]).norm());
                        }
                        // rows_index = rows_index + solve_data_yaw_rotate[i][k].observed_num;
                        
                    }
                    if(cost_matrix[j].size()!=total_observe_position_num)
                    {
                        // ROS_INFO cost_matrix[j].size();
                        // ROS_INFO total_observe_position_num - solve_data_yaw_rotate[i][i].observed_num;
                        // ROS_INFO("Drone_%d_:total_observe_position_num: %d;solve_data_yaw_rotate[i][i].observed_num: %d", drone_datainfo.drone_id, total_observe_position_num, solve_data_yaw_rotate[i][i].observed_num);
                        // ROS_INFO("Drone_%d_:cost_matrix[j].size(): %ld", drone_datainfo.drone_id, cost_matrix[j].size());
                        ROS_ERROR("Drone_%d_:cost_matrix[j].size()!=total_observe_position_num", drone_datainfo.drone_id);
                    }
                    else
                    {
                        ROS_INFO("Drone_%d:cost_matrix[j].size()==total_observe_position_num", drone_datainfo.drone_id);
                    }

                }
                // 显示cost_matrix
                // ROS_INFO("Drone_%d:cost_matrix", drone_datainfo.drone_id);
                // cout逐行显示cost_matrix
                // for(int j=0;j<solve_data_yaw_rotate[i][i].observed_num;j++)
                // {
                //     for(int k=0;k<total_observe_position_num;k++)
                //     {
                //         std::cout << cost_matrix[j][k] << " ";
                //     }
                //     std::cout << std::endl;
                // }

                // 通过匈牙利算法计算costmatrix
                std::vector<int> assignment;
                double cost = HungAlgoSolve(cost_matrix, assignment); // 有可能会解错，要担心！
                // for (unsigned int x = 0; x < cost_matrix.size(); x++)
                //     std::cout << x << "," << assignment[x] << "\t";
                // std::cout << "\ncost: " << cost << std::endl;

                // 通过assignment计算t
                // 通过assignment计算是第几个观测者的第几个观测
                // ROS_INFO("Drone_%d:assignment.size():%ld", drone_datainfo.drone_id, assignment.size());
                for(int j=0;j<assignment.size();j++)
                {
                    for(int k=0;k<drone_num;k++)
                    {
                        // ROS_INFO("Drone_%d:drones_observed_num[%d]=%d", drone_datainfo.drone_id, k, drones_observed_num[k]);
                        if(assignment[j]<drones_observed_num[k])
                        {
                            // ok
                            // j观测对应k飞机，赋值
                            // ROS_INFO("Drone_%d_:assignment[%d]:%d", drone_datainfo.drone_id, j, assignment[j]);
                            t_result[i][k].is_known = true;
                            int index=0;
                            if(k==0)
                            {
                                index = assignment[j];
                            }
                            else
                            {
                                index = assignment[j] - drones_observed_num[k-1];
                            }
                            t_result[i][k].t = solve_data_yaw_rotate[i][i].observed_positions[j];
                            // ROS_INFO("Drone_%d_:t_result[%d][%d]: %f %f %f", drone_datainfo.drone_id, i, k, t_result[i][k].t.x(), t_result[i][k].t.y(), t_result[i][k].t.z());
                            break;
                        }
                        else
                        {
                            // continue;
                        }
                    }
                }
            }
        }
        // ROS_INFO("Drone_%d:observe_position_estimate", drone_datainfo.drone_id);
        int id = drone_datainfo.drone_id;
        for(int i=0;i<drone_num;i++)
        {
            if(t_result[id][i].is_known)
            {
                ROS_INFO("Drone_%d_:t_result[%d][%d]: %f %f %f", drone_datainfo.drone_id, id, i, t_result[id][i].t.x(), t_result[id][i].t.y(), t_result[id][i].t.z());
            }
            else
            {
                // 用图搜索的方式或递归的方式找到t_result[x][i].is_known=true，回溯得到t_result[id][i].t
                ROS_INFO("Drone_%d_:t_result[%d][%d]: is unknown", drone_datainfo.drone_id, id, i);
                Eigen::Vector3d accumulatedT = Eigen::Vector3d::Zero();
                std::vector<bool> visited(drone_num, false);
                if (findPathAndAccumulateT(id, i, accumulatedT, visited)) 
                {
                    t_result[id][i].t = accumulatedT;
                    t_result[id][i].is_known = true;
                    ROS_INFO("Drone %d: t_result[%d][%d] found: %f %f %f", id, id, i, accumulatedT.x(), accumulatedT.y(), accumulatedT.z());
                } else 
                {
                    ROS_INFO("Drone %d: t_result[%d][%d]: is unknown because of no path finding!", id, id, i);
                }
            }
        }
        // 显示observe_data_yaw的所有观测
        ROS_INFO("Drone_%d:observe_position_real", drone_datainfo.drone_id);
        for(int i=0;i<observe_data_yaw.observed_num;i++)
        {
            // observe_data_yaw
            ROS_INFO("Drone_%d:observe_data_yaw[%d]: %f %f %f", drone_datainfo.drone_id, i, observe_data_yaw.observed_positions[i].x(), observe_data_yaw.observed_positions[i].y(), observe_data_yaw.observed_positions[i].z());

        }
    }

    bool FACTInitializerFSM::findPathAndAccumulateT(int start, int end, Eigen::Vector3d& accumulatedT, std::vector<bool>& visited) {
        std::queue<Node> queue;
        queue.push(Node(start, Eigen::Vector3d::Zero(), std::vector<int>()));

        while (!queue.empty()) {
            Node current = queue.front();
            queue.pop();
            if (current.droneId == end) {
                accumulatedT = current.accumulatedT;
                return true;
            }
            if (!visited[current.droneId]) 
            {
                visited[current.droneId] = true;
                for (int i = 0; i < drone_num; i++) {
                    if (t_result[current.droneId][i].is_known && !visited[i]) {
                        std::vector<int> newPath = current.path;
                        newPath.push_back(i);
                        // // [current.droneId][i] 角度信息
                        // solve_data_yaw_rotate[current.droneId][i].rotation_matrix;
                        // double x = t_result[current.droneId][i].t.x();
                        // double y = t_result[current.droneId][i].t.y();
                        // double z = t_result[current.droneId][i].t.z();
                        // Eigen::Vector2d temp_vec(x, y);
                        // // 应用旋转矩阵
                        // temp_vec = solve_data_yaw_rotate[current.droneId][i].rotation_matrix * temp_vec;
                        // Eigen::Vector3d newAccumulatedT = current.accumulatedT + Eigen::Vector3d(temp_vec.x(), temp_vec.y(), z);
                        // 更新累计T:累计yaw角旋转矩阵乘以观测值
                        // 更新累计yaw角(用观测估计的yaw即可)
                        // 定义一个3×3的旋转矩阵
                        Eigen::Matrix3d rotationMatrix_temp;
                        double yaw_temp = angle_result[current.droneId]-angle_result[start];
                        yaw_temp = (-1) * yaw_temp;
                        rotationMatrix_temp << cos(yaw_temp), -sin(yaw_temp), 0,
                                                sin(yaw_temp), cos(yaw_temp), 0,
                                                0,                  0,        1;
                        Eigen::Vector3d newAccumulatedT = current.accumulatedT + rotationMatrix_temp * t_result[current.droneId][i].t;
                        queue.push(Node(i, newAccumulatedT, newPath));
                    }
                }
            }
        }
        return false;
    }


}