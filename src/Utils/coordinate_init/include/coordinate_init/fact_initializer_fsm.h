#ifndef _FACT_INITIALIZER_FSM_H_
#define _FACT_INITIALIZER_FSM_H_

#include <Eigen/Eigen>
#include <eigen3/unsupported/Eigen/KroneckerProduct>
#include <algorithm>
#include <iostream>
#include <random>
#include <vector>

#include <stdlib.h>
#include <cfloat> // for DBL_MAX
#include <cmath>  // for fabs()

#include <queue>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/GoalSet.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <coordinate_init/ObserveData.h>

#include <coordinate_init/fact_random_init.h>
// #include <coordinate_init/fact_calipers.h>
#include "coordinate_init/fact_generate_goal.h"
#include "coordinate_init/HungAlgoSolver.h"

#include <mosek.h>
#include <fusion.h>
#include <monty.h>


using std::vector;
using namespace mosek::fusion;
using namespace monty;

namespace fact_initializer
{
    class FACTInitializerFSM
    {
        public:
            FACTInitializerFSM() {}
            ~FACTInitializerFSM() {}
            void init(ros::NodeHandle &nh, int argc_drone_id);
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        private:
            enum EXEC_TYPE{
                INIT=0,
                OBSERVE=1, // Rotation and observation are coupled.
                SOLVE=2,
                MOTION=3,
                STOP=4
            };
            struct Drone_Data {
                int drone_id;
                Eigen::Vector3d odom_pos;
                Eigen::Vector3d odom_vel;
                tf2::Quaternion quat;
                Eigen::Vector3d angleRPY;  // 欧拉角
                Eigen::Vector3d angular_vel;

                Drone_Data() : drone_id(-1), odom_pos(Eigen::Vector3d::Zero()), 
                            odom_vel(Eigen::Vector3d::Zero()), quat(tf2::Quaternion()), 
                            angleRPY(Eigen::Vector3d::Zero()), angular_vel(Eigen::Vector3d::Zero()){}

                Drone_Data(int id) : drone_id(id), odom_pos(Eigen::Vector3d::Zero()), 
                                    odom_vel(Eigen::Vector3d::Zero()), quat(tf2::Quaternion()), 
                                    angleRPY(Eigen::Vector3d::Zero()), angular_vel(Eigen::Vector3d::Zero()) {}

                void update(const nav_msgs::Odometry::ConstPtr& odom_msg) {
                    odom_pos = Eigen::Vector3d(odom_msg->pose.pose.position.x,
                                            odom_msg->pose.pose.position.y,
                                            odom_msg->pose.pose.position.z);

                    odom_vel = Eigen::Vector3d(odom_msg->twist.twist.linear.x,
                                            odom_msg->twist.twist.linear.y,
                                            odom_msg->twist.twist.linear.z);

                    tf2::fromMsg(odom_msg->pose.pose.orientation, quat);
                    tf2::Matrix3x3 m(quat);
                    m.getRPY(angleRPY[0], angleRPY[1], angleRPY[2]);

                    angular_vel = Eigen::Vector3d(odom_msg->twist.twist.angular.x,
                                                odom_msg->twist.twist.angular.y,
                                                odom_msg->twist.twist.angular.z);
                }
            };
            // FSM variables
            EXEC_TYPE exec_type;
            ros::Timer exec_timer;
            bool is_rotated, is_gotobservation, have_odom;
            ros::Subscriber mandatory_stop_sub;
            ros::Publisher  heartbeat_pub;
            void execFSMCallback(const ros::TimerEvent &e);
            void mandatoryStopCallback(const std_msgs::Empty &msg);
            void printFSMExecState();


            // state variables
            int drone_num;
            Drone_Data drone_datainfo, drone_datainfo_rotationinit;
            std::vector<Drone_Data> drone_datainfo_others;
            ros::Subscriber odom_sub;
            std::vector<ros::Subscriber> odom_sub_others;
            void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
            int seed;
            ros::Publisher mandatory_stop_pub_;

            // init

            // rotation
            double rotation_speed;
            ros::Publisher  pos_cmd_pub;
            void RotateOneCircle(bool clockwise, double rotation_speed);

            // observation
            int observe_num;  
            struct Observe_Data {
                int observer_id; // 观测者的ID
                int observed_num; // 观测到的飞机数量
                std::vector<Eigen::Vector3d> observed_positions; // 观测到的飞机位置
                Observe_Data(int id) : observer_id(id), observed_num(0) {}
                Observe_Data() : observer_id(-1), observed_num(0) {}
            };
            vector<double> init_all_yaw;// 需要观测估计的值
            Observe_Data observe_data_yaw, observe_data_quat,observe_data_yaw_part, observe_data_quat_part;
            vector<Observe_Data> observe_data_yaw_part_vector, observe_data_quat_part_vector;
            void GetAllObservation();
            void GetPartObservation();
            void observeFSMCallback(const ros::TimerEvent &e);
            bool IsinRange(const Eigen::Vector3d pc, const Eigen::Vector3d pw);
            void calculateRelativePositionQuat(const Drone_Data& other_drone);
            void calculateRelativePositionYaw(const Drone_Data& other_drone);
            int observe_counter,least_observe_num;
            ros::Publisher pub_observe;
            std::vector<ros::Subscriber> get_observe;
            std::map<int, std::vector<std::vector<Eigen::Vector3d>>> observe_others;
            void observeCallback(const coordinate_init::ObserveData::ConstPtr& msg, int drone_id);
            void publishObserveData(bool use_yaw_data);
            bool CheckObserveData();

            // Solve
            enum solver_type{
                SDP=0,
                LM=1,
                GaussNewton=2,
            };
            solver_type solver;
            int so_n;
            // static void MSKAPI printstr(void *handle, const char str[]);
            Eigen::MatrixXd GetCMatrix(int i);
            void printCMatrix(const Eigen::MatrixXd& CMatrix);
            std::vector<std::vector<Eigen::Vector2d>> P;
            std::vector<Eigen::Vector2d> Ptest;

            Eigen::MatrixXd Q_sum, Q_sumall;
            std::vector<Eigen::MatrixXd> Q;
            std::vector<double> initial_yaw_angles; // 存储所有无人机的初始yaw角
            std::vector<std::vector<double>> yaw_angle_differences; // 存储每次调用时的yaw角差值
            void recordInitialYawAngles();
            void updateYawAngleDifferences();
            void SolveMySDP();
            void GetPMatrix();
            void GetQMatrix();
            void printEigenMatrix(const Eigen::MatrixXd& matrix);
            bool add_noise;
            double noise_rate;
            double noise_mean;
            bool SDP_is_solved;
            void SolveTbyHungarian();
            struct Solve_Data {
                int drone_id; // 观测者的ID
                int observed_num; // 观测到的飞机数量
                double  angle_yaw;
                Eigen::Matrix2d rotation_matrix;
                std::vector<Eigen::Vector3d> observed_positions; // 观测到的飞机位置
                Solve_Data(int id) : drone_id(id), observed_num(0) {}
                Solve_Data() : drone_id(-1), observed_num(0) {}
            };
            std::vector<Solve_Data> solve_data_yaw;
            std::vector<std::vector<Solve_Data>>  solve_data_yaw_rotate;
            Eigen::MatrixXd Z_star;
            Eigen::MatrixXd result_end;
            std::vector<double> angle_result;
            double mean_error;
            struct T_data{
                bool is_known;
                Eigen::Vector3d t;
            };
            std::vector<std::vector<T_data>> t_result;
            // double HungAlgoSolve(std::vector <std::vector<double> >& DistMatrix, std::vector<int>& Assignment);
            // void assignmentoptimal(int *assignment, double *cost, double *distMatrix, int nOfRows, int nOfColumns);
            // void buildassignmentvector(int *assignment, bool *starMatrix, int nOfRows, int nOfColumns);
            // void computeassignmentcost(int *assignment, double *cost, double *distMatrix, int nOfRows);
            // void step2a(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
            // void step2b(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
            // void step3(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
            // void step4(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim, int row, int col);
            // void step5(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
            // 添加新的成员函数声明
            struct Node {
                int droneId;
                Eigen::Vector3d accumulatedT;
                std::vector<int> path;

                Node(int id, Eigen::Vector3d t, std::vector<int> p) : droneId(id), accumulatedT(t), path(p) {}
            };
            bool findPathAndAccumulateT(int start, int end, Eigen::Vector3d& accumulatedT, std::vector<bool>& visited);

            // motion
            ros::Publisher goal_pub_;
            void publishGoal(const Eigen::Vector3d& target_position);
            Eigen::Vector3d mygoal;
            void GenerateGoal();
            void WaitingtoArriveGoal();
            bool have_arrived_goal;
            visualization_msgs::Marker marker_endpoint;
            ros::Publisher marker_pub_endpoint;
            void ControltoGoal();
            bool OutofMap(const Eigen::Vector3d& pos);
            double my_map_size_width;
            double my_map_size_depth;
            double my_map_size_height;
            bool no_control;
            bool need_rotate;

            struct Obstacle {
                Eigen::Vector2d position; // XY position of the obstacle
                double radius; // Radius of the obstacle
            };
            std::vector<Obstacle> obstacles;
            void GoalIsInObstacle();


            // visulize yaw angle
            ros::Publisher cmd_vis_pub;
            double vis_dist;
            double hor;  
            double vert; 
            double hor_angle;
            double vert_angle;
            Eigen::Vector3d origin,left_up,left_down,right_up,right_down;
            // camera FOV parameter
            vector<Eigen::Vector3d> cam_vertices1, cam_vertices2;
            void getFOV(vector<Eigen::Vector3d>& list1, vector<Eigen::Vector3d>& list2, Eigen::Vector3d pos, double yaw);
            void drawFOV(const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2);

            // wheels
            ros::Timer observe_timer;
            bool AngleEqual(double ,double ,double,bool);
            bool AngleMore(double ,double,double);            
            int rankOfMatrix(const Eigen::MatrixXd& matrix) const;
            template<typename T>
            bool isEqual(const T& x, const T& y = T::Zero(), double tolerance = 1e-6) const;

            struct output {
                std::chrono::steady_clock::time_point start_time;
                std::chrono::duration<double> sdp_time, hung_time, total_time;
                double optimal, optimal2, optimal_real;
                std::vector<double> angle_result_real;
                bool finished = false;
                std::string solver;
            };
            output output_data;
            std::string results_path;
    };

    struct CostFunctor {
        CostFunctor(const Eigen::MatrixXd& Q) : Q_(Q) {}
        template <typename T>
        bool operator()(const T* const angle0, const T* const angle1, T* residual) const {
            Eigen::Matrix<T, 2, 2> R0 = Eigen::Rotation2D<T>(*angle0).toRotationMatrix();
            Eigen::Matrix<T, 2, 2> R1 = Eigen::Rotation2D<T>(*angle1).toRotationMatrix();
            Eigen::Matrix<T, 2, 4> R;
            R << R0, R1;
            Eigen::Matrix<T, 4, 4> Q = Q_.cast<T>();
            residual[0] = (Q * R.transpose() * R).trace();
            return true;
        }

        template <typename T>
        bool operator()(const T* const angle0, const T* const angle1, const T* const angle2, T* residual) const {
            Eigen::Matrix<T, 2, 2> R0 = Eigen::Rotation2D<T>(*angle0).toRotationMatrix();
            Eigen::Matrix<T, 2, 2> R1 = Eigen::Rotation2D<T>(*angle1).toRotationMatrix();
            Eigen::Matrix<T, 2, 2> R2 = Eigen::Rotation2D<T>(*angle2).toRotationMatrix();
            Eigen::Matrix<T, 2, 6> R;
            R << R0, R1, R2;
            Eigen::Matrix<T, 6, 6> Q = Q_.cast<T>();
            residual[0] = (Q * R.transpose() * R).trace();
            return true;
        }

        template <typename T>
        bool operator()(const T* const angle0, const T* const angle1, const T* const angle2, const T* const angle3, T* residual) const {
            Eigen::Matrix<T, 2, 2> R0 = Eigen::Rotation2D<T>(*angle0).toRotationMatrix();
            Eigen::Matrix<T, 2, 2> R1 = Eigen::Rotation2D<T>(*angle1).toRotationMatrix();
            Eigen::Matrix<T, 2, 2> R2 = Eigen::Rotation2D<T>(*angle2).toRotationMatrix();
            Eigen::Matrix<T, 2, 2> R3 = Eigen::Rotation2D<T>(*angle3).toRotationMatrix();
            Eigen::Matrix<T, 2, 8> R;
            R << R0, R1, R2, R3;
            Eigen::Matrix<T, 8, 8> Q = Q_.cast<T>();
            residual[0] = (Q * R.transpose() * R).trace();
            return true;
        }

        template <typename T>
        bool operator()(const T* const angle0, const T* const angle1, const T* const angle2, const T* const angle3, const T* const angle4, T* residual) const {
            Eigen::Matrix<T, 2, 2> R0 = Eigen::Rotation2D<T>(*angle0).toRotationMatrix();
            Eigen::Matrix<T, 2, 2> R1 = Eigen::Rotation2D<T>(*angle1).toRotationMatrix();
            Eigen::Matrix<T, 2, 2> R2 = Eigen::Rotation2D<T>(*angle2).toRotationMatrix();
            Eigen::Matrix<T, 2, 2> R3 = Eigen::Rotation2D<T>(*angle3).toRotationMatrix();
            Eigen::Matrix<T, 2, 2> R4 = Eigen::Rotation2D<T>(*angle4).toRotationMatrix();
            Eigen::Matrix<T, 2, 10> R;
            R << R0, R1, R2, R3, R4;
            Eigen::Matrix<T, 10, 10> Q = Q_.cast<T>();
            residual[0] = (Q * R.transpose() * R).trace();
            return true;
        }

        template <typename T>
        bool operator()(const T* const angle0, const T* const angle1, const T* const angle2, const T* const angle3, const T* const angle4, const T* const angle5, T* residual) const {
            Eigen::Matrix<T, 2, 2> R0 = Eigen::Rotation2D<T>(*angle0).toRotationMatrix();
            Eigen::Matrix<T, 2, 2> R1 = Eigen::Rotation2D<T>(*angle1).toRotationMatrix();
            Eigen::Matrix<T, 2, 2> R2 = Eigen::Rotation2D<T>(*angle2).toRotationMatrix();
            Eigen::Matrix<T, 2, 2> R3 = Eigen::Rotation2D<T>(*angle3).toRotationMatrix();
            Eigen::Matrix<T, 2, 2> R4 = Eigen::Rotation2D<T>(*angle4).toRotationMatrix();
            Eigen::Matrix<T, 2, 2> R5 = Eigen::Rotation2D<T>(*angle5).toRotationMatrix();
            Eigen::Matrix<T, 2, 12> R;
            R << R0, R1, R2, R3, R4, R5;
            Eigen::Matrix<T, 12, 12> Q = Q_.cast<T>();
            residual[0] = (Q * R.transpose() * R).trace();
            return true;
        }

        template <typename T>
        bool operator()(const T* const angle0, const T* const angle1, const T* const angle2, const T* const angle3, const T* const angle4, const T* const angle5, const T* const angle6, T* residual) const {
            Eigen::Matrix<T, 2, 2> R0 = Eigen::Rotation2D<T>(*angle0).toRotationMatrix();
            Eigen::Matrix<T, 2, 2> R1 = Eigen::Rotation2D<T>(*angle1).toRotationMatrix();
            Eigen::Matrix<T, 2, 2> R2 = Eigen::Rotation2D<T>(*angle2).toRotationMatrix();
            Eigen::Matrix<T, 2, 2> R3 = Eigen::Rotation2D<T>(*angle3).toRotationMatrix();
            Eigen::Matrix<T, 2, 2> R4 = Eigen::Rotation2D<T>(*angle4).toRotationMatrix();
            Eigen::Matrix<T, 2, 2> R5 = Eigen::Rotation2D<T>(*angle5).toRotationMatrix();
            Eigen::Matrix<T, 2, 2> R6 = Eigen::Rotation2D<T>(*angle6).toRotationMatrix();
            Eigen::Matrix<T, 2, 14> R;
            R << R0, R1, R2, R3, R4, R5, R6;
            Eigen::Matrix<T, 14, 14> Q = Q_.cast<T>();
            residual[0] = (Q * R.transpose() * R).trace();
            return true;
        }

        template <typename T>
        bool operator()(const T* const angle0, const T* const angle1, const T* const angle2, const T* const angle3, const T* const angle4, const T* const angle5, const T* const angle6, const T* const angle7, T* residual) const {
            Eigen::Matrix<T, 2, 2> R0 = Eigen::Rotation2D<T>(*angle0).toRotationMatrix();
            Eigen::Matrix<T, 2, 2> R1 = Eigen::Rotation2D<T>(*angle1).toRotationMatrix();
            Eigen::Matrix<T, 2, 2> R2 = Eigen::Rotation2D<T>(*angle2).toRotationMatrix();
            Eigen::Matrix<T, 2, 2> R3 = Eigen::Rotation2D<T>(*angle3).toRotationMatrix();
            Eigen::Matrix<T, 2, 2> R4 = Eigen::Rotation2D<T>(*angle4).toRotationMatrix();
            Eigen::Matrix<T, 2, 2> R5 = Eigen::Rotation2D<T>(*angle5).toRotationMatrix();
            Eigen::Matrix<T, 2, 2> R6 = Eigen::Rotation2D<T>(*angle6).toRotationMatrix();
            Eigen::Matrix<T, 2, 2> R7 = Eigen::Rotation2D<T>(*angle7).toRotationMatrix();
            Eigen::Matrix<T, 2, 16> R;
            R << R0, R1, R2, R3, R4, R5, R6, R7;
            Eigen::Matrix<T, 16, 16> Q = Q_.cast<T>();
            residual[0] = (Q * R.transpose() * R).trace();
            return true;
        }

    private :
        const Eigen::MatrixXd Q_;
    };
}


#endif