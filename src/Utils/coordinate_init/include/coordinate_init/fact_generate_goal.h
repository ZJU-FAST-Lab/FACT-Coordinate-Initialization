#ifndef _FACT_GENERATE_GOAL_H_
#define _FACT_GENERATE_GOAL_H_

#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <algorithm>
#include <numeric>

namespace fact_initializer
{
    class FACTGenerateGoal 
    {
        public:
            Eigen::Vector2d nearestPoint;  // 离原点最近的点
            double nearestDistance;        // 离原点最近的距离
            double densestAngle;           // 点分布最密集的角度
            double sparsestAngle;          // 点分布最稀疏的角度
            void initFrom3D(const std::vector<Eigen::Vector3d>& points3D, bool adaptiveBandwidth = true, double fixedBandwidth = 0.1);
        private:
            double bandwidth; // 带宽
            std::vector<double> calculateKDE(const std::vector<double>& angles);
            double computeAdaptiveBandwidth(const std::vector<double>& angles);
            double angleDifference(double angle1, double angle2);
    };
}

#endif
