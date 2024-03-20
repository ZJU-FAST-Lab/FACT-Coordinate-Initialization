#include "coordinate_init/fact_generate_goal.h"

namespace fact_initializer
{
    // 定义高斯核函数
    double gaussianKernel(double distance, double bandwidth) {
        double term = distance / bandwidth;
        return (1 / (sqrt(2 * M_PI) * bandwidth)) * exp(-0.5 * term * term);
    }

    // 考虑周期性的角度差
    double FACTGenerateGoal::angleDifference(double angle1, double angle2) {
        double diff = angle1 - angle2;
        while (diff > M_PI) diff -= 2 * M_PI;
        while (diff < -M_PI) diff += 2 * M_PI;
        return diff;
    }

    // KDE的实现
    std::vector<double> FACTGenerateGoal::calculateKDE(const std::vector<double>& angles) {
        std::vector<double> density(angles.size(), 0.0);
        for (size_t i = 0; i < angles.size(); ++i) {
            for (size_t j = 0; j < angles.size(); ++j) {
                double distance = std::abs(angleDifference(angles[i], angles[j]));
                density[i] += gaussianKernel(distance, this->bandwidth);
            }
        }

        double sum_density = std::accumulate(density.begin(), density.end(), 0.0);
        for (auto& d : density) {
            d /= sum_density;
        }

        return density;
    }

    // 自适应带宽的计算
    double FACTGenerateGoal::computeAdaptiveBandwidth(const std::vector<double>& angles) {
        // 示例：基于标准差的带宽计算
        double mean = std::accumulate(angles.begin(), angles.end(), 0.0) / angles.size();
        double sq_sum = std::inner_product(angles.begin(), angles.end(), angles.begin(), 0.0);
        double stdev = std::sqrt(sq_sum / angles.size() - mean * mean);
        return stdev * sqrt(2.0);
    }

    void FACTGenerateGoal::initFrom3D(const std::vector<Eigen::Vector3d>& points3D, bool adaptiveBandwidth, double fixedBandwidth) {
        std::vector<Eigen::Vector2d> points2D;
        std::vector<double> angles;
        nearestDistance = std::numeric_limits<double>::max();

        for (const auto& point : points3D) {
            Eigen::Vector2d point2D(point.x(), point.y());
            points2D.push_back(point2D);
            double distance = point2D.norm();
            double angle = std::atan2(point2D.y(), point2D.x());
            angles.push_back(angle);

            if (distance < nearestDistance) {
                nearestDistance = distance;
                nearestPoint = point2D;
            }
        }

        this->bandwidth = adaptiveBandwidth ? computeAdaptiveBandwidth(angles) : fixedBandwidth;
        auto density = calculateKDE(angles);

        auto max_it = std::max_element(density.begin(), density.end());
        densestAngle = angles[std::distance(density.begin(), max_it)];
        auto min_it = std::min_element(density.begin(), density.end());
        sparsestAngle = angles[std::distance(density.begin(), min_it)];
    }
}