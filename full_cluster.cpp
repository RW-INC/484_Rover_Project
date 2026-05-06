#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <Eigen/Dense>

#include <cmath>
#include <vector>
#include <limits>

class LidarObstacleProcessor : public rclcpp::Node {
public:
    LidarObstacleProcessor()
        : Node("lidar_obstacle_processor")
    {
        raw_cloud_lidar_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        cloud_body_      = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        filtered_cloud_  = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        kd_tree_         = pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>);

        R_body_lidar_ = computeMountRotation();

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/unilidar/cloud",
            10,
            std::bind(&LidarObstacleProcessor::pointCloudCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "LidarObstacleProcessor started.");
    }

    void process()
    {
        if (cloud_body_->empty()) {
            RCLCPP_WARN(this->get_logger(), "No point cloud data stored yet.");
            return;
        }

        filterPoints();
        buildKdTree();
        computeObstacleClusters();
        computeGaussianCosts();

        RCLCPP_INFO(
            this->get_logger(),
            "Processed cloud: raw=%zu, filtered=%zu, obstacles=%zu",
            cloud_body_->size(),
            filtered_cloud_->size(),
            obstacles_.size()
        );
    }

private:
    struct Obstacle {
        Eigen::Vector3f centroid;
        Eigen::Vector2f center_xy;

        Eigen::Matrix2f covariance_xy;
        Eigen::Matrix2f inv_covariance_xy;

        Eigen::Vector2f principal_axis;
        Eigen::Vector2f secondary_axis;

        float principal_variance;
        float secondary_variance;

        float height;
        float amplitude;
        int num_points;
    };

    float mount_offset_angle_rad_ = 1.0f;

    float min_x_ = 0.0f;
    float max_x_ = 10.0f;
    float max_abs_y_ = 10.0f;
    float min_z_ = 0.0f;
    float max_z_ = 10.0f;

    float ground_threshold_ = 0.03f;

    float cluster_tolerance_ = 0.03f;
    int min_cluster_size_ = 1;
    int max_cluster_size_ = 500000;

    float base_sigma_ = 0.40f;
    float sigma_per_height_ = 0.50f;
    float base_amplitude_ = 1.0f;
    float amplitude_per_height_ = 5.0f;

    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud_lidar_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_body_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree_;

    std::vector<Obstacle> obstacles_;

    Eigen::Matrix3f R_body_lidar_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        raw_cloud_lidar_->clear();
        cloud_body_->clear();

        pcl::fromROSMsg(*msg, *raw_cloud_lidar_);

        for (const auto& p_lidar : raw_cloud_lidar_->points) {
            if (!std::isfinite(p_lidar.x) ||
                !std::isfinite(p_lidar.y) ||
                !std::isfinite(p_lidar.z)) {
                continue;
            }

            Eigen::Vector3f p_L(p_lidar.x, p_lidar.y, p_lidar.z);
            Eigen::Vector3f p_B = R_body_lidar_ * p_L;

            pcl::PointXYZ p_body;
            p_body.x = p_B.x();
            p_body.y = p_B.y();
            p_body.z = p_B.z();

            cloud_body_->points.push_back(p_body);
        }

        cloud_body_->width = static_cast<uint32_t>(cloud_body_->points.size());
        cloud_body_->height = 1;
        cloud_body_->is_dense = false;

        process();
    }

    Eigen::Matrix3f computeMountRotation()
    {
        Eigen::Matrix3f R_yaw;

        R_yaw << std::cos(mount_offset_angle_rad_), -std::sin(mount_offset_angle_rad_), 0.0f,
                 std::sin(mount_offset_angle_rad_),  std::cos(mount_offset_angle_rad_), 0.0f,
                 0.0f,                               0.0f,                              1.0f;

        return R_yaw;
    }

    void filterPoints()
    {
        filtered_cloud_->clear();

        float projector_height = 4.0f;

        for (const auto& p : cloud_body_->points) {
            if (!std::isfinite(p.x) ||
                !std::isfinite(p.y) ||
                !std::isfinite(p.z)) {
                continue;
            }

            if (p.x < min_x_ || p.x > max_x_) continue;
            if (std::abs(p.y) > max_abs_y_) continue;
            if (p.z < min_z_ || p.z > projector_height) continue;

            if (p.z > 1.0f && p.z < 3.0f) continue;

            if (p.z > 1.0f) {
                RCLCPP_WARN(this->get_logger(), "Point with z > 1.0 m detected: z=%.2f", p.z);
            }

            // Optional ground removal:
            // if (p.z < ground_threshold_) continue;

            filtered_cloud_->points.push_back(p);
        }

        filtered_cloud_->width = static_cast<uint32_t>(filtered_cloud_->points.size());
        filtered_cloud_->height = 1;
        filtered_cloud_->is_dense = false;
    }

    void buildKdTree()
    {
        if (!filtered_cloud_->empty()) {
            kd_tree_->setInputCloud(filtered_cloud_);
        }
    }

    void computeObstacleClusters()
    {
        obstacles_.clear();

        if (filtered_cloud_->empty()) {
            return;
        }

        std::vector<pcl::PointIndices> cluster_indices;

        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(cluster_tolerance_);
        ec.setMinClusterSize(min_cluster_size_);
        ec.setMaxClusterSize(max_cluster_size_);
        ec.setSearchMethod(kd_tree_);
        ec.setInputCloud(filtered_cloud_);
        ec.extract(cluster_indices);

        for (const auto& indices : cluster_indices) {
            Obstacle obs;

            obs.centroid = Eigen::Vector3f::Zero();
            obs.center_xy = Eigen::Vector2f::Zero();
            obs.height = -std::numeric_limits<float>::infinity();
            obs.num_points = static_cast<int>(indices.indices.size());

            for (int idx : indices.indices) {
                const auto& p = filtered_cloud_->points[idx];

                obs.centroid += Eigen::Vector3f(p.x, p.y, p.z);

                if (p.z > obs.height) {
                    obs.height = p.z;
                }
            }

            obs.centroid /= static_cast<float>(obs.num_points);
            obs.center_xy << obs.centroid.x(), obs.centroid.y();

            Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();

            for (int idx : indices.indices) {
                const auto& p = filtered_cloud_->points[idx];

                Eigen::Vector2f d;
                d << p.x - obs.center_xy.x(),
                     p.y - obs.center_xy.y();

                cov += d * d.transpose();
            }

            cov /= static_cast<float>(obs.num_points);

            float min_sigma = 0.25f;
            cov += Eigen::Matrix2f::Identity() * min_sigma * min_sigma;

            obs.covariance_xy = cov;
            obs.inv_covariance_xy = cov.inverse();

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(cov);

            Eigen::Vector2f eigenvalues = solver.eigenvalues();
            Eigen::Matrix2f eigenvectors = solver.eigenvectors();

            obs.secondary_variance = eigenvalues(0);
            obs.principal_variance = eigenvalues(1);

            obs.secondary_axis = eigenvectors.col(0);
            obs.principal_axis = eigenvectors.col(1);

            obs.amplitude = base_amplitude_ * (1.0f + 0.10f * obs.height);

            obstacles_.push_back(obs);
        }
    }

    void computeGaussianCosts()
    {
        for (auto& obs : obstacles_) {
            obs.amplitude = base_amplitude_ * (1.0f + 0.10f * obs.height);
        }
    }

    float gaussianCostAtPoint(float x, float y) const
    {
        float total_cost = 0.0f;

        Eigen::Vector2f p;
        p << x, y;

        for (const auto& obs : obstacles_) {
            Eigen::Vector2f d = p - obs.center_xy;

            float exponent = -0.5f * d.transpose() * obs.inv_covariance_xy * d;
            float cost = obs.amplitude * std::exp(exponent);

            total_cost += cost;
        }

        return total_cost;
    }

    float pathCostStraightLine(float heading_rad, float path_length) const
    {
        int num_samples = 50;
        float total_cost = 0.0f;

        for (int i = 1; i <= num_samples; i++) {
            float s = path_length * static_cast<float>(i) / static_cast<float>(num_samples);

            float x = s * std::cos(heading_rad);
            float y = s * std::sin(heading_rad);

            total_cost += gaussianCostAtPoint(x, y);
        }

        return total_cost;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LidarObstacleProcessor>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}