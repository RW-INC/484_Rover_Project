/*
For the file to run:
- ROS2 needs to be running
- A driver is publishing to /unilidar/cloud
- The node is built in a ROS2 package and is actually running (i.e. ros2 run <package_name> <node_name>)    
*/

/* Summary:
Converts /unilidar/cloud PointCloud2 data into XYZ points, stores them in a class, then processes them into:
1. filtered obstacle-candidate points
2. KD-tree
3. obstacle clusters
4. Gaussian obstacle cost model

More assumptions
- The LiDAR topic is /unilidar/cloud
- The topic type is sensor_msgs/msg/PointCloud2
- PCL is available (point cloud library)
- Rover body frame convention:
    x = forward
    y = left
    z = up
- LiDAR points initially arrive in the LiDAR frame
- LiDAR mounting error is modeled as a yaw rotation about z then 180 deg roll about x
- Points close to ground are ignored as non-obstacles
- Obstacles are found by clustering remaining points
- Each obstacle cluster becomes one Gaussian cost source
*/

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
        raw_cloud_lidar_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(
            new pcl::PointCloud<pcl::PointXYZ>
        );

        cloud_body_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(
            new pcl::PointCloud<pcl::PointXYZ>
        );

        filtered_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(
            new pcl::PointCloud<pcl::PointXYZ>
        );

        kd_tree_ = pcl::search::KdTree<pcl::PointXYZ>::Ptr(
            new pcl::search::KdTree<pcl::PointXYZ>
        );

        R_body_lidar_ = computeMountRotation();

        // Connects to LiDAR data stream, basically says to Listen to /unilidar/cloud, and every time new data arrives
        // run function pointCloudCallback(msg)
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/unilidar/cloud",
            10,
            std::bind(
                &LidarObstacleProcessor::pointCloudCallback,
                this,
                std::placeholders::_1
            )
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
        Eigen::Vector2f center_xy;
        float height;
        float sigma;
        float amplitude;
        int num_points;
    };

    // PARAMETERS

    // Assumption - angle between LiDAR +x and rover +x
    // Positive means LiDAR frame is yawed counterclockwise relative to rover frame - may need sign change
    float mount_offset_angle_rad_ = 1.0f;

    // Region of interest in rover body frame, creates 10x10 local cell even though it is not the same as global 10x10 cell
    float min_x_ = 0.0f;
    float max_x_ = 10.0f; // Keeps everything from 0 to 10 m in positive x
    float max_abs_y_ = 10.0f; // Keeps everything in +-5 m
    float min_z_ = 0.0f; // Filters out z > 1 and z < 0. Probably need to refine this but chose these values for now, note that on
    // flat surface the expected zs should be like -.6 (rover height)
    float max_z_ = 10.0f; // In theory the LiDAR should never see above like .6 m also, so can use this knowledge to diagnose issues

    // Ground removal threshold - Assumption - Anything below this height is ground or too small to matter.
    float ground_threshold_ = 0.03f; // Probably need to refine, but 3 cm is the smallest obstacle we need to avoid so base choice

    // Clustering parameters - maybe change
    float cluster_tolerance_ = 0.03f; // this kind of makes sense, max distance between points in a cluster
    int min_cluster_size_ = 1; // i think 
    int max_cluster_size_ = 500000; // no real max i think

    // Gaussian cost parameters - probably change picked arbitrarily
    float base_sigma_ = 0.40f;
    float sigma_per_height_ = 0.50f;
    float base_amplitude_ = 1.0f;
    float amplitude_per_height_ = 5.0f;

    // STORED DATA

    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud_lidar_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_body_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree_;

    std::vector<Obstacle> obstacles_;

    Eigen::Matrix3f R_body_lidar_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

    // CALLBACK - raw ROS data to XYZ

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        raw_cloud_lidar_->clear();
        cloud_body_->clear();

        // Convert ROS PointCloud2 binary message into PCL (point cloud library) XYZ points
        pcl::fromROSMsg(*msg, *raw_cloud_lidar_);

        // Rotate points to correct for LiDAR mounting orientation - no translation offset is applied

        for (const auto& p_lidar : raw_cloud_lidar_->points) {
            Eigen::Vector3f p_L(p_lidar.x, p_lidar.y, p_lidar.z);

            // p_B = R_BL * p_L + r_BL (r_BL = 0)
            Eigen::Vector3f p_B = R_body_lidar_ * p_L;

            pcl::PointXYZ p_body;
            p_body.x = p_B.x();
            p_body.y = p_B.y();
            p_body.z = p_B.z();

            cloud_body_->points.push_back(p_body);
        }

        cloud_body_->width = cloud_body_->points.size();
        cloud_body_->height = 1;
        cloud_body_->is_dense = false;

        // For now, process immediately after each scan
        // If this becomes too slow, call process() from a timer instead
        process();
    }

    // Rotation helper

    Eigen::Matrix3f computeMountRotation()
    {
        // Assumption - mount_offset_angle_rad_ is yaw between LiDAR +x and rover +x (this may need a sign flip)
        // Sign flip might be easiest if done during actual testing
        Eigen::Matrix3f R_yaw;

        R_yaw << std::cos(mount_offset_angle_rad_), -std::sin(mount_offset_angle_rad_), 0.0f,
                std::sin(mount_offset_angle_rad_),  std::cos(mount_offset_angle_rad_), 0.0f,
                0.0f,                               0.0f,                              1.0f;

        // Assumption - LiDAR is mounted upside down by a 180 deg roll about its x-axis.
        Eigen::Matrix3f R_upside_down;

        R_upside_down << 1.0f,  0.0f,  0.0f,
                        0.0f, -1.0f,  0.0f,
                        0.0f,  0.0f, -1.0f;

        // Apply yaw, no more upside down correction
        return R_yaw;
    }

    // FILTERING STUFF

    void filterPoints()
    {
        filtered_cloud_->clear(); // initialize

        float projector_height = 4.0f; // Define the height of the projector area, meters

        for (const auto& p : cloud_body_->points) {
            if (!std::isfinite(p.x) ||
                !std::isfinite(p.y) ||
                !std::isfinite(p.z)) {
                continue;
            }

            // Keep only points in front of rover and inside local planning box - defined earlier
            if (p.x < min_x_ || p.x > max_x_) continue;
            if (std::abs(p.y) > max_abs_y_) continue;
            if (p.z < 0.0f || p.z > projector_height) continue;

            if (p.z > 1.0f && p.z < 3.0f) continue; // filter out between 1 and 3 m

            if (p.z > 1.0f) { // this is just for testing idk but want to see print statements
                RCLCPP_WARN(this->get_logger(), "Point with z > 1.0 m detected, which is unexpected: z=%.2f", p.z);
            }

            // Remove ground and features below requirement threshold
            // if (p.z < (-.6 + ground_threshold_) && p.z > (-.6 - ground_threshold_)) continue; // negative .6 is height of rover

            filtered_cloud_->points.push_back(p);
        }

        filtered_cloud_->width = filtered_cloud_->points.size();
        filtered_cloud_->height = 1;
        filtered_cloud_->is_dense = false;
    }

    // KD-tree yikes, just builds it
    void buildKdTree()
    {
        if (filtered_cloud_->empty()) {
            return;
        }

        kd_tree_->setInputCloud(filtered_cloud_);
    }

    // Clustering

    void computeObstacleClusters()
    {
        obstacles_.clear();

        if (filtered_cloud_->empty()) {
            return;
        }

        std::vector<pcl::PointIndices> cluster_indices;

        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec; // Creates clustering object
        ec.setClusterTolerance(cluster_tolerance_); // parameter - max distance between neighboring points
        ec.setMinClusterSize(min_cluster_size_); // parameter
        ec.setMaxClusterSize(max_cluster_size_); // parameter
        ec.setSearchMethod(kd_tree_); // use kd tree, part of PCL
        ec.setInputCloud(filtered_cloud_); // use filtered points
        ec.extract(cluster_indices); // run algorithm (from PCL)

        for (const auto& cluster : cluster_indices) { // Loop through each cluster
            
            // Create obstacle with initialized center height and number of points
            Obstacle obs;
            obs.center_xy = Eigen::Vector2f(0.0f, 0.0f);
            obs.height = -std::numeric_limits<float>::infinity();
            obs.num_points = static_cast<int>(cluster.indices.size());

            for (int idx : cluster.indices) { // loop through all points in cluster
                const auto& p = filtered_cloud_->points[idx]; // get the actual x, y, z point from the cloud

                obs.center_xy.x() += p.x; // sum to compute centroid
                obs.center_xy.y() += p.y;

                if (p.z > obs.height) { // get height bc this will help us understand if clusters are noise or real objects
                    obs.height = p.z;
                }
            }

            obs.center_xy /= static_cast<float>(obs.num_points); // get centroid

            // 2D covariance in xy plane
            Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();

            for (int idx : cluster.indices) {
                const auto& p = filtered_cloud_->points[idx];

                Eigen::Vector2f d;
                d << p.x - obs.center_xy.x(),
                    p.y - obs.center_xy.y();

                cov += d * d.transpose();
            }

            cov /= static_cast<float>(obs.num_points);

            // Add minimum variance to obstacles so it doesn't collapse if its line shaped
            float min_sigma = 0.25f; // meters
            cov += Eigen::Matrix2f::Identity() * min_sigma * min_sigma;

            obs.covariance = cov;
            obs.inv_covariance = cov.inverse();

            // Height adds 10% cost per meter of height
            obs.amplitude = base_amplitude_ * (1.0f + 0.10f * obs.height);

            obstacles_.push_back(obs);
        }
    }

    // Gaussian cost model - needs work

    void computeGaussianCosts() // this in theory should be fine
    {
        for (auto& obs : obstacles_) {

            // Height should increase cost a bit since it strongly correlates with larger obstacles (10%)
            obs.amplitude = base_amplitude_ * (1.0f + 0.10f * obs.height); // In theory I don't think this changes the model based on
            // current equations in next function, but it might
        }
    }

    float gaussianCostAtPoint(float x, float y) const // find cost for a given point, this onward needs work
    {
        float total_cost = 0.0f; // initialize

        Eigen::Vector2f p;
        p << x, y;

        for (const auto& obs : obstacles_) { // for each obstacle
            Eigen::Vector2f d = p - obs.center_xy;
            
            float exponent = -0.5f * d.transpose() * obs.inv_covariance * d; // mahalanobis distance

            float cost = obs.amplitude * std::exp(exponent); // amplitude factor

            total_cost += cost; // add up cost for point from each obstalce
        }

        return total_cost;
    }

    float pathCostStraightLine(float heading_rad, float path_length) const
    {
        // Simple candidate path evaluator
        // Assumption - rover starts at origin and tests straight-line paths

        int num_samples = 50;
        float total_cost = 0.0f;

        for (int i = 1; i <= num_samples; i++) {
            float s =
                path_length * static_cast<float>(i) /
                static_cast<float>(num_samples);

            float x = s * std::cos(heading_rad);
            float y = s * std::sin(heading_rad);

            total_cost += gaussianCostAtPoint(x, y);
        }

        return total_cost;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv); // initialize ros2

    auto node = std::make_shared<LidarObstacleProcessor>(); // creates node object

    rclcpp::spin(node); // keeps it alive

    rclcpp::shutdown(); // shuts down when program ends
    return 0;
}