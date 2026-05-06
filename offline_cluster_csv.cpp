#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <Eigen/Dense>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <stdexcept>

#include <fstream>

struct Cluster {
    Eigen::Vector3f centroid;
    Eigen::Matrix2f covariance_xy;
    Eigen::Vector2f principal_axis;
    Eigen::Vector2f secondary_axis;
    float principal_variance;
    float secondary_variance;
    int num_points;
};

pcl::PointCloud<pcl::PointXYZ>::Ptr loadCsvPointCloud(const std::string& filename)
{
    auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open CSV file: " + filename);
    }

    std::string line;
    std::getline(file, line); // skip header

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string xs, ys, zs;

        std::getline(ss, xs, ',');
        std::getline(ss, ys, ',');
        std::getline(ss, zs, ',');

        pcl::PointXYZ p;
        p.x = std::stof(xs);
        p.y = std::stof(ys);
        p.z = std::stof(zs);

        if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
            cloud->points.push_back(p);
        }
    }

    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = false;

    return cloud;
}

std::vector<Cluster> clusterPointCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    float cluster_tolerance,
    int min_cluster_size,
    int max_cluster_size)
{
    std::vector<Cluster> clusters;

    if (cloud->empty()) {
        return clusters;
    }

    auto kd_tree = pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>);
    kd_tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(kd_tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (const auto& indices : cluster_indices) {
        Cluster cluster;
        cluster.centroid = Eigen::Vector3f::Zero();
        cluster.num_points = static_cast<int>(indices.indices.size());

        for (int idx : indices.indices) {
            const auto& p = cloud->points[idx];
            cluster.centroid += Eigen::Vector3f(p.x, p.y, p.z);
        }

        cluster.centroid /= static_cast<float>(cluster.num_points);

        Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();

        for (int idx : indices.indices) {
            const auto& p = cloud->points[idx];

            Eigen::Vector2f d;
            d << p.x - cluster.centroid.x(),
                 p.y - cluster.centroid.y();

            cov += d * d.transpose();
        }

        cov /= static_cast<float>(cluster.num_points);

        cluster.covariance_xy = cov;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(cov);

        Eigen::Vector2f eigenvalues = solver.eigenvalues();
        Eigen::Matrix2f eigenvectors = solver.eigenvectors();

        cluster.secondary_variance = eigenvalues(0);
        cluster.principal_variance = eigenvalues(1);

        cluster.secondary_axis = eigenvectors.col(0);
        cluster.principal_axis = eigenvectors.col(1);

        clusters.push_back(cluster);
    }

    return clusters;
}

int main()
{
    std::string filename = "../cloud_0000.csv";

    float cluster_tolerance = 0.10f;
    int min_cluster_size = 5;
    int max_cluster_size = 500000;

    try {
        auto cloud = loadCsvPointCloud(filename);

        std::cout << "Loaded " << cloud->size() << " points\n";

        auto clusters = clusterPointCloud(
            cloud,
            cluster_tolerance,
            min_cluster_size,
            max_cluster_size
        );

        std::ofstream cluster_file("clusters.csv");

        cluster_file << "center_x,center_y,principal_variance,secondary_variance,"
                        "principal_axis_x,principal_axis_y\n";

        for (const auto& c : clusters) {

            cluster_file
                << c.centroid.x() << ","
                << c.centroid.y() << ","
                << c.principal_variance << ","
                << c.secondary_variance << ","
                << c.principal_axis.x() << ","
                << c.principal_axis.y() << "\n";
        }

        cluster_file.close();

        std::cout << "Found " << clusters.size() << " clusters\n\n";

        for (size_t i = 0; i < clusters.size(); i++) {
            const auto& c = clusters[i];

            std::cout << "Cluster " << i << "\n";
            std::cout << "  points: " << c.num_points << "\n";
            std::cout << "  centroid xyz: "
                      << c.centroid.x() << ", "
                      << c.centroid.y() << ", "
                      << c.centroid.z() << "\n";

            std::cout << "  covariance xy:\n"
                      << c.covariance_xy << "\n";

            std::cout << "  principal variance: " << c.principal_variance << "\n";
            std::cout << "  secondary variance: " << c.secondary_variance << "\n";

            std::cout << "  principal axis: "
                      << c.principal_axis.x() << ", "
                      << c.principal_axis.y() << "\n";

            std::cout << "  secondary axis: "
                      << c.secondary_axis.x() << ", "
                      << c.secondary_axis.y() << "\n\n";
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}