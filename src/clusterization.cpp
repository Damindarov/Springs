
#include "clusterization.h"

#define CLUSTERIZATION_EPS 2.41
#define CLUSTERIZAION_MIN_POINTS 5
#define CLUSTER_DELETE_THRESHOLD 25000

std::vector<std::shared_ptr<open3d::geometry::PointCloud>> clusterization(std::shared_ptr<open3d::geometry::PointCloud> cloud) {
    auto labels = cloud->ClusterDBSCAN(CLUSTERIZATION_EPS, CLUSTERIZAION_MIN_POINTS);

    auto unique_labels = std::set<int>(labels.begin(), labels.end());
    int number_of_clusters = unique_labels.size();

    std::vector<std::shared_ptr<open3d::geometry::PointCloud>> clusters;

    for (int i = 0; i < number_of_clusters; i++) {

        double red = (double)std::rand()/(double)RAND_MAX;
        double green = (double)std::rand()/(double)RAND_MAX;
        double blue = (double)std::rand()/(double)RAND_MAX;

        auto color = Eigen::Vector3d(red, green, blue);

        auto cluster = std::make_shared<open3d::geometry::PointCloud>();
        for (int j = 0; j < labels.size(); j ++) {
            if (labels.at(j) == i) {
                cluster->points_.push_back(cloud->points_.at(j));
                cluster->colors_.push_back(color);
            }
        }
        clusters.push_back(cluster);
    }

    auto good_clusters = std::vector<std::shared_ptr<open3d::geometry::PointCloud>>();

    for (int i = 0; i < clusters.size(); i++) {
        auto bb_max_b = clusters.at(i)->GetAxisAlignedBoundingBox().GetMaxBound();
        auto bb_min_b = clusters.at(i)->GetAxisAlignedBoundingBox().GetMinBound();
        auto width = bb_max_b[0] - bb_min_b[0];
        auto height = bb_max_b[1] - bb_min_b[1];
        auto depth = bb_max_b[2] - bb_min_b[2];
        if (clusters.at(i)->points_.size() > CLUSTER_DELETE_THRESHOLD && width > 700) {

            good_clusters.push_back(clusters.at(i));
        }
    }

    return good_clusters;
}

std::vector<std::shared_ptr<open3d::geometry::PointCloud>> clusterization_(std::shared_ptr<open3d::geometry::PointCloud> cloud, double cl_eps, double cl_min_points, double delet_threshold, double nominal_width) {
    auto labels = cloud->ClusterDBSCAN(cl_eps, cl_min_points);

    auto unique_labels = std::set<int>(labels.begin(), labels.end());
    int number_of_clusters = unique_labels.size();

    std::vector<std::shared_ptr<open3d::geometry::PointCloud>> clusters;

    for (int i = 0; i < number_of_clusters; i++) {

        double red = (double)std::rand()/(double)RAND_MAX;
        double green = (double)std::rand()/(double)RAND_MAX;
        double blue = (double)std::rand()/(double)RAND_MAX;

        auto color = Eigen::Vector3d(red, green, blue);

        auto cluster = std::make_shared<open3d::geometry::PointCloud>();
        for (int j = 0; j < labels.size(); j ++) {
            if (labels.at(j) == i) {
                cluster->points_.push_back(cloud->points_.at(j));
                cluster->colors_.push_back(color);
            }
        }
        clusters.push_back(cluster);
    }

    auto good_clusters = std::vector<std::shared_ptr<open3d::geometry::PointCloud>>();

    for (int i = 0; i < clusters.size(); i++) {
        auto bb_max_b = clusters.at(i)->GetAxisAlignedBoundingBox().GetMaxBound();
        auto bb_min_b = clusters.at(i)->GetAxisAlignedBoundingBox().GetMinBound();
        auto width = bb_max_b[0] - bb_min_b[0];
        auto height = bb_max_b[1] - bb_min_b[1];
        auto depth = bb_max_b[2] - bb_min_b[2];
        if (clusters.at(i)->points_.size() > delet_threshold && width > nominal_width) {

            good_clusters.push_back(clusters.at(i));
        }
    }

    return good_clusters;
}