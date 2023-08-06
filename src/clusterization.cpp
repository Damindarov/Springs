//
// Created by PC on 8/3/2023.
//

#include "clusterization.h"

#define CLUSTERIZATION_EPS 2.41
#define CLUSTERIZAION_MIN_POINTS 5
#define CLUSTER_DELETE_THRESHOLD 100

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
        if (clusters.at(i)->points_.size() > CLUSTER_DELETE_THRESHOLD) {
            good_clusters.push_back(clusters.at(i));
        }
    }

    return good_clusters;
}