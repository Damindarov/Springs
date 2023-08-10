#include "spring_recognition.h"

std::tuple<Eigen::Vector3d, Eigen::Matrix3d> getTransformation(std::shared_ptr<open3d::geometry::PointCloud> cluster) {
    auto obb = cluster->GetOrientedBoundingBox();
    auto center = obb.center_;
    auto R = obb.R_;

    auto principal_axis = R.col(0);
    int offset = 50;

    cluster->points_.push_back(center + Eigen::Vector3d(0, 0, 5));
    cluster->points_.push_back(center + Eigen::Vector3d(0, 0, 5) + principal_axis * offset);
    cluster->points_.push_back(center + Eigen::Vector3d(0, 0, 5) - principal_axis * offset);

    cluster->colors_.push_back(Eigen::Vector3d(1,0,0));
    cluster->colors_.push_back(Eigen::Vector3d(1,0,0));
    cluster->colors_.push_back(Eigen::Vector3d(1,0,0));


    return std::make_tuple(center, R);
}