//
// Created by PC on 8/3/2023.
//

#include "spring_recognition.h"

std::tuple<Eigen::Vector3d, Eigen::Matrix3d> getTransformation(std::shared_ptr<open3d::geometry::PointCloud> cluster) {
    auto obb = cluster->GetOrientedBoundingBox();
    auto center = obb.center_;
    auto R = obb.R_;

    return std::make_tuple(center, R);
}