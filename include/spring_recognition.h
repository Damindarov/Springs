//
// Created by PC on 8/3/2023.
//
// springs_recognition and springs_recognition_ang

#ifndef SPRINGS_SPRING_RECOGNITION_H
#define SPRINGS_SPRING_RECOGNITION_H

#include <tuple>

#include "open3d/Open3D.h"

std::tuple<Eigen::Vector3d, Eigen::Matrix3d> getTransformation(std::shared_ptr<open3d::geometry::PointCloud>);

#endif //SPRINGS_SPRING_RECOGNITION_H
