//
// Created by PC on 8/3/2023.
//
// clusterization

#ifndef SPRINGS_CLUSTERIZATION_H
#define SPRINGS_CLUSTERIZATION_H

#include "open3d/Open3D.h"
#include <vector>

std::vector<std::shared_ptr<open3d::geometry::PointCloud>> clusterization(std::shared_ptr<open3d::geometry::PointCloud>);
std::vector<std::shared_ptr<open3d::geometry::PointCloud>> clusterization_(std::shared_ptr<open3d::geometry::PointCloud>, double, double, double, double);
std::vector<std::shared_ptr<open3d::geometry::PointCloud>> clusterization_ang(std::shared_ptr<open3d::geometry::PointCloud>);

#endif //SPRINGS_CLUSTERIZATION_H
