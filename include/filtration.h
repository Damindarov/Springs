//
// Created by PC on 8/3/2023.
//
// data_filtration, selector_data_filtration, data_filtration_ang

#ifndef SPRINGS_FILTRATION_H
#define SPRINGS_FILTRATION_H

#include "open3d/Open3D.h"

std::shared_ptr<open3d::geometry::PointCloud> filtration(std::shared_ptr<open3d::geometry::PointCloud> cloud);

std::shared_ptr<open3d::geometry::PointCloud> filtration_ang(std::shared_ptr<open3d::geometry::PointCloud> cloud);

#endif //SPRINGS_FILTRATION_H
