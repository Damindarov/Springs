//
// Created by PC on 8/3/2023.
//
// is_rotation_matrix, rotation_matrix_to_euler_angles, radians_to_degrees, get_tranformation_matrix

#ifndef SPRINGS_UTILITIES_H
#define SPRINGS_UTILITIES_H
#include <float.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include "open3d/Open3D.h"

void printMessage(const char * message);

Eigen::Matrix4d getTransformationMatrix(float angle);
std::shared_ptr<open3d::geometry::LineSet> getBoxCloud(open3d::geometry::AxisAlignedBoundingBox aabb);
std::shared_ptr<open3d::geometry::LineSet> getBoxCloud(open3d::geometry::OrientedBoundingBox obb);
open3d::geometry::AxisAlignedBoundingBox getMinMaxBounds(const open3d::geometry::OrientedBoundingBox& obb);

#endif //SPRINGS_UTILITIES_H



