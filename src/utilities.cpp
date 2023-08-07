//
// Created by PC on 8/3/2023.
//
#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>
#include "utilities.h"

void printMessage(const char *message)
{
    std::cout << message << std::endl;
}

Eigen::Matrix4d getTransformationMatrix(float angle)
{
    float theta = angle * M_PI / 180.0;

    Eigen::Matrix4d transformation;
    transformation << cos(angle), -sin(angle), 0, 0,
                      sin(angle), cos(angle), 0, 0,
                      0, 0, 1, 0,
                      0, 0, 0, 1;

    return transformation;
}

std::shared_ptr<open3d::geometry::LineSet> getBoxCloud(open3d::geometry::AxisAlignedBoundingBox aabb)
{
    auto box_points = aabb.GetBoxPoints();
    auto lines = std::vector<Eigen::Vector2i>{
        {0, 1}, {0, 2}, {1, 6}, {6, 4}, {6, 3}, {4, 5}, {3, 5}, {1, 7}, {7, 2}, {0, 3}, {5, 2}, {7, 4}};

    auto box = std::make_shared<open3d::geometry::LineSet>();
    box->points_ = box_points;
    box->lines_ = lines;
    return box;
}

std::shared_ptr<open3d::geometry::LineSet> getBoxCloud(open3d::geometry::OrientedBoundingBox obb)
{
    auto box_points = obb.GetBoxPoints();
    auto lines = std::vector<Eigen::Vector2i>{
        {0, 1}, {0, 2}, {1, 6}, {6, 4}, {6, 3}, {4, 5}, {3, 5}, {1, 7}, {7, 2}, {0, 3}, {5, 2}, {7, 4}};

    auto box = std::make_shared<open3d::geometry::LineSet>();
    box->points_ = box_points;
    box->lines_ = lines;
    return box;
}

open3d::geometry::AxisAlignedBoundingBox getMinMaxBounds(const open3d::geometry::OrientedBoundingBox& obb) {
    // Get the corners of the OBB
    std::vector<Eigen::Vector3d> obb_corners = obb.GetBoxPoints();

    // Initialize the min_bound and max_bound with the first corner
    Eigen::Vector3d min_bound = obb_corners[0];
    Eigen::Vector3d max_bound = obb_corners[0];

    // Iterate over each corner, update the min_bound and max_bound
    for (const auto &corner : obb_corners) {
        min_bound = min_bound.cwiseMin(corner);
        max_bound = max_bound.cwiseMax(corner);
    }

    open3d::geometry::AxisAlignedBoundingBox aabb(min_bound, max_bound);

    return aabb;
}