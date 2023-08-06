//
// Created by PC on 8/3/2023.
//

#include "filtration.h"

#define NB_NEIGHBOURS 6
#define STD_RATIO 0.000001

#define NB_NEIGHBOURS_2 3
#define STD_RATIO_2 0.01

std::shared_ptr<open3d::geometry::PointCloud> filtration(std::shared_ptr<open3d::geometry::PointCloud> cloud) {

    auto semi_filtered_cloud = std::get<0>(cloud->RemoveStatisticalOutliers(NB_NEIGHBOURS, STD_RATIO, false));

    auto filtered_cloud = std::get<0>(semi_filtered_cloud->RemoveStatisticalOutliers(NB_NEIGHBOURS, STD_RATIO, false));

    return filtered_cloud;

}

std::shared_ptr<open3d::geometry::PointCloud> filtration_ang(std::shared_ptr<open3d::geometry::PointCloud> cloud) {

    auto filtered_cloud = std::get<0>(cloud->RemoveRadiusOutliers(20, 2.8, false));

    return filtered_cloud;

}