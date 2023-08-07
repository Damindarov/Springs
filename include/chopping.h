//
// Created by PC on 8/3/2023.
//
// adaptive_chopping and normal chopping, mean_cutter


#ifndef SPRINGS_CHOPPING_H
#define SPRINGS_CHOPPING_H

#include "open3d/Open3D.h"


std::shared_ptr<open3d::geometry::PointCloud> adaptive_chopping(std::shared_ptr<open3d::geometry::PointCloud>, bool);

#endif //SPRINGS_CHOPPING_H
