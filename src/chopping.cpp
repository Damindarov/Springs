//
// Created by PC on 8/3/2023.
//
#include "open3d/Open3D.h"
#include "utilities.h"
#include <Eigen/Dense>
#include <Eigen/Core>
#define _USE_MATH_DEFINES
#include <math.h>

#define TOP_HEIGHT 400
#define BASE_HEIGHT 1040
#define CUT_WIDTH 100

std::shared_ptr<open3d::geometry::PointCloud> adaptive_chopping(std::shared_ptr<open3d::geometry::PointCloud> cloud)
{

    auto filtered_cloud = std::get<0>(cloud->RemoveStatisticalOutliers(10, 0.0000001, false));

    auto aabb = filtered_cloud->GetAxisAlignedBoundingBox();

    auto box = std::make_shared<open3d::geometry::LineSet>();
    box = getBoxCloud(aabb);

    // std::cout << "Box points" << aabb.GetMinBound() << aabb.GetMaxBound() << std::endl;

    // for(auto point : box_points){
    //     std::cout << point << std::endl;
    // }

    auto bbox = filtered_cloud->GetAxisAlignedBoundingBox();

    Eigen::Vector3d min_bound0 = bbox.GetMinBound() + Eigen::Vector3d(0, 0, 50);
    Eigen::Vector3d max_bound0 = bbox.GetMaxBound() - Eigen::Vector3d(0, 0, 0);

    // Then, construct the AABB using these bounds
    open3d::geometry::AxisAlignedBoundingBox bbox1(min_bound0, max_bound0);

    filtered_cloud = filtered_cloud->Crop(bbox1);

    filtered_cloud = std::get<0>(filtered_cloud->RemoveStatisticalOutliers(10, 0.0001, false));

    Eigen::Matrix4d transformation;

    open3d::visualization::DrawGeometries({filtered_cloud, box}, "filtered cloud before transformation");

    // std::cout << "Transformation matrix for " << degrees << " degrees:\n" << rotation_matrix << std::endl;
    // std::cout << "aabb" << aabb.GetMinBound()[0] << std::endl;

    if (static_cast<double>(std::abs(aabb.GetMinBound()[1])) - static_cast<double>(std::abs(aabb.GetMaxBound()[1])) > 3.0)
    {
        std::cout << "greater" << std::endl;
        std::cout << std::abs(aabb.GetMinBound()[1]) - std::abs(aabb.GetMaxBound()[1]) << std::endl;
        auto transform_pcd = std::make_shared<open3d::geometry::PointCloud>();
        for (float i = 0; i < 10; i += 0.01)
        {
            // std::cout << i << std::endl;
            // std::cout << std::abs(aabb.GetMinBound()[1]) - std::abs(aabb.GetMaxBound()[1]) << std::endl;
            transformation = getTransformationMatrix(i);
            transform_pcd = filtered_cloud;
            transform_pcd->Transform(transformation);
            auto aabb = transform_pcd->GetAxisAlignedBoundingBox();
            std::cout << "Min bound: " << aabb.GetMinBound() << "\n"
                      << "Max bound: " << aabb.GetMaxBound() << std::endl;
            // get the least error in the angle
            if (static_cast<double>(std::abs(aabb.GetMinBound()[1])) - static_cast<double>(std::abs(aabb.GetMaxBound()[1])) > -3.0 && static_cast<double>(std::abs(aabb.GetMinBound()[1])) - static_cast<double>(std::abs(aabb.GetMaxBound()[1])) < 3.0)
            {
                std::cout << std::abs(aabb.GetMinBound()[1]) - std::abs(aabb.GetMaxBound()[1]) << std::endl;
                std::cout << "done" << std::endl;
                transformation = getTransformationMatrix(i / 2);
                break;
            }
        }
    }

    else if (static_cast<double>(std::abs(aabb.GetMinBound()[1])) - static_cast<double>(std::abs(aabb.GetMaxBound()[1])) < 3.0)
    {
        std::cout << "less" << std::endl;
        std::cout << std::abs(aabb.GetMinBound()[1]) - std::abs(aabb.GetMaxBound()[1]) << std::endl;
        auto transform_pcd = std::make_shared<open3d::geometry::PointCloud>();
        for (float i = 0; i > -10; i -= 0.01)
        {
            // std::cout << i << std::endl;
            // std::cout << std::abs(aabb.GetMinBound()[1]) - std::abs(aabb.GetMaxBound()[1]) << std::endl;
            transformation = getTransformationMatrix(i);
            transform_pcd = filtered_cloud;
            transform_pcd->Transform(transformation);
            auto aabb = transform_pcd->GetAxisAlignedBoundingBox();
            // std::cout << "here is the error" << std::endl;
            std::cout << "Min bound: " << aabb.GetMinBound() << "\n" << "Max bound: " << aabb.GetMaxBound() << std::endl;
            // get the least error in the angle
            if (static_cast<double>(std::abs(aabb.GetMinBound()[1])) - static_cast<double>(std::abs(aabb.GetMaxBound()[1])) > -3.0 && static_cast<double>(std::abs(aabb.GetMinBound()[1])) - static_cast<double>(std::abs(aabb.GetMaxBound()[1])) < 3.0)
            {
                std::cout << std::abs(aabb.GetMinBound()[1]) - std::abs(aabb.GetMaxBound()[1]) << std::endl;
                std::cout << "done" << std::endl;
                std::cout << "i = " << i << std::endl;
                transformation = getTransformationMatrix(i/2);
                break;
            }
        }
    }

    // std::cout << "Final Transformation matrix " << rotation_matrix << std::endl;
    Eigen::Matrix3d rotation = transformation.block<3,3>(0,0);
    Eigen::Vector3d center_of_rotation = aabb.GetCenter();
    filtered_cloud->Rotate(rotation, center_of_rotation);
    auto aabb2 = filtered_cloud->GetAxisAlignedBoundingBox();
    // Convert to OrientedBoundingBox
    open3d::geometry::OrientedBoundingBox obb = open3d::geometry::OrientedBoundingBox::CreateFromAxisAlignedBoundingBox(aabb);
    obb.Rotate(rotation, center_of_rotation);
    // obb.Transform(transformation);
    box = getBoxCloud(obb);

    std::cout << "obb center =" << obb.center_ << std::endl;
    
    // aabb = obb.GetAxisAlignedBoundingBox();
    // box = getBoxCloud(aabb);

    // box->Transform(transformation);
    open3d::visualization::DrawGeometries({filtered_cloud, box}, "filtered cloud after transformation");

    float y1 = (obb.GetMinBound()[2] - TOP_HEIGHT) * (CUT_WIDTH / (BASE_HEIGHT - TOP_HEIGHT));
    float y2 = (obb.GetMaxBound()[2] - TOP_HEIGHT) * (CUT_WIDTH / (BASE_HEIGHT - TOP_HEIGHT));

    Eigen::Vector3d min_bound = obb.GetMinBound(); //+ Eigen::Vector3d(0, -350, 0);    //-400
    Eigen::Vector3d max_bound = obb.GetMaxBound(); //- Eigen::Vector3d(0, 600, 0);     //500
    std::cout << "y1, y2 " << y1 << " " << y2 << std::endl;
    std::cout << "obb min" << std::endl << obb.GetMinBound()[1] << std::endl;
    std::cout << "obb max" << std::endl << obb.GetMaxBound()[1] << std::endl;

    // Calculate the center of the box
    Eigen::Vector3d center = (min_bound + max_bound) / 2.0;

    //TODO: Adjust the formula for z change
    min_bound = obb.GetMinBound() + Eigen::Vector3d(0, -350, 0);    //-400 
    max_bound = obb.GetMaxBound() - Eigen::Vector3d(0, 600, 0);     //500

    // Calculate the extent of the box
    Eigen::Vector3d extent = max_bound - min_bound;

    // Create the OrientedBoundingBox
    open3d::geometry::OrientedBoundingBox obb2(obb.center_, obb.R_, extent);

    open3d::geometry::OrientedBoundingBox obb3(center, obb.R_, extent);

    // Then, construct the AABB using these bounds
    open3d::geometry::AxisAlignedBoundingBox aabb3(min_bound, max_bound);
    // open3d::geometry::OrientedBoundingBox obb2()
    cloud->Rotate(obb.R_, obb.center_);
    open3d::visualization::DrawGeometries({cloud, box}, "Point Cloud Visualization");
    auto p2 = std::make_shared<open3d::geometry::PointCloud>();
    auto p3 = std::make_shared<open3d::geometry::PointCloud>();
    p2 = cloud->Crop(obb2);
    p3 = filtered_cloud->Crop(obb2);

    box = getBoxCloud(obb2);

    std::cout << "Transformation matrix" << std::endl << transformation << std::endl;
    open3d::visualization::DrawGeometries({p3, box}, "Point Cloud Visualization");

    return p3;
}
