//
// Created by PC on 8/3/2023.
//
#include <iostream>
#include "open3d/Open3D.h"
#include "utilities.h"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <chrono>
#include <ctime> 
#define _USE_MATH_DEFINES
#include <math.h>
using namespace std::chrono;
using namespace std;
#define TOP_HEIGHT 400.0
#define BASE_HEIGHT 1040.0
#define CUT_WIDTH 100.0


std::shared_ptr<open3d::geometry::PointCloud> adaptive_chopping(std::shared_ptr<open3d::geometry::PointCloud> cloud, bool first_run)
{

    auto t1 = high_resolution_clock::now();
    // std::shared_ptr<open3d::geometry::PointCloud> filtered_cloud;
    //filtration1
    auto filtered_cloud = std::get<0>(cloud->RemoveStatisticalOutliers(10, 0.0000001, false));

    auto box = std::make_shared<open3d::geometry::LineSet>();
    auto box2 = std::make_shared<open3d::geometry::LineSet>();
    open3d::geometry::OrientedBoundingBox obb5;	
    auto bbox = filtered_cloud->GetAxisAlignedBoundingBox();

    //cut the small culsters from top
    Eigen::Vector3d min_bound0 = bbox.GetMinBound() + Eigen::Vector3d(0, 0, 100);
    Eigen::Vector3d max_bound0 = bbox.GetMaxBound() - Eigen::Vector3d(0, 0, 0);

    // Then, construct the AABB using these bounds
    open3d::geometry::AxisAlignedBoundingBox crop_top_box(min_bound0, max_bound0);

    filtered_cloud = filtered_cloud->Crop(crop_top_box);

    //filtration 2
    filtered_cloud = std::get<0>(filtered_cloud->RemoveStatisticalOutliers(10, 0.0001, false));

    //actual bounding box
    obb5 = filtered_cloud->GetMinimalOrientedBoundingBox(true);
    box = getBoxCloud(obb5);

    auto center5 = obb5.center_;
    Eigen::Matrix3d rotation5 = obb5.R_;
    auto extent5 = obb5.extent_;

    // open3d::visualization::DrawGeometries({filtered_cloud, box}, "filtered cloud before transformation");

    Eigen::Vector3d min_bound5 = center5 - extent5 / 2.0;
    Eigen::Vector3d max_bound5 = center5 + extent5 / 2.0;
    double y1 = 0.0;
    double y2 = 0.0;
    double x1 = 0.0;
    double x2 = 0.0;


    if(static_cast<double>(std::abs(min_bound5[0]-max_bound5[0]))> 900.0){
        y1 = (min_bound5[2] - TOP_HEIGHT) * (CUT_WIDTH / (BASE_HEIGHT - TOP_HEIGHT));
        y2 = (max_bound5[2] - TOP_HEIGHT) * (CUT_WIDTH / (BASE_HEIGHT - TOP_HEIGHT));
        x1 = 0.0;
        x2 = 0.0;
    }
    else if (static_cast<double>(std::abs(min_bound5[0]-max_bound5[0]))< 850.0){
        x1 = (min_bound5[2] - TOP_HEIGHT) * (CUT_WIDTH / (BASE_HEIGHT - TOP_HEIGHT));
        x2 = (max_bound5[2] - TOP_HEIGHT) * (CUT_WIDTH / (BASE_HEIGHT - TOP_HEIGHT));;
        y1 = 0.0;
        y2 = 0.0;
    }

    std::cout << "x1, x2 " << x1 << " " << x2 << std::endl;
    std::cout << "y1, y2 " << y1 << " " << y2 << std::endl;

    Eigen::Vector3d min_bound_new = min_bound5 + Eigen::Vector3d(x1, y1, 0);
    Eigen::Vector3d max_bound_new = max_bound5 - Eigen::Vector3d(x2, y2, 0);
    std::cout << "min_bound5, min_bound_new " << min_bound5 << " " << min_bound_new << std::endl;
    std::cout << "max_bound5, max_bound_new " << max_bound5 << " " << max_bound_new << std::endl;


    Eigen::Vector3d center_new = (min_bound_new + max_bound_new) / 2.0;
    Eigen::Vector3d extent_new = max_bound_new - min_bound_new;

    std::cout << "center_new " << center_new << std::endl;
    std::cout << "extent_new " << extent_new << std::endl;

    // open3d::geometry::OrientedBoundingBox obb_new;
    auto obb_new = cloud->GetOrientedBoundingBox();
    obb_new.center_ = center_new;
    obb_new.extent_ = extent_new;
    obb_new.R_ = obb5.R_;
    
    //test
    // obb5.center_ = center_new;
    // obb5.extent_ = extent_new;


    auto chopped_cloud = std::make_shared<open3d::geometry::PointCloud>();
    // open3d::visualization::DrawGeometries({cloud}, "Just cloud");
    // open3d::visualization::DrawGeometries({chopped_cloud}, "Before chopping");
    chopped_cloud = cloud->Crop(obb_new);
    box2 = getBoxCloud(obb_new);
    std::cout << "obb5 R = " << obb5.R_ << std::endl;
    std::cout << "obb_new R = " << obb5.R_ << std::endl;

    // open3d::visualization::DrawGeometries({chopped_cloud,box2,box}, "After chopping");

    auto t2 = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(t2 - t1);
    std::cout << "Choping t2-t1 " << duration.count() << "\n";

    // open3d::visualization::DrawGeometries({chopped_cloud, box, box2}, "Point Cloud Visualization");
    
    return chopped_cloud;
}

std::shared_ptr<open3d::geometry::PointCloud> adaptive_chopping_(std::shared_ptr<open3d::geometry::PointCloud> cloud, bool first_run, double top_height, double base_height, double cut_width)
{

    auto t1 = high_resolution_clock::now();
    // std::shared_ptr<open3d::geometry::PointCloud> filtered_cloud;
    //filtration1
    auto filtered_cloud = std::get<0>(cloud->RemoveStatisticalOutliers(10, 0.0000001, false));

    auto box = std::make_shared<open3d::geometry::LineSet>();
    auto box2 = std::make_shared<open3d::geometry::LineSet>();
    open3d::geometry::OrientedBoundingBox obb5;	
    auto bbox = filtered_cloud->GetAxisAlignedBoundingBox();

    //cut the small culsters from top
    Eigen::Vector3d min_bound0 = bbox.GetMinBound() + Eigen::Vector3d(0, 0, 100);
    Eigen::Vector3d max_bound0 = bbox.GetMaxBound() - Eigen::Vector3d(0, 0, 0);

    // Then, construct the AABB using these bounds
    open3d::geometry::AxisAlignedBoundingBox crop_top_box(min_bound0, max_bound0);

    filtered_cloud = filtered_cloud->Crop(crop_top_box);

    //filtration 2
    filtered_cloud = std::get<0>(filtered_cloud->RemoveStatisticalOutliers(10, 0.0001, false));

    //actual bounding box
    obb5 = filtered_cloud->GetMinimalOrientedBoundingBox(true);
    box = getBoxCloud(obb5);

    auto center5 = obb5.center_;
    Eigen::Matrix3d rotation5 = obb5.R_;
    auto extent5 = obb5.extent_;

    // open3d::visualization::DrawGeometries({filtered_cloud, box}, "filtered cloud before transformation");

    Eigen::Vector3d min_bound5 = center5 - extent5 / 2.0;
    Eigen::Vector3d max_bound5 = center5 + extent5 / 2.0;
    double y1 = 0.0;
    double y2 = 0.0;
    double x1 = 0.0;
    double x2 = 0.0;


    if(static_cast<double>(std::abs(min_bound5[0]-max_bound5[0]))> 900.0){
        y1 = (min_bound5[2] - top_height) * (cut_width / (base_height - top_height));
        y2 = (max_bound5[2] - top_height) * (cut_width / (base_height - top_height));
        x1 = 0.0;
        x2 = 0.0;
    }
    else if (static_cast<double>(std::abs(min_bound5[0]-max_bound5[0]))< 850.0){
        x1 = (min_bound5[2] - top_height) * (cut_width / (base_height - top_height));
        x2 = (max_bound5[2] - top_height) * (cut_width / (base_height - top_height));;
        y1 = 0.0;
        y2 = 0.0;
    }

    std::cout << "x1, x2 " << x1 << " " << x2 << std::endl;
    std::cout << "y1, y2 " << y1 << " " << y2 << std::endl;

    Eigen::Vector3d min_bound_new = min_bound5 + Eigen::Vector3d(x1, y1, 0);
    Eigen::Vector3d max_bound_new = max_bound5 - Eigen::Vector3d(x2, y2, 0);
    std::cout << "min_bound5, min_bound_new " << min_bound5 << " " << min_bound_new << std::endl;
    std::cout << "max_bound5, max_bound_new " << max_bound5 << " " << max_bound_new << std::endl;


    Eigen::Vector3d center_new = (min_bound_new + max_bound_new) / 2.0;
    Eigen::Vector3d extent_new = max_bound_new - min_bound_new;

    std::cout << "center_new " << center_new << std::endl;
    std::cout << "extent_new " << extent_new << std::endl;

    // open3d::geometry::OrientedBoundingBox obb_new;
    auto obb_new = cloud->GetOrientedBoundingBox();
    obb_new.center_ = center_new;
    obb_new.extent_ = extent_new;
    obb_new.R_ = obb5.R_;
    
    //test
    // obb5.center_ = center_new;
    // obb5.extent_ = extent_new;


    auto chopped_cloud = std::make_shared<open3d::geometry::PointCloud>();
    // open3d::visualization::DrawGeometries({cloud}, "Just cloud");
    // open3d::visualization::DrawGeometries({chopped_cloud}, "Before chopping");
    chopped_cloud = cloud->Crop(obb_new);
    box2 = getBoxCloud(obb_new);
    std::cout << "obb5 R = " << obb5.R_ << std::endl;
    std::cout << "obb_new R = " << obb5.R_ << std::endl;

    // open3d::visualization::DrawGeometries({chopped_cloud,box2,box}, "After chopping");

    auto t2 = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(t2 - t1);
    std::cout << "Choping t2-t1 " << duration.count() << "\n";

    // open3d::visualization::DrawGeometries({chopped_cloud, box, box2}, "Point Cloud Visualization");
    
    return chopped_cloud;
}