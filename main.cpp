#define NOMINMAX
#include <iostream>
#include "open3d/Open3D.h"
#include "utilities.h"
#include "c_interface.h"
#include <chrono>
#include <ctime> 
// #include "NumCpp.hpp"

int main() {

    printMessage("Hey its working!!!!....");

    // // Example usage of NumCpp
    // nc::NdArray<double> arr1 = {1, 2, 3, 4, 5};
    // nc::NdArray<double> arr2 = {6, 7, 8, 9, 10};
    // nc::NdArray<double> result = nc::multiply(arr1, arr2);

    // std::cout << "Result: " << result << std::endl;
    
    //  // Create a point cloud
    // auto cloud = std::make_shared<open3d::geometry::PointCloud>();
    // bool status;
    // status = lib_init();
    // std::cout<<status;
    // status = do_homing();
    // std::cout<<status;
    // status = scan(1.0, 1.0, 0, 3450.0, 3450.0);
    // PointCloud PT = get_point_cloud();
    // std::cout<<PT.size<<"\n";
    // std::cout<<PT.data->x<<"\n";
    // move_to(0,0.5);
    
    // // Generate 1000 random points
    // auto start = std::chrono::system_clock::now();
    // for (int i = 0; i < PT.size; ++i) {
    //     cloud->points_.push_back(Eigen::Vector3d(PT.data[i].x, PT.data[i].y, PT.data[i].z));
    // }
    // auto end = std::chrono::system_clock::now();
    // std::chrono::duration<double> elapsed_seconds = end-start;
    // std::time_t end_time = std::chrono::system_clock::to_time_t(end);
 
    // std::cout << "finished computation at " << std::ctime(&end_time)
    //           << "elapsed time: " << elapsed_seconds.count() << "s"
    //           << std::endl;
    
    // // Write the point cloud to a PLY file
    // if (open3d::io::WritePointCloud("point_cloud.ply", *cloud)) {
    //     std::cout << "Successfully wrote the point cloud file." << std::endl;
    // } else {
    //     std::cout << "Failed to write the point cloud file." << std::endl;
    //     return 1;
    // }


    // // Create a pointer to store the point cloud
    // // auto cloud = std::make_shared<open3d::geometry::PointCloud>();

    // // Read the point cloud data from a PLY file
    // if (open3d::io::ReadPointCloud("point_cloud.ply", *cloud)) {
    //     std::cout << "Successfully read the point cloud file." << std::endl;
    // } else {
    //     std::cout << "Failed to read the point cloud file." << std::endl;
    //     return 1;
    // }

    // // Visualize the point cloud
    // open3d::visualization::DrawGeometries({cloud}, "Point Cloud Visualization");
    
    return 0;
}