#include <iostream>
#include "open3d/Open3D.h"
#include "utilities.h"
#include "c_interface.h"
#include<iostream>
#include<fstream>
#include<string>
#include "yaml-cpp/yaml.h"
using namespace std;
int main() {
    printMessage("Hey its working!!!!....");
    YAML::Node config = YAML::LoadFile("springs_params_.yaml");
    if (config["CORE_PARAMS"]) {
         std::cout << "Last logged in: " <<config["CORE_PARAMS"]<< "\n";
    }
     // Create a point cloud
    auto cloud = std::make_shared<open3d::geometry::PointCloud>();

    // Read the point cloud data from a PLY file
    if (open3d::io::ReadPointCloud("point_cloud.ply", *cloud)) {
        std::cout << "Successfully read the point cloud file." << std::endl;
    } else {
        std::cout << "Failed to read the point cloud file." << std::endl;
        return 1;
    }

    // Visualize the point cloud
    open3d::visualization::DrawGeometries({cloud}, "Point Cloud Visualization");






    return 0;
}