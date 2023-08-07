#define NOMINMAX
#include <time.h>
#include <iostream>
#include <set>
#include "open3d/Open3D.h"
#include "chopping.h"

#define NB_NEIGHBOURS 10
#define STD_RATIO 0.001

#define NB_NEIGHBOURS_2 3
#define STD_RATIO_2 0.01

#define CLUSTERIZATION_EPS 2.41
#define CLUSTERIZAION_MIN_POINTS 5
#define CLUSTER_DELETE_THRESHOLD 100

bool terminateThread = false;
char recvbuf[1024];
std::mutex mtx;
bool first_run = true;

int main() {

     // Create a point cloud
    auto cloud = std::make_shared<open3d::geometry::PointCloud>();

    if (open3d::io::ReadPointCloud("point_cloud_bottom.ply", *cloud)) {
        std::cout << "Successfully read the point cloud file." << std::endl;
    } else {
        std::cout << "Failed to read the point cloud file." << std::endl;
        return 1;
    }

    // Visualize the point cloud
    // open3d::visualization::DrawGeometries({cloud}, "Point Cloud Visualization");

    //Adaptive Chopping
    auto filtered_cloud = std::make_shared<open3d::geometry::PointCloud>();
    filtered_cloud = adaptive_chopping(cloud, first_run);


    std::cout << "Final display" << std::endl;

    open3d::visualization::DrawGeometries({filtered_cloud}, "Point Cloud Visualization");

    return 0;
}