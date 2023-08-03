#include <iostream>
#include "open3d/Open3D.h"

int main() {
    // Create a point cloud
    std::cout << "hellow" << std::endl;
    // open3d::geometry::PointCloud pc;

//    // Load the point cloud data from a file
    auto pc = open3d::io::CreatePointCloudFromFile("filtered_cloud.pcd");

    //open3d::visualization::DrawGeometries({pc});
//    // Create a visualization window
//    open3d::visualization::Visualizer visualizer;
//    visualizer.CreateVisualizerWindow("Open3D Point Cloud", 1920, 1080);

//    // Add the point cloud to the visualization window
//    visualizer.AddGeometry(pc);

//    // Set the view control parameters
//    visualizer.GetRenderOption().point_size_ = 1.0;
// //    visualizer.GetViewControl().SetViewMatrices(open3d::visualization::ViewControl::ConvertJsonStringToViewParameters(
// //            "{\"version\": \"0.0.0\", \"name\": \"Open3D\", \"eye\": [0, 0, 0], \"lookat\": [0, 0, 1], \"up\": [0, -1, 0], \"zoom\": 1.0}"));

//    // Run the visualization loop
//    visualizer.Run();

//    // Close the visualization window
//    visualizer.DestroyVisualizerWindow();

    return 0;
}