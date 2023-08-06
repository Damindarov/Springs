#define NOMINMAX
#include "server.h"

#include <iostream>
#include "open3d/Open3D.h"
#include "utilities.h"
#include "c_interface.h"
#include <chrono>
#include <ctime> 
#include "server.h"
#include <thread>

// #pragma comment(lib, "ws2_32.lib")
bool terminateThread = false;
char recvbuf[1024];
std::mutex mtx;
using namespace std::chrono;
using namespace std;
#define NB_NEIGHBOURS 6
#define STD_RATIO 0.000001

#define NB_NEIGHBOURS_2 3
#define STD_RATIO_2 0.01

#define CLUSTERIZATION_EPS 2.41
#define CLUSTERIZAION_MIN_POINTS 5
#define CLUSTER_DELETE_THRESHOLD 100
int main() {


    std::thread serverThread(server);
    serverThread.detach();
    std::string recvd_str;
    while (true)
    {
        Sleep(20);
        mtx.lock();
        recvd_str = recvbuf;
        mtx.unlock();
        std::cout << recvd_str << "\n";
        if (recvd_str[2] == '5')
        {
            auto cloud = std::make_shared<open3d::geometry::PointCloud>();
            // MPI_Init(NULL, NULL);
            if (open3d::io::ReadPointCloud("point_cloud.ply", *cloud)) {
                std::cout << "Successfully read the point cloud file." << std::endl;
            } else {
                std::cout << "Failed to read the point cloud file." << std::endl;
                return 1;
            }
            auto one = high_resolution_clock::now();
            auto semi_filtered_cloud = std::get<0>(cloud->RemoveStatisticalOutliers(NB_NEIGHBOURS, STD_RATIO, false));

            auto filtered_cloud = std::get<0>(semi_filtered_cloud->RemoveStatisticalOutliers(NB_NEIGHBOURS, STD_RATIO, false));

            auto two = high_resolution_clock::now();

            auto duration = duration_cast<milliseconds>(two - one);
            std::cout << "Filtration: " << duration.count() << "\n";
            terminateThread = true;
            break;
        }
    }



    // printMessage("Server_test");
   
    return 0;
}