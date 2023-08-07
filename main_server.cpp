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
    bool position_now = false; //variable for keeping firection of nex scan
    int station_server = 0;
    bool station = false;
    auto cloud = std::make_shared<open3d::geometry::PointCloud>();
    station = lib_init();
    station = do_homing();
    while(!station){
        station = do_homing();
    }
    station = false;
    while (true)
    {
        Sleep(20);
        mtx.lock();
        recvd_str = recvbuf;
        mtx.unlock();
        std::cout <<"Type operation: "<<recvd_str << "\n";
        if (recvd_str[2] == 4)
            cout<<"change_station, remove error"<<"/n";
            station_server = 4;
        
        
        
        if (recvd_str[2] == '2' && station_server != 7 && station_server != 8)
        {
            if (station_server == 7)
                continue;
            station_server = 3;
            while (!station){
                if(!position_now){
                    station = scan(1.0, 1.0, 0.0, 3400.0, 3400.0);
                }else{
                    station = scan(1.0, 1.0, 3400.0, 0.0, 0.0);
                }    
            }
            station = !station;
            station_server = 9;
            
            // write typeoperation = 0, recvd_str[2] = 0

            PointCloud PT = get_point_cloud();
            for (int i = 0; i < PT.size; ++i) {
                    cloud->points_.push_back(Eigen::Vector3d(PT.data[i].x, PT.data[i].y, PT.data[i].z));
            }

            auto semi_filtered_cloud = std::get<0>(cloud->RemoveStatisticalOutliers(NB_NEIGHBOURS, STD_RATIO, false));
            auto filtered_cloud = std::get<0>(semi_filtered_cloud->RemoveStatisticalOutliers(NB_NEIGHBOURS, STD_RATIO, false));
            open3d::visualization::DrawGeometries({filtered_cloud}, "Point Cloud Visualization");

            terminateThread = true;
            break;
        }
    }



    // printMessage("Server_test");
   
    return 0;
}