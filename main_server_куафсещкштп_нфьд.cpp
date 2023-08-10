// #define NOMINMAX
// #include "server.h"

// #include <iostream>
// #include "open3d/Open3D.h"
// #include "utilities.h"
// #include "c_interface.h"
// #include <chrono>
// #include <ctime> 
// #include "server.h"
// #include <thread>

// // #pragma comment(lib, "ws2_32.lib")
// bool terminateThread = false;
// char recvbuf[1024];
// std::mutex mtx;
// using namespace std::chrono;
// using namespace std;
// #define NB_NEIGHBOURS 6
// #define STD_RATIO 0.000001

// #define NB_NEIGHBOURS_2 3
// #define STD_RATIO_2 0.01

// #define CLUSTERIZATION_EPS 2.41
// #define CLUSTERIZAION_MIN_POINTS 5
// #define CLUSTER_DELETE_THRESHOLD 100
// int main() {
//     std::thread serverThread(server);
//     serverThread.detach();
//     std::string recvd_str;
//     bool position_now = false; //variable for keeping firection of nex scan
//     int station_server = 0;
//     bool station = false;
//     auto cloud = std::make_shared<open3d::geometry::PointCloud>();
//     station = lib_init();
//     station = do_homing();
//     while(!station){
//         station = do_homing();
//     }
//     station = false;
//     while (true)
//     {
//         Sleep(20);
//         mtx.lock();
//         recvd_str = recvbuf;
//         mtx.unlock();
//         std::cout <<"Type operation: "<<recvd_str << "\n";
//         if (recvd_str[2] == 4)
//             cout<<"change_station, remove error"<<"/n";
//             station_server = 4;
        
        
        
//         if (recvd_str[2] == '2' && station_server != 7 && station_server != 8)
//         {
//             if (station_server == 7)
//                 continue;
//             station_server = 3;
//             while (!station){
//                 if(!position_now){
//                     station = scan(1.0, 1.0, 0.0, 3400.0, 3400.0);
//                 }else{
//                     station = scan(1.0, 1.0, 3400.0, 0.0, 0.0);
//                 }    
//             }
//             station = !station;
//             station_server = 9;
            
//             // write typeoperation = 0, recvd_str[2] = 0

//             PointCloud PT = get_point_cloud();
//             for (int i = 0; i < PT.size; ++i) {
//                     cloud->points_.push_back(Eigen::Vector3d(PT.data[i].x, PT.data[i].y, PT.data[i].z));
//             }

//             auto semi_filtered_cloud = std::get<0>(cloud->RemoveStatisticalOutliers(NB_NEIGHBOURS, STD_RATIO, false));
//             auto filtered_cloud = std::get<0>(semi_filtered_cloud->RemoveStatisticalOutliers(NB_NEIGHBOURS, STD_RATIO, false));
//             open3d::visualization::DrawGeometries({filtered_cloud}, "Point Cloud Visualization");

//             terminateThread = true;
//             break;
//         }
//     }



//     // printMessage("Server_test");
   
//     return 0;
// }

#define NOMINMAX
#include "server.h"

#include <iostream>
#include "open3d/Open3D.h"
#include "utilities.h"
#include "c_interface.h"
#include <chrono>
#include "chopping.h"
#include "clusterization.h"
#include "spring_recognition.h"
#include <ctime> 
#include "server.h"
#include <typeinfo>
#include <thread>
#include "yaml-cpp/yaml.h"
// #include "filtration.h"

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

Eigen::Matrix4d transformation;

int main() {
    std::thread serverThread(server);
    serverThread.detach();
    std::string recvd_str;
    bool position_now = false; //variable for keeping firection of nex scan
    int station_server = 0;
    bool station = false;
    bool chopped = false;
    bool first_run = true;
    bool ang_first_run = true;
    open3d::geometry::OrientedBoundingBox obb2 = open3d::geometry::OrientedBoundingBox(Eigen::Vector3d(), Eigen::Matrix3d(), Eigen::Vector3d());
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
            auto t1 = high_resolution_clock::now();
            // YAML::Node config = YAML::LoadFile("springs_params_.yaml");




            auto cloud = std::make_shared<open3d::geometry::PointCloud>();
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
            if(station)
                position_now = !position_now;
            
            auto t2 = high_resolution_clock::now();
            auto duration = duration_cast<milliseconds>(t2 - t1);
            std::cout << "Scanning time t2-t1 " << duration.count() << "\n";
            station = !station;
            station_server = 9;
            recvbuf[2] = '0';
            // write typeoperation = 0, recvd_str[2] = 0
            auto t3 = high_resolution_clock::now();
            PointCloud PT = get_point_cloud();
            for (int i = 0; i < PT.size; ++i) {
                    cloud->points_.push_back(Eigen::Vector3d(PT.data[i].x, PT.data[i].y, PT.data[i].z));
            }
            // if(!chopped){
            //     auto t5 = high_resolution_clock::now();
            //     std::cout << "IMHERE"<< "\n";
            //     obb2 = adaptive_chopping(cloud);
            //     std::cout << typeid(obb2).name()<< "\n"; 
            //     chopped = true;
            //     auto t6 = high_resolution_clock::now();
            //     auto duration = duration_cast<milliseconds>(t6 - t5);
            //     std::cout << "Choping time t2-t1 " << duration.count() << "\n";
            // }
            auto t31 = high_resolution_clock::now();
            std::cout << "IMHERE"<< "\n";
            //TODO: adaptive choping depending on the number of clusters 
            auto chopped_point_cloud = adaptive_chopping(cloud, first_run);
            first_run = false;


            auto t4 = high_resolution_clock::now();
            duration = duration_cast<milliseconds>(t4 - t31);
            std::cout << "Chopping t2-t1 " << duration.count() << "\n";
            auto semi_filtered_cloud = std::get<0>(chopped_point_cloud->RemoveStatisticalOutliers(NB_NEIGHBOURS, STD_RATIO, false));
            auto filtered_cloud = std::get<0>(semi_filtered_cloud->RemoveStatisticalOutliers(NB_NEIGHBOURS, STD_RATIO, false));
            
            auto t5 = high_resolution_clock::now();
            duration = duration_cast<milliseconds>(t5 - t4);
            std::cout << "Filtration time t2-t1 " << duration.count() << "\n";
            
            // auto chopped_point_cloud = filtered_cloud->Crop(obb2);

            auto t6 = high_resolution_clock::now();
            duration = duration_cast<milliseconds>(t6 - t5);
            std::cout << "Cropping time t2-t1 " << duration.count() << "\n";
            // open3d::visualization::DrawGeometries({chopped_point_cloud}, "Point Cloud Visualization");
            auto clusters = std::vector<std::shared_ptr<open3d::geometry::PointCloud>>();
            clusters = clusterization(filtered_cloud);
            std::cout << clusters.size() << duration.count() << "\n";
            auto t7 = high_resolution_clock::now();
            duration = duration_cast<milliseconds>(t7 - t6);
            std::cout << "Clusterization time t2-t1 " << duration.count() << "\n";

            if (clusters.size() == 0) {
                std::cout << "Cluster size = " << clusters.size() << std::endl;
                auto filtered_cloud1 = std::get<0>(cloud->RemoveRadiusOutliers(20, 2.8, false));
                //auto filtered_cloud1 = filtration_ang(cloud);
                //TODO: adaptive choping depending on the number of clusters 
                auto chopped_point_cloud1 = adaptive_chopping(filtered_cloud1, ang_first_run);
                auto clusters1 = clusterization(chopped_point_cloud1);

                if(clusters1.size() == 0){
                    cout<<"Maybe frame is empty"<<endl;
                    double mean = 0;
                    for(int i = 0; i < chopped_point_cloud1->points_(); i++){
                        mean += chopped_point_cloud1.points_()[2];
                    }
                    mean = mean / chopped_point_cloud1->points_.size();
                    auto cloud_frame = std::make_shared<open3d::geometry::PointCloud>();

                    for(int i = 0; i < chopped_point_cloud1.points_(); i++){
                        cloud_frame->points_.push_back(Eigen::Vector3d(cloud_frame->points_[i][0], cloud_frame->points_[i][1], cloud_frame->points_[i][2]);
                    }
                    

                }
            }

            std::cout << "Springs: " << clusters.size() << std::endl;

            // std::vector<std::shared_ptr<const open3d::geometry::Geometry>> v();
            int springs = 1;
            for (auto cluster : clusters) {
                std::cout << "Spring " << springs++ << std::endl;
                auto tf = getTransformation(cluster);
                std::cout << "Coordinates:\n" << std::get<0>(tf) << std::endl;
                std::cout << "Rotational matrix:\n" << std::get<1>(tf) << std::endl;
                std::cout << "Euler angles:\n" << std::get<1>(tf).eulerAngles(0,1,2) << std::endl;
                std::cout << std::endl;

                // v.pushBack(cluster);
            }

            // open3d::visualization::DrawGeometries(v, "Point Cloud Visualization");

            std::vector<std::shared_ptr<const open3d::geometry::Geometry>> v;
            for (auto p : clusters) {v.push_back(p);} 
            const auto cv = v;
            open3d::visualization::DrawGeometries(cv, "Point Cloud Visualization");
            delete_point_cloud(PT);
            // terminateThread = true;
            // break;
        }
        if (recvd_str[2] == '5')
        {
            terminateThread = true;
            break;
        }
    }


    // printMessage("Server_test");
   
    return 0;
}