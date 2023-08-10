#define NOMINMAX
#include "server.h"
#include <iostream>
#include "open3d/Open3D.h"
#include "utilities.h"
#include "c_interface.h"
#include <chrono>
#include <ctime>
#include <thread>
#include "chopping.h"
#include "clusterization.h"
#include "spring_recognition.h"
#include "yaml-cpp/yaml.h"
// #pragma comment(lib, "ws2_32.lib")
bool terminateThread = false;
bool flag = true;

SOCKET ListenSocket = INVALID_SOCKET;
SOCKET ClientSocket = INVALID_SOCKET;
int iSendResult = 0;
char recvbuf[1024];
int rcvd = 0;

std::mutex mtx;
using namespace std::chrono;
using namespace std;
#define NB_NEIGHBOURS 6
#define STD_RATIO 0.000001

#define NB_NEIGHBOURS_2 3
#define STD_RATIO_2 0.0001

#define CLUSTERIZATION_EPS 2.41
#define CLUSTERIZAION_MIN_POINTS 5
#define CLUSTER_DELETE_THRESHOLD 100

struct msg
{
    int station_server;
    int command;
    std::string spring_type;
    double x;
    double y;
    double z;
    double phi;
    double theta;
    double psi;
};
msg mymsg;
int command = mymsg.command;
int station_server = 0;
int main()
{
    std::thread serverThread(server);
    serverThread.detach();
    std::string recvd_str;
    bool position_now = false; // variable for keeping firection of nex scan
    // int station_server = 0;
    bool station = false;
    bool first_run = true;

    station = lib_init();
    station_server = 1;
    station = do_homing();
    while (!station){
        station = do_homing();
    }
    station_server = 2;
    station = false;

    while (true)
    {
        cout<<"external comand"<<mymsg.command<<endl;

        // Sleep(5000);
        // mtx.lock();
        // recvd_str = recvbuf;
        // mtx.unlock();
        // std::cout <<"Type operation: "<<recvd_str << "\n";

        auto chopped_cloud = std::make_shared<open3d::geometry::PointCloud>();
        auto chopped_cloud_prime = std::make_shared<open3d::geometry::PointCloud>();
        auto semi_filtered_cloud = std::make_shared<open3d::geometry::PointCloud>();
        auto filtered_cloud = std::make_shared<open3d::geometry::PointCloud>();
        auto clusters = std::vector<std::shared_ptr<open3d::geometry::PointCloud>>();
        auto cloud = std::make_shared<open3d::geometry::PointCloud>();
        auto type_spring = mymsg.spring_type; //#TODO change it to variable from server
        if (mymsg.command == 4){
            station_server = 4;
        }
        if (station_server == 7 || station_server == 8){
            cout << "change_station, remove error"<<endl;
        }
        // cout << "AAAAAA"
        //      << " " << mymsg.command << endl;
        if (mymsg.command == 2 && station_server != 7 && station_server != 8)
        {
            if (station_server == 7)
            {
                continue;
            }
            station_server = 3;
            mymsg.command = 0;
            YAML::Node config = YAML::LoadFile("springs_params_.yaml");
            // if (config["CORE_PARAMS"]) {
            // auto tr = config["CORE_PARAMS"];
            // std::cout << "Last logged in: " << config["CORE_PARAMS"]["RIGHT_Y"] << "\n";
            // }
            // std::cout << "Last logged in: " << std::endl;
            // mymsg.command = 56;
            // iSendResult = send(ClientSocket, (char *)&mymsg, sizeof(msg), 0);
            // if (iSendResult == SOCKET_ERROR)
            // {
            //     printf("send failed with error: %d\n", WSAGetLastError());
            //     closesocket(ClientSocket);
            //     WSACleanup();
            // }
            // break;
            while (!station)
            {
                if (!position_now)
                {
                    station = scan(1.0, 1.0, 0.0, 3400.0, 3400.0);
                }
                else
                {
                    station = scan(1.0, 1.0, 3400.0, 0.0, 0.0);
                }
            }
            if (station)
                position_now = !position_now;

            station = !station;
            station_server = 9;
            command = 0;
            // write typeoperation = 0, recvd_str[2] = 0
            auto t1 = high_resolution_clock::now();
            PointCloud PT;
            PT = get_point_cloud();
            for (int i = 0; i < PT.size; ++i)
            {
                cloud->points_.push_back(Eigen::Vector3d(PT.data[i].x, PT.data[i].y, PT.data[i].z));
            }

            // if (open3d::io::WritePointCloud("point_cloud.ply", *cloud)) {
            //     std::cout << "Successfully wrote the point cloud file." << std::endl;
            // } else {
            //     std::cout << "Failed to write the point cloud file." << std::endl;
            //     return 1;
            // }

            chopped_cloud = adaptive_chopping_(cloud, first_run, config["CHOPPING_PARAMS"]["TOP_HEIGHT"].as<double>(), config["CHOPPING_PARAMS"]["BASE_HEIGHT"].as<double>(),config["CHOPPING_PARAMS"]["CUT_WIDTH"].as<double>());
            open3d::visualization::DrawGeometries({chopped_cloud}, "Chopped Visualization");
            for (int i = 0; i < chopped_cloud->points_.size(); ++i)
            {
                chopped_cloud_prime->points_.push_back(Eigen::Vector3d(-chopped_cloud->points_[i][0] + config["CALIBRATION_PARAMS"]["DX"].as<double>(), chopped_cloud->points_[i][1] + config["CALIBRATION_PARAMS"]["DY"].as<double>(), -chopped_cloud->points_[i][2] + config["CALIBRATION_PARAMS"]["DZ"].as<double>()));
                // chopped_cloud_prime->points_.push_back(Eigen::Vector3d(chopped_cloud->points_[i][0], chopped_cloud->points_[i][1], chopped_cloud->points_[i][2]));
            }
            semi_filtered_cloud = std::get<0>(chopped_cloud_prime->RemoveStatisticalOutliers(config[type_spring]["NB_NEIGHBOURS"].as<double>(), config[type_spring]["STD_RATIO"].as<double>(), false));
            filtered_cloud = std::get<0>(semi_filtered_cloud->RemoveStatisticalOutliers(config[type_spring]["NB_NEIGHBOURS_2"].as<double>(), config[type_spring]["STD_RATIO_2"].as<double>(), false));
            open3d::visualization::DrawGeometries({filtered_cloud}, "Point Cloud Visualization");
            clusters = clusterization_(filtered_cloud,config[type_spring]["CLUSTERIZATION_EPS"].as<double>(),config[type_spring]["CLUSTERIZAION_MIN_POINTS"].as<double>(),config[type_spring]["CLUSTER_DELETE_THRESHOLD"].as<double>(),config[type_spring]["LENGTH_LIM"].as<double>());
            // open3d::visualization::DrawGeometries({clusters[0]}, "Point Cloud Visualization");
            auto cloud11 = std::make_shared<open3d::geometry::PointCloud>();
            for (auto cluster : clusters) {
                for (int j = 0; j < cluster->points_.size(); j++) {
                    cloud11->points_.push_back(Eigen::Vector3d(-cluster->points_[j][0], cluster->points_[j][1], -cluster->points_[j][2]));
                }
            }

            int springs = 1;
            for (auto cluster : clusters)
                {
                    
                    std::cout << "Spring " << springs++ << std::endl;
                    auto tf = getTransformation(cluster);
                    std::cout << "Coordinates:\n"
                              << std::get<0>(tf) << std::endl;
                    std::cout << "Rotational matrix:\n"
                              << std::get<1>(tf) << std::endl;
                    std::cout << "Euler angles:\n"
                              << std::get<1>(tf).eulerAngles(0, 1, 2) << std::endl;
                    std::cout << std::endl;
                }
            if (clusters.size() != 0){
                auto tf = getTransformation(clusters[0]);
                mymsg.x = std::get<0>(tf)[0];
                mymsg.y = std::get<0>(tf)[0];
                mymsg.z = std::get<0>(tf)[0];
                mymsg.phi = std::get<1>(tf).eulerAngles(0,1,2)[0];
                mymsg.theta = std::get<1>(tf).eulerAngles(0,1,2)[1];
                mymsg.psi = std::get<1>(tf).eulerAngles(0,1,2)[2];
                station_server = 5;
            }

            open3d::visualization::DrawGeometries({cloud11}, "Point Cloud Visualization");
            if (clusters.size() == 0)
            {
                cout << "Try to find angles in frame" << endl;
                auto semi_filtered_cloud1 = std::get<0>(chopped_cloud_prime->RemoveStatisticalOutliers(config[type_spring]["NB_NEIGHBOURS"].as<double>(), config[type_spring]["STD_RATIO"].as<double>(), false));
                open3d::visualization::DrawGeometries({semi_filtered_cloud}, "1_level filter");
                semi_filtered_cloud = std::get<0>(semi_filtered_cloud1->RemoveStatisticalOutliers(config[type_spring]["NB_NEIGHBOURS_ANG"].as<double>(), config[type_spring]["STD_RATIO_ANG"].as<double>(), false));
                open3d::visualization::DrawGeometries({semi_filtered_cloud}, "Point Cloud Visualization");
                clusters = clusterization_(semi_filtered_cloud,config[type_spring]["CLUSTERIZATION_EPS"].as<double>(),config[type_spring]["CLUSTERIZAION_MIN_POINTS"].as<double>(),config[type_spring]["CLUSTER_DELETE_THRESHOLD"].as<double>(),config[type_spring]["LENGTH_LIM"].as<double>());


                if (clusters.size() != 0){
                    auto tf = getTransformation(clusters[0]);
                    mymsg.x = std::get<0>(tf)[0];
                    mymsg.y = std::get<0>(tf)[0];
                    mymsg.z = std::get<0>(tf)[0];
                    mymsg.phi = std::get<1>(tf).eulerAngles(0,1,2)[0];
                    mymsg.theta = std::get<1>(tf).eulerAngles(0,1,2)[1];
                    mymsg.psi = std::get<1>(tf).eulerAngles(0,1,2)[2];
                    station_server = 5;
                }


                // int springs = 1;
                // cout<<"CLUSTERSIZE"<<clusters.size()<<endl;
                // for (auto cluster : clusters)
                // {
                    
                //     std::cout << "Spring " << springs++ << std::endl;
                //     auto tf = getTransformation(cluster);
                //     std::cout << "Coordinates:\n"
                //               << std::get<0>(tf) << std::endl;
                //     std::cout << "Rotational matrix:\n"
                //               << std::get<1>(tf) << std::endl;
                //     std::cout << "Euler angles:\n"
                //               << std::get<1>(tf).eulerAngles(0, 1, 2) << std::endl;
                //     std::cout << std::endl;
                    // flag = true;
                    // if (rcvd > 0 && flag)
                    // {
                    //     mymsg.command = 5;
                    //     mymsg.x = 0;     // get<0>(tf);
                    //     mymsg.y = 0;     // get<0>(tf)];
                    //     mymsg.z = 0;     // get<0>(tf)[2];
                    //     mymsg.phi = 0;   // get<1>(tf).eulerAngles(0);
                    //     mymsg.theta = 0; // get<1>(tf).eulerAngles(1);
                    //     mymsg.psi = 0;   // get<1>(tf).eulerAngles(2);

                    //     iSendResult = send(ClientSocket, (char *)&mymsg, sizeof(msg), 0);
                    //     if (iSendResult == SOCKET_ERROR)
                    //     {
                    //         printf("send failed with error: %d\n", WSAGetLastError());
                    //         closesocket(ClientSocket);
                    //         WSACleanup();
                    //     }
                    //     printf("Bytes sent: %d\n", iSendResult);
                    //     // mtx.unlock();
                    // }
                    // flag = false;

                    // v.pushBack(cluster);
                // }
                std::vector<std::shared_ptr<const open3d::geometry::Geometry>> v;
                for (auto p : clusters)
                {
                    v.push_back(p);
                }
                const auto cv = v;
                open3d::visualization::DrawGeometries(cv, "ANGLES");

                if (clusters.size() == 0)
                {
                    cout << "Frame can be empty" << endl;
                    // filtered_cloud
                    double mean = 0;
                    auto cloud_frame_mean_filter = std::make_shared<open3d::geometry::PointCloud>();

                    open3d::visualization::DrawGeometries({semi_filtered_cloud1}, "semi_filtered_cloud1");

                    for (int i = 0; i < size(semi_filtered_cloud1->points_); i++)
                    {
                        mean = mean + semi_filtered_cloud1->points_[i][2];
                        // cout<<semi_filtered_cloud1->points_[i][2]<<endl;
                    }
                    mean = mean / size(semi_filtered_cloud1->points_) + config["MEANING_OFFSET"].as<double>();
                    for (int i = 0; i < size(semi_filtered_cloud1->points_); i++)
                    {
                        if (semi_filtered_cloud1->points_[i][2] > mean)
                        {
                            cloud_frame_mean_filter->points_.push_back(Eigen::Vector3d(semi_filtered_cloud1->points_[i][0], semi_filtered_cloud1->points_[i][1], semi_filtered_cloud1->points_[i][2]));
                        }
                    }
                    open3d::visualization::DrawGeometries({cloud_frame_mean_filter}, "cloud_frame_mean_filter");

                    auto labels = cloud_frame_mean_filter->ClusterDBSCAN(config[type_spring]["LAST_STEP_EPS"].as<double>(), config[type_spring]["LAST_STEP_MIN_P"].as<double>());

                    auto unique_labels = std::set<int>(labels.begin(), labels.end());
                    int number_of_clusters = unique_labels.size();

                    std::vector<std::shared_ptr<open3d::geometry::PointCloud>> clusters_points_color;
                    // std::vector<std::shared_ptr<open3d::geometry::PointCloud>> clusters;
                    for (int i = 0; i < number_of_clusters; i++)
                    {

                        double red = (double)std::rand() / (double)RAND_MAX;
                        double green = (double)std::rand() / (double)RAND_MAX;
                        double blue = (double)std::rand() / (double)RAND_MAX;

                        auto color = Eigen::Vector3d(red, green, blue);

                        auto cluster = std::make_shared<open3d::geometry::PointCloud>();
                        for (int j = 0; j < labels.size(); j++)
                        {
                            if (labels.at(j) == i)
                            {
                                cluster->points_.push_back(cloud_frame_mean_filter->points_.at(j));
                                cluster->colors_.push_back(color);
                            }
                        }
                        clusters_points_color.push_back(cluster);
                    }

                    auto cloud1 = std::make_shared<open3d::geometry::PointCloud>();
                    for (auto cluster : clusters_points_color)
                    {
                        for (int j = 0; j < cluster->points_.size(); j++)
                        {
                            cloud1->points_.push_back(Eigen::Vector3d(cluster->points_[j][0], cluster->points_[j][1], cluster->points_[j][2]));
                        }
                    }
                    open3d::visualization::DrawGeometries({cloud1}, "cloud1");

                    // open3d::visualization::DrawGeometries({clusters_points_color}, "clusters_points_color");

                    auto good_clusters_lines = std::vector<std::shared_ptr<open3d::geometry::PointCloud>>();

                    for (int i = 0; i < clusters_points_color.size(); i++)
                    {
                        auto bb_max_b = clusters_points_color.at(i)->GetAxisAlignedBoundingBox().GetMaxBound();
                        auto bb_min_b = clusters_points_color.at(i)->GetAxisAlignedBoundingBox().GetMinBound();
                        auto width = bb_max_b[0] - bb_min_b[0];
                        auto height = bb_max_b[1] - bb_min_b[1];
                        auto depth = bb_max_b[2] - bb_min_b[2];
                        std::cout << width << " " << height << " " << depth << " " << std::endl;
                        if (width > config[type_spring]["LAST_STEP_LENGTH_LIM"].as<double>())
                        {
                            good_clusters_lines.push_back(clusters_points_color.at(i));
                        }
                    }
                    if (good_clusters_lines.size() > 0)
                    {
                        std::cout << "Frame is not empty" << std::endl;
                        station_server = 7;
                        recvbuf[2] = '0';
                    }
                    else
                    {
                        std::cout << "Empty frame" << std::endl;
                        station_server = 8;
                        recvbuf[2] = '0';
                    }
                    // auto cloud1 = std::make_shared<open3d::geometry::PointCloud>();
                    for (auto cluster : good_clusters_lines)
                    {
                        for (int j = 0; j < cluster->points_.size(); j++)
                        {
                            cloud1->points_.push_back(Eigen::Vector3d(cluster->points_[j][0], cluster->points_[j][1], cluster->points_[j][2]));
                        }
                    }
                    open3d::visualization::DrawGeometries({cloud1}, "Point Cloud Visualization");

                    // std::cout<<mean<<" "<<good_clusters_lines.size()<<std::endl;
                    // auto t2 = high_resolution_clock::now();
                    // auto duration = duration_cast<milliseconds>(t2 - t1);
                    // std::cout << "Full detection loop" << duration.count() << "\n";
                    // open3d::visualization::DrawGeometries({cloud_frame_mean_filter}, "After meaning center Visualization");
                }

                //     std::cout << "Cluster size = " << clusters.size() << std::endl;
                //     auto filtered_cloud1 = std::get<0>(cloud->RemoveRadiusOutliers(20, 2.8, false));
                //     //auto filtered_cloud1 = filtration_ang(cloud);
                //     //TODO: adaptive choping depending on the number of clusters
                //     auto chopped_point_cloud1 = adaptive_chopping(filtered_cloud1, ang_first_run);
                //     auto clusters1 = clusterization(chopped_point_cloud1);

                //     if(clusters1.size() == 0){
                //         cout<<"Maybe frame is empty"<<endl;
                //         double mean = 0;
                //         for(int i = 0; i < chopped_point_cloud1.points_(); i++){
                //             mean += chopped_point_cloud1.points_()[2];
                //         }
                //         mean = mean / chopped_point_cloud1->points_.size();
                //         auto cloud_frame = std::make_shared<open3d::geometry::PointCloud>();

                //         for(int i = 0; i < chopped_point_cloud1.points_(); i++){
                //             cloud_frame->points_.push_back(Eigen::Vector3d(cloud_frame->points_[i][0], cloud_frame->points_[i][1], cloud_frame->points_[i][2]);
                //         }

                //     }
            }
            else
            {
                cout<<"CLUSTERSIZE"<<clusters.size()<<endl;
                int springs = 1;
                // for (auto cluster : clusters)
                // {
                    
                //     std::cout << "Spring " << springs++ << std::endl;
                //     auto tf = getTransformation(cluster);
                //     std::cout << "Coordinates:\n"
                //               << std::get<0>(tf) << std::endl;
                //     std::cout << "Rotational matrix:\n"
                //               << std::get<1>(tf) << std::endl;
                //     std::cout << "Euler angles:\n"
                //               << std::get<1>(tf).eulerAngles(0, 1, 2) << std::endl;
                //     std::cout << std::endl;
                //     flag = true;
                //     if (rcvd > 0 && flag)
                //     {
                //         mymsg.command = 5;
                //         mymsg.x = 0;     // get<0>(tf);
                //         mymsg.y = 0;     // get<0>(tf)];
                //         mymsg.z = 0;     // get<0>(tf)[2];
                //         mymsg.phi = 0;   // get<1>(tf).eulerAngles(0);
                //         mymsg.theta = 0; // get<1>(tf).eulerAngles(1);
                //         mymsg.psi = 0;   // get<1>(tf).eulerAngles(2);

                //         iSendResult = send(ClientSocket, (char *)&mymsg, sizeof(msg), 0);
                //         if (iSendResult == SOCKET_ERROR)
                //         {
                //             printf("send failed with error: %d\n", WSAGetLastError());
                //             closesocket(ClientSocket);
                //             WSACleanup();
                //         }
                //         printf("Bytes sent: %d\n", iSendResult);
                //         // mtx.unlock();
                //     }
                //     flag = false;

                //     // v.pushBack(cluster);
                // }
                station_server = 5;
                // recvbuf[2] = '0';
            }

            // terminateThread = true;
            // break;
        }
        if (recvd_str[2] == '5')
        {
            terminateThread = true;
            break;
        }

        // if(rcvd>0)
        // {
        //     mymsg.command = 56;
        //     iSendResult = send(ClientSocket, (char *)&mymsg, sizeof(msg), 0);
        //     if (iSendResult == SOCKET_ERROR)
        //     {
        //         printf("send failed with error: %d\n", WSAGetLastError());
        //         closesocket(ClientSocket);
        //         WSACleanup();
        //     }
        //     printf("Bytes sent: %d\n", iSendResult);
        //     // mtx.unlock();}
        // }
    }

    // printMessage("Server_test");

    return 0;
}