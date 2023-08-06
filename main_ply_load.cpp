#include <time.h>
#include <iostream>
#include <set>
#include "open3d/Open3D.h"
#include <chrono>
#include "c_interface.h"
#include "mpi.h"
#include "omp.h"
// #include "omp.h"
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

     // Create a point cloud
    auto cloud = std::make_shared<open3d::geometry::PointCloud>();
    // MPI_Init(NULL, NULL);
    if (open3d::io::ReadPointCloud("point_cloud.ply", *cloud)) {
        std::cout << "Successfully read the point cloud file." << std::endl;
    } else {
        std::cout << "Failed to read the point cloud file." << std::endl;
        return 1;
    }

    // // Visualize the point cloud
    // // open3d::visualization::DrawGeometries({cloud}, "Point Cloud Visualization");


    // auto arr = *cloud;
    
    // std::cout<<"riutvbij"<<cloud.use_count();
    // // std::cout<<  << std::endl;

    // // std::cout<< end(arr.points_) - begin(arr.points_)<< std::endl;
    // auto amount_points = end(arr.points_) - begin(arr.points_);

    // // auto arr = cloud+50;
    // int a = 10, b = 20;
    // std::cout << "&a: " << arr.points_[0][0] << "\n&b: " << &arr.points_[11563740] << "\n&a - &b: " << ((char*)&arr.points_[0] - (char*)&arr.points_[11563740]) << std::endl;
    // auto shared_points = std::vector<std::shared_ptr<open3d::geometry::PointCloud>>();
    // auto shared_points_result = std::vector<std::shared_ptr<open3d::geometry::PointCloud>>();
    // auto t1 = high_resolution_clock::now();
    
    // MPI_Init(NULL, NULL);

    // int groups = 10;
    // auto agroup_len_points = amount_points/groups;
    // for(int i = 0; i < groups; i++){
    //     auto cloud_1 = std::make_shared<open3d::geometry::PointCloud>();
    //     for(int j = 0; j < agroup_len_points; j++){
    //         cloud_1->points_.push_back(Eigen::Vector3d(arr.points_[i*agroup_len_points+j][0], arr.points_[i*agroup_len_points+j][1], arr.points_[i*agroup_len_points+j][2]));    
    //         // std::cout<<arr.points_[i*agroup_len_points+j][0]<<" "<< arr.points_[i*agroup_len_points+j][1]<<" "<<arr.points_[i*agroup_len_points+j][2]<<std::endl;
    //     }
    //     shared_points.push_back(cloud_1);
    // }
    // auto t2 = high_resolution_clock::now();
    // auto duration = duration_cast<milliseconds>(t2 - t1);
    // std::cout << "t2-t1 " << duration.count() << "\n";

    // auto t3 = high_resolution_clock::now();



    // for(int i = 0; i < groups; i++){
    //     auto semi_filtered_cloud = std::get<0>(shared_points[i]->RemoveStatisticalOutliers(NB_NEIGHBOURS, STD_RATIO, false));
    //     shared_points_result.push_back(std::get<0>(semi_filtered_cloud->RemoveStatisticalOutliers(NB_NEIGHBOURS, STD_RATIO, false)));
    // }
    // auto t4 = high_resolution_clock::now();
    // duration = duration_cast<milliseconds>(t4 - t3);
    // std::cout << "t4-t3 " << duration.count() << "\n";


    // auto t5 = high_resolution_clock::now();
    // auto cloud_final = std::make_shared<open3d::geometry::PointCloud>();
    // for(int i = 0; i < groups; i++){
    //     for(int j = 0; j < shared_points_result[i]->points_.size(); j++){
    //         // cloud_1->points_.push_back(Eigen::Vector3d(arr.points_[i*agroup_len_points+j][0], arr.points_[i*agroup_len_points+j][1], arr.points_[i*agroup_len_points+j][2]));    
    //         // std::cout<<arr.points_[i*agroup_len_points+j][0]<<" "<< arr.points_[i*agroup_len_points+j][1]<<" "<<arr.points_[i*agroup_len_points+j][2]<<std::endl;
    //         cloud_final->points_.push_back(shared_points_result[i]->points_[j]);
    //     }
    // }
    // MPI_Finalize();
    // std::cout << "blya"<<std::endl;
    // auto semi_filtered_cloud = std::get<0>(cloud_final->RemoveStatisticalOutliers(NB_NEIGHBOURS, STD_RATIO, false));


    // auto t6 = high_resolution_clock::now();
    // duration = duration_cast<milliseconds>(t6 - t5);
    // std::cout << "t6-t5 " << duration.count() << "\n";

    // open3d::visualization::DrawGeometries({cloud_final}, "Point Cloud Visualization");




    // std::cout<<(*(cloud_1)).points_[0]<<endl;

    // for(int i = 0; i < 10; i++){

    //     auto semi_filtered_cloud = std::get<0>(cloud->RemoveStatisticalOutliers(NB_NEIGHBOURS, STD_RATIO, false));
    // }


    
    
    //==============================Filtration==============================================
    auto one = high_resolution_clock::now();
    auto semi_filtered_cloud = std::get<0>(cloud->RemoveStatisticalOutliers(NB_NEIGHBOURS, STD_RATIO, false));

    auto filtered_cloud = std::get<0>(semi_filtered_cloud->RemoveStatisticalOutliers(NB_NEIGHBOURS, STD_RATIO, false));

    auto two = high_resolution_clock::now();

    auto duration = duration_cast<milliseconds>(two - one);
    std::cout << "Filtration: " << duration.count() << "\n";

    // //================================Clusterization================================================
    auto labels = filtered_cloud->ClusterDBSCAN(CLUSTERIZATION_EPS, CLUSTERIZAION_MIN_POINTS);

    auto three = high_resolution_clock::now();
    
    duration = duration_cast<milliseconds>(three - two);

    std::cout << "Clusterization: " << duration.count() << "\n";

    auto unique_labels = std::set<int>(labels.begin(), labels.end());
    int number_of_clusters = unique_labels.size();

    std::vector<std::shared_ptr<const open3d::geometry::PointCloud>> clusters;

    for (int i = 0; i < number_of_clusters; i++) {

        double red = (double)std::rand()/(double)RAND_MAX;
        double green = (double)std::rand()/(double)RAND_MAX;
        double blue = (double)std::rand()/(double)RAND_MAX;

        auto color = Eigen::Vector3d(red, green, blue);

        auto cluster = std::make_shared<open3d::geometry::PointCloud>();
        for (int j = 0; j < labels.size(); j ++) {
            if (labels.at(j) == i) {
                cluster->points_.push_back(filtered_cloud->points_.at(j));
                cluster->colors_.push_back(color);
            }
        }
        clusters.push_back(cluster);
    }

    auto good_clusters = std::vector<std::shared_ptr<const open3d::geometry::Geometry>>();

    for (int i = 0; i < clusters.size(); i++) {
        if (clusters.at(i)->points_.size() > CLUSTER_DELETE_THRESHOLD) {
            good_clusters.push_back(clusters.at(i));
        }
    }

    std::cout << good_clusters.size();

    //==============================Visualization======================================

    const auto vector = good_clusters;
    open3d::visualization::DrawGeometries(vector, "Point Cloud Visualization");

    return 0;
}