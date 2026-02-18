#include "livox_lidar_api.h"
#include "livox_lidar_def.h"
#include "livox_lidar_cfg.h"

#include "drawsphere.h"
#include "SafeQueue.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <cmath>
#include <thread>
#include <atomic>
#include <chrono>
#include <fstream>

SafeQueue<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudQueue;
std::atomic<bool> running{true};

std::ofstream cloudfile("cloudlog.txt");

void ImuDataCallback(uint32_t handle, const uint8_t dev_type,  LivoxLidarEthernetPacket* data, void* client_data) {
  if (data == nullptr) {
    return;
  } 
  printf("Imu data callback handle:%u, data_num:%u, data_type:%u, length:%u, frame_counter:%u.\n",
      handle, data->dot_num, data->data_type, data->length, data->frame_cnt);
}

void PointCloudCallback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data) {
  if (data == nullptr) {
    return;
  }
  cloudfile << "x y z";
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  cloud->reserve(data->dot_num);
  
  if (data->data_type == kLivoxLidarCartesianCoordinateHighData) {
    LivoxLidarCartesianHighRawPoint *src= (LivoxLidarCartesianHighRawPoint *)data->data;
    for(uint32_t i = 0; i < data->dot_num; i++){
        pcl::PointXYZ dst;
        dst.x = src->x / 1000.0f;
        dst.y = src->y / 1000.0f;
        dst.z = src->z / 1000.0f;

        cloud->push_back(dst);

        cloudfile << dst.x << ' ' << dst.y << ' ' << dst.z << std::endl;
    }
  }

  else if (data->data_type == kLivoxLidarCartesianCoordinateLowData) {
    LivoxLidarCartesianLowRawPoint *src= (LivoxLidarCartesianLowRawPoint *)data->data;
    for(uint32_t i = 0; i < data->dot_num; i++){
        pcl::PointXYZ dst;
        dst.x = src->x / 1000.0f;
        dst.y = src->y / 1000.0f;
        dst.z = src->z / 1000.0f;

        cloud->push_back(dst);
    }
  }

  cloudQueue.push(cloud);
}

int main(int argc, const char *argv[]){

    if(argc != 2){

      std::cout << "Input Config File Path" << std::endl;
      return -1;

    }

    const std::string path = argv[1];

    if(!LivoxLidarSdkInit(path.c_str())){

      std::cout << "Lidar initialization failed" << std::endl;

      LivoxLidarSdkUninit();

      drawsphere();

      return 1;
    }

    SetLivoxLidarPointCloudCallBack(PointCloudCallback, nullptr);

    SetLivoxLidarImuDataCallback(ImuDataCallback, nullptr);
    
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer("Lidar Point Cloud Viewer"));

    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem();
    viewer->initCameraParameters();

    pcl::PointCloud<pcl::PointXYZ>::Ptr last_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    bool cloud_updated = false;

    while(!viewer->wasStopped() && running){

      pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud;

      if(cloudQueue.pop(new_cloud, false)){

        last_cloud = new_cloud;
        cloud_updated = true;

      }

      if (cloud_updated && !last_cloud->empty()){

        viewer->removePointCloud("lidar_cloud");

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(last_cloud, 255, 0, 0);

        viewer->addPointCloud(last_cloud, color_handler, "lidar_cloud");

        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "lidar_cloud");

        cloud_updated = false;
      }

      viewer->spinOnce(100);
    }

    std::cout << "Shutting Down" << std::endl;
    running = false;
    cloudfile.close();

    LivoxLidarSdkUninit();

    return 0;
}