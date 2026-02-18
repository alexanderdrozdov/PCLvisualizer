#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <cmath>

void drawsphere(){
    const double radius = 1.0;
    const int steps_theta = 30;   
    const int steps_phi = 60;     

    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    
    for (int i = 0; i <= steps_theta; ++i) {
        double theta = M_PI * i / steps_theta;          
        double sin_theta = std::sin(theta);
        double cos_theta = std::cos(theta);

        for (int j = 0; j <= steps_phi; ++j) {
            double phi = 2.0 * M_PI * j / steps_phi;   
            double sin_phi = std::sin(phi);
            double cos_phi = std::cos(phi);

            pcl::PointXYZ point;
            point.x = radius * sin_theta * cos_phi;
            point.y = radius * sin_theta * sin_phi;
            point.z = radius * cos_theta;

            cloud->push_back(point);
        }
    }

    
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Sphere"));
    viewer->setBackgroundColor(0.1, 0.1, 0.1); 
    
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(cloud, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, color_handler, "sphere cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sphere cloud");

    viewer->addCoordinateSystem(1.0);

    
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }

    return;
}