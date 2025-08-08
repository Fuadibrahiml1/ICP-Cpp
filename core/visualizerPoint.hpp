#pragma once
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>

class Visualizer {
public:
    Visualizer(const std::string& window_name = "Visualizer");

    void showColoredCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                          const std::string& id,
                          int r, int g, int b);

    void spin();

private:
    pcl::visualization::PCLVisualizer::Ptr viewer;
};