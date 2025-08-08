#include "visualizerPoint.hpp"

Visualizer::Visualizer(const std::string& window_name)
    : viewer(new pcl::visualization::PCLVisualizer(window_name)) {
    viewer->setBackgroundColor(0.0, 0.0, 0.0);  // black background
    viewer->initCameraParameters();
    viewer->addCoordinateSystem(0.3);
}

void Visualizer::showColoredCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                                  const std::string& id,
                                  int r, int g, int b) {
    if (!cloud || cloud->empty()) {
        std::cerr << "[Visualizer] Skipping empty or null cloud: " << id << std::endl;
        return;
    }

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(cloud, r, g, b);
    if (!viewer->updatePointCloud(cloud, color_handler, id)) {
        viewer->addPointCloud(cloud, color_handler, id);
    }

    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id
    );
}

void Visualizer::spin() {
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
}