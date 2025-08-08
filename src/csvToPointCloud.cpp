#include <fstream>
#include <sstream>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr csvToPointCloud(const std::string& filename) {
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    std::ifstream file(filename);
    std::string line;

    if (!file.is_open()) {
        std::cerr << "[Error] Failed to open file: " << filename << std::endl;
        return cloud;
    }

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string item;
        float x, y, z;

        if (std::getline(iss, item, ',')) x = std::stof(item); else continue;
        if (std::getline(iss, item, ',')) y = std::stof(item); else continue;
        if (std::getline(iss, item, ',')) z = std::stof(item); else continue;

        cloud->push_back(pcl::PointXYZ(x, y, z));
    }

    std::cout << "[csvToPointCloud] Loaded " << cloud->size() << " points from " << filename << std::endl;
    return cloud;
}