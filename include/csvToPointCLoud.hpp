//
// Created by aze_pc0218 on 7/30/25.
//

#pragma once
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr csvToPointCloud(const std::string& filename);