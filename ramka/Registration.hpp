#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>

class Registration {
public:
    using PointT = pcl::PointXYZ;
    using PointCloudT = pcl::PointCloud<PointT>;
    Registration();
    PointCloudT::Ptr align(
        const PointCloudT::Ptr& source,
        const PointCloudT::Ptr& target,
        Eigen::Matrix4f& transformation,
        double& fitness_score
    );
};