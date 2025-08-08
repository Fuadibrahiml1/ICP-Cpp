#include "Registration.hpp"
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <limits>

Registration::Registration() {}

Registration::PointCloudT::Ptr Registration::align(
    const PointCloudT::Ptr& source,
    const PointCloudT::Ptr& target,
    Eigen::Matrix4f& transformation,
    double& fitness_score
) {
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(source);
    icp.setInputTarget(target);

    icp.setMaxCorrespondenceDistance(0.01);
    icp.setMaximumIterations(500);
    icp.setTransformationEpsilon(1e-10);
    icp.setEuclideanFitnessEpsilon(1e-8);

    PointCloudT::Ptr aligned(new PointCloudT);
    icp.align(*aligned);

    if (icp.hasConverged()) {
        transformation = icp.getFinalTransformation();
        fitness_score = icp.getFitnessScore();
        pcl::transformPointCloud(*source, *aligned, transformation);
        return aligned;
    } else {
        transformation = Eigen::Matrix4f::Identity();
        fitness_score = std::numeric_limits<double>::max();
        return PointCloudT::Ptr(new PointCloudT);
    }
}