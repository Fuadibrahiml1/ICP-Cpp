#include "pipeline.hpp"
#include "../core/visualizerPoint.hpp"
#include "../ramka/Registration.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <string>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::Normal NormalT;
typedef pcl::PointCloud<NormalT> NormalCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;

PointCloudT::Ptr preprocess(const PointCloudT::Ptr& cloud) {
    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(0.001f, 0.001f, 0.001f);
    PointCloudT::Ptr downsampled(new PointCloudT);
    voxel.filter(*downsampled);

    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(downsampled);
    sor.setMeanK(20);
    sor.setStddevMulThresh(1.0);
    PointCloudT::Ptr filtered(new PointCloudT);
    sor.filter(*filtered);

    return filtered;
}

// Utility: Compute normals + FPFH features
void computeNormalsAndFPFH(const PointCloudT::Ptr& cloud,
                           NormalCloudT::Ptr& normals,
                           FeatureCloudT::Ptr& fpfh) {
    pcl::NormalEstimation<PointT, NormalT> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.01);
    ne.compute(*normals);

    pcl::FPFHEstimation<PointT, NormalT, FeatureT> fpfh_est;
    fpfh_est.setInputCloud(cloud);
    fpfh_est.setInputNormals(normals);
    fpfh_est.setSearchMethod(tree);
    fpfh_est.setRadiusSearch(0.025);
    fpfh_est.compute(*fpfh);
}

// Step 1: Global alignment (RANSAC+FPFH)
Eigen::Matrix4f globalAlign(const PointCloudT::Ptr& src, FeatureCloudT::Ptr& src_feat,
                            const PointCloudT::Ptr& tgt, FeatureCloudT::Ptr& tgt_feat,
                            PointCloudT::Ptr& output)
{
    pcl::SampleConsensusPrerejective<PointT, PointT, FeatureT> align;
    align.setInputSource(src);
    align.setSourceFeatures(src_feat);
    align.setInputTarget(tgt);
    align.setTargetFeatures(tgt_feat);
    align.setMaximumIterations(50000);
    align.setNumberOfSamples(3);
    align.setCorrespondenceRandomness(5);
    align.setSimilarityThreshold(0.9f);
    align.setMaxCorrespondenceDistance(0.025f);
    align.setInlierFraction(0.25f);

    align.align(*output);
    if (align.hasConverged()) {
        std::cout << "[Global] Alignment succeeded." << std::endl;
        return align.getFinalTransformation();
    } else {
        std::cerr << "[Global] Alignment failed." << std::endl;
        return Eigen::Matrix4f::Identity();
    }
}

void pipeline::Pipeline(const std::string& file_path1, const std::string& file_path2)
{
    PointCloudT::Ptr cloud1(new PointCloudT), cloud2(new PointCloudT);
    if (pcl::io::loadPCDFile<PointT>(file_path1, *cloud1) == -1) {
        std::cerr << "Couldn't read file: " << file_path1 << std::endl;
        return;
    }
    if (pcl::io::loadPCDFile<PointT>(file_path2, *cloud2) == -1) {
        std::cerr << "Couldn't read file: " << file_path2 << std::endl;
        return;
    }

    cloud1 = preprocess(cloud1);
    cloud2 = preprocess(cloud2);

    NormalCloudT::Ptr normals1(new NormalCloudT), normals2(new NormalCloudT);
    FeatureCloudT::Ptr fpfh1(new FeatureCloudT), fpfh2(new FeatureCloudT);
    computeNormalsAndFPFH(cloud1, normals1, fpfh1);
    computeNormalsAndFPFH(cloud2, normals2, fpfh2);

    PointCloudT::Ptr cloud2_aligned_global(new PointCloudT);
    Eigen::Matrix4f global_transform = globalAlign(cloud2, fpfh2, cloud1, fpfh1, cloud2_aligned_global);

    Registration registration;
    Eigen::Matrix4f icp_transform;
    double fitness_score;
    PointCloudT::Ptr cloud2_aligned_icp(new PointCloudT);

    cloud2_aligned_icp = registration.align(cloud2_aligned_global, cloud1, icp_transform, fitness_score);

    if (!cloud2_aligned_icp->empty()) {
        std::cout << "[ICP] Fitness score: " << fitness_score << std::endl;
        std::cout << "[ICP] Transformation:\n" << icp_transform << std::endl;
        *cloud1 += *cloud2_aligned_icp;
        pcl::io::savePCDFileASCII("merged_result.pcd", *cloud1);
        std::cout << "Saved merged point cloud to merged_result.pcd\n";
    } else {
        std::cerr << "[ICP] Alignment failed.\n";
    }

    Visualizer vis;
    vis.showColoredCloud(cloud1, "merged", 255, 0, 0);
    vis.showColoredCloud(cloud2_aligned_icp, "aligned_cloud2", 0, 255, 0);
    vis.spin();
}