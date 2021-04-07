#ifndef PREPROCESSING_HPP
#define PREPROCESSING_HPP
/* \author Leo Wang & Varun Hegde*/
// Customized Supporting function for pointcloud preprocessing 
// using PCL

/**
 * Developer: Liangyu Wang
 * E-mail:    liangyu@student.chalmers.se
 * Date:      03/2020
 */
#include <iostream>
//#include <fstream>
#include <memory>
#include <thread>
#include <random>
#include <cstdlib>
#include <chrono>
#include <dirent.h>
#include <algorithm>
#include <string>

// PCL Library
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

// Filters
#include <pcl/filters/passthrough.h>                 // PassThrough Filter
#include <pcl/filters/voxel_grid.h>                  // VoxelGrid Down Sampling
#include <pcl/filters/statistical_outlier_removal.h> // Statistical Outlier Removal
#include <pcl/filters/radius_outlier_removal.h>      // Radius Outlier Removal
#include <pcl/filters/extract_indices.h>             // Extract pointCloud according to indices
#include <pcl/filters/crop_box.h>

// Segmentation
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

// Registration
#include <pcl/registration/icp.h>        // ICP point-to-point
#include <pcl/registration/icp_nl.h>     // ICP Nolinear point-to-plane
#include <pcl/registration/gicp.h>       // ICP plane-to-plane
#include <pcl/registration/ndt.h>        // NDT Registration
#include <pcl/registration/transforms.h> // Transformation matrix 

// Visualization
#include <pcl/visualization/pcl_visualizer.h>

#define RED Color(0.6, 0, 0)
#define GREEN  Color(0.235, 0.702, 0.443)
#define BLUE   Color(0.4, 0.698, 1)
#define VIOLET Color(0.933, 0.510, 0.933)
#define BLACK  Color(0, 0, 0)
#define WHITE  Color(1, 1, 1)

struct Box{
	float x_min;
	float y_min;
	float z_min;
	float x_max;
	float y_max;
	float z_max;
};

bool point_cmp(pcl::PointXYZ a, pcl::PointXYZ b){
    return a.z<b.z;
};

enum CameraAngle {TOP, FPS, SIDE};

struct Color{
    float R, G, B;

	Color(float setR, float setG, float setB)
		: R(setR), G(setG), B(setB)
	{}
};

/*---------------------------------------------------------------------------*/
template<typename PointT>
class Filters{
public:
    // Constructor
    Filters() = default;

    // Destructor
    ~Filters() = default;
	
    typename pcl::PointCloud<PointT>::Ptr PassThroughFilter( const typename pcl::PointCloud<PointT>::Ptr &cloud, 
															 const std::string &axis, 
															 const std::array<float, 2> &limits){
		typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
		pcl::PassThrough<PointT> passFilter;
		passFilter.setInputCloud(cloud);  	 // Set input point cloud 
		passFilter.setFilterFieldName(axis); // Set filter axis
		passFilter.setFilterLimits(limits[0], limits[1]); // Set the filter acceptable range
		passFilter.filter(*cloud_filtered);
		std::cout << "[PassFilter " << axis << "(" << limits[0] << " -> " << limits[1] << ") ] " << " Original points: " 
				<< cloud->points.size() <<  ", Filtered points: " << cloud_filtered->points.size() << std::endl;
		return cloud_filtered;
	}

	typename pcl::PointCloud<PointT>::Ptr boxFilter( const typename pcl::PointCloud<PointT>::Ptr &cloud, 
													 const Eigen::Vector4f &min_point, 
													 const Eigen::Vector4f &max_point, 
													 const bool &setNegative = false){
		pcl::CropBox<PointT> region(true);
		std::vector<int> indices;
		region.setMin(min_point);
		region.setMax(max_point);
		region.setInputCloud(cloud);
		region.filter(indices);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
		for (int index : indices)
			inliers->indices.push_back(index);
		pcl::ExtractIndices<PointT> extract;
		// Extract the noise point cloud on the roof
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		extract.setNegative(setNegative);
		extract.filter(*cloud);
		return cloud;
	}

	// DownSample the dataset using a given leaf size
    typename pcl::PointCloud<PointT>::Ptr VoxelGridDownSampling( const typename pcl::PCLPointCloud2::Ptr &cloud2, 
																 const float &filterRes){
		pcl::PCLPointCloud2::Ptr cloud2_filtered(new pcl::PCLPointCloud2()); // Create filtered object
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
		pcl::VoxelGrid<pcl::PCLPointCloud2> voxelFilter;
		voxelFilter.setInputCloud(cloud2); // Set input point cloud
		voxelFilter.setLeafSize(filterRes, filterRes, filterRes); // Set voxel size
		voxelFilter.filter(*cloud2_filtered);
		pcl::fromPCLPointCloud2(*cloud2_filtered, *cloud_filtered); // PCLPointCloud2 ---> pcl::PointXYZ
		std::cout << "[VoxelGridDownSampling]  Original points: " << cloud2->width * cloud2->height <<  ", Filtered points: " << cloud_filtered->points.size() << std::endl;
		return cloud_filtered;
	}
    typename pcl::PointCloud<PointT>::Ptr StatisticalOutlierRemoval( const typename pcl::PointCloud<PointT>::Ptr &cloud, 
																	 const int &meanK, 
																	 const double &StddevMulThresh ){
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
		pcl::StatisticalOutlierRemoval<PointT> sor;
		sor.setInputCloud(cloud);
		sor.setMeanK(meanK); // Set the number of nearest neighbors to use for mean distance estimation.
		sor.setStddevMulThresh(StddevMulThresh); // Set threshold for determining outliers [smaller -> more stringent]
		// sor.setNegative(true);
		sor.filter(*cloud_filtered);
		std::cout << "[StatisticalOutlierRemoval] " << " Original points: " 
				<< cloud->points.size() <<  ", Filtered points: " << cloud_filtered->points.size() << std::endl;
		return cloud_filtered;
	}  // Can be used for reducing noise
};

#endif /*PREPROCESSING_HPP*/