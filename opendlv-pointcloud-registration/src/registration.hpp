#ifndef REGISTRATION_HPP
#define REGISTRATION_HPP
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

// Registration
#include <pcl/registration/icp.h>        // ICP point-to-point
#include <pcl/registration/icp_nl.h>     // ICP Nolinear point-to-plane
#include <pcl/registration/gicp.h>       // ICP plane-to-plane
#include <pcl/registration/ndt.h>        // NDT Registration
#include <pcl/registration/transforms.h> // Transformation matrix 

// Visualization
#include <pcl/visualization/pcl_visualizer.h>

#define RED    Color(0.6, 0, 0)
#define GREEN  Color(0.235, 0.702, 0.443)
#define BLUE   Color(0.4, 0.698, 1)
#define VIOLET Color(0.933, 0.510, 0.933)
#define BLACK  Color(0, 0, 0)
#define WHITE  Color(1, 1, 1)

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
/*-------------------------------------------------------------------------------------*/
template<typename PointT>
class Filters{
public:
    // Constructor
    Filters() = default;

    // Destructor
    ~Filters() = default;
	
	/**
	 * @brief Pass through filter (the background can be removed if the background has a certain distance from the foreground)
	 * 
	 * @param cloud Input pointcloud
	 * @param axis Choose filter axis [Example: "z"]
	 * @param limits Set the filter constraints
	 * @return Filtered pointcloud 
	 */
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

	/**
	 * @brief Pick points inside/outside of the box
	 * 
	 * @param cloud Input pointcloud
	 * @param min_point Box minimum corner point
	 * @param max_point Box maximum corner point
	 * @param setNegative true: choose points outside of the box, false: choose points inside of the box
	 * @return Filtered pointcloud
	 */
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

	/** 
	 * @brief DownSample the dataset using a given voxel leaf size
	 * 
	 * @param cloud2 Inpout pointcloud
	 * @param filterRes voxel leaf size
	 * @return Filtered pointcloud
	**/
     typename pcl::PointCloud<PointT>::Ptr VoxelGridDownSampling( const typename pcl::PointCloud<PointT>::Ptr &cloud, 
																 const float &filterRes = 0.3f){
		typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
		pcl::VoxelGrid<PointT> voxelFilter;
		voxelFilter.setInputCloud(cloud); // Set input point cloud
		voxelFilter.setLeafSize(filterRes, filterRes, filterRes); // Set voxel size
		voxelFilter.filter(*cloud_filtered);
		std::cout << "[VoxelGridDownSampling]  Original points: " << cloud->width * cloud->height 
				  <<  ", Filtered points: " << cloud_filtered->points.size() << std::endl;
		return cloud_filtered;
	}
	
	/**
	 * @brief 
	 * 
	 * @param cloud Input pointcloud
	 * @param meanK Set the number of nearest neighbors to use for mean distance estimation.
	 * @param StddevMulThresh Threshold for determining outliers [smaller -> more stringent]
	 * @return Filtered pointcloud
	 */
     typename pcl::PointCloud<PointT>::Ptr StatisticalOutlierRemoval( const typename pcl::PointCloud<PointT>::Ptr &cloud, 
																	  const int &meanK, 
																	  const double &StddevMulThresh ){
    	typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
		pcl::StatisticalOutlierRemoval<PointT> sor;
		sor.setInputCloud(cloud);
		sor.setMeanK(meanK); 
		sor.setStddevMulThresh(StddevMulThresh); 
		sor.filter(*cloud_filtered);
		std::cout << "[StatisticalOutlierRemoval] " << " Original points: " 
				<< cloud->points.size() <<  ", Filtered points: " << cloud_filtered->points.size() << std::endl;
		return cloud_filtered;
	}  

};

/*-------------------------------------------------------------------------------------*/

template<typename PointT>
class Visual{
public:
    // Constructor
    Visual() = default;

    // Destructor
    ~Visual() = default;

	/**
	 * @brief Initialize the PCL viewer, set background colorand camera angle
	 * 
	 * @param viewer Set PCL viewer
	 * @param background_color Set viewer's background color, such as 'BLACK'
	 * @param camera_angle Set viewer's camera angle, such as 'FPS'
	 */
	void initCamera (pcl::visualization::PCLVisualizer &viewer,
					 const Color &background_color, 
					 const CameraAngle &camera_angle){
		viewer.setBackgroundColor(background_color.R, background_color.G, background_color.B); // Set black background
		viewer.initCameraParameters();
		const int distance = 90;
		if(camera_angle != FPS)
			viewer.addCoordinateSystem(1.0);
		switch(camera_angle) {
			case TOP:
			viewer.setCameraPosition(0, 0, distance, 1, 0, 1); break;
			case SIDE:
			viewer.setCameraPosition(0, -distance, 0, 0, 0, 1); break;
			case FPS:
			viewer.setCameraPosition(-10, 0, 0, 0, 0, 1); break;
		}
	}

	/**
	 * @brief Point cloud visualization, color is changing based on height
	 * 
	 * @param viewer 
	 * @param cloud Input point cloud
	 * @param point_size Set point size in the viewer
	 * @param name Viewer's name
	 */
	void showPointcloud_height( pcl::visualization::PCLVisualizer &viewer, 
                         typename pcl::PointCloud<PointT>::Ptr &cloud, 
                         const int &point_size,
                         const std::string &name ){
		pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud, "z");
		viewer.addPointCloud(cloud, fildColor, name);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, name);
	}

	/**
	 * @brief Point cloud visualization, manually set color
	 * 
	 * @param viewer 
	 * @param cloud Input point cloud
	 * @param point_size Set point size in the viewer
	 * @param color Set point color
	 * @param name Viewer's name
	 */
	void showPointcloud (pcl::visualization::PCLVisualizer &viewer, 
                         typename pcl::PointCloud<PointT>::Ptr &cloud, 
                         const int &point_size,
                         const Color &color, 
                         const std::string &name){
    viewer.addPointCloud(cloud, name);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, name);
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, color.R, color.G, color.B, name);
	}
};

/*-------------------------------------------------------------------------------------*/

/**
 * @brief Calculate the time consumption. 
 * Have to use "auto start_time = std::chrono::system_clock::now()" to start timer.
 * 
 * @param start_time Choose starting Timer
 * @param function_name Function name shown in terminal
 */
void 
 timerCalculator (const std::chrono::_V2::system_clock::time_point &start_time,
                                const std::string &function_name){
    auto end_time = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    double time_passed = (double) duration.count() * 
            std::chrono::microseconds::period::num / std::chrono::microseconds::period::den;
    std::cout << "[--Timer--] " << function_name <<" time used: " << time_passed << " [s]." << std::endl;
}
#endif /*REGISTRATION_HPP*/