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

#endif /*PREPROCESSING_HPP*/