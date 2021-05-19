#ifndef PROCESSING_HPP
#define PROCESSING_HPP
/**
 * @brief Customized Supporting function for pointcloud processing using PCL
 * 
 *
 * Developer: Liangyu Wang & Varun Hegde
 * E-mail:    liangyu@student.chalmers.se
 * 			  varunh@student.chalmers.se
 * Date:      03/2021
**/

#include <iostream>
#include <memory>
#include <thread>
#include <random>
#include <cstdlib>
#include <chrono>
#include <dirent.h>
#include <algorithm>
#include <string>

// Eigen
#include <Eigen/Dense>

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
#include <pcl/filters/random_sample.h>
#include <pcl/surface/mls.h>  // Upsampling

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


// NDT-omp
#include <pclomp/ndt_omp.h>
#include <pclomp/gicp_omp.h>

#define RED    Color(0.6, 0, 0)
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

/*-------------------------------------------------------------------------------------*/

/** 
 * @brief Load KITTI files. 
 * 
 * @param folderPath KITTI data folder
 * @param VERBOSE Whether show result
 * @return file paths & number of files
 **/
std::tuple<std::vector<std::string>, int16_t> 
 loadFile (const std::string &folderPath, bool VERBOSE){
    // Count the total number of files in the path and return the path of all files.
    std::vector<std::string> filePaths; 
    DIR *path;
    struct dirent *ep;
    char path_array[(int) folderPath.length() + 1];
    strcpy(path_array, folderPath.c_str());
    path = opendir(path_array);
    int16_t count = 0;
    if(path != NULL){
        while(ep = readdir(path)){
            if(!ep -> d_name || ep -> d_name[0] == '.')
                continue;
            filePaths.push_back(folderPath + ep -> d_name);
            count ++;
        }
        (void) closedir(path);
        std::sort(filePaths.begin(), filePaths.end());
    }
    else
        perror("Couldn't open the directory...");
    if (VERBOSE == true)
        std::cout << "Found " << count << " files in folder [" << folderPath << "]."<< std::endl;
    return std::make_tuple(filePaths, count);
}

/** 
 * @brief Load pointcloud from KITTI dataset and save into 'pcl::XYZ'
 * 
 * @param filePaths Input filePaths
 * @param NUM Choose file filePaths[NUM]
 * @return pointcloud
**/
pcl::PointCloud<pcl::PointXYZ>::Ptr
 loadKitti (const std::vector<std::string> &filePaths, 
                          const int16_t &NUM){
    pcl::PointCloud<pcl::PointXYZI>::Ptr points (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    std::fstream input(filePaths[NUM].c_str(), std::ios::in | std::ios::binary);
    if(!input.good()){
		std::cerr << "Could not read file: " << filePaths[NUM] << std::endl;
		exit(EXIT_FAILURE);
	}
	input.seekg(0, std::ios::beg);
    int i;
	for (i=0; input.good() && !input.eof(); i++) {
		pcl::PointXYZI point;
		input.read((char *) &point.x, 3*sizeof(float));
		input.read((char *) &point.intensity, sizeof(float));
		points->push_back(point);
	}
	input.close();
    pcl::copyPointCloud(*points, *cloud);
    // std::cout << "Load file: [" << filePaths[NUM] << "]." << std::endl;
    return cloud;
}

/**
 * @brief Calculate the time consumption. 
 * Have to use "auto start_time = std::chrono::system_clock::now()" to start timer.
 * 
 * @param start_time Choose starting Timer
 * @param function_name Function name shown in terminal
 */
double 
 timerCalculator (const std::chrono::_V2::system_clock::time_point &start_time,
                                const std::string &function_name){
    auto end_time = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    double time_passed = (double) duration.count() * 
            std::chrono::microseconds::period::num / std::chrono::microseconds::period::den;
    std::cout << "[--Timer--] " << function_name <<" time used: " << time_passed << " [s]." << std::endl;
	return time_passed;
}

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

	/**
	 * @brief Random down sampling to size 'sample_number'
	 * 
	 * @param cloud Input pointcloud
	 * @param sample_number Set the target pointcloud size number
	 * @return pcl::PointCloud<PointT>::Ptr 
	 */
	typename pcl::PointCloud<PointT>::Ptr RandomSampling(const typename pcl::PointCloud<PointT>::Ptr &cloud,
														 const uint32_t sample_number){
		typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
		pcl::RandomSample<PointT> rs;
		rs.setInputCloud(cloud);
		rs.setSample(sample_number);
		rs.filter(*cloud_filtered);
		std::cout << "[RandomDownSampling] " << " Original points: " 
				<< cloud->points.size() <<  ", Filtered points: " << cloud_filtered->points.size() << std::endl;
		return cloud_filtered;
	}

	/**
	 * @brief Set desired point number and change input pointcloud
	 * 
	 * @param cloud Input cloud
	 * @param sample_number Target points number
	 * @return pcl::PointCloud<PointT>::Ptr 
	 */
	typename pcl::PointCloud<PointT>::Ptr PointComplement(const typename pcl::PointCloud<PointT>::Ptr &cloud,
														  const uint32_t sample_number){
		int num = sample_number - cloud->points.size();
		if(num >= 0){
			pcl::PointXYZ point;
			point.x = 0.0f;
			point.y = 0.0f;
			point.z = 0.0f;
			for(int i = 0; i < num; i++){
				cloud->points.push_back(point);
			}
			std::cout << "[Point complement]: " << " Original points: " 
				<< sample_number - num <<  ", Filtered points: " << cloud->points.size() << std::endl;
			return cloud;
		}
		else{
			typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
			cloud_filtered = RandomSampling(cloud, sample_number);
			return cloud_filtered;
		}
	}

	/**
	 * @brief Use 'pcl_isfinite()' detection to remove invalid points.
	 * 
	 * @param cloud Input pointcloud
	 * @return pcl::PointCloud<PointT>::Ptr 
	 */
	typename pcl::PointCloud<PointT>::Ptr InvalidPointsRemoval(const typename pcl::PointCloud<PointT>::Ptr &cloud){
		int num = cloud->points.size();
		pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->points.begin();
		while (it != cloud->points.end()){
			float x, y, z, rgb;
			x = it->x;
			y = it->y;
			z = it->z;
			if (!pcl_isfinite(x) || !pcl_isfinite(y) || !pcl_isfinite(z) || !pcl_isfinite(rgb))
				it = cloud->points.erase(it);
			else
				++it;
		}
		std::cout << "Remove " << num - cloud->points.size() << " invalid points." << std::endl;
		return cloud;
	}

	/**
	 * @brief Remove Nan points from the input point cloud
	 * 
	 * @param cloud Input point cloud
	 * @return pcl::PointCloud<PointT>::Ptr 
	 */
	typename pcl::PointCloud<PointT>::Ptr RemoveNan(const typename pcl::PointCloud<PointT>::Ptr &cloud){
		typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
		std::vector<int> indices;
  		pcl::removeNaNFromPointCloud(*cloud, *cloud_filtered, indices);
		return cloud_filtered;
	}
};

/*-------------------------------------------------------------------------------------*/

template<typename PointT>
class Segmentation{
public:
    // Constructor
    Segmentation() = default;

    // Destructor
    ~Segmentation() = default;

	/**
	 * @brief 
	 * 
	 * @param cloud Input point cloud
	 * @param indices Indices of extracted points
	 * @param set_negative Whether to reverse the selection
	 * @return pcl::PointCloud<PointT>::Ptr 
	 */
	typename pcl::PointCloud<PointT>::Ptr 
	 indicesExtract(const typename pcl::PointCloud<PointT>::Ptr &cloud, 
									pcl::PointIndices::Ptr &indices,
									const bool &set_negative = false){
		typename pcl::PointCloud<PointT>::Ptr cloud_output(new pcl::PointCloud<PointT>());
		pcl::ExtractIndices<PointT> ei;
		ei.setInputCloud(cloud);
		ei.setIndices(indices);
		ei.setNegative(set_negative);
		ei.filter(*cloud_output);
		return cloud_output;
	}

	/**
	 * @brief Extract rough ground plane according to the points height
	 * 
	 * @param cloud Z-axis Sorted point cloud
	 * @param height_threshold Set height threshold to determine plane
	 * @param min_number Set the minimum number of points, which are used to extract plane
	 * @return pcl::PointCloud<PointT>::Ptr 
	 */
	typename pcl::PointCloud<PointT>::Ptr 
	 RoughGroundExtraction (const typename pcl::PointCloud<PointT>::Ptr &cloud, 
												  const float & height_threshold, 
												  const int & min_number){
		typename pcl::PointCloud<PointT>::Ptr cloud_output(new pcl::PointCloud<PointT>());
		float sum = 0.0f; 
		int num = 0;         
		for(int i = 0; (i < cloud->points.size()) && (num < min_number); i++){
			sum += cloud->points[i].z; 
			num ++;
		} 
		// Calulate average point height
		float average_height = num != 0 ? sum/num : 0;
		for(int i = 0; i < cloud->points.size(); i++){
			if(cloud->points[i].z < average_height + height_threshold)
				cloud_output->points.push_back(cloud->points[i]);
		}
		return cloud_output;
	}
	
	/**
	 * @brief Plane Segmentation using RANSAC
	 * 
	 * @param original_cloud 
	 * @param rough_ground_cloud Rough ground input cloud based on heihgt
	 * @param maxIterations Set maximum iteration of RANSAC
	 * @param distanceThreshold Distance to the model threshold [unit: meter]
	 * @return std::tuple<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
	 */
	std::tuple<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
	 PlaneSegmentationRANSAC (const typename pcl::PointCloud<PointT>::Ptr &original_cloud, 
							  const typename pcl::PointCloud<PointT>::Ptr &rough_ground_cloud, 
							  const int &maxIterations, 
							  const float &distanceThreshold) {
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices); // defeine plane inliers
		pcl::SACSegmentation<PointT> seg;
		seg.setOptimizeCoefficients (true);
		seg.setModelType (pcl::SACMODEL_PLANE); // Set plane model
		seg.setMethodType (pcl::SAC_RANSAC);    // Method: RANSAC
		seg.setMaxIterations(maxIterations);    
		seg.setDistanceThreshold (distanceThreshold); // unit [meter]
		seg.setInputCloud (rough_ground_cloud); 
		seg.segment (*inliers, *coefficients);
		if (inliers->indices.size () == 0)
			PCL_ERROR ("Could not estimate a planar model for the given dataset.");
		typename pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
		typename pcl::PointCloud<PointT>::Ptr cloud_other(new pcl::PointCloud<PointT>());
		// Extract plane and non-plane cloud
		cloud_plane = indicesExtract(original_cloud, inliers);
		cloud_other = indicesExtract(original_cloud, inliers, true);
		std::cout << "[RANSAC Plane Segmentation] Plane points: " << cloud_plane->points.size() << ", other points: " 
				<< cloud_other->points.size() << std::endl;
		return std::make_tuple(cloud_plane, cloud_other);
	}

	/**
	 * @brief Plane Segmentation using SVD
	 * 
	 * @param original_cloud 
	 * @param rough_ground_cloud 
	 * @param d_threshold 
	 * @return std::tuple<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
	 */
	std::tuple<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
	 PlaneEstimation(const typename pcl::PointCloud<PointT>::Ptr &original_cloud, 
					 const typename pcl::PointCloud<PointT>::Ptr &rough_ground_cloud,
					 const float &d_threshold = 0.3f) {
		float x_mean = 0, y_mean = 0, z_mean = 0; 
		float distance_threshold = d_threshold;
		for(int i=0; i<rough_ground_cloud->points.size(); i++){
			x_mean += rough_ground_cloud->points[i].x;
			y_mean += rough_ground_cloud->points[i].y;
			z_mean += rough_ground_cloud->points[i].z;
		}
		// Calculate mean X, Y, Z
		int size = rough_ground_cloud->points.size()!=0 ? rough_ground_cloud->points.size():1;
		x_mean /= size;
		y_mean /= size;
		z_mean /= size;
		// Calculate covariance
		float xx = 0, yy = 0, zz = 0;
		float xy = 0, xz = 0, yz = 0;
		for(int i=0;i<rough_ground_cloud->points.size();i++){
			xx += (rough_ground_cloud->points[i].x-x_mean)*(rough_ground_cloud->points[i].x-x_mean);
			xy += (rough_ground_cloud->points[i].x-x_mean)*(rough_ground_cloud->points[i].y-y_mean);
			xz += (rough_ground_cloud->points[i].x-x_mean)*(rough_ground_cloud->points[i].z-z_mean);
			yy += (rough_ground_cloud->points[i].y-y_mean)*(rough_ground_cloud->points[i].y-y_mean);
			yz += (rough_ground_cloud->points[i].y-y_mean)*(rough_ground_cloud->points[i].z-z_mean);
			zz += (rough_ground_cloud->points[i].z-z_mean)*(rough_ground_cloud->points[i].z-z_mean);
		}
		// Calculate covariance matrix
		Eigen::Matrix<float, 3, 3> cov;
		cov << xx,xy,xz,
			   xy, yy, yz,
			   xz, yz, zz;
		cov /= size;
		// Singular Value Decomposition: SVD
    	Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov,Eigen::DecompositionOptions::ComputeFullU);
		Eigen::Matrix<float, 3, 1> normal = (svd.matrixU().col(2));
		Eigen::Matrix<float, 3, 1> seeds_mean; // mean ground seeds value
		seeds_mean << x_mean, y_mean, z_mean;
		float d = -(normal.transpose()*seeds_mean)(0,0);
		distance_threshold = distance_threshold - d; // Update Distance threshold

		// Pointcloud --> Matrix
		Eigen::MatrixXf points(original_cloud -> points.size(),3);
		int j = 0;
		for(auto p : original_cloud -> points){
			points.row(j++) << p.x, p.y, p.z;
		}
		// Ground plane model
		Eigen::VectorXf result = points * normal;
		// Road plane threshold filter
		typename pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
		typename pcl::PointCloud<PointT>::Ptr cloud_other(new pcl::PointCloud<PointT>());
		for(int i = 0; i < result.rows(); i++){
			if(result[i] < distance_threshold){
				cloud_plane -> points.push_back(original_cloud->points[i]);
			}
			else{
				cloud_other -> points.push_back(original_cloud->points[i]);
			}
		}
		std::cout << "[Plane Estimation] Plane points: " << cloud_plane->points.size() << ", other points: " 
				<< cloud_other->points.size() << std::endl;
		return std::make_tuple(cloud_plane, cloud_other);
	}
};

/*-------------------------------------------------------------------------------------*/

template<typename PointT>
class Registration{
public:
    // Constructor
    Registration() = default;

    // Destructor
    ~Registration() = default;

	/**
	 * @brief PCL NDT registration
	 * 
	 * @param cloud_source 
	 * @param cloud_target 
	 * @param init_guess 
	 * @param tTransformationEpsilon 
	 * @param StepSize 
	 * @param Resolution 
	 * @param MaxIteration 
	 * @return std::tuple<typename pcl::PointCloud<PointT>::Ptr, Eigen::Matrix4f> 
	 */
	std::tuple<typename pcl::PointCloud<PointT>::Ptr, Eigen::Matrix4f> 
	 NDT_Registration(const typename pcl::PointCloud<PointT>::Ptr &cloud_source,
					  const typename pcl::PointCloud<PointT>::Ptr &cloud_target, 
					  const Eigen::Matrix4f &init_guess,
					  const float &tTransformationEpsilon,
					  const float &StepSize,
					  const float &Resolution,
					  const int &MaxIteration){
		typename pcl::PointCloud<PointT>::Ptr output(new pcl::PointCloud<PointT>);
		typename pcl::NormalDistributionsTransform<PointT, PointT> NDT;
		NDT.setTransformationEpsilon(tTransformationEpsilon);
		NDT.setStepSize(StepSize);           
		NDT.setResolution(Resolution);        
		NDT.setMaximumIterations(MaxIteration);
		NDT.setInputSource(cloud_source); 
		NDT.setInputTarget(cloud_target); 
		NDT.align(*output, init_guess);
		Eigen::Matrix4f SourceToTarget(Eigen::Matrix4f::Identity());
		if(NDT.hasConverged()){
			std::cout << "NDP has converged, score is " << NDT.getFitnessScore () << std::endl;
			std::cout << "Transformation matrix:" << std::endl;
			SourceToTarget = NDT.getFinalTransformation();
			std::cout << SourceToTarget << std::endl;
			pcl::transformPointCloud(*cloud_source, *output, SourceToTarget);
		}
		return std::make_tuple(output, SourceToTarget);
	}
	
	/**
	 * @brief Multi thread NDR registration using OpenMP
	 * 
	 * @param cloud_source 
	 * @param cloud_target 
	 * @param resolution 
	 * @return std::tuple<typename pcl::PointCloud<PointT>::Ptr, Eigen::Matrix4f> 
	 */
	std::tuple<typename pcl::PointCloud<PointT>::Ptr, Eigen::Matrix4f>
	 NDT_OMP(const typename pcl::PointCloud<PointT>::Ptr &cloud_source,
	 		 const typename pcl::PointCloud<PointT>::Ptr &cloud_target, 
			 const Eigen::Matrix4f &init_guess,
			 //const float &tTransformationEpsilon,
			 const float &resolution = 1.0){
		typename pcl::PointCloud<PointT>::Ptr output(new pcl::PointCloud<PointT>);
		typename pclomp::NormalDistributionsTransform<PointT, PointT> ndt_omp;
		ndt_omp.setNumThreads(omp_get_max_threads());
		//ndt_omp.setTransformationEpsilon(tTransformationEpsilon);
		ndt_omp.setResolution(resolution);
		ndt_omp.setNeighborhoodSearchMethod(pclomp::DIRECT7);
		ndt_omp.setInputSource(cloud_source); 
		ndt_omp.setInputTarget(cloud_target); 
		ndt_omp.align(*output, init_guess);
		Eigen::Matrix4f SourceToTarget(Eigen::Matrix4f::Identity());
		if(ndt_omp.hasConverged()){
			std::cout << "NDP-OMP has converged, score is " << ndt_omp.getFitnessScore () << std::endl;
			std::cout << "Transformation matrix:" << std::endl;
			SourceToTarget = ndt_omp.getFinalTransformation();
			std::cout << SourceToTarget << std::endl;
			pcl::transformPointCloud(*cloud_source, *output, SourceToTarget);
		}
		return std::make_tuple(output, SourceToTarget);
	}

	/**
	 * @brief ICP point to point registration
	 * 
	 * @param cloud_source 
	 * @param cloud_target 
	 * @param init_transform 
	 * @param MaxIteration 
	 * @param Epsilon 
	 * @param MaxCorrespondenceDistance 
	 * @return std::tuple<typename pcl::PointCloud<PointT>::Ptr, Eigen::Matrix4f> 
	 */
	std::tuple<typename pcl::PointCloud<PointT>::Ptr, Eigen::Matrix4f> 
 	 ICP_Point2Point( const typename pcl::PointCloud<PointT>::Ptr &cloud_source,
                      const typename pcl::PointCloud<PointT>::Ptr &cloud_target, 
                      const Eigen::Matrix4f &init_transform,
                      const int &MaxIteration,
                      const float &Epsilon,
                      const float &MaxCorrespondenceDistance){
		typename pcl::PointCloud<PointT>::Ptr output(new pcl::PointCloud<PointT>);
		typename pcl::IterativeClosestPoint<PointT, PointT> icp;
		icp.setMaximumIterations(MaxIteration);
		icp.setEuclideanFitnessEpsilon(Epsilon);  // Convergence condition: The smaller the accuracy, the slower the convergence
		icp.setMaxCorrespondenceDistance(MaxCorrespondenceDistance);
		icp.setInputSource(cloud_source);      
		icp.setInputTarget(cloud_target); 
		icp.align(*output, init_transform);  
		Eigen::Matrix4f SourceToTarget (Eigen::Matrix4f::Identity());
		if (icp.hasConverged()){
			std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
			std::cout << "Transformation matrix:" << std::endl;
			SourceToTarget = icp.getFinalTransformation();
			std::cout << SourceToTarget << std::endl;
		}
		return std::make_tuple(output, SourceToTarget);
	}

	std::tuple<typename pcl::PointCloud<PointT>::Ptr, Eigen::Matrix4f> 
	 ICP_OMP(const typename pcl::PointCloud<PointT>::Ptr &cloud_source,
             const typename pcl::PointCloud<PointT>::Ptr &cloud_target, 
             const Eigen::Matrix4f &init_transform){
		typename pcl::PointCloud<PointT>::Ptr output(new pcl::PointCloud<PointT>);
		typename pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp_omp;
		gicp_omp.setInputSource(cloud_source);      
		gicp_omp.setInputTarget(cloud_target); 
		gicp_omp.align(*output, init_transform);  
		Eigen::Matrix4f SourceToTarget (Eigen::Matrix4f::Identity());
		if (gicp_omp.hasConverged()){
			std::cout << "ICP-OMP has converged, score is " << gicp_omp.getFitnessScore () << std::endl;
			std::cout << "Transformation matrix:" << std::endl;
			SourceToTarget = gicp_omp.getFinalTransformation();
			std::cout << SourceToTarget << std::endl;
		}
		return std::make_tuple(output, SourceToTarget);
	}

};

/*-------------------------------------------------------------------------------------*/
template<typename PointT>
class LidarOdometry{
public:
    // Constructor
    LidarOdometry() = default;

    // Destructor
    ~LidarOdometry() = default;

	// void CalibrationInitialize(const Eigen::Matrix4f &gps2lidar,
	// 						   const ){
	// 	gps2lidar(0,0) = 9.999976e-01; 
	// 	gps2lidar(0,1) = 7.553071e-04;
	// 	gps2lidar(0,2) = -2.035826e-03;
	// 	gps2lidar(1,0) = -7.854027e-04;
	// 	gps2lidar(1,1) = 9.998898e-01;
	// 	gps2lidar(1,2) = -1.482298e-02;
	// 	gps2lidar(2,0) = 2.024406e-03;
	// 	gps2lidar(2,1) = 1.482454e-02;
	// 	gps2lidar(2,2) = 9.998881e-01;
	// 	gps2lidar(0,3) = -8.086759e-01;
	// 	gps2lidar(1,3) = 3.195559e-01;
	// 	gps2lidar(2,3) = -7.997231e-01; 

	// 	lidar2gps_trans(0,0) = 9.999976e-01; 
	// 	lidar2gps_trans(0,1) = 7.553071e-04;
	// 	lidar2gps_trans(0,2) = -2.035826e-03;
	// 	lidar2gps_trans(1,0) = -7.854027e-04;
	// 	lidar2gps_trans(1,1) = 9.998898e-01;
	// 	lidar2gps_trans(1,2) = -1.482298e-02;
	// 	lidar2gps_trans(2,0) = 2.024406e-03;
	// 	lidar2gps_trans(2,1) = 1.482454e-02;
	// 	lidar2gps_trans(2,2) = 9.998881e-01;
	// 	lidar2gps_trans(1,3) = 3.195559e-01;
	// }

	/**
	 * @brief Calculate quaternion with euler angles
	 * 
	 * @param rollAngle 
	 * @param pitchAngle 
	 * @param yawAngle 
	 * @return Eigen::Matrix<float, 4, 1> 
	 */
	Eigen::Matrix<float, 4, 1> 
 	 Euler2Quaternion(const float &rollAngle, const float &pitchAngle, const float &yawAngle){
		auto q1 = (float) cos(0.5 * rollAngle) * cos(0.5 * pitchAngle) * cos(0.5 * yawAngle) + (float) sin(0.5 * rollAngle) * sin(0.5 * pitchAngle) * sin(0.5 * yawAngle); 
		auto q2 = (float) sin(0.5 * rollAngle) * cos(0.5 * pitchAngle) * cos(0.5 * yawAngle) - (float) cos(0.5 * rollAngle) * sin(0.5 * pitchAngle) * sin(0.5 * yawAngle); 
		auto q3 = (float) cos(0.5 * rollAngle) * sin(0.5 * pitchAngle) * cos(0.5 * yawAngle) + (float) sin(0.5 * rollAngle) * cos(0.5 * pitchAngle) * sin(0.5 * yawAngle); 
		auto q4 = (float) cos(0.5 * rollAngle) * cos(0.5 * pitchAngle) * sin(0.5 * yawAngle) - (float) sin(0.5 * rollAngle) * sin(0.5 * pitchAngle) * cos(0.5 * yawAngle); 
		Eigen::Matrix<float, 4, 1> quaternion;
		quaternion << q1, q2, q3, q4;
		return quaternion;
	}	

	/**
	 * @brief Calculate rotation matrix with quaternion
	 * 
	 * @param q Input quaternion
	 * @return Eigen::Matrix<float, 3, 3> 
	 */
	Eigen::Matrix<float, 3, 3> 
	 Quaternion2Rotation(const Eigen::Matrix4f &q){
		auto q0(q(0, 0));
		auto q1(q(1, 0));
		auto q2(q(2, 0));
		auto q3(q(3, 0));
		Eigen::Matrix<float, 3, 3> R;
		R << q0*q0 + q1*q1 - q2*q2 - q3*q3,  2*(q1*q2 - q0*q3),  2*(q0*q2 + q1*q3),
			 2*(q1*q2 + q0*q3), q0*q0 - q1*q1 + q2*q2 - q3*q3, 2*(q2*q3 - q0*q1),
			 2*(q1*q3 - q0*q2), 2*(q0*q1 + q2*q3),  q0*q0 - q1*q1 - q2*q2 + q3*q3;
		return R;
	}

	/**
	 * @brief Get the Transform Matrix object
	 * 
	 * @param R Input rotation matrix
	 * @param delta_X Translation matrix X
	 * @param delta_Y Translation matrix Y
	 * @param delta_Z Translation matrix Z
	 * @return Eigen::Matrix<float, 4, 4> 
	 */
	Eigen::Matrix<float, 4, 4> 
     GetTransformMatrix(const Eigen::Matrix3f & R, 
	 					const float &delta_X, 
						const float &delta_Y, 
						const float &delta_Z){
		Eigen::Matrix<float, 4, 4> T(Eigen::Matrix4f::Zero(4, 4));
		T << R(0,0), R(0,1), R(0,2), delta_X,
			 R(1,0), R(1,1), R(1,2), delta_Y,
			 R(2,0), R(2,1), R(2,2), delta_Z,
			 0, 0, 0, 1;
		return T;
	}

	/**
	 * @brief convert transformation matrix to 6-Dof state transformation
	 * 
	 * @param R Rotation of the transformation matrix
	 * @return std::vector<float> 
	 */
	std::vector<float>
	 Transformmatrix_to_states(const Eigen::Matrix4f &R) {
		float epsilon = sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));
		bool singular = epsilon < 1e-6;
		// Occurs if pitch is close to +/- 90 degrees
		if (!singular) {
			// states order x,y,z,pitch,roll,yaw
			std::vector<float> states(6);
			states[0] = R(0, 3);
			states[1] = R(1, 3);
			states[2] = R(2, 3);
			states[3] = asinf(-R(2, 0));
			states[4] = atan2(R(2, 1), R(2, 2));
			states[5] = atan2(R(1, 0), R(0, 0));
			return states;
		} 
		else { 
			// states order x,y,z,pitch,roll,yaw
			std::vector<float> states(6);
			states[0] = R(0, 3);
			states[1] = R(1, 3);
			states[2] = R(2, 3);
			states[3] = asinf(-R(2, 0));
			states[4] = atan2(R(2, 1), R(2, 2));
			states[5] = 0;
			return states;
		}
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
		const int distance = 100;
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
#endif /*PROCESSING_HPP*/