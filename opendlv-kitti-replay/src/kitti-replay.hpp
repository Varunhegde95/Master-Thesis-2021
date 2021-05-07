#ifndef KITTI_REPLAY_HPP
#define KITTI_REPLAY_HPP

#include <iostream>
#include <fstream>
#include <thread>
#include <cstdint>
#include <cstdlib>
#include <chrono>
#include <cstring>
#include <dirent.h>
#include <string>
#include <sys/shm.h>

// PCL Library
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/common/copy_point.h>

// Filters
#include <pcl/filters/passthrough.h>                 // PassThrough Filter
#include <pcl/filters/voxel_grid.h>                  // VoxelGrid Down Sampling
#include <pcl/filters/statistical_outlier_removal.h> // Statistical Outlier Removal
#include <pcl/filters/radius_outlier_removal.h>      // Radius Outlier Removal
#include <pcl/filters/extract_indices.h>             // Extract pointCloud according to indices
#include <pcl/filters/crop_box.h>
#include <pcl/filters/random_sample.h>

class Oxts_Data{
    public:
        Oxts_Data(){};
        double lat;   // ** GPS ** latitude of the oxts  - unit [deg]    
        double lon;   // ** GPS ** longitude of the oxts - unit [deg]
        double alt;   // ** GPS ** altitude of the oxts  - unit [m]

        double roll;  // roll angle(rad), 0 = level, positive = left side up, range :  [-pi .. + pi]
        double pitch; // pitch angle(rad), 0 = level, positive = front down, range :   [-pi/2 .. + pi/2]
        double yaw;   // heading(rad), 0 = east, positive = counter clockwise, range : [-pi .. + pi]

        double vn; // velocity towards north [m/s]
        double ve; // velocity towards east [m/s]
        double vf; // forward velocity, i.e.parallel to earth - surface [m/s]
        double vl; // leftward velocity, i.e.parallel to earth - surface [m/s]
        double vu; // upward velocity, i.e.perpendicular to earth - surface [m/s]

        double ax; // acceleration in x, i.e.in direction of vehicle front [m/s^2]
        double ay; // acceleration in y, i.e.in direction of vehicle left [m/s^2]
        double az; // acceleration in z, i.e.in direction of vehicle top [m/s^2]
        double af; // forward acceleration [m/s^2]
        double al; // leftward acceleration [m/s^2]
        double au; // upward acceleration [m/s^2]

        double wx; // angular rate around x [rad/s]
        double wy; // angular rate around y [rad/s]
        double wz; // angular rate around z [rad/s]
        double wf; // angular rate around forward axis [rad/s]
        double wl; // angular rate around leftward axis [rad/s]
        double wu; // angular rate around upward axis [rad/s]
};

/**
 * @brief Load KITTI point cloud data file path
 * 
 * @param folderPath KITTI data folder path
 * @param VERBOSE Whether to show in terminal
 * @return File paths and total numbers 
 */
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
 * @brief Load KITTI IMU & GPS data from oxts folder
 * 
 * @param filePaths oxts file path
 * @param NUM Load file[NUM] in the file path
 * @return Oxts_Data 
 */
Oxts_Data
 loadOxts (const std::vector<std::string> &filePaths, 
                   const int16_t &NUM){
    Oxts_Data oxts_data;
    std::fstream fin;
    fin.open(filePaths[NUM], std::ios::in);
    if(fin.is_open()){
        fin >> oxts_data.lat  >> oxts_data.lon   >> oxts_data.alt;
        fin >> oxts_data.roll >> oxts_data.pitch >> oxts_data.yaw;
	    fin >> oxts_data.vn   >> oxts_data.ve;

        fin >> oxts_data.vf >> oxts_data.vl >> oxts_data.vu;
	    fin >> oxts_data.ax >> oxts_data.ay >> oxts_data.az;
	    fin >> oxts_data.af >> oxts_data.al >> oxts_data.au;
	    fin >> oxts_data.wx >> oxts_data.wy >> oxts_data.wz;
	    fin >> oxts_data.wf >> oxts_data.wl >> oxts_data.wu;
        //std::cout << "Load file: [" << filePaths[NUM] << "]." << std::endl;
        return oxts_data;
    }
    else
        std::cout << "Unable to load file ..." << std::endl;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
 loadKitti (const std::vector<std::string> &filePaths, 
                          const int16_t &NUM){
    pcl::PointCloud<pcl::PointXYZI>::Ptr points (new pcl::PointCloud<pcl::PointXYZI>);
    typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
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
    //std::cout << "Load file: [" << filePaths[NUM] << "]." << std::endl;
    return cloud;
}

/**
 * @brief Calculate time consumption
 * 
 * @param start_time Initialize the timer start time 
 * @param function Function name to print
 */
void 
 timerCalculator (const std::chrono::_V2::system_clock::time_point &start_time,
                                const std::string &function){
    // Should use "auto start_fast = std::chrono::system_clock::now()" to start timer.
    auto end_time = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    double time_passed = (double) duration.count() * 
            std::chrono::microseconds::period::num / std::chrono::microseconds::period::den;
    std::cout << "[--Timer--] " << function <<" time used: " << time_passed << " [s]." << std::endl;
    //return time_passed; // [seconds]
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
	 * @brief Statistical Outlier Removal
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
};

/*-------------------------------------------------------------------------------------*/

#endif /*KITTI_REPLAY_HPP*/