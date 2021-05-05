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