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

// PCL Library
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>

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

// Load KITTI point cloud data
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

#endif /*KITTI_REPLAY_HPP*/