/*
 * Copyright (C) 2021  Liangyu Wang
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include "preprocessing.hpp"

int32_t main(int32_t argc, char **argv) {
    int32_t retCode{1};
    const std::string PROGRAM{argv[0]};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ((0 == commandlineArguments.count("cid")) ||
        (0 == commandlineArguments.count("freq")) ||
        (0 == commandlineArguments.count("name_read")) ) {
        std::cerr << "         --cid:     CID of the OD4Session to replay other Envelopes" << std::endl;
        std::cerr << "         --freq:    Frequency to send out the KITTI data [IMU, GPS, Lidar]" << std::endl;
        std::cerr << "         --name_read:    name of the shared memory area to create" << std::endl;
        std::cerr << "         --verbose: print decoding information and display image" << std::endl;
        std::cerr << "         --id-sender: sender id of output messages" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=111 --name=data --verbose --id-sender=100" << std::endl;
    }
    else {
        const std::string NAME_READ{commandlineArguments["name_read"]};
        float const FREQ{std::stof(commandlineArguments["freq"])}; // Frequency
        const bool DISPLAY{commandlineArguments.count("display") != 0};
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};

        Filters<pcl::PointXYZ> filter;
        Segmentation<pcl::PointXYZ> segmentation;
        Visual<pcl::PointXYZ> visual;
        Registration<pcl::PointXYZ> registration;

        pcl::visualization::PCLVisualizer viewer("PCD Preprocessing"); // Initialize PCD viewer
        CameraAngle camera_angle = TOP; // Set viewercamera angle
        if(DISPLAY)
            visual.initCamera(viewer, BLACK, camera_angle); // Initialize PCL viewer

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_previous(new pcl::PointCloud<pcl::PointXYZ>);     // Point cloud previous frame
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_now(new pcl::PointCloud<pcl::PointXYZ>);          // Point cloud this frame (now)
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_NDT(new pcl::PointCloud<pcl::PointXYZ>);          // NDT Registrated point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ICP(new pcl::PointCloud<pcl::PointXYZ>);          // ICP Registrated point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ICP_output(new pcl::PointCloud<pcl::PointXYZ>);   // ICP Registrated point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_global_trans(new pcl::PointCloud<pcl::PointXYZ>); // Aligned cloud transfered into global coordinates
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZ>);        // Final result 

        Eigen::Matrix4f initial_guess_transMatrix (Eigen::Matrix4f::Identity()); // NDT initial guess
        Eigen::Matrix4f NDT_transMatrix (Eigen::Matrix4f::Identity());           // NDT transformation
        Eigen::Matrix4f ICP_transMatrix (Eigen::Matrix4f::Identity());
        Eigen::Matrix4f global_transMatrix (Eigen::Matrix4f::Identity());
            
        //Interface to a running OpenDaVINCI session (ignoring any incoming Envelopes).
        cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
        
        std::cout << "Connecting to shared memory " << NAME_READ << std::endl;
        std::unique_ptr<cluon::SharedMemory> shmRead{new cluon::SharedMemory{NAME_READ}};

        if (shmRead && shmRead->valid() ) {
            std::clog << argv[0] << ": Attached to shared PCD memory '" 
            << shmRead->name() << " (" << shmRead->size() 
            << " bytes)." << std::endl;
        
            int16_t NUM = 0;
            int16_t overtime_count = 0; // Count the times of over-time 
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCL(new pcl::PointCloud<pcl::PointXYZ>);
            cloudPCL->resize(shmRead->size()/16);

            while(od4.isRunning()){
                shmRead->wait();
                shmRead->lock();
                memcpy(&(cloudPCL->points[0]), shmRead->data(), shmRead->size());
                shmRead->unlock();
                if(VERBOSE)
                    std::cout << "Read shared memory PCD [" << NUM << "], size: " << cloudPCL->points.size() << std::endl;
                auto cloud_valid = filter.InvalidPointsRemoval(cloudPCL); // Remove invalid points

                auto frame_timer = std::chrono::system_clock::now();

                /*------ 1. Down Sampling ------*/
                auto cloud_down = filter.RandomSampling(cloud_valid, (uint32_t) cloudPCL->points.size()*0.25);  // Down sampling the reading pointcloud

                /*------ 2. Crop Box Filter [Remove roof] ------*/
                std::cout << "[Crop box filter] Original points: " << cloud_down->points.size();
                const Eigen::Vector4f min_point(-40, -25, -1, 1);
                const Eigen::Vector4f max_point(40, 25, 4, 1);
                cloud_down = filter.boxFilter(cloud_down, min_point, max_point); //Distance Crop Box

                const Eigen::Vector4f roof_min(-1.5, -1.8, -1, 1);
                const Eigen::Vector4f roof_max(2.6, 1.8, -0.4, 1);
                cloud_down = filter.boxFilter(cloud_down, roof_min, roof_max, true); // Remove roof outliers
                std::cout << ", Filtered points: " << cloud_down->points.size() << std::endl;
                
                /*------ 3. Statistical Outlier Removal ------*/
                cloud_down = filter.StatisticalOutlierRemoval(cloud_down, 30, 2.0);

                /*------ 4. Plane Segmentation ------*/
                const float SENSOR_HEIGHT = 2;
                std::sort(cloud_down->points.begin(),cloud_down->points.end(),point_cmp); // Resort points in Z axis
                auto cloud_down_sorted = filter.PassThroughFilter(cloud_down, "z", std::array<float, 2> {-SENSOR_HEIGHT-0.1, 0.5f}); // 'Z' Pass filter
                auto RoughGroundPoints = segmentation.RoughGroundExtraction(cloud_down, 1.0, 70);
                // RANSAC Segmentation
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_road(new pcl::PointCloud<pcl::PointXYZ>());
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_other(new pcl::PointCloud<pcl::PointXYZ>());
                auto plane_timer = std::chrono::system_clock::now();
                // std::tie(cloud_road, cloud_other) = segmentation.PlaneSegmentationRANSAC(cloud_down, RoughGroundPoints, 150, 0.2);
                std::tie(cloud_road, cloud_other) = segmentation.PlaneEstimation(cloud_down, RoughGroundPoints, 0.3f); // SVD method
                timerCalculator(plane_timer, "Plane segmentation");
                
                /*------ 5. Registration ------*/
                if(NUM == 0){
                    std::cout << "Frame [" << NUM << "]: Set up <cloud_previous>." << std::endl;
                    *cloud_previous = *cloud_other;
                    *cloud_final += *cloud_previous;
                }
                else{
                     std::cout << "Registration start ..." << std::endl;
                    *cloud_now = *cloud_other;   

                    /*----- [1] NDT Registration -----*/
                    //std::cout << "NDT Registration" << std::endl;
                    auto timer_registration = std::chrono::system_clock::now(); // Start NDT timer
                    std::tie(cloud_NDT, NDT_transMatrix) = registration.NDT_Registration(cloud_previous, cloud_now, initial_guess_transMatrix, 1e-2, 0.2, 2.0, 10);
                    
                    /*----- [2] ICP Registration -----*/
                    std::tie(cloud_ICP, ICP_transMatrix) = registration.ICP_Point2Point(cloud_NDT, cloud_now, NDT_transMatrix, 100, 1e-6, 0.6);
                    pcl::transformPointCloud (*cloud_now, *cloud_ICP_output, ICP_transMatrix.inverse() * NDT_transMatrix.inverse());

                    /*-------- 3. Transfer aligned cloud into global coordinate --------*/
                    pcl::transformPointCloud (*cloud_ICP_output, *cloud_global_trans, global_transMatrix);
                    global_transMatrix = global_transMatrix * ICP_transMatrix.inverse() * NDT_transMatrix.inverse();

                    /*-------- 4. Stitch aligned clouds --------*/
                    *cloud_final += *cloud_global_trans;
                    *cloud_previous = *cloud_now;

                    auto registration_time = timerCalculator(timer_registration, "Registration"); // Print time
                    if(registration_time > 0.2)
                        overtime_count ++;
                }

                /*------ 6. Visualization ------*/
                if(VERBOSE){
                    std::cout << "Frame (" << NUM << "), ";
                    timerCalculator(frame_timer, "Every Frame");
                    std::cout << "Registration over-time happens: " << overtime_count << std::endl;
                }
                if(DISPLAY){
                    viewer.removeAllPointClouds();
                    visual.showPointcloud(viewer, cloud_final, 2, GREEN, "PCD Preprocessing");
                    //visual.showPointcloud(viewer, cloud_road, 2, RED, "PCD road");
                    viewer.spinOnce();
                }
                NUM ++;
            }
        }
        retCode = 0;
    }
    return retCode;
}