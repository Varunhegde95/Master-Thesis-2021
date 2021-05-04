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
        (0 == commandlineArguments.count("name_read")) ||
        (0 == commandlineArguments.count("name_send")) ) {
        std::cerr << "         --cid:     CID of the OD4Session to replay other Envelopes" << std::endl;
        std::cerr << "         --freq:    Frequency to send out the KITTI data [IMU, GPS, Lidar]" << std::endl;
        std::cerr << "         --name_read:    name of the shared memory area to create" << std::endl;
        std::cerr << "         --verbose: print decoding information and display image" << std::endl;
        std::cerr << "         --id-sender: sender id of output messages" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=111 --name=data --verbose --id-sender=100" << std::endl;
    }
    else {
        const std::string NAME_READ{commandlineArguments["name_read"]};
        const std::string NAME_SEND{commandlineArguments["name_send"]};
        float const FREQ{std::stof(commandlineArguments["freq"])}; // Frequency
        const bool DISPLAY{commandlineArguments.count("display") != 0};
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};
        uint32_t const IDSENDER{(commandlineArguments["id-sender"].size() != 0)               
            ? static_cast<uint32_t>(std::stoi(commandlineArguments["id-sender"])) : 0};

        Filters<pcl::PointXYZ> filter;
        Segmentation<pcl::PointXYZ> segmentation;
        Visual<pcl::PointXYZ> visual;
        pcl::visualization::PCLVisualizer viewer("PCD Viewer"); // Initialize PCD viewer
        CameraAngle camera_angle = TOP; // Set viewercamera angle
        visual.initCamera(viewer, BLACK, camera_angle); // Initialize PCL viewer

        // Interface to a running OpenDaVINCI session (ignoring any incoming Envelopes).
        cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
        
        std::cout << "Connecting to shared memory " << NAME_READ << std::endl;
        std::unique_ptr<cluon::SharedMemory> shmRead{new cluon::SharedMemory{NAME_READ}};
        std::unique_ptr<cluon::SharedMemory> shmSend(nullptr); // Create shared memory
        if(!shmSend){
            shmSend.reset(new cluon::SharedMemory{NAME_SEND, shmRead->size()/2});
            std::cout << "Set shared memory: " << shmSend->name() << " (" << shmRead->size() 
            << " bytes)." << std::endl;
        }

        if (shmRead && shmRead->valid() && shmSend) {
            std::clog << argv[0] << ": Attached to shared ARGB memory '" 
            << shmRead->name() << " (" << shmRead->size() 
            << " bytes)." << std::endl;
        
            int16_t NUM = 0;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCL(new pcl::PointCloud<pcl::PointXYZ>);
            cloudPCL->resize(shmRead->size()/15);

            while(od4.isRunning()){
                auto frame_timer = std::chrono::system_clock::now();
                shmRead->wait();
                shmRead->lock();
                memcpy(&(cloudPCL->points[0]), shmRead->data(), shmRead->size());
                if(DISPLAY){
                    viewer.removeAllPointClouds();
                }
                if(VERBOSE)
                    std::cout << "Read shared memory PCD [" << NUM << "], size: " << cloudPCL->points.size() << std::endl;
                shmRead->unlock();

                /*------ 1. Down Sampling ------*/
                auto cloud_down = filter.VoxelGridDownSampling(cloudPCL, 0.5f);

                /*------ 2. Crop Box Filter [Remove roof] ------*/
                const Eigen::Vector4f min_point(-40, -25, -3, 1);
                const Eigen::Vector4f max_point(40, 25, 4, 1);
                cloud_down = filter.boxFilter(cloud_down, min_point, max_point); //Distance Crop Box

                const Eigen::Vector4f roof_min(-1.5, -1.7, -1, 1);
                const Eigen::Vector4f roof_max(2.6, 1.7, -0.4, 1);
                cloud_down = filter.boxFilter(cloud_down, roof_min, roof_max, true); // Remove roof outliers

                /*------ 3. Statistical Outlier Removal ------*/
                cloud_down = filter.StatisticalOutlierRemoval(cloud_down, 30, 2.0);

                /*------ 4. Plane Segmentation ------*/
                const float SENSOR_HEIGHT = 2;
                std::sort(cloud_down->points.begin(),cloud_down->points.end(),point_cmp); // Resort points in Z axis
                auto cloud_down_sorted = filter.PassThroughFilter(cloud_down, "z", std::array<float, 2> {-SENSOR_HEIGHT-0.1, 0.5f}); // 'Z' Pass filter
                auto RoughGroundPoints = segmentation.RoughGroundExtraction(cloud_down, 1.0, 60);
                // RANSAC Segmentation
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_road(new pcl::PointCloud<pcl::PointXYZ>());
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_other(new pcl::PointCloud<pcl::PointXYZ>());
                // std::tie(cloud_road, cloud_other) = segmentation.PlaneSegmentationRANSAC(cloud_down, RoughGroundPoints, 150, 0.2);
                std::tie(cloud_road, cloud_other) = segmentation.PlaneEstimation(cloud_down, RoughGroundPoints, 0.3f);

                // Save to the shared memory
                std::cout <<  "Saving processed pcd to shared memory.." << std::endl;
                uint32_t sizecloud = sizeof(cloud_other) + cloud_other->points.size()+300;
                cluon::data::TimeStamp ts = cluon::time::now();
                shmSend->lock();
                shmSend->setTimeStamp(ts);
                memcpy(shmSend->data(), static_cast<void const*>(cloud_other.get()), shmSend->size());
                shmSend->unlock();
	            shmSend->notifyAll();
                std::cout << "Save PCD [" << NUM << "] to shared memory" << ", PCD point size: " << cloud_other->points.size() << std::endl;

                /*------ Visualization ------*/
                if (VERBOSE){
                    std::cout << "Frame (" << NUM << "), ";
                    timerCalculator(frame_timer, "Every Frame");
                }
                if(DISPLAY == true){
                    visual.showPointcloud(viewer, cloud_other, 2, GREEN, "PCD other");
                    visual.showPointcloud(viewer, cloud_road, 2, RED, "PCD road");
                    viewer.spinOnce();
                }
                NUM ++;
            }
        }
        retCode = 0;
    }
    return retCode;
}