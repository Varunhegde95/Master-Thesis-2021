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

#include "registration.hpp"

int32_t main(int32_t argc, char **argv) {
    int32_t retCode{1};
    const std::string PROGRAM{argv[0]};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ((0 == commandlineArguments.count("cid")) ||
        (0 == commandlineArguments.count("freq")) ) {
        std::cerr << "         --cid:     CID of the OD4Session to replay other Envelopes" << std::endl;
        std::cerr << "         --freq:    Frequency to send out the KITTI data [IMU, GPS, Lidar]" << std::endl;
        std::cerr << "         --name:    name of the shared memory area to create" << std::endl;
        std::cerr << "         --verbose: print decoding information and display image" << std::endl;
        std::cerr << "         --id-sender: sender id of output messages" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=111 --name=data --verbose --id-sender=100" << std::endl;
    }
    else {
        const std::string NAME{commandlineArguments["name_processed"]};
        float const FREQ{std::stof(commandlineArguments["freq"])}; // Frequency
        const bool DISPLAY{commandlineArguments.count("display") != 0};
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};
        
        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        // Interface to a running OpenDaVINCI session (ignoring any incoming Envelopes).
        cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
        
        std::cout << "Connecting to shared memory " << NAME << std::endl;
        std::unique_ptr<cluon::SharedMemory> shmCloud{new cluon::SharedMemory{NAME}};

        Visual<pcl::PointXYZ> visual;
        Filters<pcl::PointXYZ> filter;
        pcl::visualization::PCLVisualizer viewer("PCD Registration"); // Initialize PCD viewer
        CameraAngle camera_angle = TOP; // Set viewercamera angle
        visual.initCamera(viewer, BLACK, camera_angle); // Initialize PCL viewer

        if (shmCloud && shmCloud->valid() ) {
            std::clog << argv[0] << ": Attached to shared PCD memory '" 
            << shmCloud->name() << " (" << shmCloud->size() 
            << " bytes)." << std::endl;

            int16_t NUM = 0;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            cloud->resize(shmCloud->size()/15);

            while(od4.isRunning()){
                auto frame_timer = std::chrono::system_clock::now();
                shmCloud->wait();
                shmCloud->lock();
                memcpy(&(cloud->points[0]), shmCloud->data(), shmCloud->size());
                if(VERBOSE)
                    std::cout << "Read shared memory PCD [" << NUM << "], size: " << cloud->points.size() << std::endl;
                shmCloud->unlock();
     
                /*-------------------------------------------------------------------------------------*/
                //auto cloud_down = filter.VoxelGridDownSampling(cloud, 0.6f);

                /*-------------------------------------------------------------------------------------*/
                /*------ Visualization ------*/
                if (VERBOSE){
                    std::cout << "Frame (" << NUM << "), ";
                    timerCalculator(frame_timer, "Every Frame");
                }
                if(DISPLAY){
                    viewer.removeAllPointClouds();
                    visual.showPointcloud(viewer, cloud, 2, WHITE, "PCD Registration");
                    viewer.spinOnce();
                }
                NUM ++;
            }
        }
        retCode = 0;
    }
    return retCode;
}