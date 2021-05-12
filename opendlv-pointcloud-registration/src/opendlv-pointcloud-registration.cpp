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
        Registration<pcl::PointXYZ> registration;

        // Set up visualization
        pcl::visualization::PCLVisualizer viewer("PCD Registration"); // Initialize PCD viewer
        CameraAngle camera_angle = TOP; // Set viewercamera angle
        visual.initCamera(viewer, BLACK, camera_angle); // Initialize PCL viewer

        if (shmCloud && shmCloud->valid() ) {
            std::clog << argv[0] << ": Attached to shared PCD memory '" 
            << shmCloud->name() << " (" << shmCloud->size() 
            << " bytes)." << std::endl;

            int16_t NUM = 0;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_read(new pcl::PointCloud<pcl::PointXYZ>);         // Pointcloud read from shared memory
            cloud_read->resize(shmCloud->size()/16);

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_previous(new pcl::PointCloud<pcl::PointXYZ>);     // Point cloud previous frame
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_now(new pcl::PointCloud<pcl::PointXYZ>);          // Point cloud this frame (now)
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_NDT(new pcl::PointCloud<pcl::PointXYZ>);          // NDT Registrated point cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ICP(new pcl::PointCloud<pcl::PointXYZ>);          // ICP Registrated point cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ICP_output(new pcl::PointCloud<pcl::PointXYZ>);   // ICP Registrated point cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_global_trans(new pcl::PointCloud<pcl::PointXYZ>); // Aligned cloud transfered into global coordinates
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZ>);        // Final result 

            Eigen::Matrix4f initial_guess_transMatrix = Eigen::Matrix4f::Identity (); // NDT initial guess
            Eigen::Matrix4f NDT_transMatrix = Eigen::Matrix4f::Identity ();           // NDT transformation
            Eigen::Matrix4f ICP_transMatrix = Eigen::Matrix4f::Identity ();
            Eigen::Matrix4f global_transMatrix = Eigen::Matrix4f::Identity ();
        

            while(od4.isRunning()){
                shmCloud->wait();
                shmCloud->lock();
                memcpy(&(cloud_read->points[0]), shmCloud->data(), shmCloud->size());
                if(VERBOSE)
                    std::cout << "Read shared memory PCD [" << NUM << "], size: " << cloud_read->points.size() << std::endl;
                shmCloud->unlock();
                // Statistical Outlier Removal
                //auto cloud_down = filter.StatisticalOutlierRemoval(cloud_read, 40, 2.0);
                // auto cloud_down = filter.RandomSampling(cloud_read, 6500);

                /*----- TEST -----*/
                auto cloud_valid = filter.InvalidPointsRemoval(cloud_read);
                auto cloud_down = filter.PointComplement(cloud_valid, 6500);
               // auto cloud_down = filter.VoxelGridDownSampling(cloud_valid, 0.5f);

                /*-------------------------------REGISTRATION------------------------------------------*/
                auto frame_timer = std::chrono::system_clock::now();
                if(NUM == 0){
                    std::cout << "Frame [" << NUM << "]: Set up <cloud_previous>." << std::endl;
                    *cloud_previous = *cloud_down;
                    *cloud_final += *cloud_previous;
                }
                else{
                    std::cout << "Registration start ..." << std::endl;
                    *cloud_now = *cloud_down;   

                    /*-------- 1. NDT registration --------*/
                    std::cout << "NDT Registration" << std::endl;
                    // auto timer_NDT = std::chrono::system_clock::now(); // Start NDT timer
                    std::tie(cloud_NDT, NDT_transMatrix) = registration.NDT_Registration(cloud_previous, cloud_now, initial_guess_transMatrix, 1e-2, 0.5, 1.0, 10);
                    // timerCalculator(timer_NDT, "NDT registration"); // Print time

                    /*-------- 2. ICP registration --------*/
                    // std::cout << "ICP Registration" << std::endl;
                    // auto timer_ICP = std::chrono::system_clock::now(); // Start ICP timer
                    // std::tie(cloud_ICP, ICP_transMatrix) = registration.ICP_Point2Point(cloud_NDT, cloud_now, NDT_transMatrix, 100, 1e-7, 0.6);
                    // pcl::transformPointCloud (*cloud_now, *cloud_ICP_output, ICP_transMatrix.inverse() * NDT_transMatrix.inverse());
                    // timerCalculator(timer_ICP, "ICP registration"); // Print time

                    /*-------- 3. Transfer aligned cloud into global coordinate --------*/
                    // pcl::transformPointCloud (*cloud_ICP_output, *cloud_global_trans, global_transMatrix);
                    // global_transMatrix = global_transMatrix * ICP_transMatrix.inverse()*NDT_transMatrix.inverse();
                    // std::cout << "Global Transform Matrix:\n" << global_transMatrix << std::endl;

                    /*-------- 4. Stitch aligned clouds --------*/
                    // *cloud_final += *cloud_global_trans;
                    *cloud_previous = *cloud_now;
                }
                /*-------------------------------------------------------------------------------------*/
                /*------ Visualization ------*/
                if (VERBOSE){
                    std::cout << "Frame (" << NUM << "), ";
                    timerCalculator(frame_timer, "Every Frame");
                }
                if(DISPLAY){
                    viewer.removeAllPointClouds();
                    visual.showPointcloud(viewer, cloud_NDT, 2, WHITE, "PCD Registration");
                    viewer.spinOnce();
                }
                NUM ++;
            }
        }
        retCode = 0;
    }
    return retCode;
}