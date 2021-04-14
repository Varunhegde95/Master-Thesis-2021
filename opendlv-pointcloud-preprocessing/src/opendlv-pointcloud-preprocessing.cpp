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
        (0 == commandlineArguments.count("freq")) ) {
        std::cerr << "         --cid:     CID of the OD4Session to replay other Envelopes" << std::endl;
        std::cerr << "         --freq:    Frequency to send out the KITTI data [IMU, GPS, Lidar]" << std::endl;
        std::cerr << "         --name:    name of the shared memory area to create" << std::endl;
        std::cerr << "         --verbose: print decoding information and display image" << std::endl;
        std::cerr << "         --id-sender: sender id of output messages" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=111 --name=data --verbose --id-sender=100" << std::endl;
    }
    else {
        float const FREQ{std::stof(commandlineArguments["freq"])}; // Frequency
        const bool DISPLAY{commandlineArguments.count("display") != 0};
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};
        uint32_t const IDSENDER{(commandlineArguments["id-sender"].size() != 0)               
            ? static_cast<uint32_t>(std::stoi(commandlineArguments["id-sender"])) : 0};

        Filters<pcl::PointXYZ> filter;
        Visual<pcl::PointXYZ> visual;
        pcl::visualization::PCLVisualizer viewer("PCD Viewer"); // Initialize PCD viewer
        CameraAngle camera_angle = TOP; // Set camera angle
        initCamera(viewer, BLACK, camera_angle); // Init viewer

        // Interface to a running OpenDaVINCI session (ignoring any incoming Envelopes).
        cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

        std::string kittiPath;
        for (auto e : commandlineArguments) {
            if (e.second.empty() && e.first != PROGRAM) {
                kittiPath = e.first;
                break;
            }
        }
        std::string lidarPath = kittiPath + "velodyne_points/data/";
        std::vector<std::string> files_lidar;
        int16_t fileNum;
        std::tie(files_lidar, fileNum) = loadFile(lidarPath, !VERBOSE);

        // Initialize handler to receive data 
        std::mutex GpsReadingMutex;       // GPS Data [wgs84] Reading Mutex
        float lon{0.0f};
        float lat{0.0f};

        auto onGpsReading{[&GpsReadingMutex, &lon, &lat](cluon::data::Envelope &&envelope) {
            auto msg = cluon::extractMessage<opendlv::logic::sensation::Geolocation>(std::move(envelope));
            std::lock_guard<std::mutex> lck(GpsReadingMutex);
            lon = msg.longitude();   
            lat = msg.latitude();  
        }};

        od4.dataTrigger(opendlv::logic::sensation::Geolocation::ID(), onGpsReading);

        int16_t NUM = 0;
        auto atFrequency{[&DISPLAY, &VERBOSE, &od4, &lon, &lat, &files_lidar, &filter, &visual, &viewer, &IDSENDER, &NUM]() -> bool{
            if(lon == 0 && lat == 0){
                NUM = 0;
                std::cout << "Waiting to receive data ..." << std::endl;
            }
            else{
                if(DISPLAY == true){ 
                    viewer.removeAllPointClouds(); // Clear viewer
                }   
                auto cloud = loadKitti(files_lidar, NUM);
                // auto cloud_down = filter.VoxelGridDownSampling(cloud, 0.3f);// DownSampling


                if (VERBOSE){
                    std::cout << "Frame (" << NUM << ")" << std::endl;
                }
                /*------ Visualization ------*/
                if(DISPLAY == true){
                    viewer.addCoordinateSystem(1.0);
                    visual.showPointcloud(viewer, cloud, 2, WHITE, "PCD");
                    
                }
                NUM ++;
            }
        }};

        while(NUM != fileNum && od4.isRunning()){
            od4.timeTrigger(FREQ, atFrequency);
        }
        retCode = 0;
    }
    return retCode;
}