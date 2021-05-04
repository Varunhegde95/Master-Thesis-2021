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

#include "kitti-replay.hpp"

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
        const std::string NAME{commandlineArguments["name"]};
        float const FREQ{std::stof(commandlineArguments["freq"])}; // Frequency
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};
        uint32_t const IDSENDER{(commandlineArguments["id-sender"].size() != 0)               
            ? static_cast<uint32_t>(std::stoi(commandlineArguments["id-sender"])) : 0};

        // Interface to a running OpenDaVINCI session (ignoring any incoming Envelopes).
        cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
        
        std::unique_ptr<cluon::SharedMemory> sharedMemory(nullptr); // Create shared memory

        std::string kittiPath;
        for (auto e : commandlineArguments) {
            if (e.second.empty() && e.first != PROGRAM) {
                kittiPath = e.first;
                break;
            }
        }
        
        if(kittiPath.empty())
            std::cerr << argv[0] << ": Failed to open '" << kittiPath << "'" << std::endl;
        else{
            std::string oxtsPath  = kittiPath + "oxts/data/";
            std::string lidarPath = kittiPath + "velodyne_points/data/";
            std::vector<std::string> files_oxts;
            std::vector<std::string> files_lidar;
            int16_t fileNum;
            std::tie(files_oxts, fileNum) = loadFile(oxtsPath, VERBOSE);
            std::tie(files_lidar, fileNum) = loadFile(lidarPath, VERBOSE);

            std::vector<Oxts_Data> oxts_data;
            auto timer_oxts = std::chrono::system_clock::now();
            for (int16_t i = 0; i < fileNum; i ++){
                auto oxts_read = loadOxts(files_oxts, i);
                oxts_data.push_back(oxts_read);
            }
            timerCalculator(timer_oxts, "Loading all KITTI oxts data");

            int16_t NUM = 0;

            auto atFrequency{[&VERBOSE, &od4, &sharedMemory, &NAME, &oxts_data, &files_lidar, &IDSENDER, &NUM]() -> bool{
                cluon::data::TimeStamp sampleTime{cluon::time::now()};
                auto oxts_reading = oxts_data[NUM];

                // Copy point cloud into shared memory
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
                cloud_ptr = loadKitti(files_lidar, NUM);
                uint32_t sizecloud = sizeof(cloud_ptr) + cloud_ptr->points.size()+300;
                //std::cout << "cloud point size: " << cloud_ptr->points.size() << std::endl;

                if(!sharedMemory){
                    sharedMemory.reset(new cluon::SharedMemory{NAME, sizecloud*12});
                    std::cout << "reset shared memory" << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(700)); // Temporary Delay
                }
                if(sharedMemory){
                    opendlv::logic::sensation::Geolocation geolocation;
                    geolocation.latitude((float)oxts_reading.lat);            
                    geolocation.longitude((float)oxts_reading.lon);
                    geolocation.altitude((float)oxts_reading.alt);
                    geolocation.heading((float)oxts_reading.yaw);

                    opendlv::sensor::Acceleration acceleration;
                    acceleration.ax((float)oxts_reading.ax);
                    acceleration.ay((float)oxts_reading.ay);
                    acceleration.az((float)oxts_reading.az);

                    opendlv::logic::sensation::Equilibrioception equilibrioception;
                    equilibrioception.vx((float)oxts_reading.vf);        // Forward speed
                    equilibrioception.rollRate((float)oxts_reading.wx);        
                    equilibrioception.pitchRate((float)oxts_reading.wy);
                    equilibrioception.yawRate((float)oxts_reading.wz);

                    od4.send(geolocation, sampleTime, IDSENDER);
                    od4.send(acceleration, sampleTime, IDSENDER);
                    od4.send(equilibrioception, sampleTime, IDSENDER);

                    if (VERBOSE){
                        std::cout << "Frame: " << NUM << " | Lat: " << oxts_reading.lat << ", Lon: " << oxts_reading.lon
                        << ", Alt: " << oxts_reading.alt << ", Speed: " << oxts_reading.vf << ", Yaw: " << oxts_reading.yaw 
                        << ", Roll Rate: " << oxts_reading.wx << ", Pitch Rate: " << oxts_reading.wy 
                        << ", Yaw Rate: " << oxts_reading.wz << std::endl;
                    }

                    cluon::data::TimeStamp ts = cluon::time::now();
                    sharedMemory->lock();
                    sharedMemory->setTimeStamp(ts);
                    memcpy(sharedMemory->data(), static_cast<void const*>(cloud_ptr.get()), sharedMemory->size());
                    sharedMemory->unlock();
	                sharedMemory->notifyAll();
                    //std::cout << "Save PCD [" << NUM << "] to shared memory" << ", data size: " << sharedMemory->size() << " bytes." << std::endl;
                }
                NUM ++;
            }};

            while(NUM != fileNum && od4.isRunning()){
                od4.timeTrigger(FREQ, atFrequency);
            }
            retCode = 0;
        }
    }
    return retCode;
}