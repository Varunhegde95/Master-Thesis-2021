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

#include "UKF.hpp"

int32_t main(int32_t argc, char **argv) {
    int32_t retCode{1};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ((0 == commandlineArguments.count("cid"))
    || (0 == commandlineArguments.count("freq")) ) 
    {
        std::cerr << "         --cid:        CID of the OD4Session to replay other Envelopes" << std::endl;
        std::cerr << "         --freq:       module frequency " << std::endl;
        std::cerr << "         --imu-freq:   IMU frequency" <<std::endl;
        std::cerr << "         --gps-freq:   GPS frequency" <<std::endl;
        std::cerr << "         --id-sender:  sender id of output messages" << std::endl;
        std::cerr << "         --verbose: print decoding information and display image" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=111 --imu-freq=10 --gps-freq=10 --verbose" << std::endl;
    }
    else
    {
        const float FREQ = std::stof(commandlineArguments["freq"]);
        const uint32_t IMU_FREQ = (commandlineArguments.count("imu-freq") != 0) 
          ? std::stoi(commandlineArguments["imu-freq"]) : 0;
        const uint32_t GPS_FREQ = (commandlineArguments.count("gps-freq") != 0) 
          ? std::stoi(commandlineArguments["gps-freq"]) : 0;
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};
        uint32_t const SENDER_ID{(commandlineArguments["id-sender"].size() != 0)               
            ? static_cast<uint32_t>(std::stoi(commandlineArguments["id-sender"])) : 0};

        UKF ukf;       // Define UKF
        Odometer odom; // Define Odometer

        // Interface to a running OpenDaVINCI session (ignoring any incoming Envelopes).
        cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

        // Initialize handler to receive data 
        std::mutex GpsReadingMutex;       // GPS Data [wgs84] Reading Mutex
        std::mutex AccReadingMutex;       // IMU Acceleration Reading Mutex
        std::mutex GyroReadingMutex;      // IMU Gyroscope Reading Mutex

        Sensor_Reading sensor_reading;
        Sensor_Reading sensor_reading_pre; // Used in UKF

        auto onGpsReading{[&GpsReadingMutex, &sensor_reading](cluon::data::Envelope &&envelope) {
            auto msg = cluon::extractMessage<opendlv::logic::sensation::Geolocation>(std::move(envelope));
            std::lock_guard<std::mutex> lck(GpsReadingMutex);
            sensor_reading.lon = msg.longitude();   
            sensor_reading.lat = msg.latitude();  
            sensor_reading.alt = msg.altitude();
            sensor_reading.yaw = msg.heading();
        }};

        auto onAccReading{[&AccReadingMutex, &sensor_reading](cluon::data::Envelope &&envelope) {
            auto msg = cluon::extractMessage<opendlv::sensor::Acceleration>(std::move(envelope));
            std::lock_guard<std::mutex> lck(AccReadingMutex);
            sensor_reading.ax = msg.ax();
            sensor_reading.ay = msg.ay();
            sensor_reading.az = msg.az();
        }};

        auto onGyroReading{[&GyroReadingMutex, &sensor_reading](cluon::data::Envelope &&envelope) {
            auto msg = cluon::extractMessage<opendlv::logic::sensation::Equilibrioception>(std::move(envelope));
            std::lock_guard<std::mutex> lck(GyroReadingMutex);
            sensor_reading.wx  = msg.rollRate();
            sensor_reading.wy  = msg.pitchRate();
            sensor_reading.wz  = msg.yawRate();
            sensor_reading.vf  = msg.vx(); // Forward Speed (temporarily)
        }};

        od4.dataTrigger(opendlv::sensor::Acceleration::ID(), onAccReading);
        od4.dataTrigger(opendlv::logic::sensation::Equilibrioception::ID(), onGyroReading);
        od4.dataTrigger(opendlv::logic::sensation::Geolocation::ID(), onGpsReading);

        uint64_t frameCount{0};
        auto atFrequency{[&od4, &ukf, &odom, &IMU_FREQ, &sensor_reading, &sensor_reading_pre, &frameCount, &SENDER_ID, &VERBOSE]() -> bool
        {
            if (sensor_reading.lon == 0 && sensor_reading.lat == 0 && sensor_reading.alt == 0){ // Wait until receive data 
                frameCount = 0;
                std::cout << "Waiting to receive data ..." << std::endl;
            }
            else{ 
                if(frameCount == 0){
                    /*----- Initialize UKF -----*/
                    ukf.dt_ = (float)1 / IMU_FREQ;
                    std::cout << "DT: " << ukf.dt_ << std::endl;
                    odom.Initialize();                    // Initialize Odometer
                    ukf.Initialize(odom, sensor_reading); // Initialize UKF Q, R, P0, x_f, p_f
                    sensor_reading_pre = sensor_reading;  // Initialize sensor_reading_pre for future odometer calculation
                    ukf.GetMeasurement(odom, sensor_reading); // Get first measurement (Actually not used, only for plotting.)
                    std::cout << "Frame [" << frameCount << "] ";
                }
                else
                {
                    std::cout << "Frame [" << frameCount << "] ";
                    /*------ UKF Prediction ------*/
                    ukf.Prediction(ukf.x_f_, ukf.p_f_);
                    /*------ UKF Update ------*/ 
                    odom.GPSConvertor(sensor_reading, sensor_reading_pre); // Convert GPS coordinates to mileage [meter]
                    ukf.GetMeasurement(odom, sensor_reading); // Get measurement for update step
                    ukf.Update(ukf.x_p_, ukf.p_p_, ukf.measurements_, odom); 
                    sensor_reading_pre = sensor_reading; // Update for next state
                }

                if(VERBOSE){
                    std::cout << " Filtered State || " << "X: " << ukf.x_f_(0, 0) << ", Y: " << ukf.x_f_(1, 0) << ", Z: " << ukf.x_f_(2, 0) 
                              << ", speed: " << ukf.x_f_(3, 0) << ", yaw: " << ukf.x_f_(6, 0) << ", Roll Rate: " << ukf.x_f_(8, 0)*180/M_PI 
                              << ", Pitch Rate: " << ukf.x_f_(7, 0)*180/M_PI << ", Yaw Rate: " << ukf.x_f_(9, 0)*180/M_PI << " ..." << std::endl;
                }

                //Sending message to opendlv::fused::Movement
                cluon::data::TimeStamp ts{cluon::time::now()};
                {
                    opendlv::fused::Movement movement;
                    movement.x(static_cast<float>  (ukf.x_f_(0, 0)));  // Position X
                    movement.y(static_cast<float>  (ukf.x_f_(1, 0)));  // Position Y
                    movement.vx(static_cast<float> (ukf.x_f_(3, 0)));  // Speed Vf
                    od4.send(movement, ts, SENDER_ID);
                }
                //Sending measure position to opendlv::sensor::Position
                cluon::data::TimeStamp timestamp{cluon::time::now()};
                {
                    opendlv::sensor::Position position;
                    position.x(static_cast<float>  (ukf.measurements_(0, 0)));  // Measure Position X
                    position.y(static_cast<float>  (ukf.measurements_(1, 0)));  // Measure Position Y
                    od4.send(position, timestamp, SENDER_ID);
                }
                frameCount ++;
            }
            return true;
        }};
 
        od4.timeTrigger(FREQ, atFrequency);
        retCode = 0;
    }
    return retCode;
}