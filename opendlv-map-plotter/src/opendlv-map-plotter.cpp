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
#include "matplotlibcpp.h"


int32_t main(int32_t argc, char **argv) {
    int32_t retCode{1};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ((0 == commandlineArguments.count("cid"))
    || (0 == commandlineArguments.count("freq")) ) 
    {
        std::cerr << "         --cid:        CID of the OD4Session to replay other Envelopes" << std::endl;
        std::cerr << "         --freq:       module frequency " << std::endl;
        std::cerr << "         --verbose: print decoding information and display image" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=111 --verbose" << std::endl;
    }
    else
    {
        const float FREQ = std::stof(commandlineArguments["freq"]);
        const bool DISPLAY{commandlineArguments.count("display") != 0};
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};

        // Interface to a running OpenDaVINCI session; here, you can send and receive messages.
        cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

        // Handler to receive data from sim-sensors (realized as C++ lambda).
        std::mutex UKFReadingMutex;     // EKF Reading Mutex
        std::mutex RawReadingMutex; // Raw GPS measure position

        std::vector<double> filteredX, filteredY, rawX, rawY; // Initialize Plotting Vector

        auto onUKFReading{[&UKFReadingMutex, &filteredX, &filteredY](cluon::data::Envelope &&envelope) {
            auto msg = cluon::extractMessage<opendlv::fused::Movement>(std::move(envelope));
            std::lock_guard<std::mutex> lck(UKFReadingMutex);
            auto x = msg.x();  // UKF Position X
            auto y = msg.y();  // UKF Position Y
            filteredX.push_back(x);
            filteredY.push_back(y);
            // std::cout << "Get data" << std::endl;
        }};

        auto onRawReading{[&RawReadingMutex, &rawX, &rawY](cluon::data::Envelope &&envelope){
            auto msg = cluon::extractMessage<opendlv::sensor::Position>(std::move(envelope));
            std::lock_guard<std::mutex> lck(RawReadingMutex);
            auto x = msg.x();  // Raw Position X
            auto y = msg.y();  // Raw Position Y
            rawX.push_back(x);
            rawY.push_back(y);
        }};

        // Register our lambda for the message identifier
        od4.dataTrigger(opendlv::fused::Movement::ID(), onUKFReading);
        od4.dataTrigger(opendlv::sensor::Position::ID(), onRawReading);

        uint64_t frameCount{0};
        auto atFrequency{[&od4, &filteredX, &filteredY, &rawX, &rawY, &DISPLAY, &VERBOSE, &frameCount]() -> bool
        {
            /*------ Visualization ------*/
            if(DISPLAY){
                if (frameCount % 5 == 0){
                    matplotlibcpp::clf(); // Clear [matplotlib] previous plot
                    matplotlibcpp::scatter(rawX, rawY, 8);
                    matplotlibcpp::named_plot("Filtered", filteredX, filteredY, "r-"); // Filtered positions
                    matplotlibcpp::title("UKF Result");
                    matplotlibcpp::legend(); // Enable legend
                    matplotlibcpp::grid(true); // Enable Grid
                    matplotlibcpp::pause(0.001);  // Display plot continuously
                }
            }

            if(VERBOSE)
                std::cout << "Frame (" << frameCount << ") | Plotting ..." << std::endl;

            frameCount++;
            return true;
        }};
        od4.timeTrigger(FREQ, atFrequency);
        retCode = 0;
    }
    return retCode;
}