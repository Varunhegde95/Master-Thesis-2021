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
        std::cout << " " << std::endl;
    

        retCode = 0;
    }
    return retCode;
}