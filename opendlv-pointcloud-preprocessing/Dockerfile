# Copyright (C) 2021 Liangyu Wang
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

# Part to build opendlv-pointcloud-preprocessing.
FROM ubuntu:18.04 as builder
ENV DEBIAN_FRONTEND=noninteractive
MAINTAINER Liangyu Wang "liangyu@student.chalmers.se"
RUN apt-get update -y && \
    apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
    cmake \
    git \
    build-essential \
    software-properties-common

# Install PCL library
ADD . /opt/sources
WORKDIR /opt/sources
ENV PCL_VERSION="1.8.1"

RUN apt-get update
RUN apt-get install -y \
    libeigen3-dev \
    libflann-dev \
    libglu1-mesa-dev \
    freeglut3-dev \
    mesa-common-dev \
    libboost-all-dev \
    libusb-1.0-0-dev \
    libusb-dev \
    libopenni-dev \
    libopenni2-dev \
    libpcap-dev \
    libpng-dev \
    mpi-default-dev \
    openmpi-bin \
    openmpi-common \
    libqhull-dev \
    libgtest-dev

RUN wget https://github.com/PointCloudLibrary/pcl/archive/pcl-${PCL_VERSION}.tar.gz \
    && tar -xf pcl-${PCL_VERSION}.tar.gz \
    && cd pcl-pcl-${PCL_VERSION} \
    && mkdir build \
    && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release \
                -DVTK_RENDERING_BACKEND=OpenGL2 \
    && make -j12 \
    && make install

RUN apt-get update && apt-get install -y pcl-tools

RUN unset PCL_VERSION

RUN add-apt-repository -y ppa:chrberger/libcluon
RUN apt-get update
RUN apt-get install -y libcluon

#ADD . /opt/sources
WORKDIR /opt/sources
RUN mkdir build && \
    cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/tmp/dest .. && \
    make && make install

# Part to deploy opendlv-pointcloud-preprocessing
FROM ubuntu:18.04
MAINTAINER Liangyu Wang "liangyu@student.chalmers.se"
ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /usr/bin
COPY --from=builder /tmp/dest /usr
ENTRYPOINT ["/usr/bin/opendlv-pointcloud-preprocessing"]

