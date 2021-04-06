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
    wget \
    build-essential \
    software-properties-common \
    autotools-dev \
    libicu-dev \
    libboost-all-dev \
    libeigen3-dev \

RUN apt-get install -y  \
  mc \
  lynx \
  libqhull* \
  pkg-config \
  libxmu-dev \
  libxi-dev \
  --no-install-recommends --fix-missing

RUN apt-get install -y  \
  mesa-common-dev \
  cmake  \
  git  \
  mercurial \
  freeglut3-dev \
  libflann-dev \
  --no-install-recommends --fix-missing

RUN apt-get autoremove

# Install VTK
RUN cd /opt && git clone git://vtk.org/VTK.git VTK
RUN cd /opt/VTK && git checkout tags/v8.0.0
RUN cd /opt/VTK && mkdir build
RUN cd /opt/VTK/build && cmake -DCMAKE_BUILD_TYPE:STRING=Release -D VTK_RENDERING_BACKEND=OpenGL ..
RUN cd /opt/VTK/build && make -j 32 && make install


# Install PCL
RUN cd /opt && git clone https://github.com/Airsquire/pcl pcl
RUN cd /opt/pcl && git checkout master
RUN mkdir -p /opt/pcl/build
RUN cd /opt/pcl/build && cmake -D BUILD_visualization=true -D VTK_DIR=/opt/VTK/build -D BUILD_2d=true -D BUILD_tools=false ..
RUN cd /opt/pcl/build && make -j 32 && make install
RUN cd /opt/pcl/build && make clean


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

