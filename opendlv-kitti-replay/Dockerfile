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

# Part to build opendlv-kitti-replay.
FROM ubuntu:18.04 as builder
ENV DEBIAN_FRONTEND=noninteractive
MAINTAINER Liangyu Wang "liangyu@student.chalmers.se"
RUN apt-get update -y && \
    apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
    cmake \
    build-essential \
    software-properties-common

RUN apt-get update
RUN apt-get install -y libeigen3-dev \
    libflann-dev \
    libboost-all-dev \
    libpcl-dev

RUN add-apt-repository -y ppa:chrberger/libcluon
RUN apt-get update
RUN apt-get install -y libcluon

ADD . /opt/sources
WORKDIR /opt/sources
RUN mkdir build && \
    cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/tmp/dest .. && \
    make && make install

# Part to deploy opendlv-kitti-replay
FROM ubuntu:18.04
ENV DEBIAN_FRONTEND=noninteractive
MAINTAINER Liangyu Wang "liangyu@student.chalmers.se"

RUN apt-get update
RUN apt-get install -y libeigen3-dev \
    libflann-dev \
    libboost-all-dev \
    libpcl-dev

WORKDIR /usr/bin
COPY --from=builder /tmp/dest /usr
ENTRYPOINT ["/usr/bin/opendlv-kitti-replay"]

