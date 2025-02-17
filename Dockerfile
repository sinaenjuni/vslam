FROM ubuntu:22.04

RUN apt-get update && apt-get install -y \
    wget g++ cmake libgl1-mesa-dev libegl1-mesa-dev libgles2-mesa-dev \
    libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev libepoxy-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /vslam/thirdparty
RUN mkdir yaml_cpp eigen pangolin opencv

ARG YAML_CPP_VERSION=0.8.0
ENV YAML_CPP_VERSION=${YAML_CPP_VERSION}
RUN wget -O yaml-cpp-${YAML_CPP_VERSION}.tar.gz \
https://github.com/jbeder/yaml-cpp/archive/refs/tags/0.8.0.tar.gz \
    && tar -xzf yaml-cpp-${YAML_CPP_VERSION}.tar.gz \
    && mkdir yaml-cpp-${YAML_CPP_VERSION}/build \
    && cd yaml-cpp-${YAML_CPP_VERSION}/build \
    && cmake .. -DCMAKE_INSTALL_PREFIX="/vslam/thirdparty/yaml_cpp" \
    && make install \
    && cd "/vslam/thirdparty" \
    && rm -rf yaml-cpp-${YAML_CPP_VERSION} yaml-cpp-${YAML_CPP_VERSION}.tar.gz

ARG EIGEN_VERSION=3.4.0
ENV EIGEN_VERSION=${EIGEN_VERSION}
RUN wget -O eigen-${EIGEN_VERSION}.tar.gz \
https://gitlab.com/libeigen/eigen/-/archive/${EIGEN_VERSION}/eigen-${EIGEN_VERSION}.tar.gz \
    && tar -xzf eigen-${EIGEN_VERSION}.tar.gz \
    && mkdir eigen-${EIGEN_VERSION}/build \
    && cd eigen-${EIGEN_VERSION}/build \
    && cmake .. -DCMAKE_INSTALL_PREFIX="/vslam/thirdparty/eigen" \
    && make install \
    && cd "/vslam/thirdparty" \
    && rm -rf eigen-${EIGEN_VERSION} eigen-${EIGEN_VERSION}.tar.gz

ARG PANGOLIN_VERSION=0.9.2
ENV PANGOLIN_VERSION=${PANGOLIN_VERSION}
RUN wget -O pangolin-${PANGOLIN_VERSION}.tar.gz \
https://github.com/stevenlovegrove/Pangolin/archive/refs/tags/v${PANGOLIN_VERSION}.tar.gz \
    && tar -xzf pangolin-${PANGOLIN_VERSION}.tar.gz \
    && mkdir Pangolin-${PANGOLIN_VERSION}/build \
    && cd Pangolin-${PANGOLIN_VERSION}/build \
    && cmake .. -DCMAKE_INSTALL_PREFIX="/vslam/thirdparty/pangolin" -DEigen3_DIR="/vslam/thirdparty/eigen/share/eigen3/cmake" \
    && make -j$(nproc) \
    && make install \
    && cd "/vslam/thirdparty" \
    && rm -rf Pangolin-${PANGOLIN_VERSION} Pangolin-${PANGOLIN_VERSION}.tar.gz

VOLUME /vslam/thirdparty
WORKDIR /vslam
# COPY . /vslam
CMD ["/bin/bash"]

