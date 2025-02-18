FROM ubuntu:22.04

RUN apt-get update && apt-get install -y \
    wget g++ cmake libgl1-mesa-dev libegl1-mesa-dev libgles2-mesa-dev \
    libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev libepoxy-dev \
    libeigen3-dev libspdlog-dev libsuitesparse-dev qtdeclarative5-dev \
    qt5-qmake libqglviewer-dev-qt5 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /vslam/thirdparty
RUN mkdir yaml-cpp eigen opencv g2o pangolin \
    yaml-cpp_temp eigen_temp opencv_temp g2o_temp pangolin_temp

RUN wget -O yaml-cpp_temp.tar.gz https://github.com/jbeder/yaml-cpp/archive/refs/tags/0.8.0.tar.gz \
    && tar -xzf yaml-cpp_temp.tar.gz -C yaml-cpp_temp --strip-components=1 \
    && mkdir yaml-cpp_temp/build \
    && cd yaml-cpp_temp/build \
    && cmake .. -DCMAKE_INSTALL_PREFIX="/vslam/thirdparty/yaml-cpp" \
    && make install -j$(nproc) \
    && cd "/vslam/thirdparty" \
    && rm -rf yaml-cpp_temp yaml-cpp_temp.tar.gz

RUN wget -O eigen_temp.tar.gz https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz \
    && tar -xzf eigen_temp.tar.gz -C eigen_temp --strip-components=1\
    && mkdir eigen_temp/build \
    && cd eigen_temp/build \
    && cmake .. -DCMAKE_INSTALL_PREFIX="/vslam/thirdparty/eigen" \
    && make install -j$(nproc)\
    && cd "/vslam/thirdparty" \
    && rm -rf eigen_temp eigen_temp.tar.gz

RUN wget -O opencv_temp.tar.gz https://github.com/opencv/opencv/archive/refs/tags/4.11.0.tar.gz \
    && tar -xzf opencv_temp.tar.gz -C opencv_temp --strip-components=1\
    && mkdir opencv_temp/build \
    && cd opencv_temp/build \
    && cmake .. -DCMAKE_INSTALL_PREFIX="/vslam/thirdparty/opencv" \
    && make install -j$(nproc) \
    && cd "/vslam/thirdparty" \
    && rm -rf opencv_temp opencv_temp.tar.gz

RUN wget -O g2o_temp.tar.gz https://github.com/RainerKuemmerle/g2o/archive/refs/tags/20241228_git.tar.gz \
    && tar -xzf g2o_temp.tar.gz -C g2o_temp --strip-components=1\
    && mkdir g2o_temp/build \
    && cd g2o_temp/build \
    && cmake .. -DCMAKE_INSTALL_PREFIX="/vslam/thirdparty/g2o" -DEigen3_DIR="/vslam/thirdparty/eigen/share/eigen3/cmake" \
    && make install -j$(nproc) \
    && cd "/vslam/thirdparty" \
    && rm -rf g2o_temp g2o_temp.tar.gz
    
RUN wget -O pangolin_temp.tar.gz https://github.com/stevenlovegrove/Pangolin/archive/refs/tags/v0.9.2.tar.gz \
    && tar -xzf pangolin_temp.tar.gz -C pangolin_temp --strip-components=1\
    && mkdir pangolin_temp/build \
    && cd pangolin_temp/build \
    && cmake .. -DCMAKE_INSTALL_PREFIX="/vslam/thirdparty/pangolin" -DEigen3_DIR="/vslam/thirdparty/eigen/share/eigen3/cmake" \
    && make install -j$(nproc) \
    && cd "/vslam/thirdparty" \
    && rm -rf pangolin_temp pangolin_temp.tar.gz

VOLUME /vslam/thirdparty
WORKDIR /vslam
# COPY . /vslam
CMD ["/bin/bash"]

