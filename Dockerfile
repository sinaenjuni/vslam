FROM ubuntu:22.04 AS builder

RUN apt-get update && apt-get install -y \
    # Essential utilities
    wget g++ cmake pkg-config \
    # OpenGL & graphics dependencies
    libgl1-mesa-dev libegl1-mesa-dev libgles2-mesa-dev \
    libepoxy-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev \
    # Math & optimization libraries
    libeigen3-dev libspdlog-dev libsuitesparse-dev \
    # Qt dependencies
    qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5 \
    # GTK dependencies
    libgtk2.0-dev libgtk-3-dev \
    && rm -rf /var/lib/apt/lists/*

    # apt install libboost-all-dev git (DBoW2)
    
WORKDIR /thirdparty
RUN mkdir yaml-cpp eigen opencv g2o pangolin
WORKDIR /temp
RUN mkdir yaml-cpp_temp eigen_temp opencv_temp g2o_temp pangolin_temp

RUN wget -O /temp/yaml-cpp_temp.tar.gz \
        https://github.com/jbeder/yaml-cpp/archive/refs/tags/0.8.0.tar.gz \
    && tar -xzf /temp/yaml-cpp_temp.tar.gz -C /temp/yaml-cpp_temp --strip-components=1 \
    && mkdir /temp/yaml-cpp_temp/build \
    && cd /temp/yaml-cpp_temp/build \
    && cmake .. -DCMAKE_INSTALL_PREFIX="/thirdparty/yaml-cpp" \
    && make install -j4 

RUN wget -O /temp/eigen_temp.tar.gz \
        https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz \
    && tar -xzf /temp/eigen_temp.tar.gz -C /temp/eigen_temp --strip-components=1\
    && mkdir /temp/eigen_temp/build \
    && cd /temp/eigen_temp/build \
    && cmake .. -DCMAKE_INSTALL_PREFIX="/thirdparty/eigen" \
    && make install -j4

RUN wget -O /temp/opencv_temp.tar.gz \
        https://github.com/opencv/opencv/archive/refs/tags/4.11.0.tar.gz \
    && tar -xzf /temp/opencv_temp.tar.gz -C /temp/opencv_temp --strip-components=1\
    && mkdir /temp/opencv_temp/build \
    && cd /temp/opencv_temp/build \
    && cmake .. -DCMAKE_INSTALL_PREFIX="/thirdparty/opencv" \
    && make install -j4 

RUN wget -O /temp/g2o_temp.tar.gz \
        https://github.com/RainerKuemmerle/g2o/archive/refs/tags/20241228_git.tar.gz \
    && tar -xzf /temp/g2o_temp.tar.gz -C /temp/g2o_temp --strip-components=1\
    && mkdir /temp/g2o_temp/build \
    && cd /temp/g2o_temp/build \
    && cmake .. -DCMAKE_INSTALL_PREFIX="/thirdparty/g2o" \
        -DEigen3_DIR="/thirdparty/eigen/share/eigen3/cmake" \
    && make install -j4 
    
RUN wget -O /temp/pangolin_temp.tar.gz \
        https://github.com/stevenlovegrove/Pangolin/archive/refs/tags/v0.9.2.tar.gz \
    && tar -xzf /temp/pangolin_temp.tar.gz -C /temp/pangolin_temp --strip-components=1\
    && mkdir /temp/pangolin_temp/build \
    && cd /temp/pangolin_temp/build \
    && cmake .. -DCMAKE_INSTALL_PREFIX="/thirdparty/pangolin" \
        -DEigen3_DIR="/thirdparty/eigen/share/eigen3/cmake" \
    && make install -j4 

FROM ubuntu:22.04
RUN apt-get update && apt-get install -y \
    # Build tools
    g++ cmake \
    # OpenGL & graphics dependencies
    libgl1-mesa-dev libglu1-mesa-dev freeglut3-dev \
    # Logging & utility libraries
    libspdlog-dev libepoxy-dev \
    # Wayland support
    libwayland-dev libxkbcommon-dev wayland-protocols \
    libwayland-egl1-mesa libwayland-cursor0 \
    # Clang tools
    clangd-12 clang-format \
    # GTK
    libgtk-3-dev \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /thirdparty /vslam/thirdparty
# echo 'export LD_LIBRARY_PATH=/vslam/thirdparty/pangolin/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
# echo 'export LD_LIBRARY_PATH=/vslam/thirdparty/opencv/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
# echo 'export CMAKE_PREFIX_PATH=/vslam/thirdparty/opencv:$CMAKE_PREFIX_PATH' >> ~/.bashrc

VOLUME /vslam/thirdparty
WORKDIR /vslam
# COPY . /vslam
# CMD ["/bin/bash"]
ENV LD_LIBRARY_PATH="/vslam/thirdparty/pangolin/lib:$LD_LIBRARY_PATH"
ENV LD_LIBRARY_PATH="/vslam/thirdparty/opnecv/lib:$LD_LIBRARY_PATH"
ENV CMAKE_PREFIX_PATH="/vslam/thirdparty/opencv::$CMAKE_PREFIX_PATH"

# for a pangolin GUI error 
ENV XDG_RUNTIME_DIR="/tmp"

CMD ["/bin/bash", "-c", "tail -f /dev/null"]

