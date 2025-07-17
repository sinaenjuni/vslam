FROM ubuntu:22.04 AS builder

RUN apt update && apt install -y \
    # Essential utilities
    # wget build-essential cmake g++ cmake pkg-config \
    wget build-essential cmake libsuitesparse-dev libgtk-3-dev \
    # g2o dependencies
    libglu1-mesa-dev libqglviewer-dev-qt5 libmetis-dev \ 
    # OpenGL & graphics dependencies
    # libgl1-mesa-dev libegl1-mesa-dev libgles2-mesa-dev \
    # libepoxy-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev \
    # Math & optimization libraries
    # libeigen3-dev libspdlog-dev \
    && rm -rf /var/lib/apt/lists/*
    # apt install libboost-all-dev git (DBoW2)

WORKDIR /vslam/thirdparty
RUN mkdir yaml-cpp eigen3 lapack opencv g2o pangolin dbow2
WORKDIR /temp
RUN mkdir yaml-cpp_temp eigen3_temp lapack_temp opencv_temp g2o_temp pangolin_temp dbow2_temp

RUN wget -O /temp/yaml-cpp_temp.tar.gz \
        https://github.com/jbeder/yaml-cpp/archive/refs/tags/0.8.0.tar.gz \
    && tar -xzf /temp/yaml-cpp_temp.tar.gz -C /temp/yaml-cpp_temp --strip-components=1 \
    && mkdir /temp/yaml-cpp_temp/build \
    && cd /temp/yaml-cpp_temp/build \
    && cmake .. -DCMAKE_INSTALL_PREFIX="/vslam/thirdparty/yaml-cpp" > /vslam/thirdparty/yaml-cpp_log.txt 2>&1 \
    && make install -j4

RUN wget -O /temp/eigen3_temp.tar.gz \
        https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz \
    && tar -xzf /temp/eigen3_temp.tar.gz -C /temp/eigen3_temp --strip-components=1\
    && mkdir /temp/eigen3_temp/build \
    && cd /temp/eigen3_temp/build \
    && cmake .. -DCMAKE_INSTALL_PREFIX="/vslam/thirdparty/eigen3" > /vslam/thirdparty/eigen3_log.txt 2>&1 \
    && make install -j4

RUN wget -O /temp/lapack_temp.tar.gz \
    https://github.com/OpenMathLib/OpenBLAS/releases/download/v0.3.30/OpenBLAS-0.3.30.tar.gz \
    && tar -xzf /temp/lapack_temp.tar.gz -C /temp/lapack_temp --strip-components=1\
    && mkdir /temp/lapack_temp/build \
    && cd /temp/lapack_temp/build \
    # && cmake .. -DCMAKE_INSTALL_PREFIX="/vslam/thirdparty/lapack" > /vslam/thirdparty/lapack_log.txt 2>&1 \
    && cmake .. \
    && make install -j4 

RUN wget -O /temp/opencv_temp.tar.gz \
        https://github.com/opencv/opencv/archive/refs/tags/4.11.0.tar.gz \
    && tar -xzf /temp/opencv_temp.tar.gz -C /temp/opencv_temp --strip-components=1\
    && mkdir /temp/opencv_temp/build \
    && cd /temp/opencv_temp/build \
    && cmake .. -DCMAKE_INSTALL_PREFIX="/vslam/thirdparty/opencv" \
     -DWITH_LAPACK=ON \
    #  -DLAPACK_LIBRARIES="/vslam/thirdparty/lapack/lib/libopenblas.a" \
    #  -DLAPACK_INCLUDE_DIRS="/vslam/thirdparty/lapack/include/" \
    > /vslam/thirdparty/opencv_log.txt 2>&1 \
    && make install -j4 

RUN wget -O /temp/g2o_temp.tar.gz \
        https://github.com/RainerKuemmerle/g2o/archive/refs/tags/20241228_git.tar.gz \
    && tar -xzf /temp/g2o_temp.tar.gz -C /temp/g2o_temp --strip-components=1\
    && mkdir /temp/g2o_temp/build \
    && cd /temp/g2o_temp/build \
    && cmake .. -DCMAKE_INSTALL_PREFIX="/vslam/thirdparty/g2o" \
        -DEigen3_DIR="/vslam/thirdparty/eigen3/share/eigen3/cmake" > /vslam/thirdparty/g2o_log.txt 2>&1 \
    && make install -j4 
    
RUN wget -O /temp/pangolin_temp.tar.gz \
        https://github.com/stevenlovegrove/Pangolin/archive/refs/tags/v0.9.2.tar.gz \
    && tar -xzf /temp/pangolin_temp.tar.gz -C /temp/pangolin_temp --strip-components=1\
    && mkdir /temp/pangolin_temp/build \
    && cd /temp/pangolin_temp/build \
    && cmake .. -DCMAKE_INSTALL_PREFIX="/vslam/thirdparty/pangolin" \
        -DEigen3_DIR="/vslam/thirdparty/eigen3/share/eigen3/cmake" > /vslam/thirdparty/pangolin_log.txt 2>&1 \
    && make install -j4 

RUN wget -O /temp/dbow2_temp.tar.gz \
    https://github.com/dorian3d/DBoW2/archive/refs/heads/master.tar.gz \
    && tar -xzf /temp/dbow2_temp.tar.gz -C /temp/dbow2_temp --strip-components=1 \
    && mkdir /temp/dbow2_temp/build \
    && cd /temp/dbow2_temp/build \
    && cmake .. -DCMAKE_INSTALL_PREFIX="/vslam/thirdparty/dbow2" \
        -DOpenCV_DIR="/vslam/thirdparty/opencv/lib/cmake/opencv4" > /vslam/thirdparty/dbow2_log.txt 2>&1 \
    && make install -j4 

FROM ubuntu:22.04
RUN apt update && apt install -y \
    g++ cmake libgl1-mesa-dev libepoxy-dev libspdlog-dev libglu1-mesa-dev libsuitesparse-dev libgtk-3-dev
    # OpenGL & graphics depenencies
    #  libglu1-mesa-dev freeglut3-dev \
#     # Logging & utility libraries
#     libspdlog-dev libepoxy-dev \
#     # Wayland support
#     libwayland-dev libxkbcommon-dev wayland-protocols \
#     libwayland-egl1-mesa libwayland-cursor0 \
#     # Clang tools
#     clangd-12 clang-format \
#     # GTK
#     libgtk-3-dev \
    # && rm -rf /var/lib/apt/lists/*

COPY --from=builder /vslam/thirdparty /vslam/thirdparty
VOLUME /vslam/thirdparty
ENV LD_LIBRARY_PATH="/vslam/build:\
/vslam/thirdparty/yaml-cpp/lib:\
/vslam/thirdparty/pangolin/lib:\
/vslam/thirdparty/dbow2/lib:\
/vslam/thirdparty/opencv/lib:\
/vslam/thirdparty/g2o/lib"
# for a pangolin GUI error 
ENV XDG_RUNTIME_DIR="/tmp"

# setting clangd 19 version(https://apt.llvm.org/)
RUN apt update && apt install -y wget lsb-release software-properties-common gnupg \
    && wget https://apt.llvm.org/llvm.sh -O /tmp/llvm.sh \
    && chmod +x /tmp/llvm.sh \ 
    && /tmp/llvm.sh 19 \
    && ln -s /usr/bin/clangd-19 /usr/bin/clangd

CMD ["/bin/bash", "-c", "tail -f /dev/null"]


# WORKDIR /vslam
# COPY . /vslam
# CMD ["/bin/bash"]
# ENV LD_LIBRARY_PATH="/vslam/thirdparty/pangolin/lib:/vslam/thirdparty/opencv/lib:$LD_LIBRARY_PATH"
# ENV CMAKE_PREFIX_PATH="/vslam/thirdparty/pangolin:/vslam/thirdparty/opencv:$CMAKE_PREFIX_PATH"
# echo 'export LD_LIBRARY_PATH=/vslam/thirdparty/pangolin/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
# echo 'export LD_LIBRARY_PATH=/vslam/thirdparty/opencv/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
# echo 'export CMAKE_PREFIX_PATH=/vslam/thirdparty/opencv:$CMAKE_PREFIX_PATH' >> ~/.bashrc