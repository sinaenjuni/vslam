docker build -t vslam .

docker run --rm -it -e DISPLAY=host.docker.internal:0 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v vslam_thirdparty:/vslam/thirdparty \
    -v $(pwd):/vslam \
    vslam

docker run --rm -d -e DISPLAY=host.docker.internal:0 \
    -v vslam_thirdparty:/vslam/thirdparty \
    -v $(pwd):/vslam \
    vslam

docker run --rm -d -e DISPLAY=host.docker.internal:0 \
    -v vslam_thirdparty:/vslam/thirdparty \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $(pwd):/vslam \
    vslam