docker build -t vslam .

docker run --rm -it -e DISPLAY=host.docker.internal:0 \
    -v vslam_thirdparty:/vslam/thirdparty \
    -v $(pwd):/vslam \
    vslam