docker run --rm --net=host -it --gpus 1 detic_ros:latest \
    /bin/bash -i -c \
    'source ~/.bashrc; \
    roscd detic_ros; \
    git remote add tmp https://github.com/HiroIshida/detic_ros.git; \
    git fetch tmp; \
    git checkout tmp/master; \
    rossetip; rossetmaster pr1040; \
    roslaunch detic_ros sample_detection.launch \
    standalone:=false \
    debug:=true \
    model_type:=swin \
    output_highest:=false \
    confidence_threshold:=0.5 \
    vocabulary:=custom \
    custom_vocabulary:=cigarette_case,checkbook,notebook,box \
    input_image:=/kinect_head/rgb/image_color'


