rosrun detic_ros run_container.py -host pr1040 -mount ./detic_mount -name detection.launch \
    standalone:=false \
    debug:=true \
    model_type:=swin \
    output_highest:=false \
    confidence_threshold:=0.5 \
    vocabulary:=custom \
    custom_vocabulary:=cigarette_case,checkbook,notebook,box \
    input_image:=/kinect_head/rgb/image_color
