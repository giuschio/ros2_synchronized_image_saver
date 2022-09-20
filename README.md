# Usage

ros2 run synchronized_image_saver node --ros-args -p output_path:=<ABSOLUTE_OUTPUT_PATH> -r image0:=<first_img_topic> -r image1:=<second_image_topic>     

example:    
ros2 run synchronized_image_saver node --ros-args -p output_path:=/home/harmony_asl/Desktop/saver -r image0:=/kinect/depth/image_raw -r image1:=/kinect/rgb/image_raw