# Stereolabs ZED - YOLO 3D
This package lets you use [YOLO (v2 or v3)](http://pjreddie.com/darknet/yolo/) to detect pedestrians.

## ROS integration
The image with the marked pedestrians as well as a vector containing all detected persons (spencer_tracking_msgs::DetectedPersons) are published on the two following topics:
- /zed_yolo_detected_persons/image
- /zed_yolo_detected_persons

There are two different launch files available. One uses a very lightweight YOLOv3 model which is able to run at around 18 FPS on an nvidia jetson agx xavier board. The other launch file uses a heavier YOLOv3 model with slightly better detection accuracy. It is recommended to use the lightweight model. 

lighweight model:
- roslaunch yolo_pedestrian_detector pedestrian_detector_tiny.launch

normal model:
- roslaunch yolo_pedestrian_detector pedestrian_detector.launch

The weight files can be downloaded from here: 
- https://pjreddie.com/darknet/yolo/
(YOLOv3-tiny and YOLOv3-416)

place the weights files in here: 
libdarknet/weights
