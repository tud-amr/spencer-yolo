#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <queue>
#include <fstream>
#include <thread>
#include <atomic>
#include <mutex>              // std::mutex, std::unique_lock
#include <condition_variable> // std::condition_variable
#include <algorithm>

// ROS
#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include <sstream>
#include <spencer_tracking_msgs/DetectedPersons.h>
#include <spencer_tracking_msgs/DetectedPerson.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

// This is a modified version of https://github.com/AlexeyAB/darknet/blob/master/src/yolo_console_dll.cpp
// Basically simplified and using the ZED SDK

#define OPENCV
#define GPU

#include <sl_zed/Camera.hpp>

#include "yolo_v2_class.hpp"    // https://github.com/AlexeyAB/darknet/blob/master/src/yolo_v2_class.hpp
#include <opencv2/opencv.hpp>


std::mutex data_lock;
cv::Mat cur_frame;
std::vector<bbox_t> result_vect;
std::atomic<bool> exit_flag, new_data;


class bbox_t_3d {
public:
    bbox_t bbox;
    sl::float3 coord;

    bbox_t_3d(bbox_t bbox_, sl::float3 coord_) {
        bbox = bbox_;
        coord = coord_;
    }
};


float getMedian(std::vector<float> &v) {
    size_t n = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + n, v.end());
    return v[n];
}


std::vector<bbox_t_3d> getObjectDepth(std::vector<bbox_t> &bbox_vect, sl::Mat &xyzrgba) {
    sl::float4 out(NAN, NAN, NAN, NAN);
    bool valid_measure;
    int i, j;
    const int R_max = 4;

    std::vector<bbox_t_3d> bbox3d_vect;

    for (auto &it : bbox_vect) {

		// Center of the bounding box in pixels
        int center_i = it.x + it.w * 0.5f, center_j = it.y + it.h * 0.5f;

        std::vector<float> x_vect, y_vect, z_vect;
        for (int R = 0; R < R_max; R++) {
            for (int y = -R; y <= R; y++) {
                for (int x = -R; x <= R; x++) {
                    i = center_i + x;
                    j = center_j + y;
                    xyzrgba.getValue<sl::float4>(i, j, &out, sl::MEM_CPU);
                    valid_measure = std::isfinite(out.z);
                    if (valid_measure) {
                        x_vect.push_back(out.x);
                        y_vect.push_back(out.y);
                        z_vect.push_back(out.z);
                    }
                }
            }
        } 

        if (x_vect.size() * y_vect.size() * z_vect.size() > 0) {
            float x_med = getMedian(x_vect);
            float y_med = getMedian(y_vect);
            float z_med = getMedian(z_vect);

            bbox3d_vect.emplace_back(it, sl::float3(x_med, y_med, z_med));
        }
    }

    return bbox3d_vect;
}


void draw_boxes(cv::Mat mat_img, std::vector<bbox_t_3d> result_vec, std::vector<std::string> obj_names, std::string object_name) {
    for (auto &i : result_vec) {
		cv::Scalar color = obj_id_to_color(i.bbox.obj_id);
		cv::rectangle(mat_img, cv::Rect(i.bbox.x, i.bbox.y, i.bbox.w, i.bbox.h), color, 2);
		
		std::string obj_name = object_name;
		std::stringstream streamx;
		std::stringstream streamy;
		std::stringstream streamz;
		std::stringstream streamprob;
		streamx << std::fixed << std::setprecision(2) << i.coord.x;
		streamy << std::fixed << std::setprecision(2) << i.coord.y;
		streamz << std::fixed << std::setprecision(2) << i.coord.z;
		streamprob << std::fixed << std::setprecision(2) << i.bbox.prob;
		
		obj_name += "prob = " + streamprob.str() + " ,  x = " + streamx.str() + "m,  y = " + streamy.str() + "m,  z = " + streamz.str();
		cv::Size const text_size = getTextSize(obj_name, cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, 2, 0);
		int const max_width = (text_size.width > i.bbox.w + 2) ? text_size.width : (i.bbox.w + 2);
		cv::rectangle(mat_img, cv::Point2f(std::max((int) i.bbox.x - 1, 0), std::max((int) i.bbox.y - 30, 0)),
				cv::Point2f(std::min((int) i.bbox.x + max_width, mat_img.cols - 1),
				std::min((int) i.bbox.y, mat_img.rows - 1)),
				color, CV_FILLED, 8, 0);
		putText(mat_img, obj_name, cv::Point2f(i.bbox.x, i.bbox.y - 10), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2,
				cv::Scalar(0, 0, 0), 2);
    }
}


std::vector<std::string> objects_names_from_file(std::string const filename) {
    std::ifstream file(filename);
    std::vector<std::string> file_lines;
    if (!file.is_open()) return file_lines;
    for (std::string line; getline(file, line);) file_lines.push_back(line);
    std::cout << "Object names loaded \n";
    return file_lines;
}


cv::Mat slMat2cvMat(sl::Mat &input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case sl::MAT_TYPE_32F_C1:
            cv_type = CV_32FC1;
            break;
        case sl::MAT_TYPE_32F_C2:
            cv_type = CV_32FC2;
            break;
        case sl::MAT_TYPE_32F_C3:
            cv_type = CV_32FC3;
            break;
        case sl::MAT_TYPE_32F_C4:
            cv_type = CV_32FC4;
            break;
        case sl::MAT_TYPE_8U_C1:
            cv_type = CV_8UC1;
            break;
        case sl::MAT_TYPE_8U_C2:
            cv_type = CV_8UC2;
            break;
        case sl::MAT_TYPE_8U_C3:
            cv_type = CV_8UC3;
            break;
        case sl::MAT_TYPE_8U_C4:
            cv_type = CV_8UC4;
            break;
        default:
            break;
    }
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM_CPU));
}


void detectorThread(std::string cfg_file, std::string weights_file, float thresh) {
    Detector detector(cfg_file, weights_file);
    std::shared_ptr<image_t> det_image;
    cv::Size const frame_size = cur_frame.size();

    while (!exit_flag) {
        if (new_data) {
            data_lock.lock();
            det_image = detector.mat_to_image_resize(cur_frame);
            result_vect = detector.detect_resized(*det_image, frame_size.width, frame_size.height, thresh, false); // true
            data_lock.unlock();
            new_data = false;
        } else sl::sleep_ms(1);
    }
}


std::vector<bbox_t> filterOutUnwantedDetections(std::vector<bbox_t> &bbox_vect, std::vector<std::string> obj_names, std::string object_name) {
	
	std::vector<bbox_t> filtered_detections;
	std::vector<bbox_t> filtered_detections_wo_doubles;
	std::vector<int> erase_these_elements;
	
	// filter out everything that is not a person
	for (auto &it : bbox_vect){
		if (obj_names[it.obj_id] == object_name) {
			filtered_detections.push_back(it);
		}
	}
	
	// Sometimes, there are two or even more bounding boxes per detected person. 
	// Use nonmax suppression to only keep the bounding box with the highest confidence	
	for (int i = 0; i < filtered_detections.size(); i++) {
		for (int j = 0; j < filtered_detections.size(); j++) {
			// check if one bounding box lies completely within the other one. If it does, only keep the box with the higher confidence
			if ( (j != i) && (
					(filtered_detections[i].x < filtered_detections[j].x && 
					filtered_detections[i].y < filtered_detections[j].y &&
					filtered_detections[i].x + filtered_detections[i].w > filtered_detections[j].x + filtered_detections[j].w &&
					filtered_detections[i].y + filtered_detections[i].h > filtered_detections[j].y + filtered_detections[j].h) || 
					(filtered_detections[i].x > filtered_detections[j].x && 
					filtered_detections[i].y > filtered_detections[j].y &&
					filtered_detections[i].x + filtered_detections[i].w < filtered_detections[j].x + filtered_detections[j].w &&
					filtered_detections[i].y + filtered_detections[i].h < filtered_detections[j].y + filtered_detections[j].h))) {
						
				if (filtered_detections[i].prob > filtered_detections[j].prob) {
					erase_these_elements.push_back(j);
				} else {
					erase_these_elements.push_back(i);
				}
			}
		}
	}
	
	// store all bounding boxes that lie not within each other
	for (int i = 0; i < filtered_detections.size(); i++) {
		if(std::find(erase_these_elements.begin(), erase_these_elements.end(), i) != erase_these_elements.end()) {
			
		} else {
			filtered_detections_wo_doubles.push_back(filtered_detections[i]);
		}
	}
	
	return filtered_detections_wo_doubles;
}


// Extract relevant information and fill it into a ROS message
spencer_tracking_msgs::DetectedPersons fillPeopleMessage(
		std::vector<bbox_t_3d> result_vec, 
		std::vector<std::string> obj_names, 
		std::string camera_frame_id) {
	
	spencer_tracking_msgs::DetectedPersons  detected_persons;
	spencer_tracking_msgs::DetectedPerson  detected_person;
	
    for (auto &i : result_vec) {
		detected_person.pose.pose.position.x = i.coord.x;
		detected_person.pose.pose.position.y = i.coord.y;
		detected_person.pose.pose.position.z = i.coord.z;
		detected_person.modality = spencer_tracking_msgs::DetectedPerson::MODALITY_GENERIC_YOLO;
		detected_person.confidence = i.bbox.prob;
		
		detected_persons.detections.push_back(detected_person);
    }
    
    detected_persons.header.stamp = ros::Time::now();
    detected_persons.header.frame_id = camera_frame_id;
    
    return detected_persons;
}

// Publish the images with the detections on a ROS topic
sensor_msgs::Image fillRosImgMsg(cv::Mat cv_img, std::string camera_frame_id) {
	cv_bridge::CvImage img_bridge;
	sensor_msgs::Image ros_image;
	std_msgs::Header header;
	
	header.stamp = ros::Time::now();
	header.frame_id = camera_frame_id;
	img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGRA8, cv_img);
	img_bridge.toImageMsg(ros_image);
	
	return ros_image;
}


int main(int argc, char *argv[]) {
    
    // Initialize ROS node
    // last argument of init is the node name
	ros::init(argc, argv, "yolo_pedestrian_detector");
	ros::NodeHandle n;

	//~ Load ROS parameters
	std::string names_file;
    std::string cfg_file;
    std::string weights_file;
    std::string filename;
    std::string camera_frame_id;
    std::string object_name;
    int loop_rate_param;
    float confidence_thresh;
    
    
	n.getParam("/yolo_pedestrian_detector/names_file", names_file);
	n.getParam("/yolo_pedestrian_detector/cfg_file", cfg_file);
	n.getParam("/yolo_pedestrian_detector/weights_file", weights_file);
	n.getParam("/yolo_pedestrian_detector/camera_frame_id", camera_frame_id);
	n.getParam("/yolo_pedestrian_detector/object_name", object_name);
	n.getParam("/yolo_pedestrian_detector/loop_rate_param", loop_rate_param);
	n.getParam("/yolo_pedestrian_detector/confidence_thresh", confidence_thresh);
	
	// Initialize publishers. Topics are getting remapped
	ros::Publisher people_position_pub = n.advertise<spencer_tracking_msgs::DetectedPersons>("detections_output", 1);
	ros::Publisher image_pub = n.advertise<sensor_msgs::Image>("detections_image", 1);
	
	ros::Rate loop_rate(loop_rate_param);

    sl::Camera zed;
    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION_HD720;
    init_params.coordinate_units = sl::UNIT_METER;
    if (!filename.empty()) init_params.svo_input_filename.set(filename.c_str());

	std::cout << zed.open(init_params) << std::endl;
    zed.grab();

    float const thresh = (argc > 5) ? std::stof(argv[5]) : confidence_thresh;
    auto obj_names = objects_names_from_file(names_file);

    sl::Mat left, cur_cloud;
    zed.retrieveImage(left);
    zed.retrieveMeasure(cur_cloud, sl::MEASURE_XYZ);
    slMat2cvMat(left).copyTo(cur_frame);
    exit_flag = false;
    new_data = false;

    std::thread detect_thread(detectorThread, cfg_file, weights_file, thresh);

    while (ros::ok()) {

        if (zed.grab() == sl::SUCCESS) {
            zed.retrieveImage(left);
            data_lock.lock();
            cur_frame = slMat2cvMat(left);
            data_lock.unlock();
            new_data = true;

            zed.retrieveMeasure(cur_cloud, sl::MEASURE_XYZ);
            
            data_lock.lock();
            auto result_vec_filtered = filterOutUnwantedDetections(result_vect, obj_names, object_name);
            auto result_vec_draw = getObjectDepth(result_vec_filtered, cur_cloud);
            data_lock.unlock();

            draw_boxes(cur_frame, result_vec_draw, obj_names, object_name);
            //cv::imshow("ZED", cur_frame);
            
            // Publish ROS image message
            auto ros_image_msg = fillRosImgMsg(cur_frame, camera_frame_id);
            image_pub.publish(ros_image_msg);
            
            // Publish the message that contains infos about detected persons
            auto detected_persons_msg = fillPeopleMessage(result_vec_draw, obj_names, camera_frame_id);         
			people_position_pub.publish(detected_persons_msg);
			
			ros::spinOnce();
			loop_rate.sleep();
        }
		
        int key = cv::waitKey(15); // 3 or 16ms
        if (key == 'p') while (true) if (cv::waitKey(100) == 'p') break;
        if (key == 27 || key == 'q') exit_flag = true;
        
    }

    detect_thread.join();
    zed.close();
    return 0;
}
