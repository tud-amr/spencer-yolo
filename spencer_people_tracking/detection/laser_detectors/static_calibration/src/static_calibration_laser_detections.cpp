// Subscribe to the topic on which information about detected persons is published (YOLO detections)

#include "ros/ros.h"
#include <spencer_tracking_msgs/DetectedPersons.h>
#include <spencer_tracking_msgs/DetectedPerson.h>

class StaticCalibration
{
  public:
    // Constructor
    StaticCalibration(ros::NodeHandle n_priv, ros::NodeHandle n)
    {
      // load params
      n_priv.getParam("detected_persons", sub_topic_);
      n_priv.getParam("detected_persons_high_recall", sub_topic_high_recall_);
      n_priv.getParam("static_calibration_offset_x", static_calibration_offset_x_);

      sub_ = n.subscribe(sub_topic_, 1, &StaticCalibration::detectedPersonCallback, this);
      sub_high_recall_ = n.subscribe(sub_topic_high_recall_, 1, &StaticCalibration::detectedPersonHighRecallCallback, this);

      pub_ = n.advertise<spencer_tracking_msgs::DetectedPersons>("output", 1);
      pub_high_recall_ = n.advertise<spencer_tracking_msgs::DetectedPersons>("output_high_recall", 1);
    }

    void detectedPersonCallback(const spencer_tracking_msgs::DetectedPersons::ConstPtr& msg)
    {
      spencer_tracking_msgs::DetectedPersons calibrated_msg = *msg;
      
      // Add the static offset to each detection
      for(int i=0; i < calibrated_msg.detections.size(); i++)
      {
        calibrated_msg.detections[i].pose.pose.position.x += static_calibration_offset_x_;
      }
      pub_.publish(calibrated_msg);
    }

    void detectedPersonHighRecallCallback(const spencer_tracking_msgs::DetectedPersons::ConstPtr& msg)
    {
      spencer_tracking_msgs::DetectedPersons calibrated_msg = *msg;

      // Add the static offset to each detection
      for(int i=0; i < calibrated_msg.detections.size(); i++)
      {
        calibrated_msg.detections[i].pose.pose.position.x += static_calibration_offset_x_;
      }
      pub_high_recall_.publish(calibrated_msg);
    }

  private:
    ros::Subscriber sub_;
    ros::Subscriber sub_high_recall_;
    ros::Publisher pub_;
    ros::Publisher pub_high_recall_;

    std::string sub_topic_;
    std::string sub_topic_high_recall_;
    double static_calibration_offset_x_;
}; // End of class StaticCalibration



int main(int argc, char **argv)
{
  ros::init(argc, argv, "static_calibration_node");
  // private nodehandle is used for loading parameters
  ros::NodeHandle n; 
  ros::NodeHandle n_priv("~");

  StaticCalibration static_calibration(n_priv, n);

  ros::Rate loop_rate(20);
  
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
