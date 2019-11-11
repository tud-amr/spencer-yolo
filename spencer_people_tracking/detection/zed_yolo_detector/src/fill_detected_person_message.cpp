
// Subscribe to the topic on which information about detected persons is published (YOLO detections)

#include "ros/ros.h"
#include <spencer_tracking_msgs/DetectedPersons.h>
#include <spencer_tracking_msgs/DetectedPerson.h>


class FillMessage
{
  public:
  
    FillMessage(ros::NodeHandle n_priv, ros::NodeHandle n)
    {
      n.param("pose_variance", pose_variance_, 0.05);
      n.param("person_listener", sub_topic_, std::string("/yolo_pedestrian_detector/detected_persons"));
      n.param("detection_id_increment", detection_id_increment_, 1);
      n.param("detection_id_offset",    detection_id_offset_, 0);

      LARGE_VARIANCE_ = 0.5;
      current_detection_id_ = detection_id_offset_;

      sub_ = n.subscribe(sub_topic_, 1, &FillMessage::personCallback, this);
      pub_ = n.advertise<spencer_tracking_msgs::DetectedPersons>("output", 1);
    }


    void personCallback(const spencer_tracking_msgs::DetectedPersons::ConstPtr& msg)
    {
      spencer_tracking_msgs::DetectedPersons detected_persons = *msg;

      int num_detections = detected_persons.detections.size();

      if(num_detections > 0)
      {
        for(int i = 0; i < num_detections; i++)
        {
            detected_persons.detections[i].pose.covariance[0*6 + 0] = pose_variance_;
            detected_persons.detections[i].pose.covariance[1*6 + 1] = pose_variance_;
            detected_persons.detections[i].pose.covariance[2*6 + 2] = pose_variance_;
            detected_persons.detections[i].pose.covariance[3*6 + 3] = LARGE_VARIANCE_;
            detected_persons.detections[i].pose.covariance[4*6 + 4] = LARGE_VARIANCE_;
            detected_persons.detections[i].pose.covariance[5*6 + 5] = LARGE_VARIANCE_;

            detected_persons.detections[i].detection_id = current_detection_id_;
            current_detection_id_ += detection_id_increment_;
        }
      }
      
      detected_persons.header.stamp = ros::Time::now();
      pub_.publish(detected_persons);
    }


  private:
    ros::Subscriber sub_;
    ros::Publisher pub_;
    std::string sub_topic_;
    double LARGE_VARIANCE_;
    double pose_variance_;
    int detection_id_increment_, detection_id_offset_, current_detection_id_; // added for multi-sensor use in SPENCER

}; // end of class FillMessage


int main(int argc, char **argv)
{
  ros::init(argc, argv, "spencer_yolo_listener");
  ros::NodeHandle n;
  ros::NodeHandle n_priv;

  FillMessage fill_message(n_priv, n);

  ros::Rate loop_rate(20);
  
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
