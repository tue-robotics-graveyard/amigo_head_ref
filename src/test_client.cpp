#include <ros/ros.h>
#include <amigo_head_ref/head_ref.h>
#include <geometry_msgs/PointStamped.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "test_client");
  ros::NodeHandle nh;
  
  ros::Publisher target_pub = nh.advertise<geometry_msgs::PointStamped>("head_target", 50);
  
  geometry_msgs::PointStamped target;
  
  target.header.frame_id = "grippoint_right";
  target.point.x = 0.0;
  target.point.y = 0.0;
  target.point.z = 0.0;
  
  ros::Rate rate(1.0);
  
  while (ros::ok()){
	ROS_INFO("Publish");  
    target_pub.publish(target);
    rate.sleep();
  }
  
  return true;
}
