#include <ros/ros.h>
#include <amigo_head_ref/head_ref.h>
#include <geometry_msgs/PointStamped.h>

#define PI 3.14159265

void targetCallback(const geometry_msgs::PointStamped::ConstPtr& msg){
	
	target = *msg;
	received = true;
	ROS_INFO("Received target for head: (%.2f, %.2f, %.2f) in frame '%s'",target.point.x,target.point.y,target.point.z,target.header.frame_id.c_str());
	
}

bool transformPoint(const tf::TransformListener& listener){
  
  if (received){
  
  geometry_msgs::PointStamped desired_point,torso_point, head_point, neck_point, torso_point_neck, head_point_neck, target_to_head_pan_point;
  
  
  desired_point = target;
  
  //define head point
  head_point.header.frame_id = "head";
  head_point.header.stamp = ros::Time();
  head_point.point.x = 0.0;
  head_point.point.y = 0.0;
  head_point.point.z = 0.0;
  
  //define neck_pan point
  neck_point.header.frame_id = "neck_tilt";
  neck_point.header.stamp = ros::Time();
  neck_point.point.x = 0.0;
  neck_point.point.y = 0.0;
  neck_point.point.z = 0.0;
   
  try{  //transform from neck_tilt_point to torso
        listener.transformPoint("torso", neck_point, torso_point_neck);
        
    ROS_DEBUG("neck_tilt_point: (%.3f, %.3f. %.3f) -----> torso_point: (%.3f, %.3f, %.3f) at time %.2f",
        neck_point.point.x, neck_point.point.y, neck_point.point.z,
        torso_point_neck.point.x, torso_point_neck.point.y, torso_point_neck.point.z, torso_point_neck.header.stamp.toSec());
      }
  catch(tf::TransformException& ex){
    ROS_ERROR("4. Received an exception trying to transform: %s", ex.what());
    return false;
  }
  
    try{  //transform from head to neck_tilt_point
        listener.transformPoint("head", neck_point, head_point_neck);
        
    ROS_DEBUG("head_point: (%.3f, %.3f. %.3f) -----> neck_tilt_point: (%.3f, %.3f, %.3f) at time %.2f",
        neck_point.point.x, neck_point.point.y, neck_point.point.z,
        head_point_neck.point.x, head_point_neck.point.y, head_point_neck.point.z, head_point_neck.header.stamp.toSec());
      }
  catch(tf::TransformException& ex){
    ROS_ERROR("5. Received an exception trying to transform: %s", ex.what());
    return false;
  }
  
  try{  //transform from desired_point to torso
        listener.transformPoint("torso", desired_point, torso_point);
        
    ROS_DEBUG("desired_point: (%.2f, %.2f. %.2f) -----> torso_point: (%.2f, %.2f, %.2f) at time %.2f",
        desired_point.point.x, desired_point.point.y, desired_point.point.z,
        torso_point.point.x, torso_point.point.y, torso_point.point.z, torso_point.header.stamp.toSec());
      }
  catch(tf::TransformException& ex){
    ROS_ERROR("1. Received an exception trying to transform: %s", ex.what());
    return false;
  }
  
    try{  //transform from desired_point to neck_pan
        listener.transformPoint("neck_pan", desired_point, target_to_head_pan_point);
        
    ROS_DEBUG("desired_point: (%.2f, %.2f. %.2f) -----> head_pan: (%.2f, %.2f, %.2f) at time %.2f",
        desired_point.point.x, desired_point.point.y, desired_point.point.z,
        target_to_head_pan_point.point.x, target_to_head_pan_point.point.y, target_to_head_pan_point.point.z, target_to_head_pan_point.header.stamp.toSec());
      }
  catch(tf::TransformException& ex){
    ROS_ERROR("1. Received an exception trying to transform: %s", ex.what());
    return false;
  }
 
  //get head and neck joint limits
  double neck_lower = -2; ///model_data.joint_min.data[0];
  double neck_upper = 2 ;///model_data.joint_max.data[0];
  
  double head_lower = -2;///model_data.joint_min.data[1];
  double head_upper = 2;///model_data.joint_max.data[1];
  
  ROS_DEBUG("neck_lower = %f, neck_upper = %f",neck_lower,neck_upper);
  ROS_DEBUG("head_lower = %f, head_upper = %f",head_lower,head_upper);
  
  double pan_angle, tilt_angle;
  
  //compute neck pan angle
  pan_angle = -atan2(torso_point.point.y, torso_point.point.z);
     
  //distance from neck_tilt joint to torso
  double h_neck = fabs(torso_point_neck.point.x);
  double b_neck = fabs(torso_point_neck.point.z);
  ROS_DEBUG(" h_neck = %f, b_neck = %f", h_neck, b_neck);
  
  //distance from head to neck_tilt 
  double h_head = fabs(head_point_neck.point.z);
  ROS_DEBUG(" h_head = %f", h_head);
  
  //compute vertical distance = between tracked point en neck_tilt
  double delta_h = torso_point.point.x - h_neck;
  
  //compute horizontal distance between tracked point en neck_tilt
  double delta_b = target_to_head_pan_point.point.x - b_neck;
  
  //compute distance from tracked point to 
  double r = sqrt(delta_h * delta_h + delta_b * delta_b);
  
  double alpha = atan2(delta_h, delta_b); 
  double beta = 0.5*PI - acos( h_head/r );
  
  //compute total tilt angle
  tilt_angle = (-alpha + beta);
  
  ROS_DEBUG("delta_h = %f\t, delta_b = %f\t, r=%f",delta_h, delta_b,r);
  ROS_DEBUG("alpha=%f, beta=%f\t, tilt_angle =%f",alpha,beta,tilt_angle);
  ROS_DEBUG("Pan angle = %f\t, , tilt_angle =%f",pan_angle,tilt_angle);
  
  //populate object to be published
  head_ref.head_pan = pan_angle;
  head_ref.head_tilt = tilt_angle;

  //publish angles over ROS  
  head_pub.publish(head_ref);

  //publish visualization marker for rviz
  publishMarker();
  
  //publish to dynamixel
  dynamixel_msg.id = 0;
  dynamixel_msg.goal = tilt_angle;
  
  dynamixel_pub.publish(dynamixel_msg);

  
  }
  
  return true;
}


void publishMarker(void){

  //create marker object
  visualization_msgs::Marker marker;
  
  
  uint32_t shape = visualization_msgs::Marker::SPHERE;
  
  // Set the frame ID and timestamp.  
  marker.header.frame_id = target.header.frame_id;
  marker.header.stamp = ros::Time::now();

  marker.ns = "head_target";
  marker.id = 0;

  // Set the marker type. 
  marker.type = shape;

  // Set the marker action. 
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker. 
  marker.pose.position.x = target.point.x;
  marker.pose.position.y = target.point.y;
  marker.pose.position.z = target.point.z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 0.6;

  marker.lifetime = ros::Duration();

  // Publish the marker
  marker_pub.publish(marker);

}



int main(int argc, char** argv){
  ros::init(argc, argv, "head_ref");
  ros::NodeHandle nh;

  //set topic
  head_pub = nh.advertise<amigo_msgs::head_ref>("/head_controller/set_Head", 50);
  marker_pub = nh.advertise<visualization_msgs::Marker>("target_head", 1);
  dynamixel_pub = nh.advertise<dynamixel::angle>("ax_pos", 1000);

  
  target_sub = nh.subscribe("head_target", 1, targetCallback);
  
  received = false;
  /*
  //create ModelData object
  ModelData k;  
  if (k.init()<0) {
        ROS_ERROR("Could not get amigo model");
        return -1;
  }
*/
  tf::TransformListener listener(ros::Duration(10));

  //transform points with certain time interval
  ros::Timer timer = nh.createTimer(ros::Duration(0.1), boost::bind(&transformPoint,boost::ref(listener)));
  
  ROS_INFO("Amigo_head_ref active and waiting for target");
  ros::spin();

  return true;
}
