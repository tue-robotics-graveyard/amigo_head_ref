#include <ros/ros.h>
#include <amigo_head_ref/head_ref.h>

#define PI 3.14159265

bool transformPoint(const tf::TransformListener& listener, ModelData model_data){
  
  geometry_msgs::PointStamped desired_point,torso_point, torso_point2, head_point, head_point2, camera_point;
  
  //define desired point
  desired_point.header.frame_id = target_frame;
  desired_point.header.stamp = ros::Time();
  desired_point.point.x = target_x;
  desired_point.point.y = target_y;
  desired_point.point.z = target_z;
  
  //define head point
  head_point.header.frame_id = "head";
  head_point.header.stamp = ros::Time();
  head_point.point.x = 0.0;
  head_point.point.y = 0.0;
  head_point.point.z = 0.0;
  
  //define camera_tf point
  camera_point.header.frame_id = "camera_tf";
  camera_point.header.stamp = ros::Time();
  camera_point.point.x = 0.0;
  camera_point.point.y = 0.0;
  camera_point.point.z = 0.0;
  
  
  try{  //transform from desired_point to torso
        listener.transformPoint("torso", desired_point, torso_point);
        
    ROS_DEBUG("desired_point: (%.2f, %.2f. %.2f) -----> torso_point: (%.2f, %.2f, %.2f) at time %.2f",
        desired_point.point.x, desired_point.point.y, desired_point.point.z,
        torso_point.point.x, torso_point.point.y, torso_point.point.z, torso_point.header.stamp.toSec());
      }
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform: %s", ex.what());
    return false;
  }
  
  try{  //transform from head to torso
        listener.transformPoint("torso", head_point, torso_point2);

    ROS_DEBUG("head_point: (%.2f, %.2f. %.2f) -----> torso_point: (%.2f, %.2f, %.2f) at time %.2f",
        head_point.point.x, head_point.point.y, head_point.point.z,
        torso_point2.point.x, torso_point2.point.y, torso_point2.point.z, torso_point2.header.stamp.toSec());
  }
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform: %s", ex.what());
    return false;
  }
 
   try{  //transform from camera_tf to head
        listener.transformPoint("head", camera_point, head_point2);

    ROS_DEBUG("camera_point: (%.2f, %.2f. %.2f) -----> head_point: (%.2f, %.2f, %.2f) at time %.2f",
        camera_point.point.x, camera_point.point.y, camera_point.point.z,
        head_point2.point.x, head_point2.point.y, head_point2.point.z, head_point2.header.stamp.toSec());
  }
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform: %s", ex.what());
    return false;
  }
 
  //get head and neck joint limits
  double neck_lower = model_data.joint_min.data[0];
  double neck_upper = model_data.joint_max.data[0];
  
  double head_lower = model_data.joint_min.data[1];
  double head_upper = model_data.joint_max.data[1];
  
  ROS_DEBUG("neck_lower = %f, neck_upper = %f",neck_lower,neck_upper);
  ROS_DEBUG("head_lower = %f, head_upper = %f",head_lower,head_upper);
  
  double pan_angle, tilt_angle;
  
  //compute neck pan angle
  pan_angle = -atan2(torso_point.point.y, torso_point.point.z);
  
  //limit neck pan angle
  
   
  //distance from head joint to torso
  double h_neck = fabs(torso_point2.point.x);
  double b_neck = fabs(torso_point2.point.z);
  
  //distance from camera_tf to head joint
  double h_head = fabs(head_point2.point.y);
  
  //compute vertical distance to tracked point from height of head joint
  double delta_h = torso_point.point.x - h_neck;
  double delta_b = torso_point.point.z - b_neck;
  
  //compute distance from tracked point to 
  double r = sqrt(delta_h * delta_h + delta_b * delta_b);
  
  double alpha = atan2(delta_h, delta_b);
  
  double beta = acos( h_head/r );
  
  //compute total angle
  tilt_angle = (alpha + beta) - 0.5*PI;
  
  ROS_DEBUG("delta_h = %f\t, delta_b = %f\t, r=%f",delta_h, delta_b,r);
  ROS_DEBUG("alpha=%f, beta=%f\t, tilt_angle =%f",alpha,beta,tilt_angle);
  ROS_INFO("Pan angle = %f\t, , tilt_angle =%f",pan_angle,tilt_angle);
  
  //populate object to be published
  head_ref.head_pan = pan_angle;
  head_ref.head_tilt = tilt_angle;

  //publish angles over ROS  
  head_pub.publish(head_ref);

  //publish visualization marker for rviz
  publishMarker();
  
  return true;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "head_ref");
  ros::NodeHandle nh;

  ros::Rate rate(1.0);
  //get target frame from parameter server
  while(!nh.getParam("/target_frame",target_frame) && nh.ok())
  {
    ROS_ERROR("No target_frame given. Load a /target_frame on the parameter server.");
    rate.sleep();
  }

  //get parameters from parameter server or set defaults
  nh.param<double> ("pos_x", target_x,0.0);
  nh.param<double> ("pos_y", target_y,0.0);
  nh.param<double> ("pos_z", target_z,0.0);

  //set topic
  head_pub = nh.advertise<amigo_msgs::head_ref>("/head_controller/set_Head", 50);
  marker_pub = nh.advertise<visualization_msgs::Marker>("target_head", 1);


  //create ModelData object
  ModelData k;  
  if (k.init()<0) {
        ROS_ERROR("Could not get amigo model");
        return -1;
  }

  tf::TransformListener listener(ros::Duration(10));

  //transform points with certain time interval
  ros::Timer timer = nh.createTimer(ros::Duration(0.1), boost::bind(&transformPoint,boost::ref(listener),k));
  
  ros::spin();

  return true;
}

void publishMarker(void){

  //create marker object
  visualization_msgs::Marker marker;
  
  
  uint32_t shape = visualization_msgs::Marker::SPHERE;
  
  // Set the frame ID and timestamp.  
  marker.header.frame_id = target_frame;
  marker.header.stamp = ros::Time::now();

  marker.ns = "head_target";
  marker.id = 0;

  // Set the marker type. 
  marker.type = shape;

  // Set the marker action. 
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker. 
  marker.pose.position.x = target_x;
  marker.pose.position.y = target_y;
  marker.pose.position.z = target_z;
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
