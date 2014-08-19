#ifndef HEAD_REF_H_
#define HEAD_REF_H_

// ros
#include <ros/ros.h>

// Actionlib
#include <actionlib/server/simple_action_server.h>
#include <amigo_head_ref/HeadRefAction.h>

// tf
#include <tf/transform_listener.h>

// Message types
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/JointState.h>

#include <visualization_msgs/Marker.h>

#define PI 3.14159265

// * * * * * * * * Goal Specifications * * * * * * * *
enum GoalType {
	NONE,
	LOOKAT,
	PAN_TILT
};

GoalType goal_type_;

tf::Stamped<tf::Point> goal_target_;
double goal_pan_;
double goal_tilt_;
double goal_pan_vel_;
double goal_tilt_vel_;

bool keep_tracking_;

// * * * * * * * * * * * * * * * * * * * * * * * *

// measured pan and tilt
double current_pan_, current_tilt_;

// pan and tilt goal tolerance
double goal_error_tolerance_pan_;
double goal_error_tolerance_tilt_;

// minimum and maximum pan and tilt angles
double min_pan_;
double max_pan_;
double min_tilt_;
double max_tilt_;

// maximum velocity and acceleration
double max_vel_pan_;
double max_vel_tilt_;
double max_acc_pan_;
double max_acc_tilt_;

// publishers
ros::Publisher marker_pub_;
ros::Publisher head_pub_;

// loop frequency
double frequency_;

// tf
tf::TransformListener* tf_listener_;

// actionlib server
actionlib::SimpleActionServer<amigo_head_ref::HeadRefAction>* as_;


void measurmentsCallBack(const sensor_msgs::JointState& msg);

void targetCallback(const geometry_msgs::PointStamped::ConstPtr& msg);

void goalCB();

void preemtCB();

bool targetToPanTilt(const tf::Stamped<tf::Point>& target, double& pan, double& tilt);

void generateHeadReference();

void publishMarker(const tf::Stamped<tf::Point>& target);


#endif /* HEAD_REF_H_ */
