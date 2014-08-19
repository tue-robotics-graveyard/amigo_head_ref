#include "amigo_head_ref/head_ref.h"

void measurmentsCallBack(const sensor_msgs::JointState& msg) {
    for(unsigned int i = 0; i < msg.name.size(); ++i) {
        if (msg.name[i] == "neck_pan_joint") {
            current_pan_ = msg.position[i];
        } else if  (msg.name[i] == "neck_tilt_joint") {
            current_tilt_ = msg.position[i];
        }
    }
}

void targetCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {

	tf::pointStampedMsgToTF(*msg, goal_target_);
	goal_target_.stamp_ = ros::Time();

	ROS_INFO("targetCallback");

	if (as_->isActive()) {
		amigo_head_ref::HeadRefResult result;
		result.result = "Head action canceled due published head target";
		as_->setPreempted(result);
	}

	goal_type_ = LOOKAT;
	keep_tracking_ = true;
}

void goalCB() {

	ROS_DEBUG("goalCB");

	const amigo_head_ref::HeadRefGoalConstPtr &goal = as_->acceptNewGoal();

	if (goal->goal_type == amigo_head_ref::HeadRefGoal::LOOKAT) {
		tf::pointStampedMsgToTF(goal->target_point, goal_target_);
		goal_target_.stamp_ = ros::Time();
		targetToPanTilt(goal_target_, goal_pan_, goal_tilt_);
		goal_type_ = LOOKAT;
		keep_tracking_ = goal->keep_tracking;

		publishMarker(goal_target_);
	} else if (goal->goal_type == amigo_head_ref::HeadRefGoal::PAN_TILT) {
		goal_pan_ = goal->pan;
		goal_tilt_ = goal->tilt;
		goal_type_ = PAN_TILT;
		keep_tracking_ = false;
	} else {
		goal_type_ = NONE;
	}
	goal_pan_vel_ = goal->pan_vel;
	goal_tilt_vel_ = goal->tilt_vel;

    // set default min/max
    double min_pan = min_pan_;
    double max_pan = max_pan_;
    double min_tilt = min_tilt_;
    double max_tilt = max_tilt_;

    // if set in action goal, override min/max
    if (goal->min_pan != goal->max_pan) {
        min_pan = goal->min_pan;
        max_pan = goal->max_pan;
    }
    if (goal->min_tilt != goal->max_tilt) {
        min_tilt = goal->min_tilt;
        max_tilt = goal->max_tilt;
    }

    if (keep_tracking_) {
        // constraint pan
        if (goal_pan_ < min_pan) {
            goal_pan_ = min_pan;
        } else if (goal_pan_ > max_pan) {
            goal_pan_ = max_pan;
        }

        // contraint tilt
        if (goal_tilt_ < min_tilt) {
            goal_tilt_ = min_tilt;
        } else if (goal_tilt_ > max_tilt) {
            goal_tilt_ = max_tilt;
        }
    } else {
        if ((goal_pan_ < min_pan || goal_pan_ > max_pan
                || goal_tilt_ < min_tilt || goal_tilt_ > max_tilt)) {
            // pan / tilt out of bounds
            amigo_head_ref::HeadRefResult result;
            result.result = "Pan / tilt out of bounds";
            as_->setAborted(result);
            goal_type_ = NONE;
        }
    }

}

void preemtCB() {
	// set the action state to preempted
	//ROS_INFO("Preemted");
	as_->setPreempted();
	goal_type_ = NONE;
}

bool targetToPanTilt(const tf::Stamped<tf::Point>& target, double& pan, double& tilt) {

	//ROS_INFO("TARGET: (%.2f, %.2f, %.2f) in frame '%s'",target.getX() ,target.getY(), target.getZ(), target.frame_id_.c_str());

	tf::Stamped<tf::Point> target_HEAD_MOUNT;
	try {
        tf_listener_->transformPoint("/amigo/head_mount", target, target_HEAD_MOUNT);
	} catch(tf::TransformException& ex){
		ROS_ERROR("%s", ex.what());
		return false;
	}

	tf::Stamped<tf::Point> target_NECK_TILT;
	try {
        tf_listener_->transformPoint("/amigo/neck_tilt", target, target_NECK_TILT);
	} catch(tf::TransformException& ex){
		ROS_ERROR("%s", ex.what());
		return false;
	}

	//ROS_INFO("target_NECK_PAN = %f, %f, %f", target_HEAD_MOUNT.getX(), target_HEAD_MOUNT.getY(), target_HEAD_MOUNT.getZ());

	double head_mount_to_neck;
	try {
		tf::StampedTransform transform;
        tf_listener_->lookupTransform("/amigo/head_mount", "/amigo/neck_tilt", ros::Time(), transform);
		head_mount_to_neck = transform.getOrigin().getX();
	} catch(tf::TransformException& ex){
		ROS_ERROR("%s", ex.what());
		return false;
	}

	double neck_to_cam_vert;
	try {
		tf::StampedTransform transform;
        tf_listener_->lookupTransform("/amigo/neck_tilt", "/amigo/top_kinect/openni_camera", ros::Time(), transform);
		neck_to_cam_vert = transform.getOrigin().getZ();
	} catch(tf::TransformException& ex){
		ROS_ERROR("%s", ex.what());
		return false;
	}

	pan = -atan2(target_HEAD_MOUNT.getY(), target_HEAD_MOUNT.getZ());

	double neck_to_target = target_NECK_TILT.length();

	double target_to_neck_vert = target_HEAD_MOUNT.getX() - head_mount_to_neck;

	double head_mount_to_target_flat = sqrt(target_HEAD_MOUNT.getY() * target_HEAD_MOUNT.getY()
			+ target_HEAD_MOUNT.getZ() * target_HEAD_MOUNT.getZ());

	//ROS_INFO("target_to_neck_vert = %f", target_to_neck_vert);
	//ROS_INFO("head_mount_to_target_flat = %f", head_mount_to_target_flat);
	//ROS_INFO("neck_to_cam_vert = %f", neck_to_cam_vert);
	//ROS_INFO("neck_to_target = %f", neck_to_target);

	double tilt_basic = -atan(target_to_neck_vert / head_mount_to_target_flat);
	double tilt_camera_offset_correction =  asin(neck_to_cam_vert / neck_to_target);

	tilt = tilt_basic + tilt_camera_offset_correction;

	//ROS_INFO("Pan = %f, tilt = %f", pan, tilt);

	return true;
}

void generateHeadReference() {

	if (goal_type_ == NONE) {
		return;
	}

	if (goal_type_ == LOOKAT) {
		targetToPanTilt(goal_target_, goal_pan_, goal_tilt_);
		publishMarker(goal_target_);
	}

	if (fabs(goal_pan_ - current_pan_) < goal_error_tolerance_pan_ && fabs(goal_tilt_ - current_tilt_) < goal_error_tolerance_tilt_) {
		// pan and tilt are within error bound

		if (as_->isActive() && !keep_tracking_) {
			amigo_head_ref::HeadRefResult result;
			ROS_DEBUG("Target reached, ready for next head target!");
			result.result = "Done";
			as_->setSucceeded(result);
			goal_type_ = NONE;
		}

		return;
	}

	// populate msg
    sensor_msgs::JointState head_ref;
    head_ref.name.push_back("neck_pan_joint");
    head_ref.name.push_back("neck_tilt_joint");

    head_ref.position.push_back(goal_pan_);
    head_ref.position.push_back(goal_tilt_);
    head_ref.velocity.push_back(goal_pan_vel_);
    head_ref.velocity.push_back(goal_tilt_vel_);

	//publish angles over ROS
	head_pub_.publish(head_ref);
}

void publishMarker(const tf::Stamped<tf::Point>& target) {

	//create marker object
	visualization_msgs::Marker marker;


	uint32_t shape = visualization_msgs::Marker::SPHERE;

	// Set the frame ID and timestamp.
	marker.header.frame_id = target.frame_id_;
	marker.header.stamp = ros::Time::now();

	marker.ns = "head_target";
	marker.id = 0;

	// Set the marker type.
	marker.type = shape;

	// Set the marker action.
	marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.
	marker.pose.position.x = target.getX();
	marker.pose.position.y = target.getY();
	marker.pose.position.z = target.getZ();
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = 0.08;
	marker.scale.y = 0.08;
	marker.scale.z = 0.08;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 1.0f;
	marker.color.g = 0.0f;
	marker.color.b = 1.0f;
	marker.color.a = 0.6;

	marker.lifetime = ros::Duration();

	// Publish the marker
	marker_pub_.publish(marker);

}

int main(int argc, char** argv){
	ros::init(argc, argv, "head_ref_action");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	//set topic
    head_pub_ = nh.advertise<sensor_msgs::JointState>("/amigo/neck/references", 50);
	marker_pub_ = nh.advertise<visualization_msgs::Marker>("head_target_marker", 1);

	//get namespace
	std::string ns = ros::this_node::getName();

	//get parameters
	nh_private.param<double>("min_pan", min_pan_, -2.681);
	nh_private.param<double>("max_pan", max_pan_, 2.681);
	nh_private.param<double>("min_tilt", min_tilt_, -0.3787);
	nh_private.param<double>("max_tilt", max_tilt_, 1.0236);

	nh_private.param<double>("max_vel_pan", max_vel_pan_, 0.7);
	nh_private.param<double>("max_vel_tilt", max_vel_tilt_, 0.7);
	nh_private.param<double>("max_acc_pan", max_acc_pan_, 0.7);
	nh_private.param<double>("max_acc_tilt", max_acc_tilt_, 0.7);

	nh_private.param<double>("frequency", frequency_, 25);
	nh_private.param<double>("goal_tolerance_pan", goal_error_tolerance_pan_, 0.02);
    nh_private.param<double>("goal_tolerance_tilt", goal_error_tolerance_tilt_, 0.06);

	//Subscribers
    ros::Subscriber head_meas = nh.subscribe("/amigo/neck/measurements", 1, measurmentsCallBack);

    ros::Subscriber target_sub = nh.subscribe("/head_target", 1, targetCallback);

	// Construct action server
	as_ = new actionlib::SimpleActionServer<amigo_head_ref::HeadRefAction>(nh, "/head_ref_action", false);
	as_->registerGoalCallback(boost::bind(&goalCB));
	as_->registerPreemptCallback(boost::bind(&preemtCB));

	as_->start();

	current_pan_ = 0.0;
	current_tilt_ = 0.0;

	goal_pan_ = 0;
	goal_tilt_ = 0;

	goal_pan_vel_ = 0;
	goal_tilt_vel_ = 0;

	goal_type_ = NONE;

	ROS_INFO("Amigo_head_ref active and waiting for target");

	tf_listener_ = new tf::TransformListener();

	ros::Rate r(frequency_);
	while (nh.ok()) {
		ros::spinOnce();
		generateHeadReference();
		r.sleep();
	}

	delete as_;
	delete tf_listener_;

	return true;
}

