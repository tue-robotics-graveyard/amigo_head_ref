#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <amigo_head_ref/HeadRefAction.h>

using namespace std;

int main (int argc, char **argv)
{
	ros::init(argc, argv, "dummy_action_client");

	// create the action client
	// true causes the client to spin it's own thread
	actionlib::SimpleActionClient<amigo_head_ref::HeadRefAction> ac("head_ref_action", true);

	ROS_INFO("Waiting for action server to start.");

	// wait for the action server to start
	ac.waitForServer(); //will wait for infinite time

	ROS_INFO("Action server started, sending goal.");

	// send a goal to the action
	amigo_head_ref::HeadRefGoal goal;
	goal.target_point.header.frame_id = "/base_link";
	goal.target_point.header.stamp = ros::Time::now();
	goal.target_point.point.x = atof(argv[1]);
	goal.target_point.point.y = atof(argv[2]);
	goal.target_point.point.z = atof(argv[3]);
	goal.goal_type = amigo_head_ref::HeadRefGoal::LOOKAT;
	goal.keep_tracking = atof(argv[4]);

	cout << "x: " << goal.target_point.point.x << " y: " << goal.target_point.point.y << " z: " << goal.target_point.point.z << endl;

	ac.sendGoalAndWait(goal, ros::Duration(2.0));

	actionlib::SimpleClientGoalState state = ac.getState();
	ROS_INFO("Action finished: %s",state.toString().c_str());

	/*
	cout << "Sent goal, waiting...." << endl;

	usleep(1000000);
	ROS_INFO("Cancelling all goals...");
	ac.cancelAllGoals();
	usleep(1000000);
	state = ac.getState();
	ROS_INFO("Action finished: %s",state.toString().c_str());

	goal.target_point.point.x +=1;
	ac.sendGoal(goal);
	usleep(1000000);
	state = ac.getState();
	ROS_INFO("Action finished: %s",state.toString().c_str());

	//wait for the action to return
	bool finished_before_timeout = ac.waitForResult(ros::Duration(5.0));

	if (finished_before_timeout) {
		actionlib::SimpleClientGoalState state = ac.getState();
		ROS_INFO("Action finished: %s",state.toString().c_str());
	}
	else {
		ac.cancelAllGoals();
		ROS_INFO("Action did not finish before the time out.");
	}

	//exit

	*/

	return 0;
}

