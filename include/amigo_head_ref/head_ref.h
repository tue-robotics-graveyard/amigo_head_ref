#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <cstring>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_datatypes.h>
#include <amigo_msgs/head_ref.h>
#include <visualization_msgs/Marker.h>
#include <kdl_parser/kdl_parser.hpp>
#include <dynamixel/angle.h>



class ModelData {
	
	public:
	  ModelData();
	  bool init();
	  std::vector<double> joint_min;
	  std::vector<double> joint_max;
	  
	private:
	  ros::NodeHandle nh, nh_private;
	  
	  
	  bool loadModel(const std::string xml);
      bool readJoints(urdf::Model &robot_model);
};

bool transformPoint(const tf::TransformListener& listener, ModelData model_data);
void targetCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
void publishMarker(void);

ros::Publisher marker_pub;
ros::Publisher head_pub;
ros::Publisher dynamixel_pan_pub;
ros::Publisher dynamixel_tilt_pub;


ros::Subscriber target_sub;

amigo_msgs::head_ref head_ref;



geometry_msgs::PointStamped target;
bool received;
