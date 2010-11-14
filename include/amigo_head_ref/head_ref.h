#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <cstring>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_datatypes.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kinematics_msgs/KinematicSolverInfo.h>
#include <amigo_msgs/head_ref.h>
#include <visualization_msgs/Marker.h>



class ModelData {
	
	public:
	  ModelData();
	  bool init();
	  KDL::JntArray joint_min, joint_max;
	  
	private:
	  ros::NodeHandle nh, nh_private;
	  
	  
      kinematics_msgs::KinematicSolverInfo info;
      //std::string root_name, tip_name;
	  
	  bool loadModel(const std::string xml);
      bool readJoints(urdf::Model &robot_model);
};

bool transformPoint(const tf::TransformListener& listener, ModelData model_data);
void publishMarker(void);

ros::Publisher marker_pub;
ros::Publisher head_pub;
amigo_msgs::head_ref head_ref;

std::string target_frame;
double target_x, target_y, target_z;
