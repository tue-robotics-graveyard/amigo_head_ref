#include <amigo_head_ref/head_ref.h>

ModelData::ModelData(): nh_private ("~") {
	}

bool ModelData::loadModel(const std::string xml) {
    urdf::Model robot_model;
    KDL::Tree tree;

    if (!robot_model.initString(xml)) {
        ROS_FATAL("Could not initialize robot model");
        return -1;
    }
    if (!kdl_parser::treeFromString(xml, tree)) {
        ROS_ERROR("Could not initialize tree object");
        return false;
    }
    if (!readJoints(robot_model)) {
        ROS_FATAL("Could not read information about the joints");
        return false;
    }

    return true;
}


bool ModelData::readJoints(urdf::Model &robot_model) {
    int num_joints = 0;
    // get joint maxs and mins
    boost::shared_ptr<const urdf::Link> link = robot_model.getLink("camera_tf");
    boost::shared_ptr<const urdf::Joint> joint;

    while (link && link->name != "torso") {
        joint = robot_model.getJoint(link->parent_joint->name);
        if (!joint) {
            ROS_ERROR("Could not find joint: %s",link->parent_joint->name.c_str());
            return false;
        }
        if (joint->type != urdf::Joint::UNKNOWN ){//&& joint->type != urdf::Joint::FIXED) {
            ROS_INFO( "adding joint: [%s]", joint->name.c_str() );
            num_joints++;
        }
        link = robot_model.getLink(link->getParent()->name);
    }

    joint_min.resize(num_joints);
    joint_max.resize(num_joints);
    info.joint_names.resize(num_joints);
    info.limits.resize(num_joints);

    link = robot_model.getLink("head");
    unsigned int i = 0;
    while (link && i < num_joints) {
        joint = robot_model.getJoint(link->parent_joint->name);
        if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
            ROS_INFO( "getting bounds for joint: [%s]", joint->name.c_str() );

            float lower, upper;
            int hasLimits;
            if ( joint->type != urdf::Joint::CONTINUOUS ) {
                lower = joint->limits->lower;
                upper = joint->limits->upper;
                hasLimits = 1;
            } else {
                lower = -M_PI;
                upper = M_PI;
                hasLimits = 0;
            }
            int index = num_joints - i -1;

            joint_min.data[index] = lower;
            joint_max.data[index] = upper;
            i++;
            ROS_INFO("joint name = %s\t, low lim = %f\t, up lim =%f",joint->name.c_str(),lower, upper);
        }
        link = robot_model.getLink(link->getParent()->name);
    }
    return true;
}



bool ModelData::init() {
    // Get URDF XML
    std::string urdf_xml, full_urdf_xml;
    nh.param("urdf_xml",urdf_xml,std::string("robot_description"));
    nh.searchParam(urdf_xml,full_urdf_xml);
    ROS_DEBUG("Reading xml file from parameter server");
    std::string result;
    if (!nh.getParam(full_urdf_xml, result)) {
        ROS_FATAL("Could not load the xml from parameter server: %s", urdf_xml.c_str());
        return false;
    }

    // Load and Read Models
    if (!loadModel(result)) {
        ROS_FATAL("Could not load models!");
        return false;
    }
    return true;
}
