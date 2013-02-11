#include <collada_parser/collada_parser.h>
#include <urdf_parser/urdf_parser.h>
#include <urdf/model.h>

//
#include <iostream>
#include <fstream>

#undef GAZEBO_1_0
#undef GAZEBO_1_3
//#define GAZEBO_1_0
#define GAZEBO_1_3

using namespace urdf;
using namespace std;

int main(int argc, char **argv) {
  if (argc != 5){
    std::cerr << "Usage: collada_gazebo_gen package_name dir input dae" << std::endl;
    std::cerr << "collada file: $(rospack find package_name)/dir/input.dae is needed." << std::endl;
    return -1;
  }
  bool colladafile = false;
  std::string package_name(argv[1]);
  std::string dir_name(argv[2]);
  std::string dae_fname(argv[3]);
  std::string prefix(argv[4]);

  ROS_INFO("%s %s %s -> %s",package_name.c_str(),dir_name.c_str(),dae_fname.c_str(), (dir_name + "/" + dae_fname + "." + prefix).c_str());
  // get the entire file
  std::string xml_string;
  std::fstream xml_file((dir_name + "/" + dae_fname + "." + prefix).c_str(), std::fstream::in);
  while ( xml_file.good() )
  {
      std::string line;
      std::getline( xml_file, line);
      xml_string += (line + "\n");
  }
  xml_file.close();

  boost::shared_ptr<ModelInterface> robot;
  if( xml_string.find("<COLLADA") != std::string::npos )
  {
    ROS_INFO("Parsing robot collada xml string");
    robot = parseCollada(xml_string);
    colladafile = true;
  }
  else
  {
    ROS_INFO("Parsing robot urdf xml string");
    robot = parseURDF(xml_string);
  }

  if (!robot){
    std::cerr << "ERROR: Model Parsing the xml failed" << std::endl;
    return -1;
  }

  {
    int cnt = 0;
    std::ofstream os;
    std::string fname = dir_name + "/" + dae_fname + "_transmission.xacro";
    os.open(fname.c_str());
    if (colladafile) {
      os << "<COLLADA xmlns:xacro=\"http://www.ros.org/wiki/xacro\">\n";
    } else {
      os << "<robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\">\n";
    }
    os << "  <xacro:property name=\"mechanical_reduction\" value=\"1\" />\n";
    os << "  <xacro:property name=\"motor_torque_constant\" value=\"1\" />\n";
    os << "  <xacro:property name=\"pulses_per_revolution\" value=\"90000\" />\n\n";
    os << "  <xacro:macro name=\"pr2_transmission\" params=\"joint_name link_name\">\n";
    os << "    <transmission type=\"pr2_mechanism_model/SimpleTransmission\" name=\"${joint_name}_trans\">\n";
    os << "      <actuator name=\"${joint_name}_motor\" />\n";
    os << "      <joint name=\"${joint_name}\" />\n";
    os << "      <mechanicalReduction>${mechanical_reduction}</mechanicalReduction>\n";
    os << "      <motorTorqueConstant>${motor_torque_constant}</motorTorqueConstant>\n";
    os << "      <pulsesPerRevolution>${pulses_per_revolution}</pulsesPerRevolution>\n";
    os << "    </transmission>\n";
    os << "  </xacro:macro>\n";
    for ( std::map<std::string,boost::shared_ptr<Joint> >::iterator joint = robot->joints_.begin();
          joint != robot->joints_.end(); joint++) {
      if ( joint->second->type == Joint::REVOLUTE ||
           joint->second->type == Joint::CONTINUOUS ) {
        os << "  <xacro:pr2_transmission joint_name=\"" << joint->first;
        os << "\" link_name=\"link" << cnt++ << "\"/>\n";
      }
    }
    if (colladafile) {
      os << "</COLLADA>";
    } else {
      os << "</robot>";
    }
    os.close();
  }

  {
    std::ofstream os;
    std::string fname = dir_name + "/" + dae_fname + "_gazebo.xacro";
    os.open(fname.c_str());
    if (colladafile) {
      os << "<COLLADA xmlns:controller=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#controller\"\n";
    } else {
      os << "<robot xmlns:controller=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#controller\"\n";
    }
    os << "         xmlns:xacro=\"http://www.ros.org/wiki/xacro\">\n";
#ifdef GAZEBO_1_0
    os << "  <xacro:macro name=\"gazebo_reference\" params=\"reference material gravity\">\n";
    os << "    <gazebo reference=\"${reference}\">\n";
    os << "      <material>${material}</material>\n";
    os << "      <turnGravityOff>${gravity}</turnGravityOff>\n";
    os << "    </gazebo>\n";
    os << "  </xacro:macro>\n";
#endif

    if (colladafile) {
      os << "  <extra type=\"physics\" name=\"gazebo\">\n";
      os << "    <technique profile=\"gazebo\">\n";
    }
#ifdef GAZEBO_1_0
    // old gazebo (gazebo on ROS Fuerte)
    os << "      <gazebo>\n";
    os << "        <controller:gazebo_ros_controller_manager\n";
    os << "           name=\"gazebo_ros_controller_manager\"\n";
    os << "           plugin=\"libgazebo_ros_controller_manager.so\">\n";
    os << "          <alwaysOn>true</alwaysOn>\n";
    os << "          <updateRate>1000.0</updateRate>\n";
    os << "        </controller:gazebo_ros_controller_manager>\n";
    os << "      </gazebo>\n";
#endif
#ifdef GAZEBO_1_3
    os << "  <gazebo>" << endl;
    os << "    <!--plugin filename=\"libMultiSenseSLPlugin.so\" name=\"multisense_plugin\" /-->" << endl;
    os << "    <!--plugin filename=\"libAtlasPlugin.so\" name=\"atlas_plugin\"/-->" << endl;
    os << "    <plugin filename=\"libgazebo_ros_joint_trajectory.so\" name=\"joint_trajectory_plugin\">" << endl;
    os << "      <topicName>joint_trajectory</topicName>" << endl;
    os << "      <updateRate>1000.0</updateRate>" << endl;
    os << "    </plugin>" << endl;
    os << "    <plugin filename=\"libgazebo_ros_controller_manager.so\" name=\"gazebo_ros_controller_manager\">" << endl;
    os << "      <alwaysOn>true</alwaysOn>" << endl;
    os << "      <updateRate>1000.0</updateRate>" << endl;
    os << "    </plugin>" << endl;
    os << "  </gazebo>" << endl;
#endif

#ifdef GAZEBO_1_0
    for ( std::map<std::string,boost::shared_ptr<Link> >::iterator link = robot->links_.begin();
          link != robot->links_.end(); link++) {
      os << "      <xacro:gazebo_reference reference=\"" << link->first;
      os << "\" material=\"Gazebo/Blue\" gravity=\"False\" />\n";
    }
#endif
    if (colladafile) {
      os << "    </technique>\n";
      os << "  </extra>\n";
    }

    os << "  <xacro:include filename=\"$(find " << package_name;
    os << ")/" << dir_name << "/" << dae_fname << "_transmission.xacro\" />\n";
    if (colladafile) {
      os << "</COLLADA>\n";
    } else {
      os << "</robot>\n";
    }
    os.close();
  }

  {
    std::ofstream os;
    std::string fname = dir_name + "/" + dae_fname + "_controllers.yaml";
    os.open(fname.c_str());
    os << "fullbody_controller:\n";
    os << "  type: robot_mechanism_controllers/JointTrajectoryActionController\n";
    os << "  joints:\n";
    for ( std::map<std::string,boost::shared_ptr<Joint> >::iterator joint = robot->joints_.begin();
          joint != robot->joints_.end(); joint++) {
      if ( joint->second->type == Joint::REVOLUTE ||
           joint->second->type == Joint::CONTINUOUS ) {
        os << "    - " << joint->first << "\n";
      }
    }
    os << "  gains:\n";
    for ( std::map<std::string,boost::shared_ptr<Joint> >::iterator joint = robot->joints_.begin();
          joint != robot->joints_.end(); joint++) {
      if ( joint->second->type == Joint::REVOLUTE ||
           joint->second->type == Joint::CONTINUOUS ) {
        os << "    " << joint->first << ": {p: 200.0, d: 0.0002, i: 0.0, i_clamp: 0.0}\n"; // ????
      }
    }
    os << "  joint_trajectory_action_node:\n";
    os << "    joints:\n";
    for ( std::map<std::string,boost::shared_ptr<Joint> >::iterator joint = robot->joints_.begin();
          joint != robot->joints_.end(); joint++) {
      if ( joint->second->type == Joint::REVOLUTE ||
           joint->second->type == Joint::CONTINUOUS ) {
        os << "      - " << joint->first << "\n";
      }
    }
    os << "    constraints:\n";
    os << "      goal_time: 0.0\n";
    for ( std::map<std::string,boost::shared_ptr<Joint> >::iterator joint = robot->joints_.begin();
          joint != robot->joints_.end(); joint++) {
      if ( joint->second->type == Joint::REVOLUTE ||
           joint->second->type == Joint::CONTINUOUS ) {
        os << "      " << joint->first << ":\n";
        os << "        goal: 0.02\n";
      }
    }
    os.close();
  }

  return 0;
}
