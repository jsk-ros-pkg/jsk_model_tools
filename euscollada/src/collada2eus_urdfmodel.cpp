/* Author: Yohei Kakiuchi */
#include <ros/ros.h>
#include "collada_parser/collada_parser.h"
#include "urdf/model.h"

#if IS_ASSIMP3
// assimp 3 (assimp_devel)
#include <assimp/IOSystem.hpp>
#include <assimp/IOStream.hpp>
#include <assimp/Importer.hpp>
#include <assimp/Exporter.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#else
// assimp 2
#include <assimp.hpp>
//#include <export.hpp>
#include <aiScene.h>
#include <aiPostProcess.h>
#endif

#include <iostream>
#include <fstream>

#include <boost/program_options.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>

#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Quaternion.h>


using namespace urdf;
using namespace std;

//
class ModelEuslisp {

public:
  ModelEuslisp () { } ;
  ModelEuslisp (boost::shared_ptr<ModelInterface> r);
  ~ModelEuslisp ();

  bool setRobotName (string &name) { arobot_name = name; };
  bool writeToFile (string &filename);
  void addChildLinkNames(boost::shared_ptr<const Link> link);
  void addChildJointNames(boost::shared_ptr<const Link> link);

private:
  boost::shared_ptr<ModelInterface> robot;
  string arobot_name;

  //m_link_assoc <link, link>
  //m_materials <name, material>
};

ModelEuslisp::ModelEuslisp (boost::shared_ptr<ModelInterface> r) {
  robot = r;
}

ModelEuslisp::~ModelEuslisp () {

}

void ModelEuslisp::addChildLinkNames(boost::shared_ptr<const Link> link) {
  std::cerr << "lk_name: " << link->name << std::endl;
  for (std::vector<boost::shared_ptr<Link> >::const_iterator child = link->child_links.begin();
       child != link->child_links.end(); child++) {
    // m_link_assoc (link, child);
    // 
    addChildLinkNames(*child);
  }
}

void ModelEuslisp::addChildJointNames(boost::shared_ptr<const Link> link) {
  for (std::vector<boost::shared_ptr<Link> >::const_iterator child = link->child_links.begin();
       child != link->child_links.end(); child++){

    std::cerr << "jk_name: " << (*child)->parent_joint->name << std::endl;

    addChildJointNames(*child);
  }
}

bool ModelEuslisp::writeToFile (string &filename) {
  if (!robot) {
    cerr << ";; not robot" << std::endl;
    return false;
  }

  addChildLinkNames(robot->getRoot());
  addChildJointNames(robot->getRoot());
}

namespace po = boost::program_options;
int main(int argc, char** argv)
{
  bool use_simple_visual;
  bool use_simple_collision;
  string output_file;
  string arobot_name;
  string input_file;

  po::options_description desc("Options for collada_to_urdf");
  desc.add_options()
    ("help", "produce help message")
    ("simple_visual,V", "use bounding box for visual")
    ("simple_collision,C", "use bounding box for collision")
    ("use_collision,U", "use collision geometry (default collision is the same as visual)")
    ("robot_name,N", po::value< vector<string> >(), "output robot name")
    ("output_file,O", po::value< vector<string> >(), "output file")
    ("input_file", po::value< vector<string> >(), "input file")
    ;

  po::positional_options_description p;
  p.add("input_file", -1);

  po::variables_map vm;
  try {
    po::store(po::command_line_parser(argc, argv).
              options(desc).positional(p).run(), vm);
    po::notify(vm);
  }
  catch (po::error e) {
    cerr << ";; option parse error / " << e.what() << endl;
    return 1;
  }

  if (vm.count("help")) {
    cout << desc << "\n";
    return 1;
  }
  if (vm.count("simple_visual")) {
    use_simple_visual = true;
    cerr << ";; Using simple_visual" << endl;
  }
  if (vm.count("simple_collision")) {
    use_simple_collision = true;
    cerr << ";; Using simple_collision" << endl;
  }

  if (vm.count("output_file")) {
    vector<string> aa = vm["output_file"].as< vector<string> >();
    cerr << ";; output file is: "
         <<  aa[0] << endl;
    output_file = aa[0];
  }
  if (vm.count("robot_name")) {
    vector<string> aa = vm["robot_name"].as< vector<string> >();
    cerr << ";; robot_name is: "
         <<  aa[0] << endl;
    arobot_name = aa[0];
  }
  if (vm.count("input_file")) {
    vector<string> aa = vm["input_file"].as< vector<string> >();
    cerr << ";; Input file is: "
         <<  aa[0] << endl;
    input_file = aa[0];
  }

  std::string xml_string;
  std::fstream xml_file(input_file.c_str(), std::fstream::in);
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
    ROS_DEBUG("Parsing robot collada xml string");
    robot = parseCollada(xml_string);
  }
  else
  {
    ROS_DEBUG("Parsing robot urdf xml string");
    robot = parseURDF(xml_string);
  }

  if (!robot){
    std::cerr << "ERROR: Model Parsing the xml failed" << std::endl;
    return -1;
  }

  if (arobot_name == "") {
    arobot_name = robot->getName();
  }
  if (output_file == "") {
    output_file =  arobot_name + ".urdf";
  }

  ModelEuslisp eusmodel(robot);
  eusmodel.setRobotName(arobot_name);
  eusmodel.writeToFile (output_file);
}
