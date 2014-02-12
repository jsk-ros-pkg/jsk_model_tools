/* Author: Yohei Kakiuchi */
#include <ros/ros.h>
#include "collada_parser/collada_parser.h"
#include "urdf/model.h"

#include <sys/utsname.h>
#include <math.h>

#if IS_ASSIMP3
// assimp 3 (assimp_devel)
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/IOStream.hpp>
#include <assimp/IOSystem.hpp>
#else
// assimp 2
#include <assimp/assimp.hpp>
#include <assimp/aiScene.h>
#include <assimp/aiPostProcess.h>
#include <assimp/IOStream.h>
#include <assimp/IOSystem.h>
#endif

#include <iostream>
#include <fstream>

#include <boost/program_options.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/foreach.hpp>

#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Quaternion.h>

#include <resource_retriever/retriever.h>

#include "yaml-cpp/yaml.h"

//extern "C" {
//#include <qhull/qhull_a.h>
//}

#include "rospack/rospack.h"

// copy from rviz/src/rviz/mesh_loader.cpp
class ResourceIOStream : public Assimp::IOStream
{
public:
  ResourceIOStream(const resource_retriever::MemoryResource& res)
  : res_(res)
  , pos_(res.data.get())
  {}

  ~ResourceIOStream()
  {}

  size_t Read(void* buffer, size_t size, size_t count)
  {
    size_t to_read = size * count;
    if (pos_ + to_read > res_.data.get() + res_.size)
    {
      to_read = res_.size - (pos_ - res_.data.get());
    }

    memcpy(buffer, pos_, to_read);
    pos_ += to_read;

    return to_read;
  }

  size_t Write( const void* buffer, size_t size, size_t count) { ROS_BREAK(); return 0; }

  aiReturn Seek( size_t offset, aiOrigin origin)
  {
    uint8_t* new_pos = 0;
    switch (origin)
    {
    case aiOrigin_SET:
      new_pos = res_.data.get() + offset;
      break;
    case aiOrigin_CUR:
      new_pos = pos_ + offset; // TODO is this right?  can offset really not be negative
      break;
    case aiOrigin_END:
      new_pos = res_.data.get() + res_.size - offset; // TODO is this right?
      break;
    default:
      ROS_BREAK();
    }

    if (new_pos < res_.data.get() || new_pos > res_.data.get() + res_.size)
    {
      return aiReturn_FAILURE;
    }

    pos_ = new_pos;
    return aiReturn_SUCCESS;
  }

  size_t Tell() const
  {
    return pos_ - res_.data.get();
  }

  size_t FileSize() const
  {
    return res_.size;
  }

  void Flush() {}

private:
  resource_retriever::MemoryResource res_;
  uint8_t* pos_;
};

class ResourceIOSystem : public Assimp::IOSystem
{
public:
  ResourceIOSystem()
  {
  }

  ~ResourceIOSystem()
  {
  }

  // Check whether a specific file exists
  bool Exists(const char* file) const
  {
    // Ugly -- two retrievals where there should be one (Exists + Open)
    // resource_retriever needs a way of checking for existence
    // TODO: cache this
    resource_retriever::MemoryResource res;
    try
    {
      res = retriever_.get(file);
    }
    catch (resource_retriever::Exception& e)
    {
      return false;
    }

    return true;
  }

  // Get the path delimiter character we'd like to see
  char getOsSeparator() const
  {
    return '/';
  }

  // ... and finally a method to open a custom stream
  Assimp::IOStream* Open(const char* file, const char* mode = "rb")
  {
    ROS_ASSERT(mode == std::string("r") || mode == std::string("rb"));

    // Ugly -- two retrievals where there should be one (Exists + Open)
    // resource_retriever needs a way of checking for existence
    resource_retriever::MemoryResource res;
    try
    {
      res = retriever_.get(file);
    }
    catch (resource_retriever::Exception& e)
    {
      return 0;
    }

    return new ResourceIOStream(res);
  }

  void Close(Assimp::IOStream* stream);

private:
  mutable resource_retriever::Retriever retriever_;
};

void ResourceIOSystem::Close(Assimp::IOStream* stream)
{
  delete stream;
}

////
using namespace urdf;
using namespace std;

#define FLOAT_PRECISION_FINE   "%.16e"
#define FLOAT_PRECISION_COARSE "%.3f"
//
class ModelEuslisp {

public:
  ModelEuslisp () { } ;
  ModelEuslisp (boost::shared_ptr<ModelInterface> r);
  ~ModelEuslisp ();

  // accessor
  void setRobotName (string &name) { arobot_name = name; };
  void setUseCollision(bool &b) { use_collision = b; };
  void setUseSimpleGeometry(bool &b) { use_simple_geometry = b; };
  void setAddJointSuffix(bool &b) { add_joint_suffix = b; };
  void setAddLinkSuffix(bool &b) { add_link_suffix = b; };

  // methods for parsing robot model for euslisp
  void addLinkCoords();
  void printMesh(const aiScene* scene, const aiNode* node, const string &material_name);
  void readYaml(string &config_file);

  Pose getLinkPose(boost::shared_ptr<const Link> link) {
    if (!link->parent_joint) {
      Pose ret;
      return ret;
    }
    Pose p_pose = getLinkPose(link->getParent());
    Pose l_pose = link->parent_joint->parent_to_joint_origin_transform;
    Pose ret;
    ret.rotation = p_pose.rotation * l_pose.rotation;
    ret.position = (p_pose.rotation * l_pose.position) +  p_pose.position;

    return ret;
  }

  void printLink (boost::shared_ptr<const Link> Link, Pose &pose);
  void printJoint (boost::shared_ptr<const Joint> joint);
  void printGeometry (boost::shared_ptr<Geometry> g, const Pose &pose,
                      const string &name, const string &material_name);
  void printLinks ();
  void printJoints ();
  void printEndCoords();
  void printGeometries();

  // print methods
  void copyRobotClassDefinition ();
  void printRobotDefinition();
  void printRobotMethods();

  void writeToFile (string &filename);
private:
  typedef map <string, boost::shared_ptr<const Visual> > MapVisual;
  typedef map <string, boost::shared_ptr<const Collision> > MapCollision;
  typedef pair<vector<string>, vector<string> > link_joint;
  typedef pair<string, link_joint > link_joint_pair;

  boost::shared_ptr<ModelInterface> robot;
  string arobot_name;
  map <boost::shared_ptr<const Link>, Pose > m_link_coords;
  map <boost::shared_ptr<const Link>, MapVisual > m_link_visual;
  map <boost::shared_ptr<const Link>, MapCollision > m_link_collision;
  map <string, boost::shared_ptr<const Material> > m_materials;
  vector<pair<string, string> > g_all_link_names;
  vector<link_joint_pair> limbs;

  FILE *fp;
  YAML::Node doc;

  //
  bool add_joint_suffix;
  bool add_link_suffix;
  bool use_simple_geometry;
  bool use_collision;
};

ModelEuslisp::ModelEuslisp (boost::shared_ptr<ModelInterface> r) {
  robot = r;
  add_joint_suffix = false;
  add_link_suffix = false;
  use_simple_geometry = false;
  use_collision = false;
}

ModelEuslisp::~ModelEuslisp () {

}

void ModelEuslisp::addLinkCoords() {
  for (std::map<std::string, boost::shared_ptr<Link> >::iterator link = robot->links_.begin();
       link != robot->links_.end(); link++) {
    Pose p = getLinkPose(link->second);
    m_link_coords.insert
      (map<boost::shared_ptr<const Link>, Pose >::value_type (link->second, p));
#if DEBUG
    std::cerr << "name: " << link->first;
    std::cerr << ", #f(" << p.position.x << " ";
    std::cerr << p.position.y << " ";
    std::cerr << p.position.z << ") #f(";
    std::cerr << p.rotation.w << " ";
    std::cerr << p.rotation.x << " ";
    std::cerr << p.rotation.y << " ";
    std::cerr << p.rotation.z << ")" << std::endl;
#endif
    if (use_collision) {
      //link->second->collision;
      //link->second->collision_array;
    } else {
      if(!!link->second->visual) {
        m_materials.insert
          (map <string, boost::shared_ptr<const Material> >::value_type
           (link->second->visual->material_name, link->second->visual->material));
        MapVisual mv;
        string gname(link->second->name);
        gname += "_geom0";
        mv.insert(MapVisual::value_type (gname, link->second->visual));

        m_link_visual.insert
          (map <boost::shared_ptr<const Link>, MapVisual >::value_type
           (link->second, mv));
      } else {
        int counter = 0;
        MapVisual mv;
        for (std::vector<boost::shared_ptr <Visual> >::iterator it = link->second->visual_array.begin();
             it != link->second->visual_array.end(); it++) {
          m_materials.insert
            (map <string, boost::shared_ptr<const Material> >::value_type ((*it)->material_name, (*it)->material));
          MapVisual mv;
          stringstream ss;
          ss << link->second->name << "_geom" << counter;
          mv.insert(MapVisual::value_type (ss.str(), (*it)));
          counter++;
        }
        m_link_visual.insert
          (map <boost::shared_ptr<const Link>, MapVisual >::value_type (link->second, mv));
      }
    }
  }
}

void ModelEuslisp::printMesh(const aiScene* scene, const aiNode* node,
                             const string &material_name) {
  aiMatrix4x4 transform = node->mTransformation;
  aiNode *pnode = node->mParent;
  while (pnode)  {
    if (pnode->mParent != NULL) {
      transform = pnode->mTransformation * transform;
    }
    pnode = pnode->mParent;
  }

  aiMatrix3x3 rotation(transform);
  aiMatrix3x3 inverse_transpose_rotation(rotation);
  inverse_transpose_rotation.Inverse();
  inverse_transpose_rotation.Transpose();
  for (uint32_t i = 0; i < node->mNumMeshes; i++) {
    aiMesh* input_mesh = scene->mMeshes[node->mMeshes[i]];
    fprintf(fp, "                  (list ;; mesh description\n");
    fprintf(fp, "                   (list :type :triangles)\n");
    fprintf(fp, "                   (list :material (list\n");
    if (material_name.size() > 0) {
      // TODO: using material_name on urdf
      fprintf(fp, ";; material: %s not using\n", material_name.c_str());
    } else {
      if (!!scene->mMaterials) {
        aiMaterial *am = scene->mMaterials[input_mesh->mMaterialIndex];
        //fprintf(fp, "(list :color (float-vector %f %f %f))\n", );
        //fprintf(fp, "(list :ambient (float-vector %f %f %f))\n", );
        //fprintf(fp, "(list :diffuse (float-vector %f %f %f))\n", );
      }
    }
    fprintf(fp, "))\n");

    fprintf(fp, "                   (list :indices #i(");
    for (uint32_t j = 0; j < input_mesh->mNumFaces; j++) {
      aiFace& face = input_mesh->mFaces[j];
      for (uint32_t k = 0; k < face.mNumIndices; ++k) {
        fprintf(fp, " %d", face.mIndices[k]);
      }
    }
    fprintf(fp, "))\n");

    fprintf(fp, "                   (list :vertices #2f(");
    // Add the vertices
    for (uint32_t j = 0; j < input_mesh->mNumVertices; j++)  {
      aiVector3D p = input_mesh->mVertices[j];
      p *= transform;
      //p *= scale;
      fprintf(fp, "(%f %f %f)", 1000 * p.x, 1000 * p.y, 1000 * p.z);
    }
    fprintf(fp, "))");

    if (input_mesh->HasNormals()) {
      fprintf(fp, "\n");
      fprintf(fp, "                   (list :normals #2f(");
      for (uint32_t j = 0; j < input_mesh->mNumVertices; j++)  {
        aiVector3D n = inverse_transpose_rotation * input_mesh->mNormals[j];
        n.Normalize();
        fprintf(fp, "(%f %f %f)", n.x, n.y, n.z);
      }
      fprintf(fp, "))");
    }
#if 0
    if (input_mesh->HasTextureCoords(0)) {
      for (uint32_t j = 0; j < input_mesh->mNumVertices; j++)  {
        aiVector3D n = inverse_transpose_rotation * input_mesh->mNormals[j];
        //*vertices++ = input_mesh->mTextureCoords[0][j].x;
        //*vertices++ = input_mesh->mTextureCoords[0][j].y;
      }
    }
#endif
    fprintf(fp, ")\n");
  }
  for (uint32_t i = 0; i < node->mNumChildren; ++i) {
    printMesh(scene, node->mChildren[i], material_name);
  }
}

bool limb_order_asc(const pair<string, size_t>& left, const pair<string, size_t>& right) { return left.second < right.second; }
void ModelEuslisp::readYaml (string &config_file) {
  // read yaml
  string limb_candidates[] = {"torso", "larm", "rarm", "lleg", "rleg", "head"}; // candidates of limb names

  vector<pair<string, size_t> > limb_order;
  ifstream fin(config_file.c_str());
  if (fin.fail()) {
    fprintf(stderr, "%c[31m;; Could not open %s%c[m\n", 0x1b, config_file.c_str(), 0x1b);
  } else {
    YAML::Parser parser(fin);
    parser.GetNextDocument(doc);

    /* re-order limb name by lines of yaml */
    BOOST_FOREACH(string& limb, limb_candidates) {
      if ( doc.FindValue(limb) ) {
        std::cerr << limb << "@" << doc[limb].GetMark().line << std::endl;
        limb_order.push_back(pair<string, size_t> (limb, doc[limb].GetMark().line));
      }
    }
    std::sort(limb_order.begin(), limb_order.end(), limb_order_asc);
  }

  // generate limbs including limb_name, link_names, and joint_names
  for (size_t i = 0; i < limb_order.size(); i++) {
    string limb_name = limb_order[i].first;
    vector<string> tmp_link_names, tmp_joint_names;
    try {
      const YAML::Node& limb_doc = doc[limb_name];
      for(unsigned int i = 0; i < limb_doc.size(); i++) {
        const YAML::Node& n = limb_doc[i];
        for(YAML::Iterator it=n.begin();it!=n.end();it++) {
          string key, value; it.first() >> key; it.second() >> value;
          tmp_joint_names.push_back(key);
          boost::shared_ptr<const Joint> jnt = robot->getJoint(key);
          if (!!jnt) {
            tmp_link_names.push_back(jnt->child_link_name);
          } else {
            // error
          }
          g_all_link_names.push_back(pair<string, string>(key, value));
        }
      }
      limbs.push_back(link_joint_pair(limb_name, link_joint(tmp_link_names, tmp_joint_names)));
    } catch(YAML::RepresentationException& e) {
    }
  }
}

void ModelEuslisp::copyRobotClassDefinition () {
  // load class definition
  fprintf(fp, ";;\n");
  fprintf(fp, ";; copy euscollada-robot class definition from euscollada/src/euscollada-robot.l\n");
  fprintf(fp, ";;\n");
  try {
    std::string euscollada_path;

    rospack::Rospack rp;
    std::vector<std::string> search_path;
    rp.getSearchPathFromEnv(search_path);
    rp.crawl(search_path, 1);
    std::string path;
    rp.find("euscollada",euscollada_path);

    euscollada_path += "/src/euscollada-robot.l";
    ifstream fin(euscollada_path.c_str());
    std::string buf;
    while(fin && getline(fin, buf)) {
      fprintf(fp, "%s\n", buf.c_str());
    }

    fprintf(fp, ";;\n");
    fprintf(fp, ";; end of copy from %s\n", euscollada_path.c_str());
    fprintf(fp, ";;\n");
  } catch (runtime_error &e) {
    std::cerr << "cannot resolve euscollada package path" << std::endl;
  }
}

void ModelEuslisp::printRobotDefinition() {
  fprintf(fp, "(defun %s () (setq *%s* (instance %s-robot :init)))\n",
          arobot_name.c_str(), arobot_name.c_str(), arobot_name.c_str());
  fprintf(fp, "\n");
  fprintf(fp, "(defclass %s-robot\n", arobot_name.c_str());
  fprintf(fp, "  :super euscollada-robot\n");
  fprintf(fp, "  :slots ( ;; link names\n");
  fprintf(fp, "         ");
  for (std::map<std::string, boost::shared_ptr<Link> >::iterator link = robot->links_.begin();
       link != robot->links_.end(); link++) {
    if (add_link_suffix) {
      fprintf(fp," %s_lk", link->second->name.c_str());
    } else {
      fprintf(fp," %s", link->second->name.c_str());
    }
  }
  fprintf(fp, "\n         ;; joint names\n");
  fprintf(fp, "         ");
  for (std::map<std::string, boost::shared_ptr<Joint> >::iterator joint = robot->joints_.begin();
       joint != robot->joints_.end(); joint++) {
    if(add_joint_suffix) {
      fprintf(fp, " %s_jt", joint->second->name.c_str());
    } else {
      fprintf(fp, " %s", joint->second->name.c_str());
    }
  }
  fprintf(fp, "\n         )\n  )\n");
  // TODO: add openrave manipulator tip frame
  // TODO: add sensor frames
}

void ModelEuslisp::printRobotMethods() {
  fprintf(fp, "(defmethod %s-robot\n", arobot_name.c_str());
  fprintf(fp, "  (:init\n");
  fprintf(fp, "   (&rest args)\n");
  fprintf(fp, "   (let ()\n");
  // send super :init
  fprintf(fp, "     (send-super* :init :name \"%s\" args)\n", arobot_name.c_str());
  fprintf(fp, "\n");

  printLinks();

  printJoints();

  printEndCoords();

  printGeometries();

  fprintf(fp, "  )\n");
}

void ModelEuslisp::printLinks () {
  for (std::map<boost::shared_ptr<const Link>, Pose >::iterator it = m_link_coords.begin();
       it != m_link_coords.end(); it++) {
    printLink(it->first, it->second);
  }

  if (add_link_suffix) {
    fprintf(fp, "     (send self :assoc %s_lk)\n", robot->root_link_->name.c_str());
  } else {
    fprintf(fp, "     (send self :assoc %s)\n", robot->root_link_->name.c_str());
  }
}

void ModelEuslisp::printLink (boost::shared_ptr<const Link> link, Pose &pose) {
  string thisNodeName;
  if (add_link_suffix) {
    thisNodeName.assign(link->name);
    thisNodeName += "_lk";
  } else {
    thisNodeName.assign(link->name);
  }
  fprintf(fp, "     ;; link: %s\n", thisNodeName.c_str());
  fprintf(fp, "     (let ((geom-lst (list \n");
  {
    map <boost::shared_ptr<const Link>, MapVisual >::iterator it = m_link_visual.find (link);
    if (it != m_link_visual.end()) {
      for( MapVisual::iterator vmap = it->second.begin();
           vmap != it->second.end(); vmap++) {
        fprintf(fp, "                       (send self :_make_instance_%s)", vmap->first.c_str());
      }
    }
    fprintf(fp, ")))\n");
  }
  fprintf(fp, "       (dolist (g (cdr geom-lst)) (send (car geom-lst) :assoc g))\n");
  fprintf(fp, "       (setq %s\n", thisNodeName.c_str());
  fprintf(fp, "             (instance bodyset-link\n");
  fprintf(fp, "                       :init (make-cascoords)\n");
  fprintf(fp, "                       :bodies geom-lst\n");
  fprintf(fp, "                       :name \"%s\"))\n", link->name.c_str());

  if (!!link->inertial) {
    fprintf(fp, "       ;; inertial parameter for %s\n", thisNodeName.c_str());
    fprintf(fp, "       (let ((tmp-cds (make-coords :pos ");
    fprintf(fp, "(float-vector "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE")",
            link->inertial->origin.position.x * 1000,
            link->inertial->origin.position.y * 1000,
            link->inertial->origin.position.z * 1000);
    {
      double qx, qy, qz, qw;
      link->inertial->origin.rotation.getQuaternion(qx, qy, qz, qw);
      if (qx != 0.0 || qy != 0.0 || qz != 0.0 || qw != 1.0) {
        fprintf(fp, "\n                                   :rot (quaternion2matrix ");
        fprintf(fp, "(float-vector "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE"))",
                qw, qx, qy, qz);
      }
      fprintf(fp, ")\n");
    }
    fprintf(fp, "                      ))\n");
    //
    fprintf(fp, "         (send %s :weight %.3f)\n", thisNodeName.c_str(), link->inertial->mass * 1000);
    fprintf(fp, "         (setq (%s . inertia-tensor)\n", thisNodeName.c_str());
    fprintf(fp, "               (matrix\n");
    fprintf(fp, "                 (float-vector "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE")\n",
            link->inertial->ixx * 1000 * 1000 * 1000,
            link->inertial->ixy * 1000 * 1000 * 1000,
            link->inertial->ixz * 1000 * 1000 * 1000);
    fprintf(fp, "                 (float-vector "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE")\n",
            link->inertial->ixy * 1000 * 1000 * 1000,
            link->inertial->iyy * 1000 * 1000 * 1000,
            link->inertial->iyz * 1000 * 1000 * 1000);
    fprintf(fp, "                 (float-vector "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE")))\n",
            link->inertial->ixz * 1000 * 1000 * 1000,
            link->inertial->iyz * 1000 * 1000 * 1000,
            link->inertial->izz * 1000 * 1000 * 1000);
    fprintf(fp, "         (setq (%s . acentroid) (send tmp-cds :worldpos))\n", thisNodeName.c_str());
    fprintf(fp, "         )\n");
  } else {
    fprintf(fp, "       (progn (send %s :weight 0.0) (send %s :centroid (float-vector 0 0 0)) (send %s :inertia-tensor #2f((0 0 0)(0 0 0)(0 0 0))))\n",
            thisNodeName.c_str(), thisNodeName.c_str(), thisNodeName.c_str());
  }
  //
  fprintf(fp, "       ;; global coordinates for %s\n", thisNodeName.c_str());
  fprintf(fp, "       (let ((world-cds (make-coords :pos ");
  fprintf(fp, "(float-vector "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE")",
          pose.position.x * 1000, pose.position.y * 1000, pose.position.z * 1000);
  {
    double qx, qy, qz, qw;
    pose.rotation.getQuaternion(qx, qy, qz, qw);
    if (qx != 0.0 || qy != 0.0 || qz != 0.0 || qw != 1.0) {
      fprintf(fp, "\n                                   :rot (quaternion2matrix ");
      fprintf(fp, "(float-vector "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE"))",
              qw, qx, qy, qz);
    }
    fprintf(fp, ")\n");
  }
  fprintf(fp, "                        ))\n");
  fprintf(fp, "         (send %s :transform world-cds))\n", thisNodeName.c_str());
  fprintf(fp, "       )\n\n");
}

void ModelEuslisp::printJoints () {
  fprintf(fp, "\n     ;; joint models\n");
  for (std::map<std::string, boost::shared_ptr<Joint> >::iterator joint = robot->joints_.begin();
       joint != robot->joints_.end(); joint++) {
    printJoint(joint->second);
  }
}

void ModelEuslisp::printJoint (boost::shared_ptr<const Joint> joint) {
  bool linear = (joint->type==Joint::PRISMATIC);
  if (joint->type != Joint::REVOLUTE && joint->type !=Joint::CONTINUOUS
      && joint->type !=Joint::PRISMATIC && joint->type != Joint::FIXED) {
    // error
  }
  string thisJointName;
  if (add_joint_suffix) {
    thisJointName.assign(joint->name);
    thisJointName += "_jt";
  } else {
    thisJointName.assign(joint->name);
  }
  fprintf(fp, "     ;; joint: %s\n", thisJointName.c_str());
  fprintf(fp, "     (setq %s\n", thisJointName.c_str());
  fprintf(fp, "           (instance %s :init\n", linear?"linear-joint":"rotational-joint");
  fprintf(fp, "                     :name \"%s\"\n", joint->name.c_str());
  if (add_link_suffix) {
    fprintf(fp, "                     :parent-link %s_lk :child-link %s_lk\n",
            joint->parent_link_name.c_str(), joint->child_link_name.c_str());
  } else {
    fprintf(fp, "                     :parent-link %s :child-link %s\n",
            joint->parent_link_name.c_str(), joint->child_link_name.c_str());
  }
  { // axis
    fprintf(fp, "                     :axis ");
    fprintf(fp, "(float-vector "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE")\n",
            joint->axis.x, joint->axis.y, joint->axis.z);
  }
  {
    float min = joint->limits->lower;
    float max = joint->limits->upper;
    fprintf(fp, "                     ");
    fprintf(fp, ":min ");
    if (min == -FLT_MAX) fprintf(fp, "*-inf*"); else
      fprintf(fp, "%f", joint->type ==Joint::PRISMATIC ? min * 1000 : min * 180.0 / M_PI);
    fprintf(fp, " :max ");
    if (max == FLT_MAX) fprintf(fp,  "*inf*"); else
      fprintf(fp, "%f", joint->type ==Joint::PRISMATIC ? max * 1000 : max * 180.0 / M_PI);
    fprintf(fp, "\n");
    fprintf(fp, "                     :max-joint-velocity %f\n", joint->limits->velocity);
    fprintf(fp, "                     :max-joint-torque %f\n", joint->limits->effort);
  }
  fprintf(fp, "                     ))\n");
}

void ModelEuslisp::printEndCoords () {
  // TODO: end coords from collada ...
  fprintf(fp, "     ;; end coords from yaml file\n");
  BOOST_FOREACH(link_joint_pair& limb, limbs) {
    string limb_name = limb.first;
    vector<string> link_names = limb.second.first;

    if (link_names.size()>0) {
      string end_coords_parent_name(link_names.back());
      try {
        const YAML::Node& n = doc[limb_name+"-end-coords"]["parent"];
        n >> end_coords_parent_name;
      } catch(YAML::RepresentationException& e) {
      }
      if (add_link_suffix) {
        fprintf(fp, "     (setq %s-end-coords (make-cascoords :coords (send %s_lk :copy-worldcoords) :name :%s-end-coords))\n",
                limb_name.c_str(), end_coords_parent_name.c_str(), limb_name.c_str());
      } else {
        fprintf(fp, "     (setq %s-end-coords (make-cascoords :coords (send %s :copy-worldcoords) :name %s-end-coords))\n",
                limb_name.c_str(), end_coords_parent_name.c_str(), limb_name.c_str());
      }
      try {
        const YAML::Node& n = doc[limb_name+"-end-coords"]["translate"];
        double value;
        fprintf(fp, "     (send %s-end-coords :translate (float-vector", limb_name.c_str());
        for(unsigned int i = 0; i < 3; i++) { n[i]>>value; fprintf(fp, " "FLOAT_PRECISION_FINE"", 1000*value);}
        fprintf(fp, "))\n");
      } catch(YAML::RepresentationException& e) {
      }
      try {
        const YAML::Node& n = doc[limb_name+"-end-coords"]["rotate"];
        double value;
        fprintf(fp, "     (send %s-end-coords :rotate", limb_name.c_str());
        for(unsigned int i = 3; i < 4; i++) { n[i]>>value; fprintf(fp, " "FLOAT_PRECISION_FINE"", M_PI/180*value);}
        fprintf(fp, " (float-vector");
        for(unsigned int i = 0; i < 3; i++) { n[i]>>value; fprintf(fp, " "FLOAT_PRECISION_FINE"", value);}
        fprintf(fp, "))\n");
      } catch(YAML::RepresentationException& e) {
      }
      if(add_link_suffix) {
        fprintf(fp, "     (send %s_lk :assoc %s-end-coords)\n", end_coords_parent_name.c_str(), limb_name.c_str());
      } else {
        fprintf(fp, "     (send %s :assoc %s-end-coords)\n", end_coords_parent_name.c_str(), limb_name.c_str());
      }
    }
  }
  fprintf(fp, "\n");

  // limb name
  fprintf(fp, "     ;; limbs\n");
  BOOST_FOREACH(link_joint_pair& limb, limbs) {
    string limb_name = limb.first;
    vector<string> link_names = limb.second.first;
    if ( link_names.size() > 0 ) {
      fprintf(fp, "     (setq %s (list", limb_name.c_str());
      if (add_link_suffix) {
        for (unsigned int i = 0; i < link_names.size(); i++)
          fprintf(fp, " %s_lk", link_names[i].c_str());
        fprintf(fp, "))\n");
      } else {
        for (unsigned int i = 0; i < link_names.size(); i++)
          fprintf(fp, " %s", link_names[i].c_str());
        fprintf(fp, "))\n");
      }
      fprintf(fp, "\n");
      // find root link by tracing limb's link list
      fprintf(fp, "     (setq %s-root-link\n", limb_name.c_str());
      fprintf(fp, "           (labels ((find-parent (l) (if (find (send l :parent) %s) (find-parent (send l :parent)) l)))\n",
              limb_name.c_str());
      fprintf(fp, "             (find-parent (car %s))))\n", limb_name.c_str());
    }
  }
  fprintf(fp, "\n");

  // link name
  fprintf(fp, "     ;; links\n");
  if (add_link_suffix) {
    fprintf(fp, "     (setq links (list %s_lk", robot->root_link_->name.c_str());
  } else {
    fprintf(fp, "     (setq links (list %s", robot->root_link_->name.c_str());
  }
  BOOST_FOREACH(link_joint_pair& limb, limbs) {
    string limb_name = limb.first;
    vector<string> link_names = limb.second.first;
    if (add_link_suffix) {
      for (unsigned int i = 0; i < link_names.size(); i++) {
        fprintf(fp, " %s_lk", link_names[i].c_str());
      }
    } else {
      for (unsigned int i = 0; i < link_names.size(); i++) {
        fprintf(fp, " %s", link_names[i].c_str());
      }
    }
  }
  fprintf(fp, "))\n");

  fprintf(fp, "     ;; joint-list\n");
  fprintf(fp, "     (setq joint-list (list");
  BOOST_FOREACH(link_joint_pair& limb, limbs) {
    vector<string> joint_names = limb.second.second;
    if(add_joint_suffix) {
      for (unsigned int i = 0; i < joint_names.size(); i++) {
        fprintf(fp, " %s_jt", joint_names[i].c_str());
      }
    } else {
      for (unsigned int i = 0; i < joint_names.size(); i++) {
        fprintf(fp, " %s", joint_names[i].c_str());
      }
    }
  }
  fprintf(fp, "))\n");
  fprintf(fp, "\n");

  // init ending
  fprintf(fp, "     ;; init-ending\n");
  fprintf(fp, "     (send self :init-ending)\n\n");

  // bodies
  fprintf(fp, "     ;; overwrite bodies to return draw-things links not (send link :bodies)\n");
  fprintf(fp, "     (setq bodies (flatten (mapcar #'(lambda (b) (if (find-method b :bodies) (send b :bodies))) (list");
  for (std::map<std::string, boost::shared_ptr<Link> >::iterator link = robot->links_.begin();
       link != robot->links_.end(); link++) {
    if (add_link_suffix) {
      fprintf(fp, " %s_lk", link->first.c_str());
    } else {
      fprintf(fp, " %s", link->first.c_str());
    }
  }
  fprintf(fp, "))))\n\n");

  // when - angle-vector: reset-pose is defined in yaml file
  try {
    doc["angle-vector"]["reset-pose"];
    fprintf(fp, "     (send self :reset-pose) ;; :set reset-pose\n\n");
  } catch(YAML::RepresentationException& e) {
  }

  fprintf(fp, "     self)) ;; end of :init\n\n");

  try {
    const YAML::Node& n = doc["angle-vector"];
    if ( n.size() > 0 ) fprintf(fp, "  ;; pre-defined pose methods\n");
    for(YAML::Iterator it = n.begin(); it != n.end(); it++) {
      string name; it.first() >> name;
      fprintf(fp, "  (:%s () (send self :angle-vector (float-vector", name.c_str());
      const YAML::Node& v = it.second();
      for(unsigned int i = 0; i < v.size(); i++){
        double d; v[i] >> d;
        fprintf(fp, " %f", d);
      }
      fprintf(fp, ")))\n");
    }
  } catch(YAML::RepresentationException& e) {
  }

  // all joint and link name
  fprintf(fp, "\n    ;; all joints\n");
  for (std::map<std::string, boost::shared_ptr<Joint> >::iterator joint = robot->joints_.begin();
       joint != robot->joints_.end(); joint++) {
    if(add_joint_suffix) {
      fprintf(fp, "  (:%s (&rest args) (forward-message-to %s_jt args))\n", joint->first.c_str(), joint->first.c_str());
    } else {
      fprintf(fp, "  (:%s (&rest args) (forward-message-to %s args))\n", joint->first.c_str(), joint->first.c_str());
    }
  }

  if (add_link_suffix) {
    fprintf(fp, "\n  ;; all links forwarding\n");
    fprintf(fp, "  (:links (&rest args)\n");
    fprintf(fp, "   (if (null args) (return-from :links (send-super :links)))\n");
    fprintf(fp, "   (let ((key (car args))\n           (nargs (cdr args)))\n");
    fprintf(fp, "     (unless (keywordp key)\n         (return-from :links (send-super* :links args)))\n       (case key\n");
    for (std::map<std::string, boost::shared_ptr<Link> >::iterator link = robot->links_.begin();
         link != robot->links_.end(); link++) {
      fprintf(fp, "       (:%s (forward-message-to %s_lk nargs))\n", link->first.c_str(), link->first.c_str());
    }
    fprintf(fp, "       (t (send-super* :links args)))))\n");
  }
  fprintf(fp, "\n  ;; all links\n");

  for (std::map<std::string, boost::shared_ptr<Link> >::iterator link = robot->links_.begin();
       link != robot->links_.end(); link++) {
    if (add_link_suffix) {
      fprintf(fp, "  (:%s_lk (&rest args) (forward-message-to %s_lk args))\n", link->first.c_str(), link->first.c_str());
    } else {
      fprintf(fp, "  (:%s (&rest args) (forward-message-to %s args))\n", link->first.c_str(), link->first.c_str());
    }
  }

  fprintf(fp, "\n    ;; user-defined joint\n");
  for(vector<pair<string, string> >::iterator it = g_all_link_names.begin();
      it != g_all_link_names.end(); it++){
    if(add_joint_suffix) {
      fprintf(fp, "  (:%s (&rest args) (forward-message-to %s_jt args))\n", it->second.c_str(), it->first.c_str());
    } else {
      fprintf(fp, "  (:%s (&rest args) (forward-message-to %s args))\n", it->second.c_str(), it->first.c_str());
    }
  }

#if 0
  // TODO: print sensors
  // sensor
  fprintf(fp, "\n    ;; attach_sensor\n");
  if ( g_dae->getDatabase()->getElementCount(NULL, "articulated_system", NULL) > 0 ) {
    domArticulated_system *thisArticulated;
    for ( size_t ii = 0; ii < g_dae->getDatabase()->getElementCount(NULL, "articulated_system", NULL); ii++) {
      g_dae->getDatabase()->getElement((daeElement**)&thisArticulated, ii, NULL, "articulated_system");
      if ( thisArticulated->getExtra_array().getCount() > 0 ) break;
    }
    std::vector<std::string> fsensor_list;
    std::vector<std::string> imusensor_list;
    for(size_t ie = 0; ie < thisArticulated->getExtra_array().getCount(); ++ie) {
      domExtraRef pextra = thisArticulated->getExtra_array()[ie];
      // find element which type is attach_sensor and is attached to thisNode
      if ( strcmp(pextra->getType(), "attach_sensor") == 0 ) {
        fprintf(fp, "    (:%s (&rest args) (forward-message-to %s-sensor-coords args))\n", pextra->getName(), pextra->getName());
        if (getSensorType(pextra) == "base_force6d" ) {
          fsensor_list.push_back(string(pextra->getName()));
        } else if (getSensorType(pextra) == "base_imu" ) {
          imusensor_list.push_back(string(pextra->getName()));
        }
      }
    }
    fprintf(fp, "    (:force-sensors (&rest args) (forward-message-to-all (list ");
    for (size_t i = 0; i < fsensor_list.size(); i++) {
      fprintf(fp, "(send self :%s) ", fsensor_list[i].c_str());
    }
    fprintf(fp, ") args))\n");
    fprintf(fp, "    (:imu-sensors (&rest args) (forward-message-to-all (list ");
    for (size_t i = 0; i < imusensor_list.size(); i++) {
      fprintf(fp, "(send self :%s) ", imusensor_list[i].c_str());
    }
    fprintf(fp, ") args))\n");
  }
  fprintf(fp, "  )\n\n");
#endif
}

void ModelEuslisp::printGeometry (boost::shared_ptr<Geometry> g, const Pose &pose,
                                  const string &name, const string &material_name) {
  string gname(name);
  if (g->type == Geometry::MESH) gname = ((Mesh *)(g.get()))->filename;
  fprintf(fp, "  (:_make_instance_%s ()\n", name.c_str());
  fprintf(fp, "    (let (geom glvertices qhull)\n");

  if (g->type == Geometry::SPHERE) {
    //
  } else if (g->type == Geometry::BOX) {
    //
  } else if (g->type == Geometry::CYLINDER) {
    //
  } else { // g->type == Geometry::MESH
    //TODO: qhull
    //fprintf(fp, "      (replace-object geom (send geom :qhull-faceset))\n");
    Assimp::Importer importer;
    importer.SetIOHandler(new ResourceIOSystem());
    const aiScene* scene = importer.ReadFile(gname,
                                             aiProcess_SortByPType|aiProcess_GenNormals|aiProcess_Triangulate|
                                             aiProcess_GenUVCoords|aiProcess_FlipUVs);
    if (scene && scene->HasMeshes()) {
      fprintf(fp, "      (setq\n");
      fprintf(fp, "       glvertices\n");
      fprintf(fp, "       (instance gl::glvertices :init\n");
      fprintf(fp, "                 (list ;; mesh list\n");
      printMesh(scene, scene->mRootNode, material_name);
      fprintf(fp, "                  )\n");
      fprintf(fp, "                 ))\n");
      //
      fprintf(fp, "      (let ((local-cds (make-coords :pos ");
      fprintf(fp, "(float-vector "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE")",
              pose.position.x * 1000,
              pose.position.y * 1000,
              pose.position.z * 1000);
      {
        double qx, qy, qz, qw;
        pose.rotation.getQuaternion(qx, qy, qz, qw);
        if (qx != 0.0 || qy != 0.0 || qz != 0.0 || qw != 1.0) {
          fprintf(fp, "\n                                    :rot (quaternion2matrix ");
          fprintf(fp, "(float-vector "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE"))",
                  qw, qx, qy, qz);
        }
        fprintf(fp, ")\n");
        fprintf(fp, "                       ))\n");
      }
      fprintf(fp, "        (send glvertices :transform local-cds))\n");
      //
      fprintf(fp, "      (send glvertices :calc-normals)\n");
    } else {
      // error
    }
    // qhull
  }
  fprintf(fp, "      (setq geom (instance collada-body :init :replace-obj qhull :name \"%s\"))\n", gname.c_str());
  if (g->type == Geometry::MESH) {
    fprintf(fp, "      (send geom :assoc glvertices)\n");
    fprintf(fp, "      (setq (geom . glvertices) glvertices)\n");
  }
  fprintf(fp, "      geom))\n");
}

void ModelEuslisp::printGeometries () {
  if (use_collision) {

  } else {
    for(map <boost::shared_ptr<const Link>, MapVisual >::iterator it = m_link_visual.begin();
        it != m_link_visual.end(); it++) {
      for( MapVisual::iterator vmap = it->second.begin();
           vmap != it->second.end(); vmap++) {
        printGeometry(vmap->second->geometry, vmap->second->origin,
                      vmap->first, vmap->second->material_name);
      }
    }
  }
}

void ModelEuslisp::writeToFile (string &filename) {
  if (!robot) {
    cerr << ";; not robot" << std::endl;
    return;
  }

  fp = fopen(filename.c_str(),"w");

  if (fp == NULL) {
    return;
  }

  // print header
  utsname uname_buf;
  uname(&uname_buf);
  time_t tm;
  time(&tm);
  localtime(&tm);

  fprintf(fp, ";;\n");
  fprintf(fp, ";; DO NOT EDIT THIS FILE\n");
  fprintf(fp, ";;\n");
  fprintf(fp, ";; this file is automatically generated from %s on (%s %s %s %s) at %s\n",
          filename.c_str(), uname_buf.sysname, uname_buf.nodename, uname_buf.release,
          uname_buf.machine, ctime(&tm));
  fprintf(fp, ";;\n");
  fprintf(fp, ";; %s $ ", get_current_dir_name());
  //for (int i = 0; i < argc; i++) fprintf(fp, "%s ", argv[i]); fprintf(fp, "\n");
  fprintf(fp, ";;\n");
  fprintf(fp, "\n");

  // pase
  addLinkCoords();

  // start printing
  copyRobotClassDefinition();

  printRobotDefinition();

  printRobotMethods();
}

//// main ////
namespace po = boost::program_options;
int main(int argc, char** argv)
{
  bool use_simple_geometry;
  bool use_collision;
  string arobot_name;

  string input_file;
  string config_file;
  string output_file;

  po::options_description desc("Options for collada_to_urdf");
  desc.add_options()
    ("help", "produce help message")
    ("simple_geometry,V", "use bounding box for geometry")
    ("use_collision,U", "use collision geometry (default collision is the same as visual)")
    ("robot_name,N", po::value< vector<string> >(), "output robot name")
    ("input_file,I", po::value< vector<string> >(), "input file")
    ("config_file,C", po::value< vector<string> >(), "configuration yaml file")
    ("output_file,O", po::value< vector<string> >(), "output file")
    ;

  po::positional_options_description p;
  p.add("input_file", 1);
  p.add("config_file", 1);
  p.add("output_file", 1);

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
  if (vm.count("simple_geometry")) {
    use_simple_geometry = true;
    cerr << ";; Using simple_geometry" << endl;
  }
  if (vm.count("use_collision")) {
    use_collision = true;
    cerr << ";; Using simple_geometry" << endl;
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
  if (vm.count("config_file")) {
    vector<string> aa = vm["config_file"].as< vector<string> >();
    cerr << ";; Config file is: "
         <<  aa[0] << endl;
    config_file = aa[0];
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
  if (config_file.size() > 0) {
    eusmodel.readYaml(config_file);
  }
  eusmodel.writeToFile (output_file);
}
