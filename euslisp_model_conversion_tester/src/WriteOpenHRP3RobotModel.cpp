// -*- C++ -*-
/*!
 * @file .cpp
 * @brief Standalone component
 * @date $Date$
 *
 * $Id$
 */

#include <rtm/Manager.h>
#include <rtm/CorbaNaming.h>
#include <hrpModel/Body.h>
#include <hrpModel/Link.h>
#include <hrpModel/Sensor.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <string>
#include <fstream>
//#include <algorithm>

class WriteOpenHRP3RobotModel {
private:
  hrp::BodyPtr m_robot;
  OpenHRP::BodyInfo_var bodyinfo;
  std::ofstream ofs;
public:
  WriteOpenHRP3RobotModel (hrp::BodyPtr _robot, OpenHRP::BodyInfo_var _bodyinfo, const std::string& out_fname)
    : m_robot(_robot), bodyinfo(_bodyinfo)
  {
    ofs.open(out_fname.c_str());
  };
  ~WriteOpenHRP3RobotModel ()
  {
    ofs.close();
  };
  void writeVector (const hrp::Vector3& vec, const std::string& print_name, const std::string& indent)
  {
    ofs << indent << print_name << " : " << vec(0) << "," << vec(1) << "," << vec(2) << std::endl;
  };
  void writeMatrix (const hrp::Matrix33 mat, const std::string& print_name, const std::string& indent)
  {
    ofs << indent << print_name << " : "
        << mat(0,0) << "," << mat(0,1) << "," << mat(0,2) << ","
        << mat(1,0) << "," << mat(1,1) << "," << mat(1,2) << ","
        << mat(2,0) << "," << mat(2,1) << "," << mat(2,2) << std::endl;
  };
  void writeMassProperties (hrp::Link* _link, const std::string& indent)
  {
    ofs << indent << "Mass : " << _link->m << std::endl;
    writeVector(_link->c, "LocalCom", indent);
    writeMatrix(_link->I, "LocalInertia", indent);
    ofs << indent << "ComNorm : " << _link->c.norm() << std::endl;
    ofs << indent << "InertiaNorm : " << _link->I.norm() << std::endl;
    writeVector(hrp::Vector3(_link->p + (_link->R * _link->c)), "GlobalCom", indent);
    writeMatrix(hrp::Matrix33(_link->R * _link->I * _link->R.transpose()), "GlobalInertia", indent);
  }
  void writeLinkProperties (const hrp::Vector3& localp, const hrp::Matrix33& localR, const std::string& indent)
  {
    writeVector(localp, "LocalPos", indent);
    writeMatrix(localR, "LocalRot", indent);
  };
  void writeLinkGrobalProperties (const hrp::Vector3& p, const hrp::Matrix33& R, const std::string& indent)
  {
    writeVector(p, "GlobalPos", indent);
    writeMatrix(R, "GlobalRot", indent);
  };
  void writeJointProperties (const hrp::Link* _link, const std::string& indent)
  {
    ofs << indent << "LowerLimit : " << _link->llimit << std::endl;
    ofs << indent << "UpperLimit : " << _link->ulimit << std::endl;
    // neglect lvlimit
    //ofs << indent << "lvlimit : " << _link->lvlimit << std::endl;
    ofs << indent << "VelocityLimit : " << _link->uvlimit << std::endl;
  };
  void writeSensorProperties (const hrp::Sensor* _sen, const std::string& indent)
  {
    writeVector(_sen->localPos, "LocalPos", indent);
    writeMatrix(_sen->localR, "LocalRot", indent);
    writeVector(hrp::Vector3(_sen->link->p + _sen->link->R *_sen->localPos), "GlobalPos", indent);
    writeMatrix(hrp::Matrix33(_sen->link->R *_sen->localR), "GlobalRot", indent);
  };
  void writeSensorProperties (const int sensorType, const std::string& indent)
  {
    for (size_t i = 0; i < m_robot->numSensors(sensorType); i++) {
      std::string name(m_robot->sensor(sensorType, i)->name);
      ofs << indent << name << ":" << std::endl;
      writeSensorProperties(m_robot->sensor(sensorType, i), std::string(indent+" "));
    }
  };
  hrp::Matrix33 outer_product_matrix(const hrp::Vector3 &v)
  {
    hrp::Matrix33 ret;
    (ret(0, 0) = 0);
    (ret(0, 1) = (-v(2)));
    (ret(0, 2) = v(1));
    (ret(1, 0) = v(2));
    (ret(1, 1) = 0);
    (ret(1, 2) = (-v(0)));
    (ret(2, 0) = (-v(1)));
    (ret(2, 1) = v(0));
    (ret(2, 2) = 0);
    return ret;
  }
  hrp::Matrix33 calcTotalInertia (const hrp::Vector3& total_com)
  {
    hrp::Matrix33 ret = hrp::Matrix33::Zero();
    for (size_t i = 0; i < m_robot->numLinks(); i++) {
      hrp::Link* l = m_robot->link(i);
      hrp::Vector3 tmpc = l->p + l->R * l->c - total_com;
      ret += l->R * l->I * l->R.transpose() + l->m * outer_product_matrix(tmpc).transpose() * outer_product_matrix(tmpc);
    }
    return ret;
  };
  void writeRobotModel () {
    ofs << "Total:" << std::endl;
    hrp::Vector3 tmp = m_robot->rootLink()->R.transpose() * (m_robot->calcCM() - m_robot->rootLink()->p);
    ofs << " Mass : " << m_robot->calcTotalMass() << std::endl;
    writeVector(tmp, "GlobalCom", " ");
    writeMatrix(calcTotalInertia(tmp), "GlobalInertia", " ");
    ofs << " Joints: ";
    for (size_t i = 0; i < m_robot->numLinks(); i++) {
      std::string name(m_robot->link(i)->name);
      if (name != std::string(m_robot->rootLink()->name))
        ofs << name << ", ";
    }
    ofs << std::endl;
    ofs << " ActualJoints: ";
    for (size_t i = 0; i < m_robot->numLinks(); i++) {
      if (m_robot->link(i)->jointId != -1)
        ofs << std::string(m_robot->link(i)->name) << ", ";
    }
    ofs << std::endl;

    ofs << "Links:" << std::endl;
    OpenHRP::LinkInfoSequence_var links = bodyinfo->links();
    for (size_t i = 0; i < m_robot->numLinks(); i++) {
      std::string indent(" "), name, child_indent(indent+" ");
      for ( int k = 0; k < links->length(); k++ ) {
        //std::cerr << " link: " << links[k].segments[0].name << " " <<  << std::endl;
        if ( std::string(links[k].name) == m_robot->link(i)->name )
          name = std::string(links[k].segments[0].name);
      }
      ofs << indent << name << ":" << std::endl;
      writeMassProperties(m_robot->link(i), child_indent);
      if (m_robot->link(i)->name != std::string(m_robot->rootLink()->name)) {
        //writeLinkProperties(m_robot->link(i)->b, m_robot->link(i)->Rs, child_indent);
        writeLinkProperties(hrp::Vector3(m_robot->link(i)->parent->R.transpose() * (m_robot->link(i)->p - m_robot->link(i)->parent->p)),
                            hrp::Matrix33(m_robot->link(i)->parent->R.transpose() * m_robot->link(i)->R),
                            child_indent);
        writeLinkGrobalProperties(m_robot->link(i)->p, m_robot->link(i)->R, child_indent);
      }
    }
    ofs << "Joints:" << std::endl;
    for (size_t i = 0; i < m_robot->numLinks(); i++) {
      if (m_robot->link(i)->name != std::string(m_robot->rootLink()->name)) {
        std::string indent(" "), child_indent(indent+" ");
        ofs << indent << m_robot->link(i)->name << ":" << std::endl;
        writeJointProperties(m_robot->link(i), child_indent);
      }
    }

    ofs << "Sensors:" << std::endl;
    writeSensorProperties(hrp::Sensor::FORCE, " ");
    writeSensorProperties(hrp::Sensor::RATE_GYRO, " ");
    writeSensorProperties(hrp::Sensor::ACCELERATION, " ");
  }
};

int main (int argc, char** argv)
{
  std::string output, input;

  for (int i = 1; i < argc; ++ i) {
    std::string arg(argv[i]);
    coil::normalize(arg);
    if ( arg == "--output" ) {
      if (++i < argc) output = argv[i];
    } else if ( arg == "--input" ) {
      if (++i < argc) input = argv[i];
    } else if ( arg == "-o" ) {
      ++i;
    } else if ( arg[0] == '-' ||  arg[0] == '_'  ) {
      std::cerr << argv[0] << " : Unknwon arguments " << arg << std::endl;
    } else {
    }
  }

  RTC::Manager* manager;
  manager = RTC::Manager::init(argc, argv);

  std::string nameServer = manager->getConfig()["corba.nameservers"];
  int comPos = nameServer.find(",");
  if (comPos < 0){
    comPos = nameServer.length();
  }
  nameServer = nameServer.substr(0, comPos);
  RTC::CorbaNaming naming(manager->getORB(), nameServer.c_str());
  hrp::BodyPtr m_robot(new hrp::Body());
  if (!loadBodyFromModelLoader(m_robot, input.c_str(),
			       CosNaming::NamingContext::_duplicate(naming.getRootContext())
	  )){
      std::cerr << "failed to load model[" << input << "]" << std::endl;
      return RTC::RTC_ERROR;
  }
  OpenHRP::BodyInfo_var bodyinfo;
  bodyinfo = hrp::loadBodyInfo(input.c_str(), CosNaming::NamingContext::_duplicate(naming.getRootContext()));

  for (size_t i = 0; i < m_robot->numJoints(); i++) {
    m_robot->joint(i)->q = 0;
  }
  m_robot->rootLink()->p = hrp::Vector3::Zero();
  m_robot->rootLink()->R = hrp::Matrix33::Identity();
  m_robot->calcForwardKinematics();

  WriteOpenHRP3RobotModel rmct(m_robot, bodyinfo, output);
  rmct.writeRobotModel ();

  return 0;
}
