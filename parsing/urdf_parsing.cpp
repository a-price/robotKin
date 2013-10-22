
//#define HAVE_URDF_PARSE // Uncomment this to get Qt to autocomplete below
#ifdef HAVE_URDF_PARSE

#include <urdf_parser/urdf_parser.h>
#include <urdf_model/model.h>
#include <urdf_model/link.h>
#include <urdf_model/joint.h>

#include <string>
#include <fstream>
#include <boost/shared_ptr.hpp>

#include "urdf_parsing.h"

namespace RobotKinURDF{

bool exploreLink(RobotKin::Robot& robot, boost::shared_ptr<urdf::ModelInterface> model,
                 boost::shared_ptr<urdf::Link> link, int id, int pID);
bool addURDFJoint(RobotKin::Linkage& linkage, boost::shared_ptr<urdf::ModelInterface> model,
                  boost::shared_ptr<urdf::Joint> ujoint);
bool parseURDFLink(RobotKin::Link& link, boost::shared_ptr<urdf::Link> ulink);
//int findRoot(urdf::ModelInterface model);

}



using namespace RobotKinURDF;
using namespace Eigen;
using namespace std;

bool RobotKinURDF::loadURDF(RobotKin::Robot &robot, string filename)
{
    // Read the urdf into a string
    std::string xml_model_string;
    std::fstream xml_file( filename.c_str(),
                             std::fstream::in );

    if(!xml_file.good())
    {
        std::cerr << "Could not find file \'" << filename << "\' to parse!" << endl;
        robot.name("invalid");
        return false;
    }

    while( xml_file.good() ) {
    std::string line;
    std::getline( xml_file, line );
        xml_model_string += ( line + "\n" );
    }

    xml_file.close();

    if(RobotKinURDF::loadURDFString(robot, xml_model_string))
	{
        robot.updateFrames();
        return true;
    }
    else
        return false;
}

bool RobotKinURDF::loadURDFString(RobotKin::Robot& robot, string xml_model_string)
{
    boost::shared_ptr<urdf::ModelInterface> model;


    // Parse model using the urdf_parser
    model = urdf::parseURDF( xml_model_string );

    // Output some info from the urdf

    // Robot name
    std::cout << " URDF Robot name: "<< model->getName() << std::endl;

    // Links
    std::vector< boost::shared_ptr<urdf::Link> > links;
    model->getLinks( links );

    // TODO: Have this print out in verbose mode
//    std::cout << "The robot has "<< links.size() << " links:" << std::endl;

//    for( int i = 0; i < links.size(); ++i ) {
//        std::cout << " Link [" << i << "]: "<< links[i]->name << std::endl;
//        if( links[i]->child_joints.size() > 0 ) {
//            std::cout << "\t -- with child joints: " << links[i]->child_joints[0]->name;
//            for(size_t c=1; c<links[i]->child_joints.size(); c++)
//                std::cout << ", " << links[i]->child_joints[c]->name;
//            std::cout << std::endl;
//        } else {
//            std::cout << "\t -- with NO child joint. Probably this is an end link" << std::endl;
//        }

//        if( links[i]->inertial ) {
//            std::cout << "\t -- with mass: "<< links[i]->inertial->mass << std::endl;
//            std::cout << "\t -- and inertia moments:"<< links[i]->inertial->ixx<<
//            ", "<<links[i]->inertial->ixy <<
//            ", "<<links[i]->inertial->ixz <<
//            ", "<<links[i]->inertial->iyy <<
//            ", "<<links[i]->inertial->iyz <<
//            ", "<<links[i]->inertial->izz <<std::endl;
//        }
//    }


    boost::shared_ptr<urdf::Link> rootLink = model->root_link_;
    RobotKin::Link link;
    parseURDFLink(link, rootLink);
    robot.rootLink = link;

    if(robot.name().compare("")==0)
        robot.name(model->getName());



    exploreLink(robot, model, rootLink, 0, -1);

    return true;
}




bool RobotKinURDF::exploreLink(RobotKin::Robot &robot, boost::shared_ptr<urdf::ModelInterface> model,
                               boost::shared_ptr<urdf::Link> link, int id, int pID)
{
    RobotKin::Linkage linkage(Eigen::Isometry3d::Identity(), link->name, id);

    if(link->parent_joint)
        addURDFJoint(linkage, model, link->parent_joint);


    if(link->child_joints.size()==1)
    {
        bool serial=true;
        boost::shared_ptr<urdf::Joint> childJoint = link->child_joints[0];
        boost::shared_ptr<urdf::Link> childLink;
        do
        {
            addURDFJoint(linkage, model, childJoint);

            model->getLink(childJoint->child_link_name, childLink);
            if(childLink->child_joints.size()==1)
                childJoint = childLink->child_joints[0];
            else
                serial = false;

        } while(serial);

        robot.addLinkage(linkage, pID, linkage.name());
        for(size_t i=0; i<childLink->child_joints.size(); i++)
        {
            boost::shared_ptr<urdf::Joint> nextJoint = childLink->child_joints[i];
            boost::shared_ptr<urdf::Link> nextLink;
            model->getLink(nextJoint->child_link_name, nextLink);
            size_t nextID = robot.nLinkages();
            exploreLink(robot, model, nextLink, (int)nextID, (int)id);
        }
    }
    else if(link->child_joints.size()>1)
    {
        robot.addLinkage(linkage, pID, linkage.name());
        for(size_t i=0; i<link->child_joints.size(); i++)
        {
            boost::shared_ptr<urdf::Joint> nextJoint = link->child_joints[i];
            boost::shared_ptr<urdf::Link> nextLink;
            model->getLink(nextJoint->child_link_name, nextLink);
            size_t nextID = robot.nLinkages();
            exploreLink(robot, model, nextLink, (int)nextID, (int)id);
        }
    }
    else
        robot.addLinkage(linkage, pID, linkage.name());
}

bool RobotKinURDF::addURDFJoint(RobotKin::Linkage &linkage, boost::shared_ptr<urdf::ModelInterface> model,
                                boost::shared_ptr<urdf::Joint> ujoint)
{
    Isometry3d transform(Isometry3d::Identity());
    Vector3d translation(ujoint->parent_to_joint_origin_transform.position.x,
                         ujoint->parent_to_joint_origin_transform.position.y,
                         ujoint->parent_to_joint_origin_transform.position.z);
    transform.translate(translation);

    double w, x, y, z;
    ujoint->parent_to_joint_origin_transform.rotation.getQuaternion(x,y,z,w);
    Quaterniond quat(w,x,y,z);
    transform.rotate(quat);

    RobotKin::JointType jt = RobotKin::REVOLUTE;
    if(ujoint->type == urdf::Joint::REVOLUTE)
        jt = RobotKin::REVOLUTE;
    else if(ujoint->type == urdf::Joint::PRISMATIC)
        jt = RobotKin::PRISMATIC;
    else
        jt = RobotKin::ANCHOR;

    Vector3d jointAxis(ujoint->axis.x, ujoint->axis.y, ujoint->axis.z);
    if(jointAxis.norm() == 0)
        jointAxis = Eigen::Vector3d::UnitZ();
    else
        jointAxis.normalize();

    RobotKin::Link link;
    boost::shared_ptr<urdf::Link> childLink;

    model->getLink(ujoint->child_link_name, childLink);
    parseURDFLink(link, childLink);

    if(ujoint->limits)
    {
        RobotKin::Joint joint(transform, ujoint->name, 0, jt, jointAxis, ujoint->limits->lower, ujoint->limits->upper);
        joint.link = link;
        linkage.addJoint(joint);
    }
    else
    {
        RobotKin::Joint joint(transform, ujoint->name, 0, jt, jointAxis);
        joint.link = link;
        linkage.addJoint(joint);
    }

}


bool RobotKinURDF::parseURDFLink(RobotKin::Link& link, boost::shared_ptr<urdf::Link> ulink)
{
    RobotKin::TRANSLATION com;
    Eigen::Matrix3d tensor;
    if(ulink->inertial)
    {
        com  << ulink->inertial->origin.position.x,
                ulink->inertial->origin.position.y,
                ulink->inertial->origin.position.z;
        link.setMass(ulink->inertial->mass, com);

        tensor  << ulink->inertial->ixx, ulink->inertial->ixy, ulink->inertial->ixz,
                   ulink->inertial->ixy, ulink->inertial->iyy, ulink->inertial->iyz,
                   ulink->inertial->ixz, ulink->inertial->iyz, ulink->inertial->izz;
        link.setInertiaTensor(tensor);
    }
}



#endif
