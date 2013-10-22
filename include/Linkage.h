/*
 -------------------------------------------------------------------------------
 Linkage.h
 Robot Library Project
 
 CLASS NAME:
 Linkage
 
 DESCRIPTION:
 description...
 
 FILES:
 Linkage.h
 Linkage.cpp
 
 DEPENDENCIES:
 
 
 CONSTRUCTORS:
 Linkage();
 
 PROPERTIES:
 prop1 - description... .
 
 prop2 - description... .
 
 METHODS:
 type method1(type arg1);
 Description... .
 arg1 - description... .
 
 NOTES:
 
 
 EXAMPLES:
 Example 1: description
 ----------------------------------------------------------------------------
 code...
 ----------------------------------------------------------------------------
 
 
 VERSIONS:
 1.0 - 5/11/13 - Rowland O'Flaherty ( rowlandoflaherty.com )
 
 -------------------------------------------------------------------------------
 */



#ifndef _Linkage_h_ 
#define _Linkage_h_



//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include "Frame.h"
#include <string>
#include <vector>
#include <map>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

//------------------------------------------------------------------------------
// Class Declarations
//------------------------------------------------------------------------------
namespace RobotKin
{
    class Robot;
}

//------------------------------------------------------------------------------
// Namespaces
//------------------------------------------------------------------------------

namespace RobotKin {
    
    //------------------------------------------------------------------------------
    // Typedefs and Enums
    //------------------------------------------------------------------------------
    typedef Eigen::Matrix<double, 6, Eigen::Dynamic> Matrix6Xd;


    class Link
    {
        friend class Robot;
        friend class Linkage;
        friend class Joint;

    public:
        //----------------------------------------------------------------------
        // Link Lifecycle
        //----------------------------------------------------------------------
        // Constructors
        Link(); // TODO
        Link(double newMass, TRANSLATION newCom); // TODO
        Link(double newMass, TRANSLATION newCom, Eigen::Matrix3d newInertiaTensor); // TODO

        double mass() const;
        const TRANSLATION& const_com() const;
        TRANSLATION& com();
        const Eigen::Matrix3d& const_tensor() const;
        Eigen::Matrix3d& tensor();


        void setMass(double newMass, TRANSLATION newCom); // TODO
        void setInertiaTensor(Eigen::Matrix3d newInertiaTensor); // TODO

        bool hasMass() const;
        bool hasTensor() const;

        void printInfo() const;

    protected:
        bool massProvided;
        bool tensorProvided;

        double mass_;
        TRANSLATION com_;
        Eigen::Matrix3d tensor_;

    }; // Class Link

    
    class Joint : public Frame
    {

        //----------------------------------------------------------------------
        // Joint Friends
        //----------------------------------------------------------------------
        friend class Frame;
        friend class Linkage;
        friend class Robot;
        friend class Link;

    public:

        //----------------------------------------------------------------------
        // Joint Lifecycle
        //----------------------------------------------------------------------
        // Constructors
        Joint(TRANSFORM respectToFixed = TRANSFORM::Identity(),
              std::string name = "",
              size_t id = 0,
              JointType jointType = REVOLUTE,
              TRANSLATION axis=TRANSLATION::UnitZ(),
              double minValue = -M_PI,
              double maxValue = M_PI);
        Joint(const Joint& joint);

        // Destructor
        virtual ~Joint();

        Link link;
        TRANSLATION centerOfMass(FrameType withRespectTo=ROBOT);
        double mass();


        double gravityTorque(bool downstream=true);

        //----------------------------------------------------------------------
        // Joint Overloaded Operators
        //----------------------------------------------------------------------
        // Assignment operator
        const Joint& operator=(const double value);
        Joint& operator=(const Joint& joint); // TODO: Test this

        //----------------------------------------------------------------------
        // Joint Public Member Functions
        //----------------------------------------------------------------------
        double value() const;
        rk_result_t value(double newValue, bool update=true);

        double min() const;
        double max() const;
        void min(double newMin);
        void max(double newMax);

        JointType getJointType();

        void setJointAxis(AXIS axis);
        AXIS getJointAxis();

        const TRANSFORM& respectToFixed() const;
        void respectToFixed(TRANSFORM aCoordinate);

        const TRANSFORM& respectToFixedTransformed() const;

        const TRANSFORM& respectToLinkage() const;



        TRANSFORM respectToRobot() const;

        TRANSFORM respectToWorld() const;

        Joint& parentJoint();

        size_t getLinkageID();
        std::string getLinkageName();

        size_t getRobotID();
        std::string getRobotName();

        size_t localID() const; // Joint ID inside its linkage

        Linkage& linkage();
        Robot& robot();

        void printInfo() const;

    protected:
        //----------------------------------------------------------------------
        // Joint Protected Member Variables
        //----------------------------------------------------------------------
        double value_; // Current joint value (R type = joint angle, P type = joint length)



        const Robot* parentRobot() const;

    private:
        //----------------------------------------------------------------------
        // Joint Private Member Variables
        //----------------------------------------------------------------------
        JointType jointType_; // Type of joint (REVOLUTE or PRISMATIC)
        double min_; // Minimum joint value
        double max_; // Maximum joint value
        AXIS jointAxis_;
        TRANSFORM respectToFixedTransformed_; // Coordinates transformed according to the joint value and type with respect to respectToFixed frame
        TRANSFORM respectToLinkage_; // Coordinates with respect to linkage base frame
        size_t localID_;


    }; // class Joint

    class Tool : public Frame
    {

        //----------------------------------------------------------------------
        // Tool Friends
        //----------------------------------------------------------------------
        friend class Linkage;
        friend class Robot;
        friend class Frame;

    public:
        //----------------------------------------------------------------------
        // Tool Lifecycle
        //----------------------------------------------------------------------
        // Constructors
        Tool(const Tool& tool);
        Tool(TRANSFORM respectToFixed = TRANSFORM::Identity(),
             std::string name = "",
             size_t id = 0);

        // Destructor
        virtual ~Tool();

        //----------------------------------------------------------------------
        // Tool Public Member Functions
        //----------------------------------------------------------------------
        Tool& operator =(const Tool& tool);

        const TRANSFORM& respectToFixed() const;
        void respectToFixed(TRANSFORM aCoordinate);

        const TRANSFORM& respectToLinkage() const;

        TRANSFORM respectToRobot() const;

        TRANSFORM respectToWorld() const;

        void printInfo() const;

        static Tool Identity();

        size_t getRobotID();
        std::string getRobotName();

        size_t getLinkageID();
        std::string getLinkageName();

        size_t getParentJointID();
        std::string getParentJointName();

        Link massProperties;

        TRANSLATION centerOfMass(FrameType withRespectTo=ROBOT);
        double mass();



    protected:

        const Linkage* parentLinkage() const;

        const Robot* parentRobot() const;

    private:
        //----------------------------------------------------------------------
        // Tool Private Member Variables
        //----------------------------------------------------------------------
        TRANSFORM respectToLinkage_; // Coordinates with respect to linkage base frame

    }; // class Tool

    
    class Linkage : public Frame
    {
        //--------------------------------------------------------------------------
        // Linkage Friends
        //--------------------------------------------------------------------------
        friend class Frame;
        friend class Joint;
        friend class Tool;
        friend class Robot;
        
    public:


        
        
        //--------------------------------------------------------------------------
        // Linkage Lifecycle
        //--------------------------------------------------------------------------
        // Constructors
        Linkage();
        Linkage(const Linkage& linkage);
        Linkage(TRANSFORM respectToFixed, std::string name, size_t id);
        Linkage(TRANSFORM respectToFixed,
                std::string name, size_t id,
                Joint joint,
                Tool tool = Tool::Identity());
        Linkage(TRANSFORM respectToFixed,
                std::string name,
                size_t id,
                std::vector<Joint> joints,
                Tool tool = Tool::Identity());
        
        // Destructor
        virtual ~Linkage();
        
        
        //--------------------------------------------------------------------------
        // Linkage Overloaded Operators
        //--------------------------------------------------------------------------
        // Assignment operator
        Linkage& operator =( const Linkage& linkage );
        const Linkage& operator=(const Eigen::VectorXd& values);
        
        
        //--------------------------------------------------------------------------
        // Linkage Public Member Functions
        //--------------------------------------------------------------------------
        double mass();
        TRANSLATION centerOfMass(FrameType withRespectTo=ROBOT);
        TRANSLATION centerOfMass(const std::vector<size_t> &indices, bool includeTool=true, FrameType withRespectTo=ROBOT);
        TRANSLATION centerOfMass(const std::vector<std::string> &names, bool includeTool=true, FrameType withRespectTo=ROBOT);
        double mass(const std::vector<size_t> &indices, bool includeTool=true);
        double mass(const std::vector<std::string> &names, bool includeTool=true);

        TRANSLATION centerOfMass(size_t fromJoint, bool downstream=true, bool includeTool=true, FrameType withRespectTo=ROBOT);
        TRANSLATION centerOfMass(std::string fromJoint, bool downstream=true, bool includeTool=true, FrameType withRespectTo=ROBOT);
        double mass(size_t fromJoint, bool downstream=true, bool includeTool=true);
        double mass(std::string fromJoint, bool downstream=true, bool includeTool=true);

        TRANSLATION centerOfMass(size_t fromJoint, size_t toJoint, FrameType withRespectTo=ROBOT);
        TRANSLATION centerOfMass(std::string fromJoint, std::string toJoint, FrameType withRespectTo=ROBOT);
        double mass(size_t fromJoint, size_t toJoint);
        double mass(std::string fromJoint, std::string toJoint);

        void gravityJointTorques(Eigen::VectorXd& torques, bool downstream=true);





        Linkage& parentLinkage();
        
        size_t nChildren() const;

        rk_result_t jointNamesToIndices(const std::vector<std::string> &jointNames,
                                        std::vector<size_t> &jointIndices);

        size_t jointNameToIndex(std::string jointName);
        
        rk_result_t setJointValue(size_t jointIndex, double val);
        rk_result_t setJointValue(std::string jointName, double val);
        
        size_t nJoints() const;
        const Joint& const_joint(size_t jointIndex) const;
        const Joint& const_joint(std::string jointName) const;
        
        Joint& joint(size_t jointIndex);
        Joint& joint(std::string jointName);

        Linkage& childLinkage(size_t childIndex);
        // TODO: Allow this to be called by name?
        // Sounds pretty useless, actually
        
        const std::vector<Joint*>& const_joints() const;
        std::vector<Joint*>& joints();

        void addJoint(Joint newJoint); // TODO
        void insertJoint(Joint newJoint, size_t jointIndex); // TODO

        void setTool(Tool newTool);
//        void addTool(Linkage::Tool newTool); // TODO
//        void chooseTool(size_t toolIndex); // TODO
//        void chooseTool(string toolName); // TODO
        
        const Tool& const_tool() const;
        Tool& tool();
        
        Eigen::VectorXd values() const;
        bool values(const Eigen::VectorXd &allValues);
        
        const TRANSFORM& respectToFixed() const;
        void respectToFixed(TRANSFORM aCoordinate);
        
        const TRANSFORM& respectToRobot() const;
        
        TRANSFORM respectToWorld() const;
        
        void jacobian(Eigen::MatrixXd& J, TRANSLATION location, const Frame *refFrame) const;
        void jacobian(Eigen::MatrixXd& J, const std::vector<Joint*>& jointFrames, TRANSLATION location, const Frame* refFrame) const;
        
        void printInfo() const;
        
        size_t getParentLinkageID();
        std::string getParentLinkageName();


        size_t getRobotID();
        std::string getRobotName();
        
        void getChildIDs(std::vector<size_t>& ids);
        void getChildNames(std::vector<std::string>& names);
        void printChildren();
        
        //--------------------------------------------------------------------------
        // Linkage Public Member Variables
        //--------------------------------------------------------------------------
        bool (*analyticalIK)(Eigen::VectorXd& q, const TRANSFORM& B, const Eigen::VectorXd& qPrev);
        
    protected:
        //--------------------------------------------------------------------------
        // Linkage Protected Member Variables
        //--------------------------------------------------------------------------
        
        TRANSFORM respectToRobot_; // Coordinates with respect to robot base frame
        Linkage* parentLinkage_;
        std::vector<Linkage*> childLinkages_;
        std::vector<Joint*> joints_;
        Tool tool_;
        // TODO: Consider allowing multiple switchable tools
        //vector<Linkage::Tool> tools_;
//        size_t activeTool_;

        bool hasParent;
        bool hasChildren;
        
    private:
        //--------------------------------------------------------------------------
        // Linkage Public Member Functions
        //--------------------------------------------------------------------------
        void initialize(std::vector<Joint> joints, Tool tool);
        void updateFrames();
        void updateChildLinkage();
        static bool defaultAnalyticalIK(Eigen::VectorXd& q, const TRANSFORM& B, const Eigen::VectorXd& qPrev);
        
        
        
        //--------------------------------------------------------------------------
        // Linkage Private Member Variables
        //--------------------------------------------------------------------------
        bool initializing_;
        bool needsUpdate_;
        std::map<std::string, size_t> jointNameToIndex_;
        
        
    }; // class Linkage

    
} // namespace RobotKin

#endif


