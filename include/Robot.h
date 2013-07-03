/*
 -------------------------------------------------------------------------------
 Robot.h
 robotTest Project
 
 CLASS NAME:
 Robot
 
 DESCRIPTION:
 description...
 
 FILES:
 Robot.h
 Robot.cpp
 
 DEPENDENCIES:
 
 
 CONSTRUCTORS:
 Robot();
 
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
 1.0 - 5/15/13 - Rowland O'Flaherty ( rowlandoflaherty.com )
 
 -------------------------------------------------------------------------------
 */



#ifndef _Robot_h_
#define _Robot_h_



//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include "Frame.h"
#include "Linkage.h"
#include <vector>
#include <map>
#include <string>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#ifdef HAVE_URDF_PARSE
#include "urdf_parsing.h"
#endif // HAVE_URDF_PARSE

//------------------------------------------------------------------------------
// Namespaces
//------------------------------------------------------------------------------
using namespace std;
using namespace Eigen;

// TODO: Add a user-specified level of assertiveness

namespace RobotKin {

    typedef enum {
        RK_SOLVED = 0,
        RK_CONVERGED,
        RK_DIVERGED,
        RK_NO_SOLUTION,
        RK_INVALID_JOINT,
        RK_INVALID_LINKAGE,


        RK_TYPE_SIZE
    } rk_result_t;

    
    //------------------------------------------------------------------------------
    // Typedefs
    //------------------------------------------------------------------------------
    typedef Matrix<double, 6, Dynamic> Matrix6Xd;

	// Sort parentIndices and linkages
    struct indexParentIndexPair {
        size_t I;
        int pI;
    
        bool operator<( const indexParentIndexPair& rhs ) const {
            return pI < rhs.pI;
        }
    };

    
    
    class Robot : public Frame
    {
        //--------------------------------------------------------------------------
        // Robot Friends
        //--------------------------------------------------------------------------
        friend class Linkage;
        
        
    public:
        //--------------------------------------------------------------------------
        // Robot Lifecycle
        //--------------------------------------------------------------------------
        // Constructors
        Robot();
        Robot(vector<Linkage> linkageObjs, vector<int> parentIndices);

//#ifdef HAVE_URDF_PARSE
        Robot(string filename, string name="", size_t id=0);
        bool loadURDF(string filename);
//#endif // HAVE_URDF_PARSE
        
        // Destructor
        virtual ~Robot();

        
        //--------------------------------------------------------------------------
        // Robot Public Member Functions
        //--------------------------------------------------------------------------
        size_t nLinkages() const;
        
        size_t linkageIndex(string linkageName) const;
        
        // Getting individual linkages
        const Linkage& const_linkage(size_t linkageIndex) const;
        const Linkage& const_linkage(string linkageName) const;
        
        Linkage& linkage(size_t linkageIndex);
        Linkage& linkage(string linkageName);
        
        // Getting all the linkages
        const vector<Linkage*>& const_linkages() const;
        vector<Linkage*>& linkages();
        
        // Adding new linkages
        void addLinkage(int parentIndex, string name);
        void addLinkage(string parentName, string name);
        void addLinkage(Linkage linkage, string parentName, string name);
        void addLinkage(Linkage linkage, int parentIndex, string name);
        
        // Getting joint information
        size_t nJoints() const;
        size_t jointIndex(string jointName) const;

        const Linkage::Joint& const_joint(size_t jointIndex) const;
        const Linkage::Joint& const_joint(string jointName) const;

        Linkage::Joint& joint(size_t jointIndex);
        Linkage::Joint& joint(string jointName);

        // Convenience function
        void setJointValue(size_t jointIndex, double val);
        void setJointValue(string jointName, double val);
        
        const vector<Linkage::Joint*>& const_joints() const;
        vector<Linkage::Joint*>& joints();
        
        VectorXd values() const;
        void values(const VectorXd& someValues);
        
        const Isometry3d& respectToFixed() const;
        void respectToFixed(Isometry3d aCoordinate);
        
        Isometry3d respectToWorld() const;
        
        void jacobian(MatrixXd& J, const vector<Linkage::Joint*>& jointFrames, Vector3d location, const Frame* refFrame) const;
        
        void updateFrames();
        void printInfo() const;
        
        //--------------------------------------------------------------------------
        // Kinematics Solvers
        //--------------------------------------------------------------------------

        rk_result_t dampedLeastSquaresIK_chain(const vector<size_t> &jointIndices, vector<double> &jointValues,
                                         const Isometry3d &target, const Isometry3d &finalTF = Isometry3d::Identity());

        rk_result_t dampedLeastSquaresIK_chain(const vector<string>& jointNames, vector<double>& jointValues,
                                         const Isometry3d& target, const Isometry3d &finalTF = Isometry3d::Identity());

        rk_result_t dampedLeastSquaresIK_linkage(const string linkageName, vector<double>& jointValues,
                                         const Isometry3d& target, const Isometry3d &finalTF = Isometry3d::Identity());


    protected:
        //--------------------------------------------------------------------------
        // Robot Protected Member Functions
        //--------------------------------------------------------------------------
        Isometry3d respectToWorld_; // Coordinates with respect to robot base frame
        vector<Linkage*> linkages_;
        map<string, size_t> linkageNameToIndex_;
        vector<Linkage::Joint*> joints_;
        map<string, size_t> jointNameToIndex_;
        
        
        //--------------------------------------------------------------------------
        // Robot Protected Member Variables
        //--------------------------------------------------------------------------
        virtual void initialize(vector<Linkage> linkageObjs, vector<int> parentIndices);
        
        
    private:
        //--------------------------------------------------------------------------
        // Robot Constants, Enums, and Types
        //--------------------------------------------------------------------------

        
        
        
        //--------------------------------------------------------------------------
        // Robot Private Member Variables
        //--------------------------------------------------------------------------
        bool initializing_;
        
        
        
    }; // class Robot
    
    //------------------------------------------------------------------------------
    // Postfix Increment Operators
    //------------------------------------------------------------------------------
    
} // namespace RobotKin

#endif


