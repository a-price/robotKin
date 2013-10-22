/*
 -------------------------------------------------------------------------------
 Hubo.h
 robotTest Project
 
 CLASS NAME:
    Hubo
 
 DESCRIPTION:
    description...
 
 FILES:
    Hubo.h
    Hubo.cpp

 DEPENDENCIES:
    
 
 CONSTRUCTORS:
    Hubo();    
 
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
    1.0 - 5/20/13 - Rowland O'Flaherty ( rowlandoflaherty.com )
 
 -------------------------------------------------------------------------------
 */



#ifndef _Hubo_h_
#define _Hubo_h_



//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include "Robot.h"
#include "Linkage.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <vector>


namespace RobotKin
{

//------------------------------------------------------------------------------
// Typedefs
//------------------------------------------------------------------------------
typedef Eigen::Matrix< double, 4, 1 > Vector4d;
typedef Eigen::Matrix< double, 5, 1 > Vector5d;
typedef Eigen::Matrix< double, 6, 1 > SCREW;
typedef Eigen::Matrix< double, 25, 1 > Vector25d;
typedef Eigen::Matrix<double, 1, 2> Matrix12d;
typedef Eigen::Matrix<double, 6, 2>  Matrix62d;
typedef Eigen::Matrix< double, 6, 6 > Matrix66d;

enum {
    SIDE_LEFT,
    SIDE_RIGHT
};


class Hubo : public Robot
{
public:
    //--------------------------------------------------------------------------
    // Hubo Lifecycle
    //--------------------------------------------------------------------------
    // Constructors
    Hubo();
    
    // Destructor
    virtual ~Hubo();
    
    
    //--------------------------------------------------------------------------
    // Hubo Public Member Functions
    //--------------------------------------------------------------------------
    bool leftArmAnalyticalIK(Eigen::VectorXd& q, const TRANSFORM& B, const Eigen::VectorXd& qPrev);
    bool rightArmAnalyticalIK(Eigen::VectorXd& q, const TRANSFORM& B, const Eigen::VectorXd& qPrev);
    
    void armFK(TRANSFORM& B, const SCREW& q, size_t side);
    bool armAnalyticalIK(Eigen::VectorXd& q, const TRANSFORM& B, const SCREW& qPrev, size_t side);
    
    
    bool leftLegAnalyticalIK(Eigen::VectorXd& q, const TRANSFORM& B, const Eigen::VectorXd& qPrev);
    bool rightLegAnalyticalIK(Eigen::VectorXd& q, const TRANSFORM& B, const Eigen::VectorXd& qPrev);
    
    void legFK(TRANSFORM& B, const SCREW& q, size_t side);
    bool legAnalyticalIK(Eigen::VectorXd& q, const TRANSFORM& B, const SCREW& qPrev, size_t side);
    
    //--------------------------------------------------------------------------
    // Hubo Public Member Variables
    //--------------------------------------------------------------------------
    // Lengths
    double torsoLength;
    Eigen::VectorXd armLengths;
    Eigen::VectorXd legLengths;
    
    // Limits
    Eigen::MatrixX2d torsoLimits;
    Eigen::MatrixX2d leftArmLimits;
    Eigen::MatrixX2d rightArmLimits;
    Eigen::MatrixX2d leftLegLimits;
    Eigen::MatrixX2d rightLegLimits;
    
    // Offsets
    double torsoOffset;
    Eigen::VectorXd leftArmOffsets;
    Eigen::VectorXd rightArmOffsets;
    Eigen::VectorXd leftLegOffsets;
    Eigen::VectorXd rightLegOffsets;
    
    //------------------------------------------------------------------------------
    // Hubo Public Helper Functions
    //------------------------------------------------------------------------------
    static inline double mod(double x, double y) {
        if (0 == y)
            return x;
        return x - y * floor(x/y);
    }
    
    static inline double wrapToPi(double fAng) {
        return mod(fAng + M_PI, 2*M_PI) - M_PI;
    }
    
private:
    //--------------------------------------------------------------------------
    // Hubo Private Member Functions
    //--------------------------------------------------------------------------
    void initialize();
    Linkage initializeTorso();
    Linkage initializeLeftArm();
    Linkage initializeRightArm();
    Linkage initializeLeftLeg();
    Linkage initializeRightLeg();
    
    //--------------------------------------------------------------------------
    // Hubo Private Member Variables
    //--------------------------------------------------------------------------
    double zeroSize;
    
    
}; // class Hubo

} // namespace RobotKin

#endif


