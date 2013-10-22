

#include "Robot.h"
#include "Constraints.h"

#include <time.h>


using namespace RobotKin;
using namespace Eigen;
using namespace std;

Constraints::Constraints()
    : performNullSpaceTask(false),
      hasRestingValues(false),
      maxIterations(500),
      dampingConstant(0.05),
      finalTransform(TRANSFORM::Identity()),
      convergenceTolerance(0.0001),
      performErrorClamp(true),
      translationClamp(0.2),
      rotationClamp(0.15),
      customErrorClamp(false),
      useIterativeJacobianSeed(true),
      maxAttempts(5),
      rotationScale(0.01),
      performDeltaClamp(true),
      deltaClamp(5*M_PI/180),
      wrapToJointLimits(true),
      wrapSolutionToJointLimits(true)
{

}




Constraints &Constraints::Defaults()
{
    Constraints* constraints = new Constraints;
    return *constraints;
}

void Constraints::restingValues(VectorXd newRestingValues)
{
    performNullSpaceTask = true;
    hasRestingValues = true;
    restingValues_ = newRestingValues;
}

VectorXd& Constraints::restingValues() { return restingValues_; }

bool Constraints::nullComplete()
{
    if(!performNullSpaceTask)
        return true;
    else
        return nullComplete_;
}

VectorXd Constraints::nullSpaceTask(Robot& robot, const MatrixXd& J, const std::vector<size_t> &indices,
                                    const VectorXd& values)
{
    VectorXd nullTask = values;
    nullTask.setZero();
    nullComplete_ = true;
    return nullTask;
}

void Constraints::errorClamp(Robot &robot, const std::vector<size_t> &indices, SCREW &error)
{

}

void Constraints::iterativeJacobianSeed(Robot& robot, size_t attemptNumber,
                                        const std::vector<size_t> &indices, Eigen::VectorXd &values)
{
    if( attemptNumber == 0 )
    {
//        wrapToJointLimits = true;
        return;
    }
    else if( attemptNumber == 1 && hasRestingValues
             && values.size() == restingValues_.size() )
        for(int i=0; i<values.size(); i++)
            values(i) = restingValues_(i);
    else if( attemptNumber == 2 )
    {
        wrapToJointLimits = false;
        robot.imposeLimits = false;
        for(int i=0; i<values.size(); i++)
            values(i) = 0;
    }
    else
    {
        int resolution = 1000;
        int randVal = rand();
        for(int i=0; i<values.size(); i++)
            values(i) = ((double)(randVal%resolution))/((double)resolution-1)
                    *(robot.joint(indices[i]).max() - robot.joint(indices[i]).min())
                    + robot.joint(indices[i]).min();
    }
}






