
#include "Robot.h"
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/QR>

using namespace std;
using namespace Eigen;
using namespace RobotKin;

// TODO: Add sign function


double RobotKin::mod(double x, double y)
{
    if (0 == y)
        return x;

    return x - y * floor(x/y);
}

double RobotKin::wrapToPi(double angle)
{
    return mod(angle + M_PI, 2*M_PI) - M_PI;
}


void RobotKin::clampMag(VectorXd& v, double clamp)
{
    if(v.norm() > clamp)
        v *= clamp/v.norm();
}

void RobotKin::clampMag(SCREW& v, double clamp)
{
    if(v.norm() > clamp)
        v *= clamp/v.norm();
}

void RobotKin::clampMag(TRANSLATION& v, double clamp)
{
    if(v.norm() > clamp)
        v *= clamp/v.norm();
}


void RobotKin::clampMaxAbs(VectorXd& v, double clamp)
{
    int max=0;
    for(int i=0; i<v.size(); i++)
    {
        if(fabs(v[i]) > fabs(v[max]))
            max = i;
    }

    if(fabs(v[max])>fabs(clamp))
        v *= fabs(clamp)/fabs(v[max]);
}

double RobotKin::minimum(double a, double b) { return a<b ? a : b; }


void RobotKin::wrapToJointLimits(Robot& robot, const vector<size_t>& jointIndices, VectorXd& jointValues)
{
    for(int i=0; i<jointIndices.size(); i++)
    {
        if(robot.joint(jointIndices[i]).getJointType()==RobotKin::REVOLUTE)
        {
            if( !(robot.joint(jointIndices[i]).min() <= jointValues[i]
                    && jointValues[i] <= robot.joint(jointIndices[i]).max()) )
            {
//                double tempValue = jointValues[i];
//                while(tempValue > robot.joint(jointIndices[i]).max())
//                    tempValue -= 2*M_PI;

//                while(tempValue < robot.joint(jointIndices[i]).min())
//                    tempValue += 2*M_PI;

//                if(robot.joint(jointIndices[i]).min() <= tempValue
//                        && tempValue <= robot.joint(jointIndices[i]).max())
//                    jointValues[i] = tempValue;
//                else
//                {
//                    if( fabs(wrapToPi(jointValues[i]-robot.joint(jointIndices[i]).min())) <
//                            fabs(wrapToPi(jointValues[i]-robot.joint(jointIndices[i]).max())) )
//                        jointValues[i] = robot.joint(jointIndices[i]).min();
//                    else
//                        jointValues[i] = robot.joint(jointIndices[i]).max();
//                }
                
                if( fabs(wrapToPi(jointValues[i]-robot.joint(jointIndices[i]).min())) <
                        fabs(wrapToPi(jointValues[i]-robot.joint(jointIndices[i]).max())) )
                    jointValues[i] = robot.joint(jointIndices[i]).min();
                else
                    jointValues[i] = robot.joint(jointIndices[i]).max();
            }
        }
    }
}



// Derived from code by Yohann Solaro ( http://listengine.tuxfamily.org/lists.tuxfamily.org/eigen/2010/01/msg00187.html )
void pinv(const MatrixXd &b, MatrixXd &a_pinv)
{
    // see : http://en.wikipedia.org/wiki/Moore-Penrose_pseudoinverse#The_general_case_and_the_SVD_method

    // TODO: Figure out why it wants fewer rows than columns
//    if ( a.rows()<a.cols() )
//        return false;
    bool flip = false;
    MatrixXd a;
    if( a.rows() < a.cols() )
    {
        a = b.transpose();
        flip = true;
    }
    else
        a = b;


    // SVD
    JacobiSVD< MatrixXd > svdA;
    svdA.compute(a, ComputeFullU | ComputeThinV);

    JacobiSVD<MatrixXd>::SingularValuesType vSingular = svdA.singularValues();



    // Build a diagonal matrix with the Inverted Singular values
    // The pseudo inverted singular matrix is easy to compute :
    // is formed by replacing every nonzero entry by its reciprocal (inversing).
    VectorXd vPseudoInvertedSingular(svdA.matrixV().cols());


    for (int iRow =0; iRow<vSingular.rows(); iRow++)
    {
        if ( fabs(vSingular(iRow))<=1e-10 ) // Todo : Put epsilon in parameter
        {
            vPseudoInvertedSingular(iRow)=0.;
        }
        else
        {
            vPseudoInvertedSingular(iRow)=1./vSingular(iRow);
        }

    }



    // A little optimization here
    MatrixXd mAdjointU = svdA.matrixU().adjoint().block(0,0,vSingular.rows(),svdA.matrixU().adjoint().cols());


    // Pseudo-Inversion : V * S * U'
    a_pinv = (svdA.matrixV() *  vPseudoInvertedSingular.asDiagonal()) * mAdjointU;


    if(flip)
    {
        a = a.transpose();
        a_pinv = a_pinv.transpose();
    }
}

///////////////////////////////////////////////////
//////////////// CENTER OF MASS ///////////////////
///////////////////////////////////////////////////

TRANSLATION Robot::centerOfMass(FrameType withRespectTo)
{
    TRANSLATION com; com.setZero();
    for(int i=0; i<nLinkages(); i++)
    {
        com += linkage(i).centerOfMass(withRespectTo)*linkage(i).mass();
        if(com(0) != com(0)
                || com(1) != com(1)
                || com(2) != com(2))
            cerr << "NaN at linkage " << linkage(i).name() << " (" << linkage(i).id() << ")" << endl;
    }

    if(WORLD == withRespectTo)
        com += respectToWorld()*rootLink.com()*rootLink.mass();
    else if(ROBOT == withRespectTo)
        com += rootLink.com()*rootLink.mass();
    else
    {
        cerr << "Invalid reference frame type for center of mass calculation: "
             << FrameType_to_string(withRespectTo) << endl;
        cerr << " -- Must be WORLD or ROBOT" << endl;
    }

    double tempMass = mass();
    if(tempMass>0)
        return com/tempMass;
    else
        return TRANSLATION::Zero();
}

TRANSLATION Robot::centerOfMass(const vector<size_t> &indices, FrameType typeOfIndex, FrameType withRespectTo)
{
    TRANSLATION com; com.setZero();
    if( JOINT == typeOfIndex )
        for(int i=0; i<indices.size(); i++)
            com += joint(indices[i]).centerOfMass(withRespectTo)*joint(indices[i]).mass();

    else if( LINKAGE == typeOfIndex )
        for(int i=0; i<indices.size(); i++)
            com += linkage(indices[i]).centerOfMass(withRespectTo)*joint(indices[i]).mass();

    else
    {
        cerr << "Invalid index type for center of mass calculation: "
                << FrameType_to_string(typeOfIndex) << endl;
        cerr << " -- Must be JOINT or LINKAGE" << endl;
        return TRANSLATION::Zero();
    }

    double tempMass = mass(indices, typeOfIndex);
    if(tempMass>0)
        return com/tempMass;
    else
        return TRANSLATION::Zero();
}

TRANSLATION Robot::centerOfMass(const vector<string> &names, FrameType typeOfIndex, FrameType withRespectTo)
{
    vector<size_t> indices(names.size());
    rk_result_t check;
    if( JOINT == typeOfIndex )
        check = jointNamesToIndices(names, indices);
    else if( LINKAGE == typeOfIndex )
        check = linkageNamesToIndices(names, indices);
    else
    {
        cerr << "Invalid index type for center of mass calculation: "
                << FrameType_to_string(typeOfIndex) << endl;
        return TRANSLATION::Zero();
    }

    if(check != RK_SOLVED)
    {
        cerr << "Error finding indices: " << rk_result_to_string(check) << endl;
        return TRANSLATION::Zero();
    }

    return centerOfMass(indices, typeOfIndex, withRespectTo);
}

double Robot::mass(const std::vector<size_t> &indices, FrameType typeOfIndex)
{
    double result = 0;

    if( JOINT == typeOfIndex)
        for(int i=0; i<indices.size(); i++)
            result += joint(indices[i]).mass();

    else if( LINKAGE == typeOfIndex )
        for(int i=0; i<indices.size(); i++)
            result += linkage(indices[i]).mass();

    else
    {
        cerr << "Invalid index type for mass calculation: "
                << FrameType_to_string(typeOfIndex) << endl;
        cerr << " -- Must be JOINT or LINKAGE" << endl;
        return 0;
    }

    result += rootLink.mass();

    return result;
}

double Robot::mass(const std::vector<std::string> &names, FrameType typeOfIndex)
{
    vector<size_t> indices(names.size());
    rk_result_t check;
    if( JOINT == typeOfIndex )
        check = jointNamesToIndices(names, indices);
    else if( LINKAGE == typeOfIndex )
        check = linkageNamesToIndices(names, indices);
    else
    {
        cerr << "Invalid index type for mass calculation: "
                << FrameType_to_string(typeOfIndex) << endl;
        return 0;
    }

    if(check != RK_SOLVED)
    {
        cerr << "Error finding indices: " << rk_result_to_string(check) << endl;
        return 0;
    }

    return mass(indices, typeOfIndex);
}

double Robot::mass()
{
    double result = 0;
    for(int i=0; i<nLinkages(); i++)
        result += linkage(i).mass();
    result += rootLink.mass();

    return result;
}

TRANSLATION Linkage::centerOfMass(FrameType withRespectTo)
{
    TRANSLATION com; com.setZero();
    for(int i=0; i<nJoints(); i++)
        com += joint(i).centerOfMass(withRespectTo)*joint(i).mass();


    com += tool().centerOfMass(withRespectTo)*tool().mass();

    double tempMass = mass();
    if(tempMass>0)
        return com/tempMass;
    else
        return TRANSLATION::Zero();
}

TRANSLATION Linkage::centerOfMass(const std::vector<size_t> &indices, bool includeTool, FrameType withRespectTo)
{
    TRANSLATION com; com.setZero();
    for(int i=0; i<indices.size(); i++)
        com += joint(indices[i]).centerOfMass(withRespectTo)*joint(indices[i]).mass();

    if(includeTool)
        com += tool().centerOfMass(withRespectTo)*tool().mass();

    double tempMass = mass(indices, includeTool);
    if(tempMass>0)
        return com/tempMass;
    else
        return TRANSLATION::Zero();
}

TRANSLATION Linkage::centerOfMass(size_t fromJoint, bool downstream, bool includeTool, FrameType withRespectTo)
{
    if(fromJoint < nJoints())
    {
        if(downstream)
        {
            TRANSLATION com; com.setZero();
            for(int i=fromJoint; i<nJoints(); i++)
                com += joint(i).centerOfMass(withRespectTo)*joint(i).mass();

            if(includeTool)
                com += tool().centerOfMass(withRespectTo)*tool().mass();

            double tempMass = mass(fromJoint, downstream, includeTool);
            if(tempMass>0)
                return com/tempMass;
            else
                return TRANSLATION::Zero();
        }
        else
        {
            TRANSLATION com; com.setZero();
            for(int i=fromJoint-1; i>=0; i--)
                com += joint(i).centerOfMass(withRespectTo)*joint(i).mass();

            double tempMass = mass(fromJoint, !downstream, includeTool);
            if(tempMass>0)
                return com/tempMass;
            else
                return TRANSLATION::Zero();
        }
    }

    cerr << "Index (" << fromJoint << ") out of bounds for CoM calculation of " << name() << endl;
    cerr << " -- Maximum index value is " << nJoints()-1 << endl;
    return TRANSLATION::Zero();
}

TRANSLATION Linkage::centerOfMass(string fromJoint, bool downstream, bool includeTool, FrameType withRespectTo)
{
    return centerOfMass(jointNameToIndex(fromJoint), downstream, includeTool, withRespectTo);
}

double Linkage::mass(size_t fromJoint, bool downstream, bool includeTool)
{
    if(fromJoint < nJoints())
    {
        if(downstream)
        {
            double mass = 0;
            for(int i=fromJoint; i<nJoints(); i++)
                mass += joint(i).mass();

            if(includeTool)
                mass += tool().mass();
            return mass;
        }
        else
        {
            double mass = 0;
            for(int i=fromJoint-1; i>=0; i--)
                mass += joint(i).mass();

            return mass;
        }
    }

    cerr << "Index (" << fromJoint << ") out of bounds for mass calculation of " << name() << endl;
    cerr << " -- Maximum index value is " << nJoints()-1 << endl;
    return 0;
}

double Linkage::mass(string fromJoint, bool downstream, bool includeTool)
{
    return mass(jointNameToIndex(fromJoint), downstream, includeTool);
}

TRANSLATION Linkage::centerOfMass(size_t fromJoint, size_t toJoint, FrameType withRespectTo)
{
    if( fromJoint >= nJoints() || toJoint >= nJoints() )
    {
        cerr << "Index range is invalid for center of mass calculation [" << fromJoint << " -> "
             << toJoint << "]" << endl;
        cerr << " -- Valid range must be within 0 to " << nJoints() << endl;
        return TRANSLATION::Zero();
    }

    TRANSLATION com; com.setZero();

    if(fromJoint <= toJoint)
        for(int i=fromJoint; i<toJoint; i++)
            com += joint(i).centerOfMass(withRespectTo)*joint(i).mass();

    else if(toJoint < fromJoint)
        for(int i=toJoint; i<fromJoint; i++)
            com += joint(i).centerOfMass(withRespectTo)*joint(i).mass();

    double tempMass = mass(fromJoint, toJoint);
    if(tempMass>0)
        return com/tempMass;
    else
        return TRANSLATION::Zero();
}

TRANSLATION Linkage::centerOfMass(string fromJoint, string toJoint, FrameType withRespectTo)
{
    return centerOfMass(jointNameToIndex(fromJoint), jointNameToIndex(toJoint), withRespectTo);
}

double Linkage::mass(size_t fromJoint, size_t toJoint)
{
    if( fromJoint >= nJoints() || toJoint >= nJoints() )
    {
        cerr << "Index range is invalid for mass calculation [" << fromJoint << " -> "
             << toJoint << "]" << endl;
        cerr << " -- Valid range must be within 0 to " << nJoints() << endl;
        return 0;
    }

    double mass = 0;

    if(fromJoint <= toJoint)
        for(int i=fromJoint; i<toJoint; i++)
            mass += joint(i).mass();

    else if(toJoint < fromJoint)
        for(int i=toJoint; i<fromJoint; i++)
            mass += joint(i).mass();

    return mass;
}

double Linkage::mass(string fromJoint, string toJoint)
{
    return mass(jointNameToIndex(fromJoint), jointNameToIndex(toJoint));
}

TRANSLATION Linkage::centerOfMass(const std::vector<string> &names, bool includeTool, FrameType withRespectTo)
{
    vector<size_t> indices;
    rk_result_t check = jointNamesToIndices(names, indices);
    if( check != RK_SOLVED )
    {
        cerr << "Error finding indices: " << rk_result_to_string(check) << endl;
        return TRANSLATION::Zero();
    }

    return centerOfMass(indices, includeTool, withRespectTo);
}

double Linkage::mass()
{
    double result = 0;
    for(int i=0; i<nJoints(); i++)
        result += joint(i).mass();

    result += tool().mass();

    return result;
}

double Linkage::mass(const vector<size_t> &indices, bool includeTool)
{
    double result = 0;
    for(int i=0; i<indices.size(); i++)
        result += joint(indices[i]).mass();

    if(includeTool)
        result += tool().mass();

    return result;
}

void gravityTorqueHelper(Robot& robot, size_t startLinkage, size_t nextLinkage, TRANSLATION& com, double& mass)
{
    double newMass = robot.linkage(nextLinkage).mass();
    com  += robot.linkage(nextLinkage).centerOfMass()*newMass;
    mass += newMass;

    for(int i=0; i<robot.linkage(nextLinkage).nChildren(); i++)
        if(robot.linkage(nextLinkage).childLinkage(i).id() != startLinkage)
            gravityTorqueHelper(robot, nextLinkage,
                                robot.linkage(nextLinkage).childLinkage(i).id(),
                                com, mass);

    if(robot.linkage(nextLinkage).parentLinkage().id() != startLinkage)
        gravityTorqueHelper(robot, nextLinkage,
                            robot.linkage(nextLinkage).parentLinkage().id(),
                            com, mass);
}

double Joint::gravityTorque(bool downstream)
{
    TRANSLATION com; com.setZero();
    double m_mass=0;

    m_mass += linkage().mass(localID(), downstream, true);
    com  += linkage().centerOfMass(localID(), downstream, true)*m_mass;

    if(downstream)
        for(int i=0; i<linkage().nChildren(); i++)
            gravityTorqueHelper(robot(), linkage().id(), linkage().childLinkage(i).id(), com, m_mass);

    else
        if(linkage().hasParent)
            gravityTorqueHelper(robot(), linkage().id(), linkage().parentLinkage().id(), com, m_mass);


    if(m_mass>0)
        com = com/m_mass;
    else
        return 0; // TODO: Put assertive warning


    TRANSLATION lever = com - respectToRobot().translation();
    TRANSLATION Fz;
    Fz = TRANSLATION::UnitZ()*gravity_constant*m_mass;


    if(downstream)
    {
        if(jointType_ == REVOLUTE)
            return lever.cross(Fz).dot(respectToRobot().rotation()*jointAxis_);
        else if(jointType_ == PRISMATIC)
            return Fz.dot(respectToRobot().rotation()*jointAxis_);
        else
            return 0;
    }
    else
    {
        if(jointType_ == REVOLUTE)
            return -lever.cross(Fz).dot(respectToRobot().rotation()*jointAxis_);
        else if(jointType_ == PRISMATIC)
            return -Fz.dot(respectToRobot().rotation()*jointAxis_);
        else
            return 0;
    }
}



void Robot::gravityJointTorques(const vector<size_t> &jointIndices, Eigen::VectorXd &torques, bool downstream)
{
    // TODO: Perhaps make this more efficient by using something besides gravityTorque()
    // If CoM and mass results were recycled, it could perhaps be made more efficient
    // but it would require dynamic allocation

    torques.resize(jointIndices.size());
    for(int i=0; i<jointIndices.size(); i++)
        torques[i] = joint(jointIndices[i]).gravityTorque(downstream);
}

void Linkage::gravityJointTorques(Eigen::VectorXd &torques, bool downstream)
{
    // TODO: Same todo as Robot::gravityJointTorques

    torques.resize(nJoints());
    for(int i=0; i<nJoints(); i++)
        torques[i] = joint(i).gravityTorque(downstream);
}





// Based on a paper by Samuel R. Buss and Jin-Su Kim // TODO: Cite the paper properly
rk_result_t Robot::selectivelyDampedLeastSquaresIK_chain(const vector<size_t> &jointIndices, VectorXd &jointValues,
                                              const TRANSFORM &target, const TRANSFORM &finalTF)
{
    return RK_SOLVER_NOT_READY;
    // FIXME: Make this work


    // Arbitrary constant for maximum angle change in one step
    double gammaMax = M_PI/4; // TODO: Put this in the constructor so the user can change it at a whim


    vector<Joint*> joints;
    joints.resize(jointIndices.size());
    // FIXME: Add in safety checks
    for(int i=0; i<joints.size(); i++)
        joints[i] = joints_[jointIndices[i]];

    // ~~ Declarations ~~
    MatrixXd J;
    JacobiSVD<MatrixXd> svd;
    TRANSFORM pose;
    AngleAxisd aagoal;
    AngleAxisd aastate;
    SCREW goal;
    SCREW state;
    SCREW err;
    SCREW alpha;
    SCREW N;
    SCREW M;
    SCREW gamma;
    VectorXd delta(jointValues.size());
    VectorXd tempPhi(jointValues.size());
    // ~~~~~~~~~~~~~~~~~~

//    cout << "\n\n" << endl;

    double tolerance = 1*M_PI/180; // TODO: Put this in the constructor so the user can set it arbitrarily
    int maxIterations = 1000; // TODO: Put this in the constructor so the user can set it arbitrarily

    size_t iterations = 0;
    do {

        values(jointIndices, jointValues);

        jacobian(J, joints, joints.back()->respectToRobot().translation()+finalTF.translation(), this);

        svd.compute(J, ComputeFullU | ComputeThinV);

    //    cout <<  "\n\n" << svd.matrixU() << "\n\n\n" << svd.singularValues().transpose() << "\n\n\n" << svd.matrixV() << endl;

    //    for(int i=0; i<svd.matrixU().cols(); i++)
    //        cout << "u" << i << " : " << svd.matrixU().col(i).transpose() << endl;


    //    std::cout << "Joint name: " << joint(jointIndices.back()).name()
    //              << "\t Number: " << jointIndices.back() << std::endl;
        pose = joint(jointIndices.back()).respectToRobot()*finalTF;

    //    std::cout << "Pose: " << std::endl;
    //    std::cout << pose.matrix() << std::endl;

    //    AngleAxisd aagoal(target.rotation());
        aagoal = target.rotation();
        goal << target.translation(), aagoal.axis()*aagoal.angle();

        aastate = pose.rotation();
        state << pose.translation(), aastate.axis()*aastate.angle();

        err = goal-state;

    //    std::cout << "state: " << state.transpose() << std::endl;
    //    std::cout << "err: " << err.transpose() << std::endl;

        for(int i=0; i<6; i++)
            alpha[i] = svd.matrixU().col(i).dot(err);

    //    std::cout << "Alpha: " << alpha.transpose() << std::endl;

        for(int i=0; i<6; i++)
        {
            N[i] = svd.matrixU().block(0,i,3,1).norm();
            N[i] += svd.matrixU().block(3,i,3,1).norm();
        }

    //    std::cout << "N: " << N.transpose() << std::endl;

        double tempMik = 0;
        for(int i=0; i<svd.matrixV().cols(); i++)
        {
            M[i] = 0;
            for(int k=0; k<svd.matrixU().cols(); k++)
            {
                tempMik = 0;
                for(int j=0; j<svd.matrixV().cols(); j++)
                    tempMik += fabs(svd.matrixV()(j,i))*J(k,j);
                M[i] += 1/svd.singularValues()[i]*tempMik;
            }
        }

    //    std::cout << "M: " << M.transpose() << std::endl;

        for(int i=0; i<svd.matrixV().cols(); i++)
            gamma[i] = minimum(1, N[i]/M[i])*gammaMax;

    //    std::cout << "Gamma: " << gamma.transpose() << std::endl;

        delta.setZero();
        for(int i=0; i<svd.matrixV().cols(); i++)
        {
    //        std::cout << "1/sigma: " << 1/svd.singularValues()[i] << std::endl;
            tempPhi = 1/svd.singularValues()[i]*alpha[i]*svd.matrixV().col(i);
    //        std::cout << "Phi: " << tempPhi.transpose() << std::endl;
            clampMaxAbs(tempPhi, gamma[i]);
            delta += tempPhi;
    //        std::cout << "delta " << i << ": " << delta.transpose() << std::endl;
        }

        clampMaxAbs(delta, gammaMax);

        jointValues += delta;

        std::cout << iterations << " | Norm:" << delta.norm() << "\tdelta: "
                  << delta.transpose() << "\tJoints:" << jointValues.transpose() << std::endl;

        iterations++;
    } while(delta.norm() > tolerance && iterations < maxIterations);
}

rk_result_t Robot::selectivelyDampedLeastSquaresIK_chain(const vector<string> &jointNames, VectorXd &jointValues,
                                              const TRANSFORM &target, const TRANSFORM &finalTF)
{
    // TODO: Make the conversion from vector<string> to vector<size_t> its own function
    vector<size_t> jointIndices;
    jointIndices.resize(jointNames.size());
    map<string,size_t>::iterator j;
    for(int i=0; i<jointNames.size(); i++)
    {
        j = jointNameToIndex_.find(jointNames[i]);
        if( j == jointNameToIndex_.end() )
            return RK_INVALID_JOINT;
        jointIndices[i] = j->second;
    }

    return selectivelyDampedLeastSquaresIK_chain(jointIndices, jointValues, target);
}


rk_result_t Robot::selectivelyDampedLeastSquaresIK_linkage(const string linkageName, VectorXd &jointValues,
                                                const TRANSFORM &target, const TRANSFORM &finalTF)
{
    vector<size_t> jointIndices;
    jointIndices.resize(linkage(linkageName).joints_.size());
    for(size_t i=0; i<linkage(linkageName).joints_.size(); i++)
        jointIndices[i] = linkage(linkageName).joints_[i]->id();

    TRANSFORM linkageFinalTF;
    linkageFinalTF = linkage(linkageName).tool().respectToFixed()*finalTF;

    return selectivelyDampedLeastSquaresIK_chain(jointIndices, jointValues, target, linkageFinalTF);
}



rk_result_t Robot::pseudoinverseIK_chain(const vector<size_t> &jointIndices, VectorXd &jointValues,
                                  const TRANSFORM &target, const TRANSFORM &finalTF)
{
    return RK_SOLVER_NOT_READY;
    // FIXME: Make this solver work


    vector<Joint*> joints;
    joints.resize(jointIndices.size());
    // FIXME: Add in safety checks
    for(int i=0; i<joints.size(); i++)
        joints[i] = joints_[jointIndices[i]];

    // ~~ Declarations ~~
    MatrixXd J;
    MatrixXd Jinv;
    TRANSFORM pose;
    AngleAxisd aagoal;
    AngleAxisd aastate;
    SCREW goal;
    SCREW state;
    SCREW err;
    VectorXd delta(jointValues.size());

    MatrixXd Jsub;
    aagoal = target.rotation();
    goal << target.translation(), aagoal.axis()*aagoal.angle();

    double tolerance = 1*M_PI/180; // TODO: Put this in the constructor so the user can set it arbitrarily
    int maxIterations = 100; // TODO: Put this in the constructor so the user can set it arbitrarily
    double errorClamp = 0.25; // TODO: Put this in the constructor
    double deltaClamp = M_PI/4; // TODO: Put this in the constructor

    size_t iterations = 0;
    do {

        values(jointIndices, jointValues);

        jacobian(J, joints, joints.back()->respectToRobot().translation()+finalTF.translation(), this);
        Jsub = J.block(0,0,3,jointValues.size());

        pinv(Jsub, Jinv);

        pose = joint(jointIndices.back()).respectToRobot()*finalTF;
        aastate = pose.rotation();
        state << pose.translation(), aastate.axis()*aastate.angle();

        err = goal-state;
        for(int i=3; i<6; i++)
            err[i] *= 0;
        err.normalize();

        TRANSLATION e = (target.translation() - pose.translation()).normalized()*0.005;

//        delta = Jinv*err*0.1;
//        clampMag(delta, deltaClamp);
        VectorXd d = Jinv*e;

//        jointValues += delta;
        jointValues += d;
        std::cout << iterations << " | Norm:" << delta.norm()
//                  << "\tdelta: " << delta.transpose() << "\tJoints:" << jointValues.transpose() << std::endl;
                  << " | " << (target.translation() - pose.translation()).norm()
                  << "\tErr: " << (goal-state).transpose() << std::endl;


        iterations++;
    } while(delta.norm() > tolerance && iterations < maxIterations);

}



rk_result_t Robot::pseudoinverseIK_chain(const vector<string> &jointNames, VectorXd &jointValues,
                                              const TRANSFORM &target, const TRANSFORM &finalTF)
{
    // TODO: Make the conversion from vector<string> to vector<size_t> its own function
    vector<size_t> jointIndices;
    jointIndices.resize(jointNames.size());
    map<string,size_t>::iterator j;
    for(int i=0; i<jointNames.size(); i++)
    {
        j = jointNameToIndex_.find(jointNames[i]);
        if( j == jointNameToIndex_.end() )
            return RK_INVALID_JOINT;
        jointIndices[i] = j->second;
    }

    return pseudoinverseIK_chain(jointIndices, jointValues, target);
}


rk_result_t Robot::pseudoinverseIK_linkage(const string linkageName, VectorXd &jointValues,
                                                const TRANSFORM &target, const TRANSFORM &finalTF)
{
    vector<size_t> jointIndices;
    jointIndices.resize(linkage(linkageName).joints_.size());
    for(size_t i=0; i<linkage(linkageName).joints_.size(); i++)
        jointIndices[i] = linkage(linkageName).joints_[i]->id();

    TRANSFORM linkageFinalTF;
    linkageFinalTF = linkage(linkageName).tool().respectToFixed()*finalTF;

    return pseudoinverseIK_chain(jointIndices, jointValues, target, linkageFinalTF);
}


rk_result_t Robot::jacobianTransposeIK_chain(const vector<size_t> &jointIndices, VectorXd &jointValues, const TRANSFORM &target, const TRANSFORM &finalTF)
{
    return RK_SOLVER_NOT_READY;
    // FIXME: Make this solver work


    vector<Joint*> joints;
    joints.resize(jointIndices.size());
    // FIXME: Add in safety checks
    for(int i=0; i<joints.size(); i++)
        joints[i] = joints_[jointIndices[i]];

    // ~~ Declarations ~~
    MatrixXd J;
    MatrixXd Jinv;
    TRANSFORM pose;
    AngleAxisd aagoal;
    AngleAxisd aastate;
    SCREW state;
    SCREW err;
    VectorXd delta(jointValues.size());
    SCREW gamma;
    double alpha;

    aagoal = target.rotation();

    double Tscale = 3; // TODO: Put these as a class member in the constructor
    double Rscale = 0;

    double tolerance = 1*M_PI/180; // TODO: Put this in the constructor so the user can set it arbitrarily
    int maxIterations = 100; // TODO: Put this in the constructor so the user can set it arbitrarily

    size_t iterations = 0;
    do {
        values(jointIndices, jointValues);

        jacobian(J, joints, joints.back()->respectToRobot().translation()+finalTF.translation(), this);

        pose = joint(jointIndices.back()).respectToRobot()*finalTF;
        aastate = pose.rotation();
        state << pose.translation(), aastate.axis()*aastate.angle();

        err << (target.translation()-pose.translation()).normalized()*Tscale,
               (aagoal.angle()*aagoal.axis()-aastate.angle()*aastate.axis()).normalized()*Rscale;

        gamma = J*J.transpose()*err;
        alpha = err.dot(gamma)/gamma.norm();

        delta = alpha*J.transpose()*err;

        jointValues += delta;
        iterations++;

        std::cout << iterations << " | Norm:" << delta.norm()
//                  << "\tdelta: " << delta.transpose() << "\tJoints:" << jointValues.transpose() << std::endl;
                  << " | " << (target.translation() - pose.translation()).norm()
                  << "\tErr: " << (target.translation()-pose.translation()).transpose() << std::endl;

    } while(err.norm() > tolerance && iterations < maxIterations);

}


rk_result_t Robot::jacobianTransposeIK_chain(const vector<string> &jointNames, VectorXd &jointValues,
                                              const TRANSFORM &target, const TRANSFORM &finalTF)
{
    // TODO: Make the conversion from vector<string> to vector<size_t> its own function
    vector<size_t> jointIndices;
    jointIndices.resize(jointNames.size());
    map<string,size_t>::iterator j;
    for(int i=0; i<jointNames.size(); i++)
    {
        j = jointNameToIndex_.find(jointNames[i]);
        if( j == jointNameToIndex_.end() )
            return RK_INVALID_JOINT;
        jointIndices[i] = j->second;
    }

    return jacobianTransposeIK_chain(jointIndices, jointValues, target);
}


rk_result_t Robot::jacobianTransposeIK_linkage(const string linkageName, VectorXd &jointValues,
                                                const TRANSFORM &target, const TRANSFORM &finalTF)
{
    vector<size_t> jointIndices;
    jointIndices.resize(linkage(linkageName).joints_.size());
    for(size_t i=0; i<linkage(linkageName).joints_.size(); i++)
        jointIndices[i] = linkage(linkageName).joints_[i]->id();

    TRANSFORM linkageFinalTF;
    linkageFinalTF = linkage(linkageName).tool().respectToFixed()*finalTF;

    return jacobianTransposeIK_chain(jointIndices, jointValues, target, linkageFinalTF);
}



// TODO: Make a constraint class instead of restValues
rk_result_t Robot::dampedLeastSquaresIK_chain(const vector<size_t> &jointIndices, VectorXd &jointValues,
                                              const TRANSFORM &target, Constraints& constraints )
{
    bool storedImposeLimits = imposeLimits;

    vector<Joint*> pJoints;
    pJoints.resize(jointIndices.size());
    // FIXME: Add in safety checks
    for(int i=0; i<pJoints.size(); i++)
        pJoints[i] = joints_[jointIndices[i]];

    // ~~ Declarations ~~
    MatrixXd J;
    MatrixXd Jinv;
    TRANSFORM pose;
    AngleAxisd aagoal(target.rotation());
    AngleAxisd aastate;
    AngleAxisd aaerr;
    TRANSLATION Terr;
    TRANSLATION Rerr;
    SCREW err;
    VectorXd nullErr(jointValues.size());
    VectorXd delta(jointValues.size());
    VectorXd stored(jointValues.size());
    VectorXd f(jointValues.size());
    double rotAngle=0;

    VectorXd deltaNull(jointValues.size());


    stored = jointValues;

    double tolerance = constraints.convergenceTolerance;
    int maxIterations = constraints.maxIterations;
    double damp = constraints.dampingConstant;


    size_t maxAttempts = 1;
    if(constraints.useIterativeJacobianSeed)
        maxAttempts = constraints.maxAttempts;

    for(size_t attempt=0; attempt<maxAttempts; attempt++)
    {
        if(constraints.useIterativeJacobianSeed)
            constraints.iterativeJacobianSeed(*this, attempt, jointIndices, jointValues);

        values(jointIndices, jointValues);

        pose = joint(jointIndices.back()).respectToRobot()*constraints.finalTransform;
//        pose = constraints.finalTransform*joint(jointIndices.back()).respectToRobot();
        aastate = pose.rotation();

//        aaerr = pose.rotation().transpose()*target.rotation(); // FAILED
//        aaerr = target.rotation().transpose()*pose.rotation(); // FAILED
//        aaerr = pose.rotation()*target.rotation().transpose(); // FAILED
        aaerr = target.rotation()*pose.rotation().transpose();
        if(fabs(aaerr.angle()) <= M_PI)
            Rerr = aaerr.angle()*aaerr.axis();
        else
            Rerr = (aaerr.angle()-2*M_PI)*aaerr.axis();

        Terr = target.translation()-pose.translation();

    //    Rerr.setZero();
        err << Terr, Rerr;//*constraints.rotationScale;

        size_t iterations = 0;
        do {

            if(constraints.performErrorClamp)
            {
                clampMag(Terr, constraints.translationClamp);
                clampMag(Rerr, constraints.rotationClamp);
            }
            err << Terr, Rerr;//*constraints.rotationScale;

            if(constraints.customErrorClamp)
                constraints.errorClamp(*this, jointIndices, err);

            if(verbose)
            {
                cout << "Clamped Error: " << err.transpose() << endl;

                cout << "-----------------------------------" << endl;
            }

//            jacobian(J, pJoints, pJoints.back()->respectToRobot().translation()+constraints.finalTransform.translation(), this);
            jacobian(J, pJoints, pose.translation(), this);

            /////////////////////////////////////////////////////////////////////////
            /////////////////////////  STANDARD APPROACH  ///////////////////////////
            /////////////////////////////////////////////////////////////////////////

            f = (J*J.transpose() + damp*damp*Matrix6d::Identity()).colPivHouseholderQr().solve(err);
            delta = J.transpose()*f;

            /////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////


            if(constraints.performNullSpaceTask)
                delta += constraints.nullSpaceTask(*this, J, jointIndices, jointValues);



            ///////////////////////////////////////////////////////////////////
            ///////////////////////  NULL SPACE APPROACH //////////////////////
            ///////////////////////////////////////////////////////////////////
/*
            Jinv = J.transpose()*(J*J.transpose() + damp*damp*Matrix6d::Identity()).inverse();

            delta = Jinv*err;

            if(constraints.performDeltaClamp)
                clampMaxAbs(delta, constraints.deltaClamp);


            if(constraints.performNullSpaceTask)
                constraints.nullSpaceTask(*this, jointIndices, jointValues, nullErr);
            else
                nullErr.setZero();

            // Try pure nullspace first
            deltaNull = (Matrix6d::Identity() - J.transpose()*(J*J.transpose()).inverse()*J)*nullErr;

            // The damped nullspace is much better for avoiding NaNs
            for(int n=0; n<deltaNull.size(); n++)
            {
                if(deltaNull[n] != deltaNull[n])
                {
                    deltaNull = (Matrix6d::Identity() - Jinv*J)*nullErr;
                    cout << "NaN in the Nullspace!!" << endl;
                    break;
                }
            }

            delta += deltaNull;
*/
            ///////////////////////////////////////////////////////////////////


            jointValues += delta;

            if(constraints.wrapToJointLimits)
                wrapToJointLimits(*this, jointIndices, jointValues);


            values(jointIndices, jointValues);

            // Catch any joint limits
            for(int k=0; k<jointIndices.size(); k++)
                jointValues(k) = joint(jointIndices[k]).value();

            pose = joint(jointIndices.back()).respectToRobot()*constraints.finalTransform;
//            pose = constraints.finalTransform*joint(jointIndices.back()).respectToRobot();

            if(verbose)
            {
                cout << "req delta: " << delta.transpose() << endl;
                cout << "act delta: " << (jointValues-stored).transpose() << endl;
                cout << "angles: " << jointValues.transpose() << endl;
                cout << "Limits: ";
                for(int k=0; k<jointIndices.size(); k++)
                    cout << "(" << joint(jointIndices[k]).min() << ", "
                         << joint(jointIndices[k]).max() << ")\t";
                cout << endl;
                cout << pose.matrix() << endl;
                cout << "Rotation needed: " << endl;
                cout << (pose.rotation().transpose()*target.rotation()).matrix() << endl;
            }
            
            
            Terr = target.translation()-pose.translation();

//            aaerr = pose.rotation().transpose()*target.rotation(); // FAILED
//            aaerr = target.rotation().transpose()*pose.rotation(); // FAILED
//            aaerr = pose.rotation()*target.rotation().transpose();
            aaerr = target.rotation()*pose.rotation().transpose();

//            cout << "Angle-Axis: (" << aaerr.angle()/M_PI*180 << ")\t" << aaerr.axis().transpose() << endl;

            if(aaerr.angle() > 2*M_PI || aaerr.angle() < 0)
                cout << "BROKEN ANGLE AXIS: " << aaerr.angle() << endl
                     << " -- Please contact mxgrey@gatech.edu and report this." << endl;
                
            if(fabs(aaerr.angle()) <= M_PI)
                Rerr = aaerr.angle()*aaerr.axis();
            else
                Rerr = (aaerr.angle()-2*M_PI)*aaerr.axis();

//            rotAngle = wrapToPi(aaerr.angle()); // This failed badly
//            Rerr = rotAngle*aaerr.axis();

            err << Terr, Rerr;

            if(verbose)
            {
                cout << "Error: " << err.transpose() << endl;
            }

            iterations++;


//        } while(err.norm() > tolerance && iterations < maxIterations);
        } while( (Terr.norm() > tolerance || Rerr.norm() > tolerance || !constraints.nullComplete())
                 && iterations < maxIterations);

        if(verbose)
        {
            cout << "Iterations: -- " << iterations << endl;
        }


        if(constraints.wrapSolutionToJointLimits)
            wrapToJointLimits(*this, jointIndices, jointValues);

        imposeLimits = storedImposeLimits;
        values(jointIndices, jointValues);


        pose = joint(jointIndices.back()).respectToRobot()*constraints.finalTransform;
//        pose = constraints.finalTransform*joint(jointIndices.back()).respectToRobot();
        
        
//        aaerr = pose.rotation().transpose()*target.rotation(); // FAILED
//        aaerr = target.rotation().transpose()*pose.rotation(); // FAILED
//        aaerr = pose.rotation()*target.rotation().transpose(); // FAILED
        aaerr = target.rotation()*pose.rotation().transpose();
        if(fabs(aaerr.angle()) <= M_PI)
            Rerr = aaerr.angle()*aaerr.axis();
        else
            Rerr = (aaerr.angle()-2*M_PI)*aaerr.axis();
        
        Terr = target.translation()-pose.translation();
//            Rerr = -aaerr.angle()*aaerr.axis();

        err << Terr, Rerr;


        if(Terr.norm() <= tolerance && Rerr.norm() <= tolerance)
            return RK_SOLVED;
    }




    return RK_DIVERGED;

}

rk_result_t Robot::dampedLeastSquaresIK_chain(const vector<string> &jointNames, VectorXd &jointValues,
                                              const TRANSFORM &target, Constraints& constraints)
{
    vector<size_t> jointIndices;

    if( jointNamesToIndices(jointNames, jointIndices) == RK_INVALID_JOINT )
        return RK_INVALID_JOINT;

    return dampedLeastSquaresIK_chain(jointIndices, jointValues, target, constraints);
}


rk_result_t Robot::dampedLeastSquaresIK_linkage(const string linkageName, VectorXd &jointValues,
                                                const TRANSFORM &target, Constraints& constraints)
{
    if(linkage(linkageName).name().compare("invalid")==0)
        return RK_INVALID_LINKAGE;

    vector<size_t> jointIndices;
    jointIndices.resize(linkage(linkageName).joints_.size());
    for(size_t i=0; i<linkage(linkageName).joints_.size(); i++)
        jointIndices[i] = linkage(linkageName).joints_[i]->id();

    constraints.finalTransform = linkage(linkageName).tool().respectToFixed();

    return dampedLeastSquaresIK_chain(jointIndices, jointValues, target, constraints);
}





