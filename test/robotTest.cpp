/*
 -------------------------------------------------------------------------------
 robotTest.cpp
 robotTest Project
 
 Initially created by Rowland O'Flaherty ( rowlandoflaherty.com ) on 5/11/13.
 
 Version 1.0
 -------------------------------------------------------------------------------
 */

// For measuring time performance
// Taken from http://aufather.wordpress.com/2010/09/08/high-performance-time-measuremen-in-linux/
//#include <stdint.h> /* for uint64_t */
//#include <time.h>  /* for struct timespec */

///* assembly code to read the TSC */
//static inline uint64_t RDTSC()
//{
//  unsigned int hi, lo;
//  __asm__ volatile("rdtsc" : "=a" (lo), "=d" (hi));
//  return ((uint64_t)hi << 32) | lo;
//}

//const int NANO_SECONDS_IN_SEC = 1000000000;
///* returns a static buffer of struct timespec with the time difference of ts1 and ts2
//   ts1 is assumed to be greater than ts2 */
//struct timespec *TimeSpecDiff(struct timespec *ts1, struct timespec *ts2)
//{
//  static struct timespec ts;
//  ts.tv_sec = ts1->tv_sec - ts2->tv_sec;
//  ts.tv_nsec = ts1->tv_nsec - ts2->tv_nsec;
//  if (ts.tv_nsec < 0) {
//    ts.tv_sec--;
//    ts.tv_nsec += NANO_SECONDS_IN_SEC;
//  }
//  return &ts;
//}

//double g_TicksPerNanoSec;
//static void CalibrateTicks()
//{
//  struct timespec begints, endts;
//  uint64_t begin = 0, end = 0;
//  clock_gettime(CLOCK_MONOTONIC, &begints);
//  begin = RDTSC();
//  uint64_t i;
//  for (i = 0; i < 1000000; i++); /* must be CPU intensive */
//  end = RDTSC();
//  clock_gettime(CLOCK_MONOTONIC, &endts);
//  struct timespec *tmpts = TimeSpecDiff(&endts, &begints);
//  uint64_t nsecElapsed = tmpts->tv_sec * 1000000000LL + tmpts->tv_nsec;
//  g_TicksPerNanoSec = (double)(end - begin)/(double)nsecElapsed;
//}

///* Call once before using RDTSC, has side effect of binding process to CPU1 */
//void InitRdtsc()
//{
//  unsigned long cpuMask;
//  cpuMask = 2; // bind to cpu 1
//  sched_setaffinity(0, sizeof(cpuMask), &cpuMask);
//  CalibrateTicks();
//}

//void GetTimeSpec(struct timespec *ts, uint64_t nsecs)
//{
//  ts->tv_sec = nsecs / NANO_SECONDS_IN_SEC;
//  ts->tv_nsec = nsecs % NANO_SECONDS_IN_SEC;
//}

///* ts will be filled with time converted from TSC reading */
//void GetRdtscTime(struct timespec *ts)
//{
//  GetTimeSpec(ts, RDTSC() / g_TicksPerNanoSec);
//}








//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include <iostream>
#include <vector>
#include "Frame.h"
#include "Linkage.h"
#include "Robot.h"
#include "Hubo.h"

#include <time.h>

//------------------------------------------------------------------------------
// Namespaces
//------------------------------------------------------------------------------
using namespace std;
using namespace Eigen;
using namespace RobotKin;

//------------------------------------------------------------------------------
// Global Varible Declarations
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
// Function Declarations
//------------------------------------------------------------------------------
void tutorial();


//------------------------------------------------------------------------------
// Main Function
//------------------------------------------------------------------------------

int main(int argc, char *argv[])
{
    tutorial();

    return 0;
}


//------------------------------------------------------------------------------
// Function Definitions
//------------------------------------------------------------------------------
void tutorial()
{
    cout << "---------------------------" << endl;
    cout << "|  Creating A Hubo Robot  |" << endl;
    cout << "---------------------------" << endl << endl;
    
    // Create an instance of a Hubo
    Hubo hubo;
    
/*
    
    // Print info about hubo
    cout << "The robot " << hubo.name() << " consist of " << hubo.nLinkages() << " linkages and " << hubo.nJoints() << " joints all together." << endl << endl;
    
    
    cout << "-------------------------" << endl;
    cout << "|  Linkages and Joints  |" << endl;
    cout << "-------------------------" << endl << endl;
    
    
    cout << "The linkages are: " << endl;
    cout << "(ID, Name, Parent)" << endl;
    for (vector<Linkage*>::iterator linkageIt = hubo.linkages().begin(); linkageIt != hubo.linkages().end(); ++linkageIt) {
        if ((*linkageIt)->parentLinkage() == 0) {
            cout << (*linkageIt)->id() << ", " << (*linkageIt)->name() << " <- " << hubo.name() << endl;
        } else {
            cout << (*linkageIt)->id() << ", " << (*linkageIt)->name() << " <- " << (*linkageIt)->parentLinkage()->name() << endl;
        }
    }
    cout << endl;
    
    size_t rightArmIndex = hubo.linkageIndex("RIGHT_ARM"); // Get index to right arm or can refer to right arm directly (see line below).
    cout << "Each linkage consist of joints." << endl << endl;
    cout << "For example the " << hubo.linkage(rightArmIndex).name() << " linkage has " << hubo.linkage(rightArmIndex).nJoints() << " joints." << endl << endl;
    
    cout << "The joints for the " << hubo.linkage(rightArmIndex).name() << " are:" << endl;
    cout << "(ID, Name, Value): " << endl;
    
    // Notice that there is a const_joints() as well as a joints() method (this exist for the robot class well), which allows for when the "this" pointer is const as is the case with the const iterator below.
    for (vector<Linkage::Joint*>::const_iterator jointIt = hubo.linkage("RIGHT_ARM").const_joints().begin();
         jointIt != hubo.linkage("RIGHT_ARM").const_joints().end(); ++jointIt) {
        cout << (*jointIt)->id() << ", " << (*jointIt)->name() << ", " << (*jointIt)->value() << endl;
    }
    cout << endl;
    
    cout << "---------------------------" << endl;
    cout << "|  Changing joint values  |" << endl;
    cout << "---------------------------" << endl << endl;
    
    cout << "Let's change the joint values." << endl << endl;
    
    // Set the joints all at once
    cout << "Set all the joints to have a value of PI/2." << endl;
    
    Linkage* rightArm = &hubo.linkage("RIGHT_ARM"); // Notice that this is a pointer
    VectorXd q = M_PI/2 * VectorXd::Ones(rightArm->nJoints(), 1);
    rightArm->values(q); // Set all joints at once
    
    cout << rightArm->values() << endl << endl;
    
    // Set the joints individually
    cout << "Set the joints individually." << endl;
    
    Linkage::Joint* joint0 = &rightArm->joint(0); // Again notice that this is a pointer
    Linkage::Joint joint1 = rightArm->joint(1); // This is not a pointer but return a const reference, but updating this joint will not update the robot
    Linkage::Joint* joint2 = &rightArm->joint("RSY"); // Just like linkages, joints can be index by name
    
    joint0->value(0.1);
    joint1.value(0.2);
    joint2->value(0.3);
    hubo.joint("REP")->value(0.4); // Notice that the joint() method of the robot class returns a pointer but the linkage() method returned a reference
    hubo.joint(23)->value(0.4); // Notice that the joint index in the robot is different than the joint index in the linkage
    
    cout << hubo.linkage("RIGHT_ARM").values() << endl << endl;
    
    // Reset all to position minus offset
    cout << "Reset joint values minus offsets." << endl;
    
    hubo.linkage("RIGHT_ARM").values(-hubo.rightArmOffsets);
    
    cout << hubo.linkage("RIGHT_ARM").values() << endl << endl;
    
    cout << "------------" << endl;
    cout << "|  Frames  |" << endl;
    cout << "------------" << endl << endl;
    
    cout << "Robots, Linkages, Joints, and Tools are all frames which have homogenous transformation with respect to differenct frames associated with them." << endl << endl;
    
    Linkage::Tool* rightHand = &rightArm->tool();
    cout << "For example the " << rightHand->name() << " tool at the end of the " << rightArm->name() << " linkage has the following homogenous transformations associated with it." << endl << endl;
    
    rightHand->printInfo();
    
    cout << "Every frame has a HG transformation with respect to a fixed frame and the world frame." << endl << endl;
    
    cout << "For example, the " << hubo.name() << " robot has the following: " << endl << endl;
    hubo.Frame::printInfo();
    
    cout << "Some types of frames (e.g. Linkage::Joint) have more HG transformations associated with their frame." << endl << endl;
    
    cout << "For example, the " << joint2->name() << " joint has the following: " << endl << endl;
    joint2->printInfo();
    
    cout << "These frames automatically get updated when the joint values change." << endl << endl;
    
    cout << "For example, let's move the right arm straight out in front of the robot." << endl << endl;
    cout << "Joint values of the " << rightArm->name() << " are:" << endl;
    joint0->value(-M_PI/2);
    cout << hubo.linkage("RIGHT_ARM").values() << endl << endl;
    
    cout << "The " << rightHand->name() << " tool now has the following values:" << endl;
    cout << "Respect to fixed frame (" << rightHand->getParentJointName() << " joint)" << endl;
    cout << rightHand->respectToFixed().matrix() << endl << endl;
    cout << "Respect to linkage frame (" << rightHand->getLinkageName()<< " linkage)" << endl;
    cout << rightHand->respectToLinkage().matrix() << endl << endl;
    cout << "Respect to robot frame (" << rightHand->getRobotName() << " robot)" << endl;
    cout << rightHand->respectToRobot().matrix() << endl << endl;
    
    cout << "Notice the transform with respect to the fixed frame did not change but the others did." << endl << endl;
    
    cout << "Let's now twist the robot at the waist." << endl << endl;
    cout << "Joint values of the " << hubo.linkage("TORSO").name() << " are:" << endl;
    hubo.linkage("TORSO").joint(0).value(M_PI/2);
    cout << hubo.linkage("TORSO").values() << endl << endl;
    
    cout << "The " << rightHand->name() << " tool now has the following values:" << endl;
    cout << "Respect to fixed frame (" << rightHand->getParentJointName() << " joint)" << endl;
    cout << rightHand->respectToFixed().matrix() << endl << endl;
    cout << "Respect to linkage frame (" << rightHand->getLinkageName() << " linkage)" << endl;
    cout << rightHand->respectToLinkage().matrix() << endl << endl;
    cout << "Respect to robot frame (" << rightHand->getRobotName() << " robot)" << endl;
    cout << rightHand->respectToRobot().matrix() << endl << endl;
    
    cout << "Notice the transform with respect to the fixed frame and the linkage frame did not change but the with respect to the robot frame did." << endl << endl;
    
    cout << "---------------" << endl;
    cout << "|  Jacobians  |" << endl;
    cout << "---------------" << endl << endl;
    
    cout << "Jacobians for each linkage and the entire robot can be obtained easily." << endl << endl;
    
    cout << "For example, let's get the Jacobian for the " << rightArm->name() << " linkage where the location of the Jacobian is at " << rightArm->tool().name() << " and referenced with respect to the " << rightArm->name() << " linkage base coordinate frame." << endl << endl;
    rightArm->joint(0).value(0);
    cout << "Current joint values are:" << endl;
    cout << rightArm->values() << endl << endl;
    MatrixXd J;
    rightArm->jacobian(J,
                       rightArm->const_joints(),
                       rightArm->const_tool().respectToLinkage().translation(),
                       &hubo.linkage("RIGHT_ARM"));
    cout << "Jacobian is:" << endl;
    cout << J.matrix() << endl << endl;
    
    cout << "Or we can get the Jacobian from the " << hubo.linkage("RIGHT_LEG").tool().name() << " to the " << hubo.linkage("RIGHT_ARM").tool().name() << " and referenced with respect to " << hubo.name() << "." << endl << endl;
    
    vector<Linkage::Joint*> joints;
    for (vector<Linkage::Joint*>::reverse_iterator jointIt = hubo.linkage("RIGHT_LEG").joints().rbegin(); jointIt != hubo.linkage("RIGHT_LEG").joints().rend(); ++jointIt) {
        joints.push_back(*jointIt);
    }
    for (vector<Linkage::Joint*>::iterator jointIt = hubo.linkage("TORSO").joints().begin(); jointIt != hubo.linkage("TORSO").joints().end(); ++jointIt) {
        joints.push_back(*jointIt);
    }
    for (vector<Linkage::Joint*>::iterator jointIt = hubo.linkage("RIGHT_ARM").joints().begin(); jointIt != hubo.linkage("RIGHT_ARM").joints().end(); ++jointIt) {
        joints.push_back(*jointIt);
    }
    
    cout << "There are " << joints.size() << " joints involved in this Jacobian with the values of" << endl;

    cout << "(Name, Value)" << endl;
    for (vector<Linkage::Joint*>::iterator jointIt = joints.begin(); jointIt != joints.end(); ++jointIt) {
        cout << (*jointIt)->name() << ", " << (*jointIt)->value() << endl;
    }
    cout << endl;

     hubo.jacobian(J,
                   joints,
                   hubo.linkage("RIGHT_ARM").const_tool().respectToRobot().translation(),
                   &hubo);
    
    cout << "Jacobian is:" << endl;
    cout << J.matrix() << endl << endl;
    
//    rightHand->printInfo();
    
//    rightArm->printInfo();
    

    {

        Linkage* rightArmT = &hubo.linkage("RIGHT_ARM"); // Notice that this is a pointer
        size_t torsoID = rightArmT->getParentLinkageID();
        hubo.linkages()[torsoID]->printChildren();
        rightArmT->printChildren();

        rightArmT->printInfo();

        Linkage* rightLegT = &hubo.linkage("RIGHT_LEG");
        rightLegT->printInfo();

    }
    
    cout << "--------------------------------------" << endl;
    cout << "|  Analytical Arm Inverse Kinematics |" << endl;
    cout << "--------------------------------------" << endl << endl;
    
    cout << "Inverse kinematics of a linkage can also easily be obtained." << endl << endl;
    
    cout << "For example, let's set the joint values of the " << rightArm->name() << " to the following:" << endl;
    
    TRANSFORM B0;
    VectorXd q0(6), q1(6);
    q0 <<
    -.1,
    -.2,
    -.3,
    -.4,
    -.5,
    -.6;
    
    rightArm->values(q0);
    
    cout << rightArm->values() << endl << endl;
    
    cout << "Then the " << rightArm->tool().name() << " with respect to the linkage has a pose of " << endl;
    
    B0 = rightArm->tool().respectToLinkage();
    
    cout << B0.matrix() << endl << endl;
    
    cout << "This HG transformation can be given to the IK to get the following joint values:" << endl;
    
    hubo.rightArmAnalyticalIK(q1, B0, q0);
    
    cout << q1 << endl << endl;
    
    
    Linkage* rightLeg = &hubo.linkage("RIGHT_LEG");
    cout << "Let's now show the IK for the " << rightLeg->name() << ". Let's set the joint values to the following:" << endl;

    q0 <<
     .11,
     .22,
     .13,
     .24,
     .15,
     .16;
    
    rightLeg->values(q0);
    
    cout << rightLeg->values() << endl << endl;
    
    cout << "Then the " << rightLeg->tool().name() << " with respect to the linkage has a pose of " << endl;
    
    B0 = rightLeg->tool().respectToLinkage();
    
    cout << B0.matrix() << endl << endl;
    
    cout << "This HG transformation can be given to the IK to get the following joint values:" << endl;
    
    hubo.rightLegAnalyticalIK(q1, B0, q0);
    
    cout << q1 << endl << endl;

    
    cout << "------------------" << endl;
    cout << "|  Still to come |" << endl;
    cout << "------------------" << endl << endl;
    
    cout << "Things that still need to be added or fixed are the following:" << endl;
    cout << "-- Numerical Inverse Kinematics" << endl;
    cout << "-- Documentation has to be written" << endl;
    cout << "-- Add methods to add and remove joints" << endl;
    */


    Robot parseTest("../urdf/huboplus.urdf");
    parseTest.linkage("Body_RSP").name("RightArm");
    parseTest.linkage("Body_LSP").name("LeftArm");
    parseTest.linkage("Body_RHY").name("RightLeg");
    parseTest.linkage("Body_LHY").name("LeftLeg");




    VectorXd jointValues(parseTest.linkage("LeftLeg").nJoints());

    jointValues <<  1.79315, 0.490851, -1.26039, 1.38938, 0.503333, 0.0365566, -2.97178;

    for(int i=0; i<parseTest.linkage("LeftLeg").nJoints(); i++)
        parseTest.linkage("LeftLeg").setJointValue(i, jointValues[i]);

    cout << "Start:" << endl << parseTest.linkage("LeftLeg").tool().respectToRobot().matrix() << endl;

    TRANSFORM target = parseTest.linkage("LeftLeg").tool().respectToRobot();
//    target.rotate(AngleAxisd(10*M_PI/180, Vector3d::UnitY()));
//    target.translate(Vector3d(0.02, 0, 0));

    jointValues << 0, 0, 0, 0, 0, 0, 0;

    cout << "Goal:" << endl << target.matrix() << endl;

    rk_result_t result = parseTest.dampedLeastSquaresIK_linkage("LeftLeg", jointValues, target);

    cout << "End:" << endl << parseTest.linkage("LeftLeg").tool().respectToRobot().matrix() << endl;

    cout << "Result: " << rk_result_to_string(result) << endl;

//    parseTest.joint("RSR").value(-90*M_PI/180);
//    parseTest.linkage("RightArm").printInfo();


    parseTest.linkage("RightArm").tool().respectToFixed(parseTest.joint("RWR_dummy").respectToFixed());
    parseTest.linkage("RightArm").printInfo();


//    TRANSFORM target = parseTest.linkage("LeftLeg").tool().respectToRobot();
//    target.translate(Vector3d(0.1, 0, 0.1));
//    target.rotate(AngleAxisd(10*M_PI/180, Vector3d::UnitZ()));


//    cout << "Start:" << endl;
//    cout << parseTest.linkage("LeftLeg").tool().respectToRobot().matrix() << endl;

//    parseTest.dampedLeastSquaresIK_linkage("LeftLeg", jointValues, target);

//    cout << "Target:" << endl;
//    cout << target.matrix() << endl;

//    cout << "End:" << endl;
//    cout << parseTest.linkage("LeftLeg").tool().respectToRobot().matrix() << endl;

//    cout << "Joint Angles: " << jointValues.transpose() << endl;

//    cout << jointValues[2] + jointValues[3] + jointValues[4] << endl;

//    cout << "Error: " << (parseTest.linkage("LeftLeg").tool().respectToRobot().translation()
//                          - target.translation()).norm() << endl;








//    parseTest.linkage("LeftLeg").printInfo();

//    parseTest.printInfo();

//    VectorXd torques;


//    clock_t time;
//    time = clock();

//    int count = 100000;
//    for(int i=0; i<count; i++)
//        parseTest.linkage("Body_RSP").gravityJointTorques(torques, false);

//    clock_t endTime;
//    endTime = clock();
//    cout << (endTime - time)/((double)CLOCKS_PER_SEC/count/2+1) << " : " <<
//            (double)CLOCKS_PER_SEC*count*2/(endTime-time) << endl;




//    cout << "Mass:" << parseTest.mass() << endl;

//    cout << "Center of Mass (home config): " << parseTest.linkage("Body_RSP").centerOfMass().transpose() << endl;

//    parseTest.linkage("Body_RSP").gravityJointTorques(torques);
//    cout << "RA torques: " << torques.transpose() << endl;

//    parseTest.setJointValue("RSP", -45*M_PI/180);
//    cout << "CoM (pitched 45): " << parseTest.linkage("Body_RSP").centerOfMass().transpose() << endl;
//    parseTest.linkage("Body_RSP").gravityJointTorques(torques);
//    cout << "RA torques: " << torques.transpose() << endl;

//    parseTest.setJointValue("RSP", -90*M_PI/180);
//    cout << "CoM (pitched 90): " << parseTest.linkage("Body_RSP").centerOfMass().transpose() << endl;
//    parseTest.linkage("Body_RSP").gravityJointTorques(torques);
//    cout << "RA torques: " << torques.transpose() << endl;

//    parseTest.setJointValue("RSR", -90*M_PI/180);
//    cout << "CoM (pointed right): " << parseTest.linkage("Body_RSP").centerOfMass().transpose() << endl;
//    parseTest.linkage("Body_RSP").gravityJointTorques(torques);
//    cout << "RA torques: " << torques.transpose() << endl;

//    parseTest.setJointValue("RSR", -140*M_PI/180);
//    cout << "CoM (pointed back a bit): " << parseTest.linkage("Body_RSP").centerOfMass().transpose() << endl;
//    parseTest.linkage("Body_RSP").gravityJointTorques(torques);
//    cout << "RA torques: " << torques.transpose() << endl;




//    parseTest.setJointValue("", M_PI/8);

//    parseTest.setJointValue("RKP", M_PI/2);
//    parseTest.setJointValue("LKP", M_PI/2);

//    cout << "Center of Mass (bent knees): " << parseTest.centerOfMass().transpose() << endl;

//    parseTest.setJointValue("LHP", -M_PI/2);
//    parseTest.setJointValue("RHP", -M_PI/2);

//    cout << "Center of Mass (sitting config): " << parseTest.centerOfMass().transpose() << endl;

//    parseTest.setJointValue("RKP", 0);
//    parseTest.setJointValue("LKP", 0);

//    cout << "Center of Mass (legs forward): " << parseTest.centerOfMass().transpose() << endl;

//    parseTest.setJointValue("LSR", M_PI/2);

//    cout << "Center of Mass (left shoulder rolled): " << parseTest.centerOfMass().transpose() << endl;


//    parseTest.setJointValue("RSR", -M_PI/2);

//    cout << "Center of Mass (both shoulders rolled): " << parseTest.centerOfMass().transpose() << endl;
//    parseTest.setJointValue("LEP", -M_PI/2);
//    parseTest.setJointValue("REP", -M_PI/2);

//    cout << "Center of Mass (bend elbows up): " << parseTest.centerOfMass().transpose() << endl;

//    cout << "RSP Torque: " << parseTest.joint("RSP").gravityTorque() << endl;

//    parseTest.printInfo();

//    parseTest.linkage("Body_RSP").printInfo();

//    cout << "print info on joint Body_LHY" << endl;
//    parseTest.linkage("Body_LHY").printInfo();







    // ________________________ IK Speed Test Area __________________________



//    TRANSFORM target(TRANSFORM::Identity());
//    target.translate(Vector3d(0.1, -0.1, 0));
//    target.rotate(AngleAxisd(M_PI/4, Vector3d::UnitZ()));
//    target.rotate(AngleAxisd(M_PI/4, Vector3d::UnitY()));



//    parseTest.linkage("Body_RSP").name("RightArm");


//    VectorXd jointVals;
//    jointVals.resize(parseTest.linkage("RightArm").nJoints());
//    jointVals.setZero();


//    TRANSFORM start = parseTest.linkage("RightArm").tool().respectToRobot();
//    cout << "Start:" << endl << start.matrix() << endl;

//    cout << "Target:" << endl << target.matrix() << endl;


//    clock_t time;
//    time = clock();

//    int count = 1000;
//    for(int i=0; i<count; i++)
//    {
//        parseTest.dampedLeastSquaresIK_linkage("RightArm", jointVals, target);
//        parseTest.dampedLeastSquaresIK_linkage("RightArm", jointVals, start);
//    }

//    parseTest.dampedLeastSquaresIK_linkage("RightArm", jointVals, target);


//    clock_t endTime;
//    endTime = clock();
//    cout << (endTime - time)/((double)CLOCKS_PER_SEC/count/2+1) << " : " <<
//            (double)CLOCKS_PER_SEC*count*2/(endTime-time) << endl;

//    cout << "End:" << endl << parseTest.linkage("RightArm").tool().respectToRobot().matrix() << endl;



}



