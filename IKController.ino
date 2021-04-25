/***********************************************************************************
 *  }--\     InterbotiX Robotic Arm            /--{
 *      |       Playback Code                 |
 *   __/           Minimal                    \__
 *  |__|                                       |__|
 *
 *
 *  The following sketch will playback a sequence one time automatically once
 *  the arm starts up.
 *
 *=============================================================================
 * Based upon Kurt's PX Reactor arm code.
 * https://github.com/KurtE
 * This code provides serial control of the Interbotix line of robotic arms, which are sold by Trossen Robotics:
 * http://www.trossenrobotics.com/robotic-arms.aspx
 * http://learn.trossenrobotics.com/interbotix/robot-arms
 *=============================================================================
 * 
 *   This code is a Work In Progress and is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
 *   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
 ****************************************************************************************************************/

//=============================================================================
// Define Options
//=============================================================================

#define PINCHER 1
#define REACTOR 2
#define WIDOWX 3

//uncomment one of the following lines depending on which arm you want to use
//#define ARMTYPE PINCHER
#define ARMTYPE REACTOR
//#define ARMTYPE WIDOWX

#if !defined(ARMTYPE) 
   #error YOU HAVE TO SELECT THE ARM YOU ARE USING! Uncomment the correct line above for your arm
#endif

#define MAX_SERVO_DELTA_PERSEC 512
//#define DEBUG             // Enable Debug mode via serial

//=============================================================================
// Global Include files
//=============================================================================
//DYNAMIXEL Control libraries
#include <ax12.h>
#include <BioloidController.h>


//input control file - local
#include "Kinematics.h"
#include <ros.h>

#include<std_msgs/Int16.h>
#include<std_msgs/Bool.h>

//=============================================================================
// Global Objects
//=============================================================================
BioloidController bioloid = BioloidController(1000000);

//===================================================================================================
// Setup 
//====================================================================================================

int x;
int y;
int z;
boolean lock = false;

ros::NodeHandle node_handle;

std_msgs::Int16 x_input;
std_msgs::Int16 y_input;
std_msgs::Int16 z_input;

void xCallback(const std_msgs::Int16& x_input) {
  if ((x + x_input.data) >= -100 && (x + x_input.data) <= 100) {
    IKEngine(x_input.data, 0, 0);
  }
}

void yCallback(const std_msgs::Int16& y_input) {
  if ((y_input.data + z) >= 20) {
    IKEngine(0, 0, y_input.data);
  }
}

void zCallback(const std_msgs::Int16& z_input) {
  if ((y + z_input.data) <= 200 && (z >= 200)) {
    IKEngine(0, z_input.data, 0);
  }
  else if ((y + z_input.data) >= 200 && (y + z_input.data) <= 250 && (z >= 100)) {
    IKEngine(0, z_input.data, 0);
  }
  else if ((y + z_input.data) >= 250 && (y + z_input.data <= 300) && (z <= 350) && z >= 20) {
    IKEngine(0, z_input.data, 0);
  }
  else if ((y + z_input.data) >= 300 && (y + z_input.data <= 400) && z >= 20 && z <= 250) {
    IKEngine(0, z_input.data, 0);
  }
}

void lockCallback(const std_msgs::Bool& lock_input) {
  lock = lock_input.data;
}

void resetCallback(const std_msgs::Bool& reset) {
  if (reset.data) {
    x = 0;
    y = 50;
    z = 300;
    IKEngine(0, 0, 0); 
  }
}


ros::Subscriber<std_msgs::Int16> x_subscriber("arm_control/x", &xCallback);
ros::Subscriber<std_msgs::Int16> y_subscriber("arm_control/y", &yCallback);
ros::Subscriber<std_msgs::Int16> z_subscriber("arm_control/z", &zCallback);
ros::Subscriber<std_msgs::Bool> lock_subscriber("arm_control/stop", &lockCallback);
ros::Subscriber<std_msgs::Bool> reset_subscriber("arm_control/reset", &resetCallback);

void setup() {

  //Serial.begin(9600);
  // Next initialize the Bioloid
  bioloid.poseSize = CNT_SERVOS;

  // Read in the current positions...
  bioloid.readPose();
  delay(100);
  
  // Start off to put arm to sleep...
  //PutArmToSleep();
  
  //set Gripper Compliance so it doesn't tear itself apart
  ax12SetRegister(SID_GRIP, AX_CW_COMPLIANCE_SLOPE, 128);
  ax12SetRegister(SID_GRIP, AX_CCW_COMPLIANCE_SLOPE, 128);


  boolean fChanged = false;
  
  g_bIKMode = IKM_IK3D_CARTESIAN;

  
  x = 0;
  y = 50;
  z = 300;
  IKSequencingControl(x, y, z, 0 , 512 , 0 , 2000 , 1000, 1);

  //X moves left and right
  //Z is up and down
  //Y is forward and back

  delay (500);   
  //Serial.println("###########################");    
  //Serial.println("Serial Communication Established.");    
  
  dxlVoltageReport(7);  //serial report for the system voltage  
  dxlServoReport(7);    //Scan Servos, return position and error (if there are any)


   node_handle.initNode();
   node_handle.subscribe(x_subscriber);
   node_handle.subscribe(y_subscriber);
   node_handle.subscribe(z_subscriber);
   node_handle.subscribe(lock_subscriber);
   node_handle.subscribe(reset_subscriber);
  
}//end setup




//===================================================================================================
// loop: Our main Loop!
//===================================================================================================
void loop() 
{
  node_handle.spinOnce();
  delay(100);
  
  /*
  if (Serial.available()) {
     
    char inChar = Serial.read();
    
    Serial.println(inChar);
    switch (inChar) {
      case 'm':
        IKEngine(x, y, z);
        break;
    }
   }
   */
} //end Main


void IKEngine(int xOffset, int yOffset, int zOffset) {
  //Serial.println("###########################");
  //Serial.println("Running Vector Command");
  //Serial.println("###########################");
  delay (500);

  if (!lock) {
    x = x + xOffset;
    y = y + yOffset;
    z = z + zOffset;
    IKSequencingControl(x, y, z, 0 , 512 , 0 , 2000 , 1000, 1);
  }
}



//===================================================================================================
// functions
//===================================================================================================


void IKSequencingControl(float X, float Y, float Z, float GA, float WR, int grip, int interpolate, int pause, int enable)
{
  if(enable == 1)
  {

    if(g_bIKMode == IKM_IK3D_CARTESIAN || g_bIKMode == IKM_IK3D_CARTESIAN_90)
    {
      doArmIK(true, X, Y, Z, GA); 
      
    }
    else if(g_bIKMode == IKM_CYLINDRICAL || g_bIKMode ==IKM_CYLINDRICAL_90)
    {  
    //  sBase = X;
      doArmIK(false, X, Y, Z, GA); 
      
    }
    else if(g_bIKMode == IKM_BACKHOE)
    {
      sBase = X;
      sShoulder = Y;
      sElbow = Z;
      sWrist = GA;
      
    }
    
    sWristRot = WR;
    sGrip = grip;
  
    MoveArmTo(sBase, sShoulder, sElbow, sWrist, sWristRot, sGrip, interpolate, true);  
    delay(pause);    
    
  }
}


int readInput() {
  
  char input = Serial.read();
  while (input == ' ') {
    input = Serial.read();
  }

  int value = 0;
  int sign = 1;
  while(input != ' ') {
    if (input >= '0' && input <= '9') {
      value = (value * 10) + (input - '0');
    }
    if (input == '-') {
      sign = -1;
    }
    input = Serial.read();
  }

  Serial.println(value * sign);

  return value * sign;
}
