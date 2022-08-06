#include <ros.h>
#include <stdlib.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
#include <math.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>

#include <DynamixelWorkbench.h>

#define DEVICE_NAME ""
#define DEBUG_SERIAL  SerialBT2

#define BAUDRATE  1000000
#define DXL_ID1    1
#define DXL_ID2    2
#define DXL_ID3    3

double ack_max = 20.2; //max Ackermann angle in degrees
#define wheelbase 0.14 //Wheelbas in meters
double ax3_ctr = 2900; //axis3 center position
double ax3_ll = 3400; //axis3 left limit position
double ax3_lr = 2400; //axis3 right limit position
#define WHEEL_SEPARATION 0.16 //rear wheel separation in meters

double imp_rad; //motor encoder pulse / radian
double ax3_ang_rad; //calculated Ackermann angle in radian
double ack_max_rad; //maximum Ackermann angle in radian
double ax3_dif; //pulse deviation from center
double ax3_abs_pos; //relative motor position from center

double lin_vel = 0;  //vehicle real linear velocity in m/s
double ang_vel = 0;  //vehicle real angular in rad/s

double lin_vel1;  //axis1 real velocity in m/s
double lin_vel2;  //axis2 real velocity in m/s

double lin_vel1_param;  //axis1 real velocity in mm/s
double lin_vel2_param;  //axis2 real velocity in mm/s

bool ax3_protect;   //Axis3 physical protection state

DynamixelWorkbench dxl_wb;
ros::NodeHandle nh;
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel) {

  if (Serial.available()) {
    Serial.read();
  }

  lin_vel = cmd_vel.linear.x;
  ang_vel = cmd_vel.angular.z;
  Serial.println(lin_vel);
  Serial.println(ang_vel);
}

//************************************************************************
//*************************** S e t u p **********************************
//************************************************************************

void setup()
{

  int c = 0;  //working cycle variable
  ax3_protect = false;    //Variable of phisycal protect of axis3

  Serial.begin(57600);
  //while(!Serial); // Wait for Opening Serial Monitor

  const char *log;
  bool result = false;

  uint8_t dxl_id1 = DXL_ID1;  //Left rear axis
  uint8_t dxl_id2 = DXL_ID2;  //Right rear axis
  uint8_t dxl_id3 = DXL_ID3;  //Steering axis
  uint16_t model_number = 0;

  result = dxl_wb.init(DEVICE_NAME, BAUDRATE, &log);    //Axes setup
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to init");
  }
  else
  {
    Serial.print("Succeeded to init : ");
    Serial.println(BAUDRATE);
  }
  for (int i = 1; i <= 3; i++)
  {
    result = dxl_wb.ping(i, &model_number, &log);       //Axes connection test
    if (result == false)
    {
      Serial.println(log);
      Serial.println("Failed to ping");
    }
    else
    {
      Serial.println("Succeeded to ping");
      Serial.print("id : ");
      Serial.print(i);
      Serial.print(" model_number : ");
      Serial.println(model_number);
    }
  }
  for (int i = 1; i <= 2; i++)
  {
    result = dxl_wb.wheelMode(i, 0, &log);              //Rear mode axes setup as wheel
    if (result == false)
    {
      Serial.println(log);
      Serial.println("Failed to change wheel mode");
    }
  }
  result = dxl_wb.jointMode(dxl_id3, 0, 0, &log);       //Steering axis setup as positioning axis
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to change joint mode");
  }

  ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);    //ROS subscription command
  nh.subscribe(cmd_vel_sub);


  DEBUG_SERIAL.begin(57600);
  /******************************************************************************************/
  /**************************************Ackermann*******************************************/
  /******************************************************************************************/

  do {
    //Test

      //lin_vel = 0.1;
      //ang_vel = 0.05;

    /**************ROS Subscriber node************************/

    void commandVelocityCallback(const geometry_msgs::Twist & cmd_vel);

    /*********************************************************/

    if (ang_vel == 0) {                                                             //Straight drive check and control

      lin_vel1 = lin_vel;
      lin_vel2 = lin_vel;
      ax3_abs_pos = ax3_ctr;

      lin_vel1_param = lin_vel1 * 1000;
      lin_vel2_param = lin_vel2 * 1000;

      dxl_wb.goalPosition(dxl_id3, (int32_t)ax3_abs_pos);
      dxl_wb.goalVelocity(dxl_id1, (int32_t)lin_vel1_param);
      dxl_wb.goalVelocity(dxl_id2, (int32_t)lin_vel2_param);

    } else {                                                                         //Angular drive control

      lin_vel1 = ((lin_vel / ang_vel) - (WHEEL_SEPARATION / 2)) * ang_vel;        //Left axis velocity count
      lin_vel2 = ((lin_vel / ang_vel) + (WHEEL_SEPARATION / 2)) * ang_vel;        //Right axis velocity count

      ack_max_rad = (ack_max * PI) / 180;                                         //Maximum ackermann angle in radian
      ax3_ang_rad = atan(wheelbase / (lin_vel / ang_vel));                        //Recommended ackermann angle
      ax3_ang_rad = ax3_ang_rad * -1;

      if (ang_vel < 0) {                                                          //Count number of impulses per radian

        imp_rad = (ax3_ctr - ax3_lr) / ack_max_rad;
        imp_rad = abs(imp_rad);

      } else {

        imp_rad = ((ax3_ctr - ax3_ll) / ack_max_rad);
        imp_rad = abs(imp_rad);

      }

      ax3_dif = ax3_ang_rad * imp_rad;                                            //Convert angle to impulses

      /****************steering motor incrementation way check*********************************/
      /*
          if (lin_vel > 0){                     //forward   +
            if (ang_vel < 0){                   //left      -
              if (ax3_ctr < ax3_ll){

                  ax3_dif = ax3_dif * -1;

              }
            }
            if (ang_vel > 0){                   //right     +
              if (ax3_ctr > ax3_lr){

                  ax3_dif = ax3_dif * -1;
              }
            }
          }
          if (lin_vel < 0){                     //reverse   -
            if (ang_vel < 0){                   //left      -
              if (ax3_ctr > ax3_ll){

                  ax3_dif = ax3_dif * -1;

              }
            }
            if (ang_vel > 0){                   //right     +
              if (ax3_ctr < ax3_lr){

                  ax3_dif = ax3_dif * -1;

              }
            }
          }
      */
      ax3_abs_pos = ax3_dif + ax3_ctr;                              //Absolut encoder position of axis3

      lin_vel1_param = lin_vel1 * 1000;                             //Convert to mm/s
      lin_vel2_param = lin_vel2 * 1000;                             //Convert to mm/s

      /*************************Physical protection of steering axis*****************/

      ax3_ang_rad = abs(ax3_ang_rad);
      if (ax3_ang_rad > ack_max_rad) {                                    //Comparsion of actual and maximum axis3 angle
        if (ax3_protect == false) {
          for (int i = 0; i <= 1; i++) {                                  //Overturning signal
            dxl_wb.goalPosition(dxl_id3, (int32_t)ax3_ll);
            delay(500);
            dxl_wb.goalPosition(dxl_id3, (int32_t)ax3_lr);
            delay(500);
          }
          dxl_wb.goalPosition(dxl_id3, (int32_t)ax3_ctr);                 //Safety movement blocking
          dxl_wb.goalVelocity(dxl_id1, (int32_t)0);
          dxl_wb.goalVelocity(dxl_id2, (int32_t)0);
          ax3_protect = true;
        }
      } else {
        ax3_protect = false;                                              //Axes parameter setting
        dxl_wb.goalPosition(dxl_id3, (int32_t)ax3_abs_pos);
        dxl_wb.goalVelocity(dxl_id1, (int32_t)lin_vel1_param);
        dxl_wb.goalVelocity(dxl_id2, (int32_t)lin_vel2_param);
      }
    }

  } while (c == 0);

  /******************************Ackermann end*************************************/

}

void loop(uint8_t dxl_id1, uint8_t dxl_id2, uint8_t dxl_id3) {


}
