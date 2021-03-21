/*pseudo code for the balancing process:
- sensors aquisition, smoothing and zeroing
- Acc angle and Gyro rate calculation and scaling
- Acc and Gyro data fusing though Kalman algorithm
- PWM calculation using a PID control algorithm
- PWM feeding to DC motors 
*/

#include <ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>
#include <Wire.h>
#include "Kalman.h"
#include "defs.h"

ros::NodeHandle nh;

void targetAngleCallback(const std_msgs::Float64& msg) {
  speed_control_result = msg.data;
}

/*void targetSteerCallback(const std_msgs::Float64& msg) {
  steer_command = msg.data;
}*/

//std_msgs::Float64 angleValue;
sensor_msgs::Imu imuValues;
//std_msgs::Float64 loopRate;
ros::Subscriber<std_msgs::Float64> angleSub("targetAngle", &targetAngleCallback);
//ros::Subscriber<std_msgs::Float64> steerSub("targetSteer", &targetSteerCallback);
//ros::Publisher anglePub("angle", &angleValue);
ros::Publisher imuPub("imuUnadjusted", &imuValues);
//ros::Publisher loopRatePub("loopRate", &loopRate);

Kalman kalmanX; // Create the Kalman instance

//one time setup
void setup() {

  nh.initNode();
  nh.subscribe(angleSub);
  //nh.subscribe(steerSub);
  //nh.advertise(anglePub);
  nh.advertise(imuPub);
  //nh.advertise(loopRatePub);
  
  //Serial.begin(SERIAL_BAUD); //initialize serial communication
  Wire.begin(); // initialize i2c

  i2c_imu_init();
  speed_control_result = 0;
  steer_command = 0;

  hover_serial_init();
  //Serial.println();

}

void loop() {
  //loop duration measurement
  current_time_microsec = micros();
  dt_microsec = current_time_microsec - old_time_microsec;
  old_time_microsec = current_time_microsec;

  current_time_millisec = millis();
  dt_millisec = current_time_millisec - old_time_millisec;
  old_time_millisec = current_time_millisec;
  //loopRate.data = dt_millisec;
  //loopRatePub.publish(&loopRate);

  //handle new angular data if available
  adjusted_angle_old = adjusted_angle;
  getImuAngle();
  //angleValue.data = adjusted_angle;
  //anglePub.publish(&angleValue);


  /*  // Testschalter nach rechts
  if (test_item == 1){
    // bandStopFilter gibt den gefilterten imuValue zurueck
  float vor_filter = imuValue; 
  getEmaA();
  bandStopFilter(); 
  float nach_filter = imuValue;

  }*/
  
  //estimated_speed = speed_command; // Positive: forward
  //estimated_speed_filtered = estimated_speed_filtered * 0.9 + (float)estimated_speed * 0.1; // low pass filter on estimated speed

  //measured_angular_speed = adjusted_angle - adjusted_angle_old;
  
  // piSpeedControl  
  //speed_control_result = speedControl(estimated_speed_filtered, throttle);

  // pidBalanceControl
  speed_command = (int)pidBalanceControl(speed_control_result, adjusted_angle);
  //Serial.print(adjusted_angle);
     
  // communication with the hoverboard
  //receive feedback form hoverboard (debugging, ...)
  Receive();
  // Send commands
  Send(steer_command, speed_command);


  // Blink the build in LED to confirm programm execution
  digitalWrite(LED_BUILTIN, ((int)current_time_millisec%2000)<1000);

  //ENDING
  nh.spinOnce();
  //Serial.println(); //debug: print every iteration's prints in a new line

}
