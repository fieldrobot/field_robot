double pidBalanceControl(double set_angle, double actual_angle){ 
      angle_error = set_angle - actual_angle;  //Vorzeichen!

      angle_error_cumulated += (angle_error * dt_millisec)/100; //(angle_error/dt_microsec);
      angle_error_cumulated = constrain(angle_error_cumulated, -maximum_angle_error_cumulated, maximum_angle_error_cumulated);

      balance_control_result = kp*angle_error + ki*angle_error_cumulated + kd*adjusted_angular_speed;     

      //just for testing balance_control_result = constrain(balance_control_result, -maximum_speed, maximum_speed); // Limit max output from control

      balance_control_result_old = balance_control_result;
      angle_error_old = angle_error;
      
      return balance_control_result;
}                             

  // Differenz (throttle (Sollgeschwindigkeit, set_speed) - estimated_speed_filtered (actual_speed))
  // gibt den Sollwinkel zurueck
float speedControl(double actual_speed, double set_speed)
{
 
  speed_error = set_speed - actual_speed;
  /*Serial.print("actual_speed:           "); Serial.print(actual_speed);
  Serial.print("speed_error:           "); Serial.print(speed_error);*/

  /*speed_error_cumulated += constrain((speed_error * dt_millisec * 0.00002), -(0.004 * dt_millisec), (0.004 * dt_millisec)); //10000000
  speed_error_cumulated = constrain(speed_error_cumulated, -(maximum_speed_error_cumulated), (maximum_speed_error_cumulated));*/

  //speed_error_cumulated += constrain((speed_error * dt_millisec * 0.00002), (maximum_speed_error_cumulated), (maximum_speed_error_cumulated));

  speed_control -= (kp_throttle * speed_error);// + -(ki_throttle * speed_error_cumulated);
  speed_control = constrain(speed_control, -maximum_speed_control, maximum_speed_control);

  return speed_control;
}

/*
 * https://github.com/LuSeKa/HoverBot/blob/master/HoverBot.ino
 * 
 *   // IMU sampling
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

 * // balance controller
  float balanceControllerOutput = euler.z() * kp_b_BALANCE + gyro.x() * kd_b_BALANCE;

  // planar controllera (lateral position and steering angle)
  float positionControllerOutput = kp_b_POSITION * (pwmDutyCycle_throttle - PWM_CENTER);
  float steeringControllerOutput = kp_b_STEERING * (pwmDutyCycle_steering - PWM_CENTER) + gyro.z() * kd_b_ORIENTATION;  

  float controllerOutput_right = balanceControllerOutput + positionControllerOutput + steeringControllerOutput;
  float controllerOutput_left  = balanceControllerOutput + positionControllerOutput - steeringControllerOutput;
 */
