#include "FlightController/FlightController.h"

void FlightController::begin()
{
  pinMode(4, INPUT_ANALOG);                           // This is needed for reading the analog value of port A4.
                                                      // Port PB3 and PB4 are used as JTDO and JNTRST by default.
                                                      // The following function connects PB3 and PB4 to the
                                                      // alternate output function.
                                                      // afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY); // Connects PB3 and PB4 to output function.
  __IO uint32_t *mapr = &AFIO->MAPR;                  // CONFIGURE afio_cfg_debug_ports TEST
  *mapr = (*mapr & ~AFIO_MAPR_SWJ_CFG) | (0x2 << 24); // CONFIGURE afio_cfg_debug_ports TEST

  pinMode(PB3, OUTPUT);                // Set PB3 as output for green LED.
  pinMode(PB4, OUTPUT);                // Set PB4 as output for red LED.
  pinMode(STM32_board_LED, OUTPUT);    // This is the LED on the STM32 board. Used for GPS indication.
  digitalWrite(STM32_board_LED, HIGH); // Turn the LED on the STM32 off. The LED function is inverted. Check the STM32 schematic.

  Serial.begin(9600);

  green_led(LOW); // Set output PB3 low.
  red_led(HIGH);  // Set output PB4 high.

  // EEPROM emulation setup
  // EEPROM.PageBase0 = 0x801F000;
  // EEPROM.PageBase1 = 0x801F800;
  // EEPROM.PageSize = 0x400;

  // Serial.begin(57600);                                        //Set the serial output to 57600 kbps. (for debugging only)
  // delay(250);                                                 //Give the serial port some time to start to prevent data loss.

  timer_setup(); // Setup the timers for the receiver inputs and ESC's output.
  delay(50);     // Give the timers some time to start.

  gps_setup(); // Set the baud rate and output refreshrate of the GPS module.

  // Check if the MPU-6050 is responding.
  HWire.begin((uint32_t)I2C_SDA, (uint32_t)I2C_CLK); // Start the I2C as master
  HWire.beginTransmission(gyro_address);             // Start communication with the MPU-6050.
  error = HWire.endTransmission();                   // End the transmission and register the exit status.
  while (error != 0)
  {                 // Stay in this loop because the MPU-6050 did not responde.
    error = 1;      // Set the error status to 1.
    error_signal(); // Show the error via the red LED.
    delay(4);       // Simulate a 250Hz refresch rate as like the main loop.
  }

  // Check if the compass is responding.
  HWire.beginTransmission(compass_address); // Start communication with the HMC5883L.
  error = HWire.endTransmission();          // End the transmission and register the exit status.
  while (error != 0)
  {                 // Stay in this loop because the HMC5883L did not responde.
    error = 2;      // Set the error status to 2.
    error_signal(); // Show the error via the red LED.
    delay(4);       // Simulate a 250Hz refresch rate as like the main loop.
  }

  // Check if the MS5611 barometer is responding.
  HWire.beginTransmission(MS5611_address); // Start communication with the MS5611.
  error = HWire.endTransmission();         // End the transmission and register the exit status.
  while (error != 0)
  {                 // Stay in this loop because the MS5611 did not responde.
    error = 3;      // Set the error status to 2.
    error_signal(); // Show the error via the red LED.
    delay(4);       // Simulate a 250Hz refresch rate as like the main loop.
  }

  gyro_setup();                       // Initiallize the gyro and set the correct registers.
  setup_compass();                    // Initiallize the compass and set the correct registers.
  read_compass();                     // Read and calculate the compass data.
  angle_yaw = actual_compass_heading; // Set the initial compass heading.

  // Create a 5 second delay before calibration.
  for (count_var = 0; count_var < 1250; count_var++)
  { // 1250 loops of 4 microseconds = 5 seconds.
    if (count_var % 125 == 0)
    {                                       // Every 125 loops (500ms).
      digitalWrite(PB4, !digitalRead(PB4)); // Change the led status.
    }
    delay(4); // Simulate a 250Hz refresch rate as like the main loop.
  }
  count_var = 0;    // Set start back to 0.
  calibrate_gyro(); // Calibrate the gyro offset.

  // Wait until the receiver is active.
  while (channel_1 < 990 || channel_2 < 990 || channel_3 < 990 || channel_4 < 990)
  {
    error = 4;      // Set the error status to 4.
    error_signal(); // Show the error via the red LED.
    delay(4);       // Delay 4ms to simulate a 250Hz loop
  }
  error = 0; // Reset the error status to 0.

  // When everything is done, turn off the led.
  red_led(LOW); // Set output PB4 low.

  // Load the battery voltage to the battery_voltage variable.
  // The STM32 uses a 12 bit analog to digital converter.
  // analogRead => 0 = 0V ..... 4095 = 3.3V
  // The voltage divider (1k & 10k) is 1:11.
  // analogRead => 0 = 0V ..... 4095 = 36.3V
  // 36.3 / 4095 = 112.81.
  // The variable battery_voltage holds 1050 if the battery voltage is 10.5V.
  battery_voltage = (float)analogRead(4) / 112.81;

  // For calculating the pressure the 6 calibration values need to be polled from the MS5611.
  // These 2 byte values are stored in the memory location 0xA2 and up.
  for (start = 1; start <= 6; start++)
  {
    HWire.beginTransmission(MS5611_address); // Start communication with the MPU-6050.
    HWire.write(0xA0 + start * 2);           // Send the address that we want to read.
    HWire.endTransmission();                 // End the transmission.

    HWire.requestFrom(MS5611_address, 2);        // Request 2 bytes from the MS5611.
    C[start] = HWire.read() << 8 | HWire.read(); // Add the low and high byte to the C[x] calibration variable.
  }

  OFF_C2 = C[2] * pow(2, 16);  // This value is pre-calculated to offload the main program loop.
  SENS_C1 = C[1] * pow(2, 15); // This value is pre-calculated to offload the main program loop.

  // The MS5611 needs a few readings to stabilize.
  for (start = 0; start < 100; start++)
  {                   // This loop runs 100 times.
    read_barometer(); // Read and calculate the barometer data.
    delay(4);         // The main program loop also runs 250Hz (4ms per loop).
  }
  actual_pressure = 0; // Reset the pressure calculations.

  // Before starting the avarage accelerometer value is preloaded into the variables.
  for (start = 0; start <= 24; start++)
    acc_z_average_short[start] = acc_z;
  for (start = 0; start <= 49; start++)
    acc_z_average_long[start] = acc_z;
  acc_z_average_short_total = acc_z * 25;
  acc_z_average_long_total = acc_z * 50;
  start = 0;

  if (motor_idle_speed < 1000)
    motor_idle_speed = 1000; // Limit the minimum idle motor speed to 1000us.
  if (motor_idle_speed > 1200)
    motor_idle_speed = 1200; // Limit the maximum idle motor speed to 1200us.

  loop_timer = micros(); // Set the timer for the first loop.
}

void FlightController::process()
{
  if (receiver_watchdog < 750)
    receiver_watchdog++;
  if (receiver_watchdog == 750 && start == 2)
  {
    channel_1 = 1500;
    channel_2 = 1500;
    channel_3 = 1500;
    channel_4 = 1500;
    error = 8;
    if (number_used_sats > 5)
    {
      if (home_point_recorded == 1)
        channel_5 = 2000;
      else
        channel_5 = 1750;
    }
    else
      channel_5 = 1500;
  }
  // Some functions are only accessible when the quadcopter is off.
  if (start == 0)
  {
    // For compass calibration move both sticks to the top right.
    if (channel_1 > 1900 && channel_2 < 1100 && channel_3 > 1900 && channel_4 > 1900)
      calibrate_compass();
    // Level calibration move both sticks to the top left.
    if (channel_1 < 1100 && channel_2 < 1100 && channel_3 > 1900 && channel_4 < 1100)
      calibrate_level();
    // Change settings
    if (channel_6 >= 1900 && previous_channel_6 == 0)
    {
      previous_channel_6 = 1;
      if (setting_adjust_timer > millis())
        setting_click_counter++;
      else
        setting_click_counter = 0;
      setting_adjust_timer = millis() + 1000;
      if (setting_click_counter > 3)
      {
        setting_click_counter = 0;
        change_settings();
      }
    }
    if (channel_6 < 1900)
      previous_channel_6 = 0;
  }

  heading_lock = 0;
  if (channel_6 > 1200)
    heading_lock = 1; // If channel 6 is between 1200us and 1600us the flight mode is 2

  flight_mode = 1; // In all other situations the flight mode is 1;
  if (channel_5 >= 1200 && channel_5 < 1600)
    flight_mode = 2; // If channel 6 is between 1200us and 1600us the flight mode is 2
  if (channel_5 >= 1600 && channel_5 < 1950)
    flight_mode = 3; // If channel 6 is between 1600us and 1900us the flight mode is 3
  if (channel_5 >= 1950 && channel_5 < 2100)
  {
    if (waypoint_set == 1 && home_point_recorded == 1 && start == 2)
      flight_mode = 4;
    else
      flight_mode = 3;
  }
  if (flight_mode != 4)
  {
    return_to_home_step = 0;
    return_to_home_lat_factor = 0;
    return_to_home_lon_factor = 0;
  }

  // Run some subroutines
  fly_waypoints();      // Jump to the fly waypoint step program.
  return_to_home();     // Jump to the return to home step program.
  flight_mode_signal(); // Show the flight_mode via the green LED.
  error_signal();       // Show the error via the red LED.
  gyro_signalen();      // Read the gyro and accelerometer data.
  read_barometer();     // Read and calculate the barometer data.
  read_compass();       // Read and calculate the compass data.
  si_translate_bytes();

  if (gps_add_counter >= 0)
    gps_add_counter--;

  read_gps();

  // 65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
  gyro_roll_input = (gyro_roll_input * 0.7) + (((float)gyro_roll / 65.5) * 0.3);    // Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.7) + (((float)gyro_pitch / 65.5) * 0.3); // Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.7) + (((float)gyro_yaw / 65.5) * 0.3);       // Gyro pid input is deg/sec.

  ////////////////////////////////////////////////////////////////////////////////////////////////////
  // This is the added IMU code from the videos:
  // https://youtu.be/4BoIE8YQwM8
  // https://youtu.be/j-kE0AMEWy4
  ////////////////////////////////////////////////////////////////////////////////////////////////////

  // Gyro angle calculations
  // 0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += (float)gyro_pitch * 0.0000611; // Calculate the traveled pitch angle and add this to the angle_pitch variable.
  angle_roll += (float)gyro_roll * 0.0000611;   // Calculate the traveled roll angle and add this to the angle_roll variable.
  angle_yaw += (float)gyro_yaw * 0.0000611;     // Calculate the traveled yaw angle and add this to the angle_yaw variable.
  if (angle_yaw < 0)
    angle_yaw += 360; // If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
  else if (angle_yaw >= 360)
    angle_yaw -= 360; // If the compass heading becomes larger then 360, 360 is subtracted to keep it in the 0 till 360 degrees range.

  // 0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians and not degrees.
  angle_pitch -= angle_roll * sin((float)gyro_yaw * 0.000001066); // If the IMU has yawed transfer the roll angle to the pitch angel.
  angle_roll += angle_pitch * sin((float)gyro_yaw * 0.000001066); // If the IMU has yawed transfer the pitch angle to the roll angel.

  angle_yaw -= course_deviation(angle_yaw, actual_compass_heading) / 1200.0; // Calculate the difference between the gyro and compass heading and make a small correction.
  if (angle_yaw < 0)
    angle_yaw += 360; // If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
  else if (angle_yaw >= 360)
    angle_yaw -= 360; // If the compass heading becomes larger then 360, 360 is subtracted to keep it in the 0 till 360 degrees range.

  // Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z)); // Calculate the total accelerometer vector.

  if (abs(acc_y) < acc_total_vector)
  {                                                                   // Prevent the asin function to produce a NaN.
    angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296; // Calculate the pitch angle.
  }
  if (abs(acc_x) < acc_total_vector)
  {                                                                  // Prevent the asin function to produce a NaN.
    angle_roll_acc = asin((float)acc_x / acc_total_vector) * 57.296; // Calculate the roll angle.
  }

  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004; // Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;    // Correct the drift of the gyro roll angle with the accelerometer roll angle.

  pitch_level_adjust = angle_pitch * 15; // Calculate the pitch angle correction.
  roll_level_adjust = angle_roll * 15;   // Calculate the roll angle correction.

  vertical_acceleration_calculations(); // Calculate the vertical accelration.

  channel_1_base = channel_1;         // Normally channel_1 is the pid_roll_setpoint input.
  channel_2_base = channel_2;         // Normally channel_2 is the pid_pitch_setpoint input.
  gps_man_adjust_heading = angle_yaw; //
  // When the heading_lock mode is activated the roll and pitch pid setpoints are heading dependent.
  // At startup the heading is registerd in the variable course_lock_heading.
  // First the course deviation is calculated between the current heading and the course_lock_heading.
  // Based on this deviation the pitch and roll controls are calculated so the responce is the same as on startup.
  if (heading_lock == 1)
  {
    heading_lock_course_deviation = course_deviation(angle_yaw, course_lock_heading);
    channel_1_base = 1500 + ((float)(channel_1 - 1500) * cos(heading_lock_course_deviation * 0.017453)) + ((float)(channel_2 - 1500) * cos((heading_lock_course_deviation - 90) * 0.017453));
    channel_2_base = 1500 + ((float)(channel_2 - 1500) * cos(heading_lock_course_deviation * 0.017453)) + ((float)(channel_1 - 1500) * cos((heading_lock_course_deviation + 90) * 0.017453));
    gps_man_adjust_heading = course_lock_heading;
  }
  if (flight_mode >= 3 && waypoint_set == 1)
  {
    pid_roll_setpoint_base = 1500 + gps_roll_adjust;
    pid_pitch_setpoint_base = 1500 + gps_pitch_adjust;
  }
  else
  {
    pid_roll_setpoint_base = channel_1_base;
    pid_pitch_setpoint_base = channel_2_base;
  }

  // Because we added the GPS adjust values we need to make sure that the control limits are not exceded.
  if (pid_roll_setpoint_base > 2000)
    pid_roll_setpoint_base = 2000;
  if (pid_roll_setpoint_base < 1000)
    pid_roll_setpoint_base = 1000;
  if (pid_pitch_setpoint_base > 2000)
    pid_pitch_setpoint_base = 2000;
  if (pid_pitch_setpoint_base < 1000)
    pid_pitch_setpoint_base = 1000;

  calculate_pid(); // Calculate the pid outputs based on the receiver inputs.

  start_stop_takeoff(); // Starting, stopping and take-off detection

  // The battery voltage is needed for compensation.
  // A complementary filter is used to reduce noise.
  battery_voltage = (battery_voltage * 0.92) + ((((float)analogRead(4) / 112.81) + battery_voltage_calibration) * 0.08);

  // Turn on the led if battery voltage is to low. Default setting is 10.5V
  if (battery_voltage > 6.0 && battery_voltage < low_battery_warning && error == 0)
    error = 1;

  // The variable base_throttle is calculated in the following part. It forms the base throttle for every motor.
  if (takeoff_detected == 1 && start == 2)
  {                                          // If the quadcopter is started and flying.
    throttle = channel_3 + takeoff_throttle; // The base throttle is the receiver throttle channel + the detected take-off throttle.
    if (flight_mode >= 2)
    {                                                                             // If altitude mode is active.
      throttle = 1500 + takeoff_throttle + pid_output_altitude + manual_throttle; // The base throttle is the receiver throttle channel + the detected take-off throttle + the PID controller output.
    }
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////
  // Creating the pulses for the ESC's is explained in this video:
  // https://youtu.be/Nju9rvZOjVQ
  ////////////////////////////////////////////////////////////////////////////////////////////////////

  if (start == 2)
  { // The motors are started.
    if (throttle > 1800)
      throttle = 1800;                                                      // We need some room to keep full control at full throttle.
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; // Calculate the pulse for esc 1 (front-right - CCW).
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; // Calculate the pulse for esc 2 (rear-right - CW).
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; // Calculate the pulse for esc 3 (rear-left - CCW).
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; // Calculate the pulse for esc 4 (front-left - CW).

    if (battery_voltage < 12.40 && battery_voltage > 6.0)
    {                                                            // Is the battery connected?
      esc_1 += (12.40 - battery_voltage) * battery_compensation; // Compensate the esc-1 pulse for voltage drop.
      esc_2 += (12.40 - battery_voltage) * battery_compensation; // Compensate the esc-2 pulse for voltage drop.
      esc_3 += (12.40 - battery_voltage) * battery_compensation; // Compensate the esc-3 pulse for voltage drop.
      esc_4 += (12.40 - battery_voltage) * battery_compensation; // Compensate the esc-4 pulse for voltage drop.
    }

    if (esc_1 < motor_idle_speed)
      esc_1 = motor_idle_speed; // Keep the motors running.
    if (esc_2 < motor_idle_speed)
      esc_2 = motor_idle_speed; // Keep the motors running.
    if (esc_3 < motor_idle_speed)
      esc_3 = motor_idle_speed; // Keep the motors running.
    if (esc_4 < motor_idle_speed)
      esc_4 = motor_idle_speed; // Keep the motors running.

    if (esc_1 > 2000)
      esc_1 = 2000; // Limit the esc-1 pulse to 2000us.
    if (esc_2 > 2000)
      esc_2 = 2000; // Limit the esc-2 pulse to 2000us.
    if (esc_3 > 2000)
      esc_3 = 2000; // Limit the esc-3 pulse to 2000us.
    if (esc_4 > 2000)
      esc_4 = 2000; // Limit the esc-4 pulse to 2000us.
  }

  else
  {
    esc_1 = 1000; // If start is not 2 keep a 1000us pulse for ess-1.
    esc_2 = 1000; // If start is not 2 keep a 1000us pulse for ess-2.
    esc_3 = 1000; // If start is not 2 keep a 1000us pulse for ess-3.
    esc_4 = 1000; // If start is not 2 keep a 1000us pulse for ess-4.
  }

  TIM4->CCR1 = esc_1; // Set the throttle receiver input pulse to the ESC 1 output pulse.
  TIM4->CCR2 = esc_2; // Set the throttle receiver input pulse to the ESC 2 output pulse.
  TIM4->CCR3 = esc_3; // Set the throttle receiver input pulse to the ESC 3 output pulse.
  TIM4->CCR4 = esc_4; // Set the throttle receiver input pulse to the ESC 4 output pulse.
  TIM4->CNT = 5000;   // This will reset timer 4 and the ESC pulses are directly created.

  send_telemetry_data(); // Send telemetry data to the ground station.

  //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
  // Because of the angle calculation the loop time is getting very important. If the loop time is
  // longer or shorter than 4000us the angle calculation is off. If you modify the code make sure
  // that the loop time is still 4000us and no longer! More information can be found on
  // the Q&A page:
  //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !

  if (micros() - loop_timer > 4050)
    error = 2; // Output an error if the loop time exceeds 4050us.
  while (micros() - loop_timer < 4000)
    ;                    // We wait until 4000us are passed.
  loop_timer = micros(); // Set the timer for the next loop.
}
