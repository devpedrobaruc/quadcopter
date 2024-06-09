#include "FlightController/FlightController.h"

void FlightController::si_translate_bytes(void)
{
  while (Serial.available())
  {
    if (si_received_bytes_counter >= HC12_R_BUFFER_SIZE)
    {
      si_received_bytes_counter = HC12_R_BUFFER_SIZE - 1;
    }

    si_received_bytes[si_received_bytes_counter] = Serial.read();
    si_received_bytes_counter++;
    si_last_input_change = millis();
  }

  if (millis() - si_last_input_change > 50 && si_print_flag == 0)
  {
    si_received_bytes_counter = 0;
    si_check_byte = 0x00;
    for (count_var = 0; count_var <= 10; count_var++)
      si_check_byte ^= si_received_bytes[count_var];

    if (si_check_byte == si_received_bytes[11])
    {
      if (si_received_bytes[0] == 'M' && si_received_bytes[1] == 'S')
      {
        if (si_received_bytes[2] == 2)
        {
          throttle = motor_idle_speed;                  // Set the base throttle to the motor_idle_speed variable.
          angle_pitch = angle_pitch_acc;                // Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
          angle_roll = angle_roll_acc;                  // Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.
          ground_pressure = actual_pressure;            // Register the pressure at ground level for altitude calculations.
          course_lock_heading = angle_yaw;              // Set the current compass heading as the course lock heading.
          acc_total_vector_at_start = acc_total_vector; // Register the acceleration when the quadcopter is started.
          lat_gps_home = l_lat_gps;
          lon_gps_home = l_lon_gps;
          home_point_recorded = 1;
          start = 2;              // Set the start variable to 2 to indicate that the quadcopter is started.
          acc_alt_integrated = 0; // Reset the integrated acceleration value.
          if (manual_takeoff_throttle > 1400 && manual_takeoff_throttle < 1600)
          {                                                    // If the manual hover throttle is used and valid (between 1400us and 1600us pulse).
            takeoff_throttle = manual_takeoff_throttle - 1500; // Use the manual hover throttle.
            takeoff_detected = 1;                              // Set the auto take-off detection to 1, indicated that the quadcopter is flying.
            // Reset the PID controllers for a smooth take-off.
            pid_i_mem_roll = 0;
            pid_last_roll_d_error = 0;
            pid_output_roll = 0;
            pid_i_mem_pitch = 0;
            pid_last_pitch_d_error = 0;
            pid_output_pitch = 0;
            pid_i_mem_yaw = 0;
            pid_last_yaw_d_error = 0;
            pid_output_yaw = 0;
          }
          else if (manual_takeoff_throttle)
          {                       // If the manual hover throttle value is invalid.
            error = 5;            // Error = 5.
            takeoff_throttle = 0; // No hover throttle compensation.
            start = 0;            // Set the start variable to 0 to stop the motors.
          }

          if (number_used_sats < 6)
          {
            start = 0;            // Set the start variable to 0 to disable the motors.
            takeoff_detected = 0; // Reset the auto take-off detection.
          }
        }
        else if (si_received_bytes[2] == 0)
        {
          start = 0;            // Set the start variable to 0 to disable the motors.
          takeoff_detected = 0; // Reset the auto take-off detection.
        }
        else
        {
          start = si_received_bytes[2];
        }
      }

      if (si_received_bytes[0] == 'W' && si_received_bytes[1] == 'P')
      {
        new_waypoint_available = 1;
        si_received_bytes[0] = 0x00;
        wp_lat_gps = (int32_t)si_received_bytes[2] | (int32_t)si_received_bytes[3] << 8 | (int32_t)si_received_bytes[4] << 16 | (int32_t)si_received_bytes[5] << 24;
        wp_lon_gps = (int32_t)si_received_bytes[6] | (int32_t)si_received_bytes[7] << 8 | (int32_t)si_received_bytes[8] << 16 | (int32_t)si_received_bytes[9] << 24;
        if (waypoint_set == 1 && home_point_recorded == 1 && flight_mode == 3)
        {
          fly_to_new_waypoint = 1;
          fly_to_new_waypoint_step = 0;
          fly_to_waypoint_lat_factor = 0;
          fly_to_waypoint_lon_factor = 0;
        }
      }
    }
    si_print_flag = 1;
  }

  if (millis() - si_last_input_change < 4 && si_print_flag == 1)
  {
    si_print_flag = 0;
  }
}