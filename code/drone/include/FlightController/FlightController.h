#ifndef __FLIGHT_CONTROLLER__
#define __FLIGHT_CONTROLLER__

#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <timer.h>
#include <stm32f1xx_hal_gpio.h>
#include <stm32/LL/stm32yyxx_ll_gpio.h>
#include <string.h>

#define STM32_board_LED PC13
#define variable_1_to_adjust dummy_float
#define variable_2_to_adjust dummy_float
#define variable_3_to_adjust dummy_float

#define I2C_SDA PB11
#define I2C_CLK PB10

class FlightController
{
private:
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // PID gain and limit settings
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    float pid_p_gain_roll = 1.3;  // Gain setting for the pitch and roll P-controller (default = 1.3).
    float pid_i_gain_roll = 0.04; // Gain setting for the pitch and roll I-controller (default = 0.04).
    float pid_d_gain_roll = 18.0; // Gain setting for the pitch and roll D-controller (default = 18.0).
    int pid_max_roll = 400;       // Maximum output of the PID-controller (+/-).

    float pid_p_gain_pitch = pid_p_gain_roll; // Gain setting for the pitch P-controller.
    float pid_i_gain_pitch = pid_i_gain_roll; // Gain setting for the pitch I-controller.
    float pid_d_gain_pitch = pid_d_gain_roll; // Gain setting for the pitch D-controller.
    int pid_max_pitch = pid_max_roll;         // Maximum output of the PID-controller (+/-).

    float pid_p_gain_yaw = 4.0;  // Gain setting for the pitch P-controller (default = 4.0).
    float pid_i_gain_yaw = 0.02; // Gain setting for the pitch I-controller (default = 0.02).
    float pid_d_gain_yaw = 0.0;  // Gain setting for the pitch D-controller (default = 0.0).
    int pid_max_yaw = 400;       // Maximum output of the PID-controller (+/-).

    // During flight the battery voltage drops and the motors are spinning at a lower RPM. This has a negative effecct on the
    // altitude hold function. With the battery_compensation variable it's possible to compensate for the battery voltage drop.
    // Increase this value when the quadcopter drops due to a lower battery voltage during a non altitude hold flight.
    float battery_compensation = 40.0;

    float pid_p_gain_altitude = 1.4;  // Gain setting for the altitude P-controller (default = 1.4).
    float pid_i_gain_altitude = 0.2;  // Gain setting for the altitude I-controller (default = 0.2).
    float pid_d_gain_altitude = 0.75; // Gain setting for the altitude D-controller (default = 0.75).
    int pid_max_altitude = 400;       // Maximum output of the PID-controller (+/-).

    float gps_p_gain = 2.7; // Gain setting for the GPS P-controller (default = 2.7).
    float gps_d_gain = 6.5; // Gain setting for the GPS D-controller (default = 6.5).

    float declination = 0.0; // Set the declination between the magnetic and geographic north.

    int16_t manual_takeoff_throttle = 0; // Enter the manual hover point when auto take-off detection is not desired (between 1400 and 1600).
    int16_t motor_idle_speed = 1100;     // Enter the minimum throttle pulse of the motors when they idle (between 1000 and 1200). 1170 for DJI

    uint8_t gyro_address = 0x68;    // The I2C address of the MPU-6050 is 0x68 in hexadecimal form.
    uint8_t MS5611_address = 0x77;  // The I2C address of the MS5611 barometer is 0x77 in hexadecimal form.
    uint8_t compass_address = 0x1E; // The I2C address of the HMC5883L is 0x1E in hexadecimal form.

    float battery_voltage_calibration = 0.0; // Battery voltage offset calibration.
    float low_battery_warning = 10.5;        // Set the battery warning at 10.5V (default = 10.5V).

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Declaring global variables
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // int16_t = signed 16 bit integer
    // uint16_t = unsigned 16 bit integer

    uint8_t last_channel_1, last_channel_2, last_channel_3, last_channel_4;
    uint8_t check_byte, flip32, start;
    uint8_t error, error_counter, error_led;
    uint8_t flight_mode, flight_mode_counter, flight_mode_led;
    uint8_t takeoff_detected, manual_altitude_change;
    uint8_t telemetry_send_byte, telemetry_bit_counter, telemetry_loop_counter;
    uint8_t channel_select_counter;
    uint8_t level_calibration_on;

    uint32_t telemetry_buffer_byte;

    int16_t esc_1, esc_2, esc_3, esc_4;
    int16_t manual_throttle;
    int16_t throttle, takeoff_throttle, cal_int;
    int16_t temperature, count_var;
    int16_t acc_x, acc_y, acc_z;
    int16_t gyro_pitch, gyro_roll, gyro_yaw;

    int32_t channel_1_start, channel_1, channel_1_base, pid_roll_setpoint_base;
    int32_t channel_2_start, channel_2, channel_2_base, pid_pitch_setpoint_base;
    int32_t channel_3_start, channel_3;
    int32_t channel_4_start, channel_4;
    int32_t channel_5_start, channel_5;
    int32_t channel_6_start, channel_6;
    int32_t measured_time, measured_time_start, receiver_watchdog;
    int32_t acc_total_vector, acc_total_vector_at_start;
    int32_t gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
    int16_t acc_pitch_cal_value;
    int16_t acc_roll_cal_value;

    int32_t acc_z_average_short_total, acc_z_average_long_total, acc_z_average_total;
    int16_t acc_z_average_short[26], acc_z_average_long[51];

    uint8_t acc_z_average_short_rotating_mem_location, acc_z_average_long_rotating_mem_location;

    int32_t acc_alt_integrated;

    uint32_t loop_timer, error_timer, flight_mode_timer;
    uint32_t delay_micros_timer;

    float roll_level_adjust, pitch_level_adjust;
    float pid_error_temp;
    float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
    float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
    float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
    float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll, angle_yaw;
    float battery_voltage, dummy_float;

    // Compass variables
    uint8_t compass_calibration_on, heading_lock;
    int16_t compass_x, compass_y, compass_z;
    int16_t compass_cal_values[6];
    float compass_x_horizontal, compass_y_horizontal, actual_compass_heading;
    float compass_scale_y, compass_scale_z;
    int16_t compass_offset_x, compass_offset_y, compass_offset_z;
    float course_a, course_b, course_c, base_course_mirrored, actual_course_mirrored;
    float course_lock_heading, heading_lock_course_deviation;

    // Pressure variables.
    float pid_error_gain_altitude, pid_throttle_gain_altitude;
    uint16_t C[7];
    uint8_t barometer_counter, temperature_counter, average_temperature_mem_location;
    int64_t OFF, OFF_C2, SENS, SENS_C1, P;
    uint32_t raw_pressure, raw_temperature, temp, raw_temperature_rotating_memory[6], raw_average_temperature_total;
    float actual_pressure, actual_pressure_slow, actual_pressure_fast, actual_pressure_diff;
    float ground_pressure, altutude_hold_pressure, return_to_home_decrease;
    int32_t dT, dT_C5;
    // Altitude PID variables
    float pid_i_mem_altitude, pid_altitude_setpoint, pid_altitude_input, pid_output_altitude, pid_last_altitude_d_error;
    uint8_t parachute_rotating_mem_location;
    int32_t parachute_buffer[35], parachute_throttle;
    float pressure_parachute_previous;
    int32_t pressure_rotating_mem[50], pressure_total_avarage;
    uint8_t pressure_rotating_mem_location;
    float pressure_rotating_mem_actual;

    // GPS variables
    uint8_t read_serial_byte, incomming_message[100], number_used_sats, fix_type;
    uint8_t waypoint_set, latitude_north, longiude_east;
    uint16_t message_counter;
    int16_t gps_add_counter;
    int32_t l_lat_gps, l_lon_gps, lat_gps_previous, lon_gps_previous;
    int32_t lat_gps_actual, lon_gps_actual, l_lat_waypoint, l_lon_waypoint;
    float gps_pitch_adjust_north, gps_pitch_adjust, gps_roll_adjust_north, gps_roll_adjust;
    float lat_gps_loop_add, lon_gps_loop_add, lat_gps_add, lon_gps_add;
    uint8_t new_line_found, new_gps_data_available, new_gps_data_counter;
    uint8_t gps_rotating_mem_location, return_to_home_step;
    int32_t gps_lat_total_avarage, gps_lon_total_avarage;
    int32_t gps_lat_rotating_mem[40], gps_lon_rotating_mem[40];
    int32_t gps_lat_error, gps_lon_error;
    int32_t gps_lat_error_previous, gps_lon_error_previous;
    uint32_t gps_watchdog_timer;

    float l_lon_gps_float_adjust, l_lat_gps_float_adjust, gps_man_adjust_heading;
    float return_to_home_lat_factor, return_to_home_lon_factor, return_to_home_move_factor;
    uint8_t home_point_recorded;
    int32_t lat_gps_home, lon_gps_home;

    // Software Serial data input handling
    uint8_t si_check_byte;
    // uint8_t temp_byte;
    int8_t si_rising_edge_set;
    int16_t si_time_array[200];
    int8_t si_print_flag = 1;
    uint8_t si_time_array_counter, si_time_array_counter_2, si_received_bytes_counter;
    uint8_t si_received_bytes[30], si_level, si_byte_counter, new_waypoint_available;
    int32_t wp_lat_gps, wp_lon_gps;
    int32_t si_measured_time, si_measured_time_start, si_last_input_change, si_last_input_change_previous;

    // Fly waypoints
    uint8_t fly_to_new_waypoint, fly_to_new_waypoint_step, waypoint_monitor;
    float fly_to_waypoint_move_factor, fly_to_waypoint_lat_factor, fly_to_waypoint_lon_factor;

    // Adjust settings online
    uint32_t setting_adjust_timer;
    uint16_t setting_click_counter;
    uint8_t previous_channel_6;
    float adjustable_setting_1, adjustable_setting_2, adjustable_setting_3;

    TwoWire HWire;
    HardwareTimer *Timer3;

    void read_barometer(void);
    void calculate_pid(void);
    void calibrate_compass(void);
    void calibrate_level(void);
    void change_settings(void);
    void fly_waypoints(void);
    void gyro_setup(void);
    void calibrate_gyro(void);
    void gyro_signalen(void);
    void handler_channel_1(void);
    void red_led(int8_t level);
    void green_led(int8_t level);
    void error_signal(void);
    void flight_mode_signal(void);
    void read_compass();
    void setup_compass();
    float course_deviation(float course_b, float course_c);
    void gps_setup(void);
    void read_gps(void);
    void return_to_home(void);
    void send_telemetry_data(void);
    void si_translate_bytes(void);
    void start_stop_takeoff(void);
    void timer_setup(void);
    void vertical_acceleration_calculations(void);

public:
    void begin();
    void process();
    void Serial_input_handler(void);
};

#endif