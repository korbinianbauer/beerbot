#include "Streaming.h"
#include "sw_pid_controller.cpp"

#define OUT_INTERVAL_MS 100

PidController SpeedErrorControllerLeft;
PidController SpeedErrorControllerRight;




void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);
  attach_motors();
  attach_encoders();
  attach_IMU();
  attach_nunchuk();
  pinMode(13, OUTPUT);

  //calib_mag_and_print_result(); // Remove writing the calb values to IMU in IMU attach() before doing this

  // Flush Nunchuk buffer or something
  get_nunchuk_x();
  get_nunchuk_y();

  // Reset SpeedController Integral Parts

  SpeedErrorControllerLeft.I_out = 0;
  SpeedErrorControllerRight.I_out = 0;


  SpeedErrorControllerLeft.setpoint = 0;
  SpeedErrorControllerLeft.Kp = 0.4;
  SpeedErrorControllerLeft.Ki = 0.001;
  SpeedErrorControllerLeft.Kd = 0;
  SpeedErrorControllerLeft.I_limit = 0.2;

  SpeedErrorControllerRight.setpoint = 0;
  SpeedErrorControllerRight.Kp = 0.4;
  SpeedErrorControllerRight.Ki = 0.001;
  SpeedErrorControllerRight.Kd = 0;
  SpeedErrorControllerRight.I_limit = 0.2;

  boolean joystick_mode = false;
  float max_speed = 1;

  float left_wheel_speed_target = 0;
  float right_wheel_speed_target = 0;

  String serial_cmds = "";

  long last_valid_cmd_input = millis();
  long last_out = millis();
  long last_loop = millis();

  unsigned long acc_accumu_counter = 0;
  float acc_x_accumu = 0;
  float acc_y_accumu = 0;
  float acc_z_accumu = 0;

  while (true) {

    bool data_requested = false;

    if (Serial.available() > 0) {
      serial_cmds = Serial.readStringUntil('\n');
      //Serial.println(serial_cmds);


      // check if valid joystick mode activate String received
      if (String("JOYSTICK") == serial_cmds.substring(0, 8)) {
        joystick_mode = true;
      }


      // check if valid data request String received
      if (String("REQ") == serial_cmds.substring(0, 3)) {
        data_requested = true;
      }



      // check if cmd string received via Serial is valid
      bool is_valid = true;

      if (String("CMDIN") != serial_cmds.substring(0, 5)) {
        is_valid = false;
        //Serial.println("CMDIN prefix missing");
      }
      if (String("END") != serial_cmds.substring(18, 21)) {
        is_valid = false;
        //Serial.println("END suffix missing");
      }

      float v_links = serial_cmds.substring(6, 11).toInt() / 1000.0;
      float v_rechts = serial_cmds.substring(12, 17).toInt() / 1000.0;

      //Serial << "v_links: " << v_links << " v_rechts: " << v_rechts << "\n";

      if (is_valid) {
        joystick_mode = false;
        left_wheel_speed_target = v_links;
        right_wheel_speed_target = v_rechts;
        last_valid_cmd_input = millis();
      }

      serial_cmds = "";

    }

    if ((millis() - last_valid_cmd_input) > 1000) { // STOP if no valid input for more than 1 second
      left_wheel_speed_target = 0;
      right_wheel_speed_target = 0;
    }

    update_encoders();
    update_IMU();

    if (joystick_mode) {
      float x = map(get_nunchuk_x(), 26, 229, -100, 100);
      float y = map(get_nunchuk_y() - 4, 35, 225, -100, 100);


      if (abs(x) < 20) {
        x = 0;
      } else if (x < 0) {
        x += 20;
      } else {
        x -= 20;
      }
      if (abs(y) < 3) {
        y = 0;
      }


      x = pow(x, 3) / 10000;
      y = pow(y, 3) / 10000;

      left_wheel_speed_target = (-y + x) * max_speed / 100;
      right_wheel_speed_target = (-y - x) * max_speed / 100;
    }

    acc_x_accumu += get_IMU_Acc_X();
    acc_y_accumu += get_IMU_Acc_Y();
    acc_z_accumu += get_IMU_Acc_Z();
    acc_accumu_counter++;

    float left_wheel_start_pos = get_left_wheel_distance();
    float right_wheel_start_pos = get_right_wheel_distance();

    float left_wheel_actual_pos = get_left_wheel_distance();
    float right_wheel_actual_pos = get_right_wheel_distance();

    float left_wheel_speed_actual = get_left_wheel_speed();
    float right_wheel_speed_actual = get_right_wheel_speed();

    float left_wheel_speed_error = left_wheel_speed_actual - left_wheel_speed_target;
    float right_wheel_speed_error = right_wheel_speed_actual - right_wheel_speed_target;

    float left_wheel_speed_correction = SpeedErrorControllerLeft.evaluate(left_wheel_speed_error);
    float right_wheel_speed_correction = SpeedErrorControllerRight.evaluate(right_wheel_speed_error);

    float left_wheel_speed_output = left_wheel_speed_target + left_wheel_speed_correction;
    float right_wheel_speed_output = right_wheel_speed_target + right_wheel_speed_correction;



    digitalWrite(13, LOW);

    if (left_wheel_speed_target == 0) {
      left_wheel_set_speed(0);
    } else {
      left_wheel_set_speed(left_wheel_speed_output);
      digitalWrite(13, HIGH);
    }

    if (right_wheel_speed_target == 0) {
      right_wheel_set_speed(0);
    } else {
      right_wheel_set_speed(right_wheel_speed_output);
      digitalWrite(13, HIGH);
    }


    //Serial << "Target Position: [L" << left_wheel_target_pos << " R" << right_wheel_target_pos << "]";
    //Serial << " Actual Position: [L" << left_wheel_actual_pos << " R" << right_wheel_actual_pos << "]";
    //Serial << " pos_error_max_magnitude: " << pos_error_max_magnitude;
    //Serial << " Rel. Left Pos Error: " << pos_error_left_relative << " Rel. Right Pos Error: " << pos_error_right_relative;
    //Serial << " left_wheel_speed_output: " << left_wheel_speed_output << " right_wheel_speed_output: " << right_wheel_speed_output;
    //Serial << "\n";

    long left_wheel_actual_pos_int = left_wheel_actual_pos * 1000;
    long right_wheel_actual_pos_int = right_wheel_actual_pos * 1000;

    long curr_time = millis();
    long loop_dt = curr_time - last_loop;
    last_loop = curr_time;
    //if (curr_time - last_out > OUT_INTERVAL_MS) {
    if (data_requested) {

      Serial << "{"; // Start python dictionary string

      Serial << "'Time': " << millis(); // Time entry in milliseconds
      Serial << ", 'Left_wheel_dist': " << left_wheel_actual_pos_int;
      Serial << ", 'Right_wheel_dist': " << right_wheel_actual_pos_int;
      Serial << ", 'IMU_roll': " << get_IMU_roll_radians();
      Serial << ", 'IMU_pitch': " << get_IMU_pitch_radians();
      Serial << ", 'IMU_yaw': " << get_IMU_yaw_radians();
      Serial << ", 'IMU_Acc_x': " << acc_x_accumu / acc_accumu_counter;
      Serial << ", 'IMU_Acc_y': " << acc_y_accumu / acc_accumu_counter;
      Serial << ", 'IMU_Acc_z': " << acc_z_accumu / acc_accumu_counter;
      Serial << ", 'Loop_dt': " << loop_dt;
      Serial << ", 'Acc_Accu': " << acc_accumu_counter;

      Serial << "}\n"; // Close python dictionary string

      acc_x_accumu = 0;
      acc_y_accumu = 0;
      acc_z_accumu = 0;
      acc_accumu_counter = 0;

      /*Serial << "TIM_OUT," << millis() / 1000.0 << ",END\n";
        Serial << "POS_OUT," << left_wheel_actual_pos_int << "," << right_wheel_actual_pos_int << ",END\n";
        Serial << "DIR_OUT," << get_IMU_roll_radians() << "," << get_IMU_pitch_radians() << "," << get_IMU_yaw_radians() << ",END\n";
        Serial << "ACC_OUT," << get_IMU_Acc_X() << "," << get_IMU_Acc_Y() << "," << get_IMU_Acc_Z() << ",END\n";*/
      last_out += OUT_INTERVAL_MS;
      data_requested = false;
    }




    //delay(20);

  }

}


void loop() {


}
