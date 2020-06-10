#include "Streaming.h"
#include "sw_pid_controller.cpp"

#define WAYPOINT_COUNT 3

PidController SpeedErrorControllerLeft;
PidController SpeedErrorControllerRight;




void setup() {
  Serial.begin(1000000);
  attach_nunchuk();
  attach_motors();
  attach_encoders();
  //attach_IMU();


  SpeedErrorControllerLeft.setpoint = 0;
  SpeedErrorControllerLeft.Kp = 0.4;
  SpeedErrorControllerLeft.Ki = 0.0035;
  SpeedErrorControllerLeft.Kd = 10;

  SpeedErrorControllerRight.setpoint = 0;
  SpeedErrorControllerRight.Kp = 0.4;
  SpeedErrorControllerRight.Ki = 0.0035;
  SpeedErrorControllerRight.Kd = 10;

  // Flush Nunchuk buffer or something
  get_nunchuk_x();
  get_nunchuk_y();


  // record
  Serial.println("Recording Waypoints");
  unsigned long last_loop = millis();
  float left_wheel_target_pos = 0;
  float right_wheel_target_pos = 0;
  float waypoint[WAYPOINT_COUNT][2];
  int waypoint_idx = 1;
  waypoint[0][0] = 0;
  waypoint[0][1] = 0;
  bool record_finished = false;
  while (not record_finished) {

    float x = map(get_nunchuk_x(), 26, 229, -100, 100);
    float y = map(get_nunchuk_y() - 4, 35, 225, -100, 100);

    Serial << x << " " << y << "\n";

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

    x = x / 2;


    float max_speed = 0.5;
    unsigned long dt = millis() - last_loop;

    left_wheel_target_pos += dt * (-y + x) * max_speed / 100000.0;
    right_wheel_target_pos += dt * (-y - x) * max_speed / 100000.0;

    update_encoders();

    float left_wheel_actual_pos = get_left_wheel_distance();
    float right_wheel_actual_pos = get_right_wheel_distance();

    float left_wheel_target_pos_error = left_wheel_target_pos - left_wheel_actual_pos;
    float right_wheel_target_pos_error = right_wheel_target_pos - right_wheel_actual_pos;

    float left_wheel_speed_target = constrain(2 * left_wheel_target_pos_error, -max_speed, max_speed);
    float right_wheel_speed_target = constrain(2 * right_wheel_target_pos_error, -max_speed, max_speed);

    float left_wheel_speed_actual = get_left_wheel_speed();
    float right_wheel_speed_actual = get_right_wheel_speed();

    float left_wheel_speed_error = left_wheel_speed_actual - left_wheel_speed_target;
    float right_wheel_speed_error = right_wheel_speed_actual - right_wheel_speed_target;

    float left_wheel_speed_correction = SpeedErrorControllerLeft.evaluate(left_wheel_speed_error);
    float right_wheel_speed_correction = SpeedErrorControllerRight.evaluate(right_wheel_speed_error);

    left_wheel_set_speed(left_wheel_speed_target + left_wheel_speed_correction);
    right_wheel_set_speed(right_wheel_speed_target + right_wheel_speed_correction);

    //    Serial.print(100 * left_wheel_speed_target);
    //    Serial.print(" ");
    //    Serial.print(100 * left_wheel_speed_actual);
    //    Serial.print(" ");
    //    Serial.print(100 * right_wheel_speed_target);
    //    Serial.print(" ");
    //    Serial.println(100 * right_wheel_speed_actual);

    Serial << " Actual Position: [L" << left_wheel_actual_pos << " R" << right_wheel_actual_pos << "]";
    Serial << "\n";


    if (get_nunchuk_button_c()) {
      while (get_nunchuk_button_c()) {
        ;//debouncing}
      }
      Serial.println("Saved waypoint");
      waypoint[waypoint_idx][0] = left_wheel_actual_pos;
      waypoint[waypoint_idx][1] = right_wheel_actual_pos;
      waypoint_idx++;

      if (waypoint_idx >= WAYPOINT_COUNT) {
        record_finished = true;
      }

    }
    last_loop = millis();
    delay(10);
  }

  // Print waypoints
  Serial.println("Recording finished. Waypoints:");
  for (int i = 0; i < WAYPOINT_COUNT; i++) {
    Serial.print(waypoint[i][0]);
    Serial.print(" ");
    Serial.println(waypoint[i][1]);
  }

  // Replay backwards
  Serial.println("Press Z Button on Nunchuk to drive back");

  while (not get_nunchuk_button_z()) {
    ;
  }
  while (get_nunchuk_button_z()) {
    ;
  }

  // Reset SpeedController Integral Parts

  SpeedErrorControllerLeft.I_out = 0;
  SpeedErrorControllerRight.I_out = 0;


  SpeedErrorControllerLeft.setpoint = 0;
  SpeedErrorControllerLeft.Kp = 0.4;
  SpeedErrorControllerLeft.Ki = 0.001;
  SpeedErrorControllerLeft.Kd = 0;

  SpeedErrorControllerRight.setpoint = 0;
  SpeedErrorControllerRight.Kp = 0.4;
  SpeedErrorControllerRight.Ki = 0.001;
  SpeedErrorControllerRight.Kd = 0;

  float delta_v_max = 1.0; // m/s^2



  for (int i = WAYPOINT_COUNT; i >= 1; i--) {
    Serial.print("Going to waypoint Nr. ");
    Serial.println(i);



    
    float left_wheel_target_pos = waypoint[i - 1][0];
    float right_wheel_target_pos = waypoint[i - 1][1];



    
    float max_speed = 0.1;
    float target_plan_delta = 0.1;
    float left_wheel_target_pos_error = 99999;
    float right_wheel_target_pos_error = 99999;
    unsigned long last_loop = millis();
    while ((abs(left_wheel_target_pos_error) > 0.01) or (abs(right_wheel_target_pos_error) > 0.01)) {

      unsigned long dt = millis() - last_loop;
      update_encoders();

      float left_wheel_actual_pos = get_left_wheel_distance();
      float right_wheel_actual_pos = get_right_wheel_distance();

      left_wheel_target_pos_error = left_wheel_target_pos - left_wheel_actual_pos;
      right_wheel_target_pos_error = right_wheel_target_pos - right_wheel_actual_pos;

      float pos_error_max_magnitude = max(abs(left_wheel_target_pos_error), abs(right_wheel_target_pos_error));

      float pos_error_left_relative;
      if (abs(left_wheel_target_pos_error) < 0.01) {
        pos_error_left_relative = 0;
      } else {
        pos_error_left_relative = left_wheel_target_pos_error / pos_error_max_magnitude;
      }
      float pos_error_right_relative;
      if (abs(right_wheel_target_pos_error) < 0.01) {
        pos_error_right_relative = 0;
      } else {
        pos_error_right_relative = right_wheel_target_pos_error / pos_error_max_magnitude;
      }

      float left_wheel_speed_target = pos_error_left_relative * max_speed;
      float right_wheel_speed_target = pos_error_right_relative * max_speed;

      float left_wheel_speed_actual = get_left_wheel_speed();
      float right_wheel_speed_actual = get_right_wheel_speed();

      //left_wheel_speed_target = constrain(left_wheel_speed_target, left_wheel_speed_actual - 0.001 * dt * delta_v_max, left_wheel_speed_actual + 0.001 * dt * delta_v_max);
      //right_wheel_speed_target = constrain(right_wheel_speed_target, right_wheel_speed_actual - 0.001 * dt * delta_v_max, right_wheel_speed_actual + 0.001 * dt * delta_v_max);

      float left_wheel_speed_error = left_wheel_speed_actual - left_wheel_speed_target;
      float right_wheel_speed_error = right_wheel_speed_actual - right_wheel_speed_target;

      float left_wheel_speed_correction = SpeedErrorControllerLeft.evaluate(left_wheel_speed_error);
      float right_wheel_speed_correction = SpeedErrorControllerRight.evaluate(right_wheel_speed_error);

      float left_wheel_speed_output = left_wheel_speed_target + left_wheel_speed_correction;
      float right_wheel_speed_output = right_wheel_speed_target + right_wheel_speed_correction;

      left_wheel_set_speed(left_wheel_speed_output);
      right_wheel_set_speed(right_wheel_speed_output);

      Serial << "Target Position: [L" << left_wheel_target_pos << " R" << right_wheel_target_pos << "]";
      Serial << " Actual Position: [L" << left_wheel_actual_pos << " R" << right_wheel_actual_pos << "]";
      Serial << " pos_error_max_magnitude: " << pos_error_max_magnitude;
      Serial << " Rel. Left Pos Error: " << pos_error_left_relative << " Rel. Right Pos Error: " << pos_error_right_relative;
      Serial << " left_wheel_speed_output: " << left_wheel_speed_output << " right_wheel_speed_output: " << right_wheel_speed_output;
      Serial << "\n";
      last_loop = millis();
      delay(10);
    }
    left_motor_set_power(0);
    right_motor_set_power(0);
    delay(1000);
  }


  

  left_motor_set_power(0);
  right_motor_set_power(0);


  delay(1000000L);




}


void loop() {


  delay(10);

}
