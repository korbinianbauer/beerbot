#include "Streaming.h"
#include "sw_pid_controller.cpp"
#include "sbus.h"

#define OUT_INTERVAL_MS 100

#define SBUS_CH_THROTTLE 1
#define SBUS_CH_STEER 0


void setup() {

  //delay(1000);

  attach_encoders();

  attach_motors();

  pinMode(13, OUTPUT);


  Serial.begin(115200);


  PidController SpeedErrorControllerLeft;
  PidController SpeedErrorControllerRight;

  /* SBUS */
  bfs::SbusRx sbus_rx(&Serial2);
  bfs::SbusTx sbus_tx(&Serial2);
  bfs::SbusData data;

  sbus_rx.Begin();
  sbus_tx.Begin();

  int throttle = 0;
  int steer = 0;

  bool armed = false;
  Serial.print("Waiting for arming...");
  while (!armed) {
    sbus_rx.Read();
    data = sbus_rx.data();
    throttle = map(data.ch[SBUS_CH_THROTTLE], 173, 1811, -1000, 1000);
    steer = map(data.ch[SBUS_CH_STEER], 173, 1811, -1000, 1000);
    //Serial << throttle << " " << steer << "\n";
    if ((throttle == 0) and (steer == 0)) {
      armed = true;
      Serial.println("Armed!");
    }
  }
  //delay(100);



  // Reset SpeedController Integral Parts
  SpeedErrorControllerLeft.I_out = 0;
  SpeedErrorControllerRight.I_out = 0;

  SpeedErrorControllerLeft.setpoint = 0;
  SpeedErrorControllerLeft.Kp = 0.4;
  SpeedErrorControllerLeft.Ki = 0.001;
  SpeedErrorControllerLeft.Kd = 0;
  SpeedErrorControllerLeft.I_limit = 0.5;

  SpeedErrorControllerRight.setpoint = 0;
  SpeedErrorControllerRight.Kp = 0.4;
  SpeedErrorControllerRight.Ki = 0.001;
  SpeedErrorControllerRight.Kd = 0;
  SpeedErrorControllerRight.I_limit = 0.5;

  float max_speed = 2.0; // m/s

  float left_wheel_speed_target = 0;
  float right_wheel_speed_target = 0;

  float left_wheel_speed_output = 0;
  float right_wheel_speed_output = 0;

  long last_valid_cmd_input = millis();
  long last_out = millis();
  long last_loop = millis();

  while (true) {


    update_encoders();
    sbus_rx.Read();

    data = sbus_rx.data();

    throttle = map(data.ch[SBUS_CH_THROTTLE], 173, 1811, -1000*max_speed, 1000*max_speed);
    steer = map(data.ch[SBUS_CH_STEER], 173, 1811, -1000*max_speed, 1000*max_speed);

    if (abs(throttle) < 2) {
      throttle = 0;
    }
    if (abs(steer) < 2) {
      steer = 0;
    }

    //Serial.print(throttle / 1000.0);
    //Serial.print("\t");
    //Serial.println(steer / 1000.0);

    //Serial << "sbus_throttle: " << data.ch[SBUS_CH_THROTTLE] << ", sbus_steer: " << data.ch[SBUS_CH_STEER] << "\n";

    float scale = constrain(1000*max_speed/(abs(steer)+abs(throttle)), 0.0, 1.0);

    left_wheel_speed_target = (throttle + steer) / 1000.0 * scale;
    right_wheel_speed_target = (throttle - steer) / 1000.0 * scale;

    


    float left_wheel_speed_actual = get_left_wheel_speed();
    float right_wheel_speed_actual = get_right_wheel_speed();

    float left_wheel_speed_correction = 0;
    float right_wheel_speed_correction = 0;

    SpeedErrorControllerLeft.setpoint = left_wheel_speed_target;
    SpeedErrorControllerRight.setpoint = right_wheel_speed_target;

    left_wheel_speed_correction = SpeedErrorControllerLeft.evaluate(left_wheel_speed_actual);
    right_wheel_speed_correction = SpeedErrorControllerRight.evaluate(right_wheel_speed_actual);

    float left_correction_limit = abs(0.5 * left_wheel_speed_output) + 0.01;
    float right_correction_limit = abs(0.5 * right_wheel_speed_output) + 0.01;

    left_wheel_speed_correction = constrain(left_wheel_speed_correction, -left_correction_limit, left_correction_limit);
    right_wheel_speed_correction = constrain(right_wheel_speed_correction, -right_correction_limit, right_correction_limit);

    left_wheel_speed_output = left_wheel_speed_target + left_wheel_speed_correction;
    right_wheel_speed_output = right_wheel_speed_target + right_wheel_speed_correction;

    left_wheel_speed_output = constrain(left_wheel_speed_output, -1000*max_speed, 1000*max_speed);
    right_wheel_speed_output = constrain(right_wheel_speed_output, -1000*max_speed, 1000*max_speed);



    digitalWrite(13, LOW);

    //left_motor_set_rpm(10);

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


    long curr_time = millis();
    long loop_dt = curr_time - last_loop;
    last_loop = curr_time;


  }

}


void loop() {


}
