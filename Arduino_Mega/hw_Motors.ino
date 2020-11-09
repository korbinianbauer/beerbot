#include <PWM.h>

bool motors_attached = false;

class Motor {
  public:
    float armature_resistance = 0.3;
    float brushes_voltage_drop = 0.8;
    float motor_gear_ratio = 1 / 9.778;
    float kv = 155; // rpm/V

    float max_current = 5;
    float last_current_limit_volt = 25;
    
    int pin_fwd_en;
    int pin_bwd_en;
    int pin_fwd;
    int pin_bwd;

    int fwd_deadzone_low;
    int fwd_deadzone_high;
    int bwd_deadzone_low;
    int bwd_deadzone_high;

    float supply_voltage = 22.3;

    int max_power = 1000;


    void attach(int fwd_en, int bwd_en, int fwd, int bwd, int forward_deadzone_low, int forward_deadzone_high, int backward_deadzone_low, int backward_deadzone_high) {
      InitTimersSafe();
      pin_fwd_en = fwd_en;
      pin_bwd_en = bwd_en;
      pin_fwd = fwd;
      pin_bwd = bwd;
      fwd_deadzone_low = forward_deadzone_low;
      fwd_deadzone_high = forward_deadzone_high;
      bwd_deadzone_low = backward_deadzone_low;
      bwd_deadzone_high = backward_deadzone_high;

      set_power(0);

    }

    float volt_to_power(float volt) {
      if (volt > 0) {
        volt += brushes_voltage_drop;
      } else if (volt < 0) {
        volt -= brushes_voltage_drop;
      }
      return constrain(1000.0 * volt / supply_voltage, -1000, 1000);
    }

    void setPinPWM(int pin, int value) {
      if (not motors_attached) {
        Serial.println("ERROR: Motors are not attached, cannot control pwm!");
      }
      // pin: Physical Arduino Pin Number
      // value: 0...1000
      pinMode(pin, OUTPUT);

      int pwm_pseudo = 0;
      switch (pin) {
        case 2: pwm_pseudo = 5; break;
        case 3: pwm_pseudo = 2; break;
        case 7: pwm_pseudo = 6; break;
        case 8: pwm_pseudo = 7; break;
        case 11: pwm_pseudo = 11; break;
        case 12: pwm_pseudo = 12; break;
        case 45: pwm_pseudo = 46; break;
        case 44: pwm_pseudo = 45; break;
      }
      SetPinFrequency(pwm_pseudo, 20000);
      long pwm_value = value * 65.534;
      pwmWriteHR(pwm_pseudo, pwm_value);

    }

    void set_voltage(float voltage) {
      int power = volt_to_power(voltage);
      set_power(power);
    }

    void set_power(int value) {
      pinMode(pin_fwd, OUTPUT);
      pinMode(pin_bwd, OUTPUT);
      pinMode(pin_fwd_en, OUTPUT);
      pinMode(pin_bwd_en, OUTPUT);

      digitalWrite(pin_fwd_en, HIGH);
      digitalWrite(pin_bwd_en, HIGH);

      value = constrain(value, -max_power, max_power);



      // Map value to remove deadzone

      if (value > 0) {
        // Low PWM values are not transfered due to MosFET Rise- and Fall time, so we have to give it a little extra
        value = map(value, 1, 1000, fwd_deadzone_low, fwd_deadzone_high);
        //Serial.print("Setting pin_fwd to ");
        //Serial.print(value / 10.0);
        //Serial.println("%");
        setPinPWM(pin_fwd, value);
        digitalWrite(pin_bwd, LOW);
      } else if (value < 0) {
        // Low PWM values are not transfered due to MosFET Rise- and Fall time, so we have to give it a little extra
        value = map(value, -1, -1000, bwd_deadzone_low, bwd_deadzone_high);
        setPinPWM(pin_bwd, value);
        //Serial.print("Setting pin_bwd to ");
        //Serial.print(value / 10.0);
        //Serial.println("%");
        digitalWrite(pin_fwd, LOW);
      } else {
        //Serial.println("Setting pin_fwd and pin_bwd to 0%");
        digitalWrite(pin_fwd, LOW);
        digitalWrite(pin_bwd, LOW);
      }



    }

    void set_brake(int value) {
      pinMode(pin_fwd, OUTPUT);
      pinMode(pin_bwd, OUTPUT);
      pinMode(pin_fwd_en, OUTPUT);
      pinMode(pin_bwd_en, OUTPUT);

      digitalWrite(pin_fwd_en, HIGH);
      digitalWrite(pin_bwd_en, HIGH);
      setPinPWM(pin_fwd, value);
      setPinPWM(pin_bwd, value);
    }

    void freewheel() {
      pinMode(pin_fwd, OUTPUT);
      pinMode(pin_bwd, OUTPUT);
      pinMode(pin_fwd_en, OUTPUT);
      pinMode(pin_bwd_en, OUTPUT);

      digitalWrite(pin_fwd_en, LOW);
      digitalWrite(pin_bwd_en, LOW);
    }

};

Motor MotorLeft;
Motor MotorRight;

void attach_motors() {
  //Serial.print("Attaching Motors... ");
  MotorLeft.attach(M_LEFT_FWD_EN, M_LEFT_BWD_EN, M_LEFT_FORWARD, M_LEFT_BACKWARD, 92, 890, 100, 880);
  MotorRight.attach(M_RIGHT_FWD_EN, M_RIGHT_BWD_EN, M_RIGHT_FORWARD, M_RIGHT_BACKWARD, 100, 880, 92, 890);
  MotorLeft.set_power(0);
  MotorRight.set_power(0);
  //Serial.println("Motors attached.");
  motors_attached = true;

}

float left_motor_get_current(){
  return (float(analogRead(A2)) - 509.5) * -0.0488; // ACS 712 mit 20A range
}

float right_motor_get_current(){
  return (float(analogRead(A3)) - 513.5) * -0.0526; // ACS 712 mit 5A range und Bypass
}


void left_motor_set_power(int power) {
  MotorLeft.set_power(power);
}

void left_motor_set_voltage(float voltage) {
  // Sets the armature voltage by compensating for voltage drop at armature resistance
  // Has to be called repeatedly to keep compensation updated
  if (voltage == 0){
    MotorLeft.set_voltage(0);
    return;
  }
  float mot_left_current_actual = left_motor_get_current();
  if (abs(mot_left_current_actual) > MotorLeft.max_current){
    MotorLeft.last_current_limit_volt = 0.9*MotorLeft.last_current_limit_volt;
  }else{
    MotorLeft.last_current_limit_volt = constrain(1.1*MotorLeft.last_current_limit_volt, -25, 25);
  }
  float left_motor_armature_resistance_voltage_drop = mot_left_current_actual * MotorLeft.armature_resistance;
  float volt_out = voltage + left_motor_armature_resistance_voltage_drop;
  volt_out = constrain(volt_out, -MotorLeft.last_current_limit_volt, MotorLeft.last_current_limit_volt);
  MotorLeft.set_voltage(volt_out);
}

void left_motor_set_rpm(float rpm) {
  float encoder_rpm = rpm * MotorLeft.motor_gear_ratio;
  float estimated_armature_voltage = rpm / MotorLeft.kv;
  left_motor_set_voltage(estimated_armature_voltage);
}

void left_wheel_set_speed(float wheel_speed) {
  float motor_rpm = wheel_speed * 60 / (PI * 0.250) / 0.5 / MotorLeft.motor_gear_ratio;
  left_motor_set_rpm(motor_rpm);
}

void right_motor_set_power(int power) {
  MotorRight.set_power(power);
}

void right_motor_set_voltage(float voltage) {
  // Sets the armature voltage by compensating for voltage drop at armature resistance
  // Has to be called repeatedly to keep compensation updated
  if (voltage == 0){
    MotorRight.set_voltage(0);
    return;
  }
  float mot_right_current_actual = right_motor_get_current();
  if (abs(mot_right_current_actual) > MotorRight.max_current){
    MotorRight.last_current_limit_volt = 0.9*MotorRight.last_current_limit_volt;
  }else{
    MotorRight.last_current_limit_volt = constrain(1.1*MotorRight.last_current_limit_volt, -25, 25);
  }
  float right_motor_armature_resistance_voltage_drop = mot_right_current_actual * MotorRight.armature_resistance;
  float volt_out = voltage + right_motor_armature_resistance_voltage_drop;
  volt_out = constrain(volt_out, -MotorRight.last_current_limit_volt, MotorRight.last_current_limit_volt);
  MotorRight.set_voltage(volt_out);
}

void right_motor_set_rpm(float rpm) {
  float encoder_rpm = rpm * MotorRight.motor_gear_ratio;
  float estimated_armature_voltage = rpm / MotorRight.kv;
  right_motor_set_voltage(estimated_armature_voltage);
}

void right_wheel_set_speed(float wheel_speed) {
  float motor_rpm = wheel_speed * 60 / (PI * 0.250) / 0.5 / MotorRight.motor_gear_ratio;
  right_motor_set_rpm(motor_rpm);
}
