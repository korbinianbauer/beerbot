bool motors_attached = false;

char dashLine[] = "=====================================================================================";

class Motor {
  public:
    float armature_resistance = 0.4;
    float brushes_voltage_drop = 0.0;
    float motor_gear_ratio = 1 / 9.778;
    float kv = 155; // rpm/V

    float max_current = 10;
    float last_current_limit_volt = 25;

    int pin_dir;
    int pin_pwm;

    int fwd_deadzone_low;
    int fwd_deadzone_high;
    int bwd_deadzone_low;
    int bwd_deadzone_high;

    float supply_voltage = 22.3;

    int max_power = 1000;


    void attach(int dir, uint8_t pwm, int forward_deadzone_low, int forward_deadzone_high, int backward_deadzone_low, int backward_deadzone_high) {
      pin_dir = dir;
      pin_pwm = pwm;



      pinMode(pin_dir, OUTPUT);
      pinMode(pin_pwm, OUTPUT);

      fwd_deadzone_low = forward_deadzone_low;
      fwd_deadzone_high = forward_deadzone_high;
      bwd_deadzone_low = backward_deadzone_low;
      bwd_deadzone_high = backward_deadzone_high;

      set_power(0);

      Serial.print("Attached Motor with DIR-Pin ");
      Serial.print(pin_dir);
      Serial.print(" and PWM-Pin ");
      Serial.println(pin_pwm);

    }

    float volt_to_power(float volt) {
      return constrain(1000.0 * volt / supply_voltage, -1000, 1000);
    }

    void set_voltage(float voltage) {
      int power = volt_to_power(voltage);
      set_power(power);
    }

    void set_power(int value) {
      value = constrain(value, -max_power, max_power);



      // Map value to remove deadzone

      if (value > 0) {
        // Low PWM values are not transfered due to MosFET Rise- and Fall time, so we have to give it a little extra
        value = map(value, 1, 1000, fwd_deadzone_low, fwd_deadzone_high);
        setPinPWM(pin_pwm, value / 1000.0f);
        //        Serial.print("Setting pin_pwm (");
        //        Serial.print(pin_pwm);
        //        Serial.print(") to ");
        //        Serial.print(value / 1000.0f);
        //        Serial.println("%");
        digitalWrite(pin_dir, LOW);
      } else if (value < 0) {
        // Low PWM values are not transfered due to MosFET Rise- and Fall time, so we have to give it a little extra
        value = map(value, -1, -1000, bwd_deadzone_low, bwd_deadzone_high);
        setPinPWM(pin_pwm, value / 1000.0f);
        //        Serial.print("Setting pin_pwm (");
        //        Serial.print(pin_pwm);
        //        Serial.print(") to ");
        //        Serial.print(value / 1000.0f);
        //        Serial.println("%");
        digitalWrite(pin_dir, HIGH);
      } else {
        //        Serial.println("Setting pin_dir to LOW and pin_pwm to 0");
        digitalWrite(pin_dir, LOW);
        //setPinPWM(pin_dir, 0);
        //digitalWrite(pin_pwm, LOW);
        setPinPWM(pin_pwm, 0);
      }



    }

    void setPinPWM(int pin, float dutycycle) {
      // dutycycle in percent, 0.0 = off, 1.0 = fully on

      //      Serial.print("Called SetPinPWM with pin=");
      //      Serial.print(pin);
      //      Serial.print(" and dutycycle=");
      //      Serial.println(dutycycle);

      if (not motors_attached) {
        Serial.println("ERROR: Motors are not attached, cannot control pwm!");
      }

      if (pin == 2) {
        //        Serial.print("Writing to OCR3B");
        //        Serial.print(" with value=");
        //        Serial.println((int) (ICR3 * dutycycle));
        OCR3B = (int) (ICR3 * dutycycle);
      } else if (pin == 5) {
        //        Serial.print("Writing to OCR3A");
        //        Serial.print(" with value=");
        //        Serial.println((int) (ICR3 * dutycycle));
        OCR3A = (int) (ICR3 * dutycycle);
      } else {
        Serial.println("setPinPWM: Pin not supported!!!");
      }

    }

    /*
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
        }*/

};

Motor MotorLeft;
Motor MotorRight;

void attach_motors() {
  //Serial.print("Attaching Motors... ");
  TCCR3B = 0x18; // 0001 1000, Disable Timer
  TCCR3A = 0xA2; // 1010 0010
  ICR3 = 1000 - 1; // Frequency divider. 16 MHz/1000 -> 16 KHz
  TCNT3 = 0x0;// Reset Time Counter I guess? Not sure.
  TCCR3B |= 1; // Prescale=1, Enable Timer

  MotorLeft.attach(M_LEFT_DIR, M_LEFT_PWM, 50, 1000, 50, 1000);
  MotorRight.attach(M_RIGHT_DIR, M_RIGHT_PWM, 50, 1000, 50, 1000);
  MotorLeft.set_power(0);
  MotorRight.set_power(0);
  //Serial.println("Motors attached.");
  motors_attached = true;

}

float left_motor_get_current() {
  return (float(analogRead(A2)) - 509.5) * -0.0488; // ACS 712 mit 20A range
}

float right_motor_get_current() {
  return (float(analogRead(A3)) - 513.5) * -0.0526; // ACS 712 mit 5A range und Bypass
}


void left_motor_set_power(int power) {
  MotorLeft.set_power(power);
}

void left_motor_set_voltage(float voltage) {
  // Sets the armature voltage by compensating for voltage drop at armature resistance
  // Has to be called repeatedly to keep compensation updated
  if (voltage == 0) {
    MotorLeft.set_voltage(0);
    return;
  }
  float mot_left_current_actual = left_motor_get_current();
  if (abs(mot_left_current_actual) > MotorLeft.max_current) {
    MotorLeft.last_current_limit_volt = 0.9 * MotorLeft.last_current_limit_volt;
  } else {
    MotorLeft.last_current_limit_volt = constrain(1.1 * MotorLeft.last_current_limit_volt, -25, 25);
  }
  float left_motor_armature_resistance_voltage_drop = mot_left_current_actual * MotorLeft.armature_resistance;

  if (mot_left_current_actual > 0) {
    left_motor_armature_resistance_voltage_drop += MotorLeft.brushes_voltage_drop;
  } else if (mot_left_current_actual < 0) {
    left_motor_armature_resistance_voltage_drop -= MotorLeft.brushes_voltage_drop;
  }


  if ((voltage < 0) and (left_motor_armature_resistance_voltage_drop > 0)) {
    left_motor_armature_resistance_voltage_drop = 0;
  }
  if ((voltage > 0) and (left_motor_armature_resistance_voltage_drop < 0)) {
    left_motor_armature_resistance_voltage_drop = 0;
  }

  float volt_out = voltage + left_motor_armature_resistance_voltage_drop;
  volt_out = constrain(volt_out, -MotorLeft.last_current_limit_volt, MotorLeft.last_current_limit_volt);
  MotorLeft.set_voltage(volt_out);
  //Serial << "voltage_L:" << voltage << ", left_motor_armature_resistance_voltage_drop:" << left_motor_armature_resistance_voltage_drop << ", volt_out_L:" << volt_out << "\n";
}

void left_motor_set_rpm(float rpm) {
  //Serial << "left_rpm:" << rpm << "\n";
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
  if (voltage == 0) {
    MotorRight.set_voltage(0);
    return;
  }
  float mot_right_current_actual = right_motor_get_current();
  if (abs(mot_right_current_actual) > MotorRight.max_current) {
    MotorRight.last_current_limit_volt = 0.9 * MotorRight.last_current_limit_volt;
  } else {
    MotorRight.last_current_limit_volt = constrain(1.1 * MotorRight.last_current_limit_volt, -25, 25);
  }
  float right_motor_armature_resistance_voltage_drop = mot_right_current_actual * MotorRight.armature_resistance;

  if (mot_right_current_actual > 0) {
    right_motor_armature_resistance_voltage_drop += MotorRight.brushes_voltage_drop;
  } else if (mot_right_current_actual < 0) {
    right_motor_armature_resistance_voltage_drop -= MotorRight.brushes_voltage_drop;
  }

  if ((voltage < 0) and (right_motor_armature_resistance_voltage_drop > 0)) {
    right_motor_armature_resistance_voltage_drop = 0;
  }
  if ((voltage > 0) and (right_motor_armature_resistance_voltage_drop < 0)) {
    right_motor_armature_resistance_voltage_drop = 0;
  }

  float volt_out = voltage + right_motor_armature_resistance_voltage_drop;
  volt_out = constrain(volt_out, -MotorRight.last_current_limit_volt, MotorRight.last_current_limit_volt);
  MotorRight.set_voltage(volt_out);
  //Serial << "voltage_R:" << voltage << ", right_motor_armature_resistance_voltage_drop:" << right_motor_armature_resistance_voltage_drop << ", volt_out_R:" << volt_out << "\n";

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
