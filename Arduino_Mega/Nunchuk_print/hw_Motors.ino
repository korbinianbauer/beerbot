#include <PWM.h>

bool motors_attached = false;

class Motor {
  public:
    int pin_fwd_en;
    int pin_bwd_en;
    int pin_fwd;
    int pin_bwd;

    void attach(int fwd_en, int bwd_en, int fwd, int bwd) {
      InitTimersSafe();
      pin_fwd_en = fwd_en;
      pin_bwd_en = bwd_en;
      pin_fwd = fwd;
      pin_bwd = bwd;

      set_power(0);
    }



    void setPinPWM(int pin, int value) {
      if (not motors_attached) {
        Serial.println("ERROR: Motors are not attached, cannot control pwm!");
      }
      // pin: Physical Arduino Pin Number
      // value: 0...100
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
      long pwm_value = value * 655;
      pwmWriteHR(pwm_pseudo, pwm_value);

    }

    void set_power(int value) {
      pinMode(pin_fwd, OUTPUT);
      pinMode(pin_bwd, OUTPUT);
      pinMode(pin_fwd_en, OUTPUT);
      pinMode(pin_bwd_en, OUTPUT);

      digitalWrite(pin_fwd_en, HIGH);
      digitalWrite(pin_bwd_en, HIGH);

      if (value > 0) {
        // Low PWM values are not transfered due to MosFET Rise- and Fall time, so we have to give it a little extra
        value = map(value, 1, 100, 16, 100);
        setPinPWM(pin_fwd, value);
        digitalWrite(pin_bwd, LOW);
      } else if (value < 0) {
        // Low PWM values are not transfered due to MosFET Rise- and Fall time, so we have to give it a little extra
        value = map(value, -1, -100, 16, 100);
        setPinPWM(pin_bwd, value);
        digitalWrite(pin_fwd, LOW);
      } else {
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
  Serial.print("Attaching Motors... ");
  MotorLeft.attach(M_LEFT_FWD_EN, M_LEFT_BWD_EN, M_LEFT_FORWARD, M_LEFT_BACKWARD);
  MotorRight.attach(M_RIGHT_FWD_EN, M_RIGHT_BWD_EN, M_RIGHT_FORWARD, M_RIGHT_BACKWARD);
  MotorLeft.set_power(0);
  MotorRight.set_power(0);
  Serial.println("Motors attached.");
  motors_attached = true;
}


void left_motor_set_power(int power){
  MotorLeft.set_power(power);
}

void right_motor_set_power(int power){
  MotorRight.set_power(power);
}

