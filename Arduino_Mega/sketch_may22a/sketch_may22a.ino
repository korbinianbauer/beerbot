#include <PWM.h>

#define M_LEFT_FORWARD 3
#define M_LEFT_BACKWARD 2
#define M_LEFT_FWD_EN 5
#define M_LEFT_BWD_EN 4

#define M_LEFT_POS A0

#define M_RIGHT_FORWARD 7
#define M_RIGHT_BACKWARD 8
#define M_RIGHT_FWD_EN 9
#define M_RIGHT_BWD_EN 6



class Motor {
  public:
    int pin_fwd_en;
    int pin_bwd_en;
    int pin_fwd;
    int pin_bwd;

    Motor(int fwd_en, int bwd_en, int fwd, int bwd) {
      InitTimersSafe();
      pin_fwd_en = fwd_en;
      pin_bwd_en = bwd_en;
      pin_fwd = fwd;
      pin_bwd = bwd;
    }



    void setPinPWM(int pin, int value) {
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

};


void setup() {
  Serial.begin(250000);
  Motor MotorLeft = Motor(M_LEFT_FWD_EN, M_LEFT_BWD_EN, M_LEFT_FORWARD, M_LEFT_BACKWARD);
  Motor MotorRight = Motor(M_RIGHT_FWD_EN, M_RIGHT_BWD_EN, M_RIGHT_FORWARD, M_RIGHT_BACKWARD);

  delay(5000);
  MotorLeft.set_power(20);
  delay(2000);
  MotorLeft.set_brake(100);
  delay(100);
  MotorLeft.set_brake(0);


}

void loop() {
  // Wait for zero position
  while (analogRead(M_LEFT_POS) != 0) {}
  while (analogRead(M_LEFT_POS) == 0) {}
  Serial.println(0);
  int pos = analogRead(M_LEFT_POS);
  while (pos != 1023) {
    Serial.println(pos);
    delayMicroseconds(2000);
    pos = analogRead(M_LEFT_POS);
  }
  delay(100000L);


}
