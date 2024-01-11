#include <PWM.h>
#include "Motor.cpp"

#define M_LEFT_FORWARD 3
#define M_LEFT_BACKWARD 2
#define M_LEFT_FWD_EN 5
#define M_LEFT_BWD_EN 4

#define M_LEFT_POS A0

#define M_RIGHT_FORWARD 7
#define M_RIGHT_BACKWARD 8
#define M_RIGHT_FWD_EN 9
#define M_RIGHT_BWD_EN 6

#define M_RIGHT_POS A1


// Lookup table format: {end angle of section, offset in degrees, slope in degrees/degree}
#define LEFT_MAG_ENC_LOOKUP_TABLE_SECTIONS 6
float left_mag_enc_lookup_table[LEFT_MAG_ENC_LOOKUP_TABLE_SECTIONS][4] =
{ {  0,   60,    0,         0.2585},
  { 60,  120,   15.5121,   -0.0128},
  {120,  180,   14.7458,   -0.1398},
  {180,  240,    6.3595,    0.0084},
  {240,  300,    6.8662,   -0.0614},
  {300,  360,    3.1807,   -0.0530}
};

float mot_left_raw_angle = 0;
float mot_left_angle = 0;
float mot_left_last_raw_angle = 0;
float mot_left_last_angle = 0;

class MagEncoder {
  public:
    int pin;
    long abs_angle = 0;
    int last_angle = 0;
    int offset_angle = 0;
    bool inverted = false;
    float gear_reduction = 0;
    int circum = 0; // Wheel circumference in mm

    void attach(int pin_analog, bool invert) {
      pin = pin_analog;
      inverted = invert;
      offset_angle = get_angle();
      update_reading();
    }

    void update_reading() {
      // This has to be called >2x per revolution
      int angle = get_angle();
      int delta_angle = angle - last_angle;

      if (delta_angle < -180) {
        delta_angle += 360;
      } else if (delta_angle > 180) {
        delta_angle -= 360;
      }

      abs_angle += delta_angle;
      last_angle = angle;
    }

    long get_abs_angle() {
      return abs_angle;
    }

    void set_wheel_circumference(int circumference_mm){
      //Serial.print("Set wheel circumference to ");
      //Serial.println(circumference_mm);
      circum = circumference_mm;
    }

    void set_gear_reduction_ratio(float ratio){
      //Serial.print("Set gear reduction ratio to ");
      //Serial.println(ratio);
      gear_reduction = ratio;
    }

    long get_abs_pos(){
      return long(get_abs_angle()/360.0 * gear_reduction * circum);
    }

  private:

    int get_angle() {
      int raw_angle = int(analogRead(pin) / 1023.0 * 360);
      if (inverted) {
        raw_angle = 360 - raw_angle;
      }
      int offseted_angle = (raw_angle - offset_angle + 360) % 360;
      return offseted_angle;
    }



};



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

    void freewheel(){
      pinMode(pin_fwd, OUTPUT);
      pinMode(pin_bwd, OUTPUT);
      pinMode(pin_fwd_en, OUTPUT);
      pinMode(pin_bwd_en, OUTPUT);

      digitalWrite(pin_fwd_en, LOW);
      digitalWrite(pin_bwd_en, LOW);
    }

};

float linearize_left_encoder_reading(float raw_angle)
{
  for (int i = 0; i < LEFT_MAG_ENC_LOOKUP_TABLE_SECTIONS; i++) {
    float section_start_angle = left_mag_enc_lookup_table[i][0];
    float section_end_angle = left_mag_enc_lookup_table[i][1];
    if (section_end_angle >= raw_angle) {
      float offset = left_mag_enc_lookup_table[i][2];
      float slope = left_mag_enc_lookup_table[i][3];

      float linearized_angle = raw_angle + offset + (raw_angle - section_start_angle) * slope;


      while (linearized_angle < 0) {
        linearized_angle += 360;
      }
      while (linearized_angle > 360) {
        linearized_angle -= 360;
      }
      return linearized_angle;
    }
  }

}

Motor MotorLeft;
Motor MotorRight;

MagEncoder EncoderLeft;
MagEncoder EncoderRight;

void setup() {
  Serial.begin(250000);
  MotorLeft.attach(M_LEFT_FWD_EN, M_LEFT_BWD_EN, M_LEFT_FORWARD, M_LEFT_BACKWARD);
  MotorRight.attach(M_RIGHT_FWD_EN, M_RIGHT_BWD_EN, M_RIGHT_FORWARD, M_RIGHT_BACKWARD);
  EncoderLeft.attach(M_LEFT_POS, false);
  EncoderLeft.set_gear_reduction_ratio(0.5);
  EncoderLeft.set_wheel_circumference(int(PI*250));
  
  EncoderRight.attach(M_RIGHT_POS, true);
  EncoderRight.set_gear_reduction_ratio(0.5);
  EncoderRight.set_wheel_circumference(int(PI*250));


  MotorLeft.set_power(0);
  MotorRight.set_power(0);

  delay(3000);

  MotorLeft.freewheel();
  MotorRight.freewheel();


}

void loop() {
  EncoderLeft.update_reading();
  int left_angle = EncoderLeft.get_abs_angle();
  int left_pos = EncoderLeft.get_abs_pos();
  Serial.print("left_pos: ");
  Serial.print(left_pos);

  EncoderRight.update_reading();
  int right_angle = EncoderRight.get_abs_angle();
  int right_pos = EncoderRight.get_abs_pos();
  Serial.print(", right_pos: ");
  Serial.println(right_pos);
  // Wait for zero position
  float mot_left_raw_angle = analogRead(M_LEFT_POS) / 1023.0 * 360;
  float mot_left_angle = linearize_left_encoder_reading(mot_left_raw_angle);

  float mot_left_delta_raw_angle = mot_left_raw_angle - mot_left_last_raw_angle;
  float mot_left_delta_angle = mot_left_angle - mot_left_last_angle;

  mot_left_last_raw_angle = mot_left_raw_angle;
  mot_left_last_angle = mot_left_angle;



  delay(50);


}
