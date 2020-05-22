#include <PWM.h>


int RPWM=2;
int LPWM=3;
int L_EN=4;
int R_EN=5;

//int RPWM=6;
//int LPWM=7;
//int L_EN=8;
//int R_EN=9;
int POS_L = A0;

int pos_offset_l = 0;
int old_pos_l = 0;
long abs_pos_l = 0;

long last_cycle_micros = 0;
int speed_l = 0;

float current_L = 0;

float read_cur_L(){
  long sum = 0;
  for (int i = 0; i < 100; i++){
    sum += (analogRead(A2) - 510);
    delay(1);
  }
  return sum/100.0*74;
}

void setup() {
   InitTimersSafe();
  
   pinMode(RPWM,OUTPUT);
   pinMode(LPWM,OUTPUT);
   pinMode(L_EN,OUTPUT);
   pinMode(R_EN,OUTPUT);

   digitalWrite(RPWM,LOW);
   digitalWrite(LPWM,LOW);
   digitalWrite(L_EN,LOW);
   digitalWrite(R_EN,LOW);

   SetPinFrequency(RPWM, 15000);

   Serial.begin(9600);

   Serial.print("RPWM resolution:");
   Serial.println(GetPinResolution(RPWM));
   Serial.print("LPWM resolution:");
   Serial.println(GetPinResolution(LPWM));

   Serial.println("EN High");
   digitalWrite(R_EN,HIGH);
   digitalWrite(L_EN,HIGH);
   pwmWriteHR(RPWM,9500);
   //digitalWrite(LPWM, HIGH);

   pos_offset_l = analogRead(POS_L)/1023.0*360;
   last_cycle_micros = micros();

   current_L = analogRead(A2);
  }



void loop() {
  // put your main code here, to run repeatedly:
  int new_pos_l = analogRead(POS_L)/1023.0*360 - pos_offset_l;

  int delta_pos_l = new_pos_l - old_pos_l;

  if (delta_pos_l < -180){
    delta_pos_l += 360;
  }else if (delta_pos_l > 180){
    delta_pos_l -= 360;
  }

  abs_pos_l += delta_pos_l;
  old_pos_l = new_pos_l;

  int dt = micros() - last_cycle_micros;
  last_cycle_micros = micros();
  speed_l = (1000000L * delta_pos_l) / dt;

  
  //Serial.print(abs_pos_l);
  //Serial.print(", ");
  //Serial.println(speed_l);
  //Serial.print(millis());
  //Serial.print(",");
  current_L = 0.8 * current_L + 0.2 * analogRead(A2);
  Serial.println(current_L - 510);
  //delay(1);
}
