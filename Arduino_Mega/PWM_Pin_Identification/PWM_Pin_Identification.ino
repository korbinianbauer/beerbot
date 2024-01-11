#include <PWM.h>

void setup() {
  Serial.begin(9600);
  InitTimersSafe();

  for (int j = 2; j <= 53; j++) {
    pinMode(j, OUTPUT);
  }

  for (int i = 2; i <= 53; i++) {
    if (4 == i) {
      continue;
    }
    for (int j = 2; j <= 53; j++) {
      digitalWrite(j, LOW);
    }
    Serial.print("Trying Pin ");
    Serial.println(i);
    pinMode(i, OUTPUT);
    SetPinFrequency(i, 15000);
    pwmWriteHR(i, 32768);
    delay(1000);
  }



}

void loop() {
}
