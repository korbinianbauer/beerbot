#include "Streaming.h"

#define sample_target 3000
#define SAMPLE_INTERVAL 200

void setup() {
  // put your setup code here, to run once:
  Serial.begin(1000000);
  Serial.println("Start");
  attach_motors();
  left_motor_set_power(10);
}

unsigned long last_time = 0;
unsigned int samples[sample_target];

void loop() {

  // Collect
  unsigned long start_time = micros();
  for (int i = 0; i < sample_target; i++) {
    unsigned long time_now = micros();
    samples[i] = analogRead(A2);



    last_time = time_now;
    while (micros() - last_time < SAMPLE_INTERVAL-4) {}
  }

  // Evaluate
  unsigned long end_time = micros();
  unsigned long duration = end_time - start_time;
  float actual_interval = duration / float(sample_target);
  float quality = abs(100 * float(SAMPLE_INTERVAL) / actual_interval);

  // Output
  Serial.println("Result:\n");
  for (int i = 0; i < sample_target; i++) {
    Serial << samples[i] << ", ";
  }
  Serial << "\n\nCollected " << sample_target << " samples in " << duration << "us. (" << actual_interval << "us per sample). Target was " << SAMPLE_INTERVAL << "us per sample\nQuality index: " << quality << "%" << "\n";


  // Hold
  left_motor_set_power(0);
  while (true) {}


}

