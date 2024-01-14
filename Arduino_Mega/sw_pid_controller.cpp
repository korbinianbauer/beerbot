#include <Arduino.h>
#include "Streaming.h"

class PidController {
  public:
    float setpoint = 0;
    float input = 0;
    float error = 0;
    float Kp = 0;
    float Ki = 0;
    float Kd = 0;
    float I_limit = 1000000000;
    float D_limit = 1000000000;

    float P_out = 0;
    float I_out = 0;
    float D_out = 0;
    float output = 0;

    float lastError = 0;
    long lastRun = 0;
    bool started = false;


    float evaluate(float input) {

      long time_now = micros();

      error = setpoint - input;

      if (not started) {
        started = true;
        lastRun = time_now;
        lastError = error;
      }


      float dt = (time_now - lastRun) / 1000.0; // ms

      P_out = Kp * error;
      I_out += Ki * error * dt;
      I_out = constrain(I_out, -I_limit, I_limit);
      D_out = Kd * (error - lastError) / dt;
      D_out = constrain(D_out, -D_limit, D_limit);

      if (setpoint == 0){
        // reset integrator when speed = 0 requested
        I_out = 0;
      }

      output = P_out + I_out;
      if (not isnan(D_out)){
        // If both (error - lastError) and dt are 0, D_out is invalid, so we don't include it.
        output += D_out;
      }

      

      lastError = error;
      lastRun = time_now;

      //Serial << "Setpoint:" << setpoint*100 << ", Input:" << input*100 << ", Error:" << error*100 << ", P_out:" << P_out*100 << ", I_out:" << I_out*100 << ", D_out:" << D_out*100 << ", Output:" << output*100 << "\n";

      return output;
    }


};
