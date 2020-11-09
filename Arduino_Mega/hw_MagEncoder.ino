class MagEncoder {
  public:
    int pin;
    long abs_angle = 0;
    long angular_speed = 0;
    int last_angle = 0;
    int offset_angle = 0;
    bool inverted = false;
    float gear_reduction = 0;
    int circum = 0; // Wheel circumference in mm
    unsigned long last_update;
    unsigned long last_speed_update;
    long last_abs_speed_angle = 0;

    void attach(int pin_analog, bool invert) {
      pin = pin_analog;
      inverted = invert;
      offset_angle = get_angle();
      update_reading();
    }

    void update_reading() {
      // This has to be called >2x per revolution
      unsigned long time_now = millis();

      int angle = get_angle();
      int delta_angle = angle - last_angle;
      unsigned long dt = time_now - last_update;

      if (delta_angle < -1800) {
        delta_angle += 3600;
      } else if (delta_angle > 1800) {
        delta_angle -= 3600;
      }

      abs_angle += delta_angle;


      // angle change since last speed calculation
      long delta_abs_speed_angle = abs_angle - last_abs_speed_angle;
      long dt_speed = time_now - last_speed_update;
      if (abs(delta_abs_speed_angle) > 8) {
        // Avoid speed readings due to jitter
        float new_angular_speed = 100.0 * delta_abs_speed_angle / dt_speed; // degrees per second
        //angular_speed = 0.9 * angular_speed + 0.1 * new_angular_speed;
        angular_speed = new_angular_speed;
        last_speed_update = time_now;
        last_abs_speed_angle = abs_angle;
      } else if (dt_speed >= 100) {
        // If no significant angle change since 100ms, assume speed == 0
        angular_speed = 0;
        last_speed_update = time_now;
        last_abs_speed_angle = abs_angle;
      }





      last_angle = angle;
      last_update = time_now;
    }

    long get_abs_angle() {
      // returns encoder absolute angle in degrees
      return abs_angle / 10;
    }

    long get_abs_wheel_angle() {
      // returns wheel absolute angle in degrees
      return abs_angle / 10 * gear_reduction;
    }

    float get_angular_wheel_speed() {
      // returns wheel angular speed in degrees per second
      return angular_speed * gear_reduction;
    }

    float get_wheel_speed() {
      // returns wheel speed in meters per second
      return get_angular_wheel_speed() / 360 * circum / 1000.0;
    }

    void set_wheel_circumference(int circumference_mm) {
      //Serial.print("Set wheel circumference to ");
      //Serial.println(circumference_mm);
      circum = circumference_mm;
    }

    void set_gear_reduction_ratio(float ratio) {
      //Serial.print("Set gear reduction ratio to ");
      //Serial.println(ratio);
      gear_reduction = ratio;
    }

    float get_abs_pos() {
      // returns wheel cumulated distance in meters
      return get_abs_wheel_angle() / 360.0 / 1000.0 * circum;
    }

  private:

    int get_angle() {
      int raw_angle = int(analogRead(pin) / 1023.0 * 3600); // Zehntel grad -> 1 Umdrehung = 3600
      if (inverted) {
        raw_angle = 3600 - raw_angle;
      }
      int offseted_angle = (raw_angle - offset_angle + 3600) % 3600;
      return offseted_angle;
    }



};

MagEncoder EncoderLeft;
MagEncoder EncoderRight;

void attach_encoders() {
  //Serial.print("Attaching Encoders... ");

  EncoderLeft.attach(M_LEFT_POS, false);
  EncoderLeft.set_gear_reduction_ratio(0.5);
  EncoderLeft.set_wheel_circumference(int(PI * 254));

  EncoderRight.attach(M_RIGHT_POS, true);
  EncoderRight.set_gear_reduction_ratio(0.5);
  EncoderRight.set_wheel_circumference(int(PI * 254));

  //Serial.println("Encoders attached.");
}

void update_encoders() {
  EncoderLeft.update_reading();
  EncoderRight.update_reading();
}


long get_left_wheel_abs_angle() {
  // returns left wheel cumulated angle in degrees
  return EncoderLeft.get_abs_wheel_angle();
}

float get_left_wheel_distance() {
  // returns left wheel cumulated distance in meters
  return EncoderLeft.get_abs_pos();
}

long get_left_wheel_angular_speed() {
  // returns left wheel angular speed in degrees per second
  return EncoderLeft.angular_speed;
}

float get_left_wheel_speed() {
  // returns left wheel speed in meters per second
  return EncoderLeft.get_wheel_speed();
}


long get_right_wheel_abs_angle() {
  // returns right wheel cumulated angle in degrees
  return EncoderRight.get_abs_wheel_angle();
}

float get_right_wheel_distance() {
  // returns right wheel cumulated distance in meters
  return EncoderRight.get_abs_pos();
}

long get_right_wheel_angular_speed() {
  // returns right wheel angular speed in degrees per second
  return EncoderRight.get_angular_wheel_speed();
}

float get_right_wheel_speed() {
  // returns right wheel speed in meters per second
  return EncoderRight.get_wheel_speed();
}




