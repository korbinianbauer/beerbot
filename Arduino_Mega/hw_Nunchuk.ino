#include <Wire.h>
#include "nunchuk.h"

bool nunchuk_attached = false;

void attach_nunchuk() {
  //Serial.print("Attaching Nunchuk... ");
  Wire.begin();
  nunchuk_init();
  nunchuk_attached = true;
  //Serial.println("Nunchuk attached.");
}

int get_nunchuk_x() {
  if (not nunchuk_attached) {
    Serial.println("ERROR: Nunchuk is not attached, cannot read!");
    return 0;
  }
  delayMicroseconds(500);
  nunchuk_read();
  return nunchuk_joystickX_raw();
}

int get_nunchuk_y() {
  if (not nunchuk_attached) {
    Serial.println("ERROR: Nunchuk is not attached, cannot read!");
    return 0;
  }
  delayMicroseconds(500);
  nunchuk_read();
  return nunchuk_joystickY_raw();
}

bool get_nunchuk_button_c(){
  delayMicroseconds(500);
  nunchuk_read();
  return nunchuk_buttonC();
}


bool get_nunchuk_button_z(){
  delayMicroseconds(500);
  nunchuk_read();
  return nunchuk_buttonZ();
}

