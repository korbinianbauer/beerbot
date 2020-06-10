#include <Wire.h>
#include "nunchuk.h"    // http://www.xarg.org/2016/12/arduino-nunchuk-library/
#include "MPU9250.h"    // https://github.com/bolderflight/MPU9250  
MPU9250 IMU9250(Wire, 0x68);


// IMU Methods
void attach_IMU() {

  Serial.print("Attaching IMU... ");
  int IMU_status;
  // start communication with IMU9250
  IMU_status = IMU9250.begin();
  if (IMU_status < 0) {
    Serial.println("IMU9250 initialization unsuccessful");
    Serial.println("Check IMU9250 wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(IMU_status);
    while (1) {}
  }
  IMU_status = IMU9250.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_10HZ);
  if (IMU_status < 0) {
    Serial.println("Setting IMU9250 bandwidth unsuccessful");
    Serial.println("Check IMU9250 wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(IMU_status);
    while (1) {}
  }
  write_mag_calib_values(31.99, 1.07, 41.07, 0.95, -32.06, 0.99);
  Serial.println("IMU attached.");
}

void write_mag_calib_values(int x_bias, int x_scale, int y_bias, int y_scale, int z_bias, int z_scale) {
  IMU9250.setMagCalX(x_bias, x_bias);
  IMU9250.setMagCalY(y_bias, y_bias);
  IMU9250.setMagCalZ(z_bias, z_bias);
}


// Nunchuk Methods
bool nunchuk_attached = false;

void attach_nunchuk() {
  Serial.print("Attaching Nunchuk... ");
  Wire.begin();
  nunchuk_init();
  nunchuk_attached = true;
  Serial.println("Nunchuk attached.");
}

int get_nunchuk_y() {
  nunchuk_read();
  return nunchuk_joystickY_raw();
}


// Main code
void setup() {
  Serial.begin(1000000);
  attach_IMU();
  attach_nunchuk();
}


void loop() {
  Serial.println(get_nunchuk_y());
}
