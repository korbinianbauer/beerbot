#include "MPU9250.h"		// https://github.com/bolderflight/MPU9250 	
#include "Streaming.h" 		// needed for the Serial output https://github.com/geneReeves/ArduinoStreaming
#include "SensorFusion.h"
SF IMU_fusion;
MPU9250 IMU9250(Wire, 0x68);

float IMU_gx, IMU_gy, IMU_gz, IMU_ax, IMU_ay, IMU_az, IMU_mx, IMU_my, IMU_mz, IMU_temp;
float IMU_pitch, IMU_roll, IMU_yaw;
float IMU_deltat;

void attach_IMU() {

  //Serial.print("Attaching IMU... ");

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

void update_IMU() {
  IMU9250.readSensor();

  IMU_ax = IMU9250.getAccelX_mss();
  IMU_ay = IMU9250.getAccelY_mss();
  IMU_az = IMU9250.getAccelZ_mss();
  IMU_gx = IMU9250.getGyroX_rads();
  IMU_gy = IMU9250.getGyroY_rads();
  IMU_gz = IMU9250.getGyroZ_rads();
  IMU_mx = IMU9250.getMagX_uT();
  IMU_my = IMU9250.getMagY_uT();
  IMU_mz = IMU9250.getMagZ_uT();
  IMU_temp = IMU9250.getTemperature_C();

  IMU_deltat = IMU_fusion.deltatUpdate();
  IMU_fusion.MadgwickUpdate(IMU_gx, IMU_gy, IMU_gz, IMU_ax, IMU_ay, IMU_az, IMU_mx, IMU_my, IMU_mz, IMU_deltat);
}

float get_IMU_roll() {
  return IMU_fusion.getRoll();
}

float get_IMU_pitch() {
  return IMU_fusion.getPitch();
}

float get_IMU_yaw() {
  return IMU_fusion.getYaw();
}

float get_IMU_roll_radians() {
  return IMU_fusion.getRollRadians();
}

float get_IMU_pitch_radians() {
  return IMU_fusion.getPitchRadians();
}

float get_IMU_yaw_radians() {
  return IMU_fusion.getYawRadians();
}

void calib_mag_and_print_result() {
  Serial.println("Starting MAG calibration");
  IMU9250.calibrateMag();
  Serial.println("MAG calibration done. Results:");
  Serial.println(IMU9250.getMagBiasX_uT());
  Serial.println(IMU9250.getMagScaleFactorX());
  Serial.println(IMU9250.getMagBiasY_uT());
  Serial.println(IMU9250.getMagScaleFactorY());
  Serial.println(IMU9250.getMagBiasZ_uT());
  Serial.println(IMU9250.getMagScaleFactorZ());
  Serial.println("End of results");
}

void write_mag_calib_values(int x_bias, int x_scale, int y_bias, int y_scale, int z_bias, int z_scale) {
  IMU9250.setMagCalX(x_bias, x_bias);
  IMU9250.setMagCalY(y_bias, y_bias);
  IMU9250.setMagCalZ(z_bias, z_bias);
}

