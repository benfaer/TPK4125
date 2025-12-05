#include <Wire.h>
#include <SparkFun_BMI270_Arduino_Library.h>
#include <Adafruit_AHRS.h> 

BMI270 imu;
Adafruit_Mahony filter;

const float angleOffset = 90;

const int sdaPin = 2;
const int sclPin = 3;
uint8_t i2cAddress = BMI2_I2C_PRIM_ADDR; // 0x68

float thetaEstimat = 0.0; 
const float alpha = 0.9804; //tidskonstant
const float dt = 0.01; //tidsteg ved 100Hz


void setupSensor() {
  filter.begin(100); // Oppdater filteret med 100 Hz
  Wire.setPins(sdaPin, sclPin);
  Wire.begin();
  Wire.setClock(400000);

  if (imu.beginI2C(i2cAddress) != BMI2_OK) {
    Serial.println("BMI270 not connected!");
    while (1);
  }

  bmi2_sens_config accelConfig;
  accelConfig.type = BMI2_ACCEL; // Konfigurasjon for akselerasjon
  accelConfig.cfg.acc.odr = BMI2_ACC_ODR_100HZ; // 100Hz avlesingshastighet
  accelConfig.cfg.acc.bwp = BMI2_ACC_OSR2_AVG2; // normalt lavpassfilter
  accelConfig.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE; // optimal filterytelse
  accelConfig.cfg.acc.range = BMI2_ACC_RANGE_4G; // Måleområde +- 4g
  int8_t err = imu.setConfig(accelConfig);

  if (err != BMI2_OK){
    Serial.println("Error: Could not set accelerometer configuration!");
    return;
  }

  bmi2_sens_config gyroConfig;
  gyroConfig.type = BMI2_GYRO; // Konfigurasjon for gyroskop
  gyroConfig.cfg.gyr.odr = BMI2_GYR_ODR_100HZ; // 100Hz avlesingshastighet
  gyroConfig.cfg.gyr.bwp = BMI2_GYR_OSR2_MODE; // normalt lavpassfilter
  gyroConfig.cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE; // optimal filterytelse
  gyroConfig.cfg.gyr.ois_range = BMI2_GYR_OIS_250; // (Ubrukt)
  gyroConfig.cfg.gyr.range = BMI2_GYR_RANGE_2000; // Måleområde 2000 grader per sekund
  gyroConfig.cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE; // Optimal støybehandling
  err = imu.setConfig(gyroConfig);

  if (err != BMI2_OK){
    Serial.println("Error: Could not set gyroscope configuration!");
    return;
  }
  Serial.println("BMI270 connected!");
}

void sensorTask() {
  uint8_t status;
  imu.getStatus(&status);
  while (!((status & BMI2_DRDY_ACC) && (status & BMI2_DRDY_GYR))){
    imu.getStatus(&status);
  }

  imu.getSensorData();

    filter.updateIMU(
    imu.data.gyroX, imu.data.gyroY, imu.data.gyroZ,
    imu.data.accelX, imu.data.accelY, imu.data.accelZ
    );
    
    float roll = filter.getRoll();
    angle = filter.getRoll() - angleOffset;

  Serial.println(angle); 
}
