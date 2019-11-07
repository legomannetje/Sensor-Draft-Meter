#include "MPU9250.h"

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);
int status;

#define accConstant 0.05
#define sampleFreq 5      // Sampling at 100HZ
float accel[3];
float gyro[3];
float pitch;
float roll;
float draft;

unsigned long previousMillis = 0;

void setup() {
  // serial to display data
  Serial.begin(115200);
  while (!Serial) {}

  // start communication with IMU
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }
  // setting the accelerometer full scale range to +/-8G
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(19);
}

void getData() {
  // Place the data in arrays
  accel[0] = IMU.getAccelX_mss();
  accel[1] = IMU.getAccelY_mss();
  accel[2] = IMU.getAccelZ_mss();
  gyro[0] = IMU.getGyroX_rads();
  gyro[1] = IMU.getGyroY_rads();
  gyro[2] = IMU.getGyroZ_rads();
}


void ConvertIMU()
{
  //Some magic from the Internet, adopted it to work with our thing
  
  static float pitchAcc, rollAcc;

  // Integrate the gyroscope data -> int(angularSpeed) = angle
  unsigned long currentMillis = millis();
  pitch += gyro[0] / pow(currentMillis - previousMillis, -1);
  roll += gyro[1] / pow(currentMillis - previousMillis, -1);
  previousMillis = currentMillis;
  // Compensate for drift with accelerometer data if !bullshit
  // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
  int forceMagnitudeApprox = abs(accel[0]) + abs(accel[1]) + abs(accel[2]);
  if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
  {
    pitchAcc = atan2(accel[0], accel[2]);
    pitch = pitch * (1 - accConstant) + pitchAcc * accConstant;

    rollAcc = atan2(accel[1], accel[2]);
    roll = roll * (1 - accConstant) + rollAcc * accConstant;
  }
}

float calcAccel() {
  float draft = sqrt((cos(pitch) - 1) / (0.055 * 0.4 * sin(pitch)));
  return draft;
}
void loop() {
  // read the sensor data
  IMU.readSensor();

  //Set the data in the arrays
  getData();
  // Convert the data to pitch and roll
  ConvertIMU();
  // Calculate the angle of the sensor
  draft = calcAccel();
  
  //Print out the Pitch, Roll and Angle
  Serial.print(pitch);
  Serial.print("\t");
  Serial.print(roll);
  Serial.print("\t");
  Serial.print(draft);
  Serial.print('\n');
  
  //Wait a little while for the next reading
  delay(150);
}
