#include <Wire.h>
#include "Kalman.h" 
#include <Servo.h>

Servo HubMotor;
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/*             IMU Data                */

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;
double dt=0;
float angle_estimate=0;
double throttle=1500; //initial value of throttle to the motors
double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
float angle;
float CorrectedAngle; //initial 0 value for footpad safety
float PWMangle;
float PWMout;
float VESCPWM, VESCPWMRawVal;
uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data
int count=0; //not really using this in the code. Should this be used? Maybe..

void setup() {
  Serial.begin(115200); //increase?
  Wire.begin();
 #if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }
  HubMotor.attach(3); //attatch the Hub motor to pin 3. This is from the servo library. Attach is needed before running
  HubMotor.writeMicroseconds(1500); //Initial pwm data sent to the wheel. 1500 = zero throttle. Range  1000 - 2000
  delay(100); // Wait for sensor to stabilize
  
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
}

void loop() {

/*           Foot Button                    */

double footpad1 = analogRead(A6); //A6 and A7 pins are reading the voltage from the footpad voltage divider
double footpad2 = analogRead(A7);

  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);

  dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();
  
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
    
angle = -kalAngleX;  //angle estimate to use. For some reason the calculated angle is inverted. I might be using the gyro wrong, but this is how i prefer. 
Serial.println(angle);


//Serial.println("Original Angle:");
//Serial.println(angle); //remove in final code
//CorrectedAngle = angle*15 + 1500;

CorrectedAngle = map (angle, -30, 30, 0, 255); //mapping the angle from -30 to 30, to 1000 to 2000. i.e. making it from an angle in degrees to PWM output from 1000 to 2000
VESCPWM = constrain (CorrectedAngle, 0, 255); //  make sure the angle is with the defined constraints. Anything crazy outside the lmits will be constrained by this command
//PWMangle = map (angle, -30, 30, 0, 255);
//PWMout = constrain (PWMangle, 0, 255);
//Serial.println (PWMangle);
//Main command to write to vesc if footpads are active

//do {
//HubMotor.writeMicroseconds(VESCPWM); //sending pwm to the vesc..
analogWrite (VESCPWM, 3);
analogWrite (VESCPWM, A6);
//  Serial.println("Corrected Angle:  " + String(CorrectedAngle)); //remove all prints before running.
//  Serial.println("Output: ");
Serial.println(VESCPWM);
//  Serial.println ("footpadV" );
//  Serial.println(footpad1);
 delay(10);
  }
//while ((footpadV1 > 1000) && (footpadV2 > 1000)); //if both footpads are pressed. 
//  }
