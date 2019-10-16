#include "Waveshare_10Dof-D.h"
bool gbSenserConnectState = false;

void setup() {
  // put your setup code here, to run once:
  bool bRet;
  IMU_EN_SENSOR_TYPE enMotionSensorType, enPressureType;
  Serial.begin(115200);

  imuInit(&enMotionSensorType, &enPressureType);
  if(IMU_EN_SENSOR_TYPE_ICM20948 == enMotionSensorType)
  {
    Serial.println("Motion sersor is ICM-20948");
  }
  else
  {
    Serial.println("Motion sersor NULL");
  }
  if(IMU_EN_SENSOR_TYPE_BMP280 == enPressureType)
  {
    Serial.println("Pressure sersor is BMP280");
  }
  else
  {
    Serial.println("Pressure sersor NULL");
  }
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  IMU_ST_ANGLES_DATA stAngles;
  IMU_ST_SENSOR_DATA stGyroRawData;
  IMU_ST_SENSOR_DATA stAccelRawData;
  IMU_ST_SENSOR_DATA stMagnRawData;
  int32_t s32PressureVal = 0, s32TemperatureVal = 0, s32AltitudeVal = 0;
  
  imuDataGet( &stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
  pressSensorDataGet(&s32TemperatureVal, &s32PressureVal, &s32AltitudeVal);
Serial.print(stAngles.fRoll);
Serial.print(",");
Serial.print(stAngles.fPitch);
Serial.print(",");
Serial.println(stAngles.fYaw);

/*
  Serial.println();
  Serial.println("/-------------------------------------------------------------/");
  Serial.print("Roll : "); Serial.print(stAngles.fRoll);
  Serial.print("    Pitch : "); Serial.print(stAngles.fPitch);
  Serial.print("    Yaw : "); Serial.print(stAngles.fYaw);
  Serial.println();
  
  Serial.print("Acceleration: X : "); Serial.print(stAccelRawData.s16X);
  Serial.print("    Acceleration: Y : "); Serial.print(stAccelRawData.s16Y);
  Serial.print("    Acceleration: Z : "); Serial.print(stAccelRawData.s16Z);
  Serial.println();
  Serial.print("Gyroscope: X : "); Serial.print(stGyroRawData.s16X);
  Serial.print("       Gyroscope: Y : "); Serial.print(stGyroRawData.s16Y);
  Serial.print("       Gyroscope: Z : "); Serial.print(stGyroRawData.s16Z);
  Serial.println();
  Serial.print("Magnetic: X : "); Serial.print(stMagnRawData.s16X);
  Serial.print("      Magnetic: Y : "); Serial.print(stMagnRawData.s16Y);
  Serial.print("      Magnetic: Z : "); Serial.print(stMagnRawData.s16Z);
  Serial.println();
  Serial.print("Pressure : "); Serial.print((float)s32PressureVal / 100);
  Serial.print("     Altitude : "); Serial.print((float)s32AltitudeVal / 100);
  Serial.println();  
  Serial.print("Temperature : "); Serial.print((float)s32TemperatureVal / 100);
  Serial.println();  */
  delay(100);
}
