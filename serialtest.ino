#include "MPU9250.h"

MPU9250 mpu;
//float velocity = 0;
//float displacement = 0;
float interval = 0;
char start_instru_1 = 1;
char start_instru_2 = 9;
char start_instru_3 = 6;

void convFloatToByteArr(float val, uint8_t* byteArr)
{
  union{
    float floatVal;
    uint8_t byteArr[4];
  }uFloatByteArr;

  uFloatByteArr.floatVal = val;
  memcpy(byteArr, uFloatByteArr.byteArr, 4);
}

void send_float_data(float val)
{
  float hypoTemp = val;
  uint8_t byteArrTemp[4];

  convFloatToByteArr(hypoTemp, &byteArrTemp[0]);
  Serial.write(byteArrTemp, 4);
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(2000);
    if (!mpu.setup(0x68)) {  
        while (1) {
            delay(2000);
        }
    }

    /*
    // calibrate anytime you want to
    Serial.println("Accel Gyro calibration will start in 5sec.");
    Serial.println("Please leave the device still on the flat plane.");
    mpu.verbose(true);
    delay(5000);
    mpu.calibrateAccelGyro();

    Serial.println("Mag calibration will start in 5sec.");
    Serial.println("Please Wave device in a figure eight until done.");
    delay(5000);
    mpu.calibrateMag();

    print_calibration();
    mpu.verbose(false);

    Serial.println("Please close the serial port.");
    delay(3000);

    Serial.write(start_instru_1);
    Serial.write(start_instru_2);
    Serial.write(start_instru_3);
    */
}

void loop() {
    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        interval = millis() - prev_ms;
        if (interval > 50) {
            //velocity = velocity + interval * mpu.getAccX();
            //displacement = displacement + interval/1000 * velocity;
          
            send_float_data(millis());
            send_float_data(mpu.getAccX());
            send_float_data(mpu.getAccY());
            send_float_data(mpu.getAccZ());
            send_float_data(mpu.getMagX());
            send_float_data(mpu.getMagY());
            send_float_data(mpu.getMagZ());
            send_float_data(mpu.getGyroX());
            send_float_data(mpu.getGyroY());
            send_float_data(mpu.getGyroZ());
            send_float_data(mpu.getRoll());
            send_float_data(mpu.getPitch());
            send_float_data(mpu.getYaw());


            
            //send_float_data(velocity);
            //send_float_data(displacement);

            prev_ms = millis();
        }
    }
}



void print_calibration() {
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(mpu.getMagBiasX());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasY());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(mpu.getMagScaleX());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleY());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleZ());
    Serial.println();
}
