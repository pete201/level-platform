#include <MPU9250.h>        // https://github.com/hideakitai/MPU9250
#include <Servo.h>      // Standard Servo library
#include <Ticker.h>     // include the ticker library 

// create instance of MPU9250
MPU9250 mpu;
// float yaw;
// float pitch;
// float roll;

// Creates servo instances of the Servo library
Servo servo1;
Servo servo2;

// Creates ticker1 instance of the Ticker library
Ticker ticker1;

// Ticker1 is set up in setup()
volatile bool writeSerial = false;
void writeToSerial() {
    writeSerial = true;
}


void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(1000);

    if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }

    // calibrate anytime you want to
    Serial.println("Accel Gyro calibration will start in 2sec.");
    Serial.println("Please leave the device still on the flat plane.");
    mpu.verbose(true);
    delay(2000);
    mpu.calibrateAccelGyro();

    Serial.println("Mag calibration will start in 3sec.");
    Serial.println("Please Wave device in a figure eight until done.");
    delay(3000);
    mpu.calibrateMag();

    print_calibration();
    mpu.verbose(false);

    // ticker1 is used to trigger write to serial periodically
    ticker1.attach_ms(100, writeToSerial); // Add ISR Function

    // Servo 1 is on Pin 13(D7), 2 is 15(D8)
    servo1.attach(13);
    // servo2.attach(15);
    
}

void loop() {
    if (mpu.update()) {

        // print to the serial port each time Ticker1 triggers:
        if (writeSerial)
        {
            writeSerial = false;
            print_roll_pitch_yaw();

            // Move the servos to follow the pitch and roll pos

            servo1.write((mpu.getPitch()) +90);
            //servo2.write(-(roll * 180/M_PI) +90);
        }
    }
}

void print_roll_pitch_yaw() {
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(mpu.getYaw(), 2);
    Serial.print(", ");
    Serial.print(mpu.getPitch(), 2);
    Serial.print(", ");
    Serial.println(mpu.getRoll(), 2);
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
