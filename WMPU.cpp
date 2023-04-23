#include "WMPU.h"
#include <Wire.h>
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif


namespace dev {
    MPU6050 mpu;
}


void WMPU::Init()
{
        // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
        uint8_t devStatus;
        dev::mpu.initialize();
        dev::mpu.testConnection();
        devStatus = dev::mpu.dmpInitialize();

        guessOffsets();

        if (devStatus == 0) {
            dev::mpu.setDMPEnabled(true);

        }

}

void WMPU::getYawPitchRoll(float* y, float* p, float* r)
{
    uint8_t fifoBuffer[64];
    Quaternion q;
    if (dev::mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        dev::mpu.dmpGetQuaternion(&q, fifoBuffer);
        float q0 = q.w;
        float q1 = q.x;
        float q2 = q.y;
        float q3 = q.z;

        float yr = -atan2(-2 * q1 * q2 + 2 * q0 * q3, q2 * q2 - q3 * q3 - q1 * q1 + q0 * q0);
        float pr = asin(2 * q2 * q3 + 2 * q0 * q1);
        float rr = atan2(-2 * q1 * q3 + 2 * q0 * q2, q3 * q3 - q2 * q2 - q1 * q1 + q0 * q0);

        *y = yr * 180 / M_PI;
        *p = pr * 180 / M_PI;
        *r = rr * 180 / M_PI;

        /*y = q0;
        p = q1;
        r = q2;*/

    }

}

void WMPU::guessOffsets() {
    // supply your own gyro offsets here, scaled for min sensitivity
    dev::mpu.setXGyroOffset(220);
    dev::mpu.setYGyroOffset(76);
    dev::mpu.setZGyroOffset(-85);
    dev::mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
}