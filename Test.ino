/*
 Name:		Test.ino
 Created:	3/18/2023 8:30:34 PM
 Author:	ashut
*/

// the setup function runs once when you press reset or power the board
#include "IMPU.h"
#include "SampleConfig.h"
SampleConfig config;
IMPU& mpu = config.getMPU();

void setup() {
	Serial.begin(115200);
	while (!Serial);
	mpu.Init();
}

// the loop function runs over and over again until power down or reset
void loop() {
	float* y, *p, *r;
	mpu.getYawPitchRoll(y,p,r);
	Serial.print("Yaw : ");
	Serial.print(*y);
	Serial.print("\tPitch : ");
	Serial.print(*p);
	Serial.print("\tRoll : ");
	Serial.println(*r);
}
