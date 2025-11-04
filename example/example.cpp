#include <Arduino.h>
#include "IMUKernel.h"

HardwareSerial serial(2);
il::IMUKernel imu(serial);

void callbackFunc(const uint8_t id, const void* data)
{
	switch (id) {
	case il::MSG_ORIENTATION: {
		const il::OrientationData* d = reinterpret_cast<const il::OrientationData*>(data);
		Serial.printf("IMU Orientation[ "
			"heading=%.02f, pitch=%.02f, roll=%.02f, "
			"gyroX=%.01f, gyroY=%.01f, gyroZ=%.01f, "
			"accX=%.03f, accY=%.03f, accZ=%.03f, "
			"t=%.01f]\n",
			static_cast<float>(d->heading) / 100, static_cast<float>(d->pitch) / 100, static_cast<float>(d->roll) / 100,
			static_cast<float>(d->gyroX) / il::KG, static_cast<float>(d->gyroY) / il::KG, static_cast<float>(d->gyroZ) / il::KG,
			static_cast<double>(d->accX) / il::KA, static_cast<double>(d->accY) / il::KA, static_cast<double>(d->accZ) / il::KA,
			static_cast<float>(d->temperature) / 10
		);
	} break;
	}
}

void setup()
{
  Serial.begin(115200);
  serial.begin(115200, SERIAL_8N1, 16, 17);

  if (!imu.begin()) {
    Serial.printf("IMU Begin Error!\n");
    while (true) {}
  }
  
  auto info = imu.getDeviceInfo();
  Serial.printf("IMU type: %u\n", info.imuType);
  Serial.printf("IMU serial number: %s\n", info.idSN);
  Serial.printf("IMU firmware version: %s\n", info.idFW);
  Serial.printf("IMU data rate: %u Hz\n", imu.dataRate());
  uint32_t bps = 0;
  switch (imu.baudRate()) {
  case il::BPS_DEFAULT: bps = 115200; break;
  case il::BPS_4800: bps = 4800; break;
  case il::BPS_9600: bps = 9600; break;
  case il::BPS_14400: bps = 14400; break;
  case il::BPS_19200: bps = 19200; break;
  case il::BPS_38400: bps = 38400; break;
  case il::BPS_57600: bps = 57600; break;
  case il::BPS_115200: bps = 115200; break;
  case il::BPS_230400: bps = 230400; break;
  case il::BPS_460800: bps = 460800; break;
  case il::BPS_921600: bps = 921600; break;
  case il::BPS_2000000: bps = 2000000; break;
  case il::BPS_375000: bps = 375000; break;
  case il::BPS_1843200: bps = 1843200; break;
  case il::BPS_3686400: bps = 3686400; break;
  case il::BPS_1000000: bps = 1000000; break;
  case il::BPS_4000000: bps = 4000000; break;
  }
  Serial.printf("IMU baud rate: %u bps\n", bps);
  Serial.printf("IMU auto start: %02X\n", imu.autoStart());
  Serial.printf("IMU output data: %s\n", (imu.averageOutputData() ? "Averaged" : "Instant"));
  Serial.printf("IMU initial alignment time: %u s\n", imu.initialAlignmentTime());
  Serial.printf("IMU alignment Angles[ yaw: %f, pitch: %f, roll: %f]\n", imu.alignYaw(), imu.alignPitch(), imu.alignRoll());

  imu.setCallback(&callbackFunc);
  imu.setDataRate(50);
  imu.provide(il::MSG_ORIENTATION);
}

float x, y, z, L = 0.0;
const float threshold = 1.001;

void loop()
{
  imu.update();
  // Detect motion
  const il::OrientationData& data = imu.getOrientationData();
  x = static_cast<float>(data.accX) / il::KA;
  y = static_cast<float>(data.accY) / il::KA;
  z = static_cast<float>(data.accZ) / il::KA;
  L = sqrtf(x * x + y * y + z * z);
  if (L > threshold) Serial.printf("Motion Detected on: %f\n", L);
}
