#include "STSServoDriver.h"
#include <cassert>
#include <cstdio>
#include <type_traits>
#include <utility>
template <class Driver, class = void> struct HasRawWrite : std::false_type
{
};
template <class Driver>
struct HasRawWrite<Driver,
	std::void_t<decltype(std::declval<Driver>().writeRegister(byte{}, byte{}, byte{}))>>
		: std::true_type
{
};
int main()
{
	HardwareSerial serial;
	STSServoDriver driver;
	assert(driver.init(255, &serial));
	assert(driver.getCurrentTemperature(1) == 70);
	assert(serial.packets.back()[6] == 1 && "Temperature must read exactly one byte");
	static_assert(!HasRawWrite<STSServoDriver>::value, "Raw writes must remain private");
	assert(!driver.getLastError()[0]);
	assert(driver.getCurrentPosition(1) == 0 && !driver.getLastError()[0]);
	assert(driver.setTorqueLimit(6, 450));
	assert(serial.packets.back()[5] == 48);
	assert(serial.packets.back()[3] == 5);
	assert(serial.packets.back()[6] == 194 && serial.packets.back()[7] == 1);
	assert(driver.setMinimumAngle(1, 300));
	assert(serial.packets.back()[5] == 9 && serial.packets.back()[3] == 5);
	assert(serial.packets.back()[6] == 44 && serial.packets.back()[7] == 1);
	assert(driver.setMaximumAngle(1, 0));
	assert(serial.packets.back()[5] == 11 && serial.packets.back()[3] == 5);
	assert(driver.setPositionOffset(1, -100, false));
	assert(serial.packets.back()[5] == 31);
	assert(serial.packets.back()[6] == 100 && serial.packets.back()[7] == 8);
	assert(driver.getCurrentLoad(6) == 450 && !driver.getLastError()[0]);
	assert(driver.getTorqueLimit(6) == 450 && serial.packets.back()[6] == 2);
	const auto sent = serial.packets.size();
	assert(!driver.setTorqueLimit(6, 1001));
	assert(strstr(driver.getLastError(), "1001") && strstr(driver.getLastError(), "0..1000"));
	assert(!driver.setTargetAcceleration(1, 300));
	assert(!driver.setPositionOffset(1, 2048));
	assert(!driver.setMode(1, STSMode(2)));
	assert(!driver.setTargetPosition(1, 32767));
	assert(!driver.setTargetVelocity(1, -32767));
	assert(serial.packets.size() == sent && "Invalid values must not reach the bus");
	serial.timeout = true;
	assert(driver.getCurrentTemperature(1) == -1);
	assert(strstr(driver.getLastError(), "timed out"));
	assert(!driver.isMoving(1) && driver.getLastError()[0]);
	assert(driver.getCurrentLoad(6) == -1);
	assert(!driver.setTorqueLimit(6, 450) && driver.getLastError()[0]);
	assert(strstr(driver.getLastError(), "torque limit"));
	serial.timeout = false;
	serial.corrupt = true;
	assert(driver.getCurrentTemperature(1) == -1);
	assert(strstr(driver.getLastError(), "checksum"));
	serial.corrupt = false;
	serial.fault = 4;
	assert(!driver.setTorqueLimit(6, 450));
	assert(strstr(driver.getLastError(), "hardware fault"));
	assert(strstr(driver.getLastError(), "temperature"));
	serial.fault = 0;
	serial.shortWrite = true;
	assert(!driver.setTargetPosition(1, 100));
	assert(strstr(driver.getLastError(), "send write"));
	serial.shortWrite = false;
	assert(driver.setTorqueLimit(6, 450) && !driver.getLastError()[0]);
	const byte ids[] = {1, 2};
	const int positions[] = {100, 200}, speeds[] = {100, 100};
	assert(driver.setTargetPositions(2, ids, positions, speeds));
	assert(serial.packets.back()[4] == 131 && serial.packets.back()[2] == 254);
	const byte duplicateIds[] = {1, 1};
	assert(!driver.setTargetPositions(2, duplicateIds, positions, speeds));
	assert(strstr(driver.getLastError(), "Duplicate"));
	const byte invalidIds[] = {1, 254};
	assert(!driver.setTargetPositions(2, invalidIds, positions, speeds));
	assert(strstr(driver.getLastError(), "ID"));
	HardwareSerial scsSerial;
	scsSerial.model = 5;
	scsSerial.speed = 0x0464;
	STSServoDriver scsDriver;
	assert(scsDriver.init(255, &scsSerial));
	assert(scsDriver.getCurrentSpeed(1) == -100);
	assert(scsDriver.setTargetVelocity(1, -100));
	assert(scsSerial.packets.back()[6] == 4 && scsSerial.packets.back()[7] == 100);
	assert(!scsDriver.setTargetVelocity(1, 1024));
	STSServoDriver uninitialized;
	assert(!uninitialized.setTorqueLimit(1, 450));
	assert(strstr(uninitialized.getLastError(), "not initialized"));
	puts("STS driver checks passed");
}
