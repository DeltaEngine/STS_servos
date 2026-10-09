#pragma once
#include <Arduino.h>
enum STSMode
{
	POSITION = 0,
	VELOCITY = 1,
	STEP = 3
};
enum ServoType
{
	UNKNOWN = 0,
	STS = 1,
	SCS = 2
};
class STSServoDriver
{
public:
	STSServoDriver();
	bool init(
		byte const &dirPin, HardwareSerial *serialPort = nullptr, long const &baudRate = 1000000);
	bool init(HardwareSerial *serialPort = nullptr, long const &baudRate = 1000000);
	bool ping(byte const &servoId);
	bool setId(byte const &oldServoId, byte const &newServoId);
	bool setPositionOffset(byte const &servoId, int const &offset, bool persist = true);
	int getCurrentPosition(byte const &servoId);
	int getCurrentSpeed(byte const &servoId);
	int getCurrentTemperature(byte const &servoId);
	int getCurrentLoad(byte const &servoId);
	int getTorqueLimit(byte const &servoId);
	float getCurrentCurrent(byte const &servoId);
	bool isMoving(byte const &servoId);
	bool setTargetPosition(byte const &servoId, int const &position, int const &speed = 4095,
		bool const &asynchronous = false);
	bool setTargetVelocity(
		byte const &servoId, int const &velocity, bool const &asynchronous = false);
	bool setTargetAcceleration(
		byte const &servoId, int const &acceleration, bool const &asynchronous = false);
	bool setMode(byte const &servoId, STSMode const &mode);
	bool setTorqueLimit(byte const &servoId, int value);
	bool setTorqueEnabled(byte const &servoId, bool enabled);
	bool setWriteLocked(byte const &servoId, bool locked);
	bool setMinimumAngle(byte const &servoId, int value);
	bool setMaximumAngle(byte const &servoId, int value);
	bool setAngularResolution(byte const &servoId, int value);
	bool setResponseDelay(byte const &servoId, int value);
	bool setMaximumTemperature(byte const &servoId, int value);
	bool setTorqueProtection(byte const &servoId, int value);
	bool setOverloadTorque(byte const &servoId, int value);
	bool setTorqueProtectionTime(byte const &servoId, int value);
	bool setOvercurrentTime(byte const &servoId, int value);
	bool setClockwiseDeadBand(byte const &servoId, int value);
	bool setCounterclockwiseDeadBand(byte const &servoId, int value);
	bool setPositionProportionalGain(byte const &servoId, int value);
	bool setPositionDerivativeGain(byte const &servoId, int value);
	bool trigerAction();
	bool setTargetPositions(
		byte const &numberOfServos, const byte servoIds[], const int positions[], const int speeds[]);
	// Empty after success. Copy before another operation; one driver belongs to one bus task.
	const char *getLastError() const { return lastError_; }
private:
	bool fail(byte id, byte address, const char *reason);
	void failHardware(byte id, byte flags);
	bool checkRange(byte id, byte address, int value, int minimum, int maximum);
	bool setByte(byte id, byte address, int value, int minimum, int maximum);
	bool setWord(byte id, byte address, int value, int minimum, int maximum);
	int sendMessage(
		byte const &servoId, byte const &commandId, byte const &paramLength, byte *parameters);
	int receiveMessage(byte const &servoId, byte const &readLength, byte *outputBuffer);
	bool writeRegister(byte const &servoId, byte const &registerId, byte const &value,
		bool const &asynchronous = false);
	bool writeTwoBytesRegister(byte const &servoId, byte const &registerId, int16_t const &value,
		bool const &asynchronous = false);
	byte readRegister(byte const &servoId, byte const &registerId);
	int16_t readTwoBytesRegister(byte const &servoId, byte const &registerId);
	bool writeRegisters(byte const &servoId, byte const &startRegister, byte const &writeLength,
		byte const *parameters, bool const &asynchronous = false);
	int readRegisters(
		byte const &servoId, byte const &startRegister, byte const &readLength, byte *outputBuffer);
	bool convertIntToBytes(byte const &servoId, int const &value, byte result[2]);
	bool determineServoType(byte const &servoId);
	HardwareSerial *port_ = nullptr;
	byte dirPin_ = 255;
	byte register_ = 0;
	ServoType servoType_[256] = {};
	char lastError_[128] = {};
};
