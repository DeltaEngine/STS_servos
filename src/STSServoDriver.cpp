#include "STSServoDriver.h"
#include "HardwareSerial.h"
#include <cstdio>
#include <cstring>
namespace STSRegisters
{
	byte const FIRMWARE_MAJOR = 0x00;
	byte const FIRMWARE_MINOR = 0x01;
	byte const SERVO_MAJOR = 0x03;
	byte const SERVO_MINOR = 0x04;
	byte const ID = 0x05;
	byte const BAUDRATE = 0x06;
	byte const RESPONSE_DELAY = 0x07;
	byte const RESPONSE_STATUS_LEVEL = 0x08;
	byte const MINIMUM_ANGLE = 0x09;
	byte const MAXIMUM_ANGLE = 0x0B;
	byte const MAXIMUM_TEMPERATURE = 0x0D;
	byte const MAXIMUM_VOLTAGE = 0x0E;
	byte const MINIMUM_VOLTAGE = 0x0F;
	byte const MAXIMUM_TORQUE = 0x10;
	byte const UNLOADING_CONDITION = 0x13;
	byte const LED_ALARM_CONDITION = 0x14;
	byte const POS_PROPORTIONAL_GAIN = 0x15;
	byte const POS_DERIVATIVE_GAIN = 0x16;
	byte const POS_INTEGRAL_GAIN = 0x17;
	byte const MINIMUM_STARTUP_FORCE = 0x18;
	byte const CK_INSENSITIVE_AREA = 0x1A;
	byte const CCK_INSENSITIVE_AREA = 0x1B;
	byte const CURRENT_PROTECTION_TH = 0x1C;
	byte const ANGULAR_RESOLUTION = 0x1E;
	byte const POSITION_CORRECTION = 0x1F;
	byte const OPERATION_MODE = 0x21;
	byte const TORQUE_PROTECTION_TH = 0x22;
	byte const TORQUE_PROTECTION_TIME = 0x23;
	byte const OVERLOAD_TORQUE = 0x24;
	byte const SPEED_PROPORTIONAL_GAIN = 0x25;
	byte const OVERCURRENT_TIME = 0x26;
	byte const SPEED_INTEGRAL_GAIN = 0x27;
	byte const TORQUE_SWITCH = 0x28;
	byte const TARGET_ACCELERATION = 0x29;
	byte const TARGET_POSITION = 0x2A;
	byte const RUNNING_TIME = 0x2C;
	byte const RUNNING_SPEED = 0x2E;
	byte const TORQUE_LIMIT = 0x30;
	byte const WRITE_LOCK = 0x37;
	byte const CURRENT_POSITION = 0x38;
	byte const CURRENT_SPEED = 0x3A;
	byte const CURRENT_DRIVE_VOLTAGE = 0x3C;
	byte const CURRENT_VOLTAGE = 0x3E;
	byte const CURRENT_TEMPERATURE = 0x3F;
	byte const ASYNCHRONOUS_WRITE_ST = 0x40;
	byte const STATUS = 0x41;
	byte const MOVING_STATUS = 0x42;
	byte const CURRENT_CURRENT = 0x45;
};

namespace instruction
{
	byte const PING_ = 0x01;
	byte const READ = 0x02;
	byte const WRITE = 0x03;
	byte const REGWRITE = 0x04;
	byte const ACTION = 0x05;
	byte const SYNCWRITE = 0x83;
	byte const RESET = 0x06;
};

STSServoDriver::STSServoDriver() {}

bool STSServoDriver::init(byte const &dirPin, HardwareSerial *serialPort, long const &baudRate)
{
#if defined(SERIAL_H) || defined(HardwareSerial_h)
	if (serialPort == nullptr) serialPort = &Serial;
#endif
	lastError_[0] = 0;
	if (!serialPort || baudRate <= 0) return fail(0, 0, "Invalid serial port or baud rate");
	port_ = serialPort;
	port_->begin(baudRate);
	port_->setTimeout(2);
	dirPin_ = dirPin;
	if (this->dirPin_ < 255)
	{
		pinMode(dirPin_, OUTPUT);
	}

	for (int i = 0; i < 256; i++) servoType_[i] = ServoType::UNKNOWN;

	// Initialize bus only; caller discovers configured servo IDs.
	return true;
}

bool STSServoDriver::init(HardwareSerial *serialPort, long const &baudRate)
{
	return this->init(255, serialPort, baudRate);
}

bool STSServoDriver::ping(byte const &servoId)
{
	lastError_[0] = 0;
	register_ = 0;
	if (servoId >= 254) return fail(servoId, 0, "ID must be 0..253");
	byte response = 0;
	if (sendMessage(servoId, instruction::PING_, 0, &response) != 6)
		return fail(servoId, 0, "Failed to send ping");
	if (receiveMessage(servoId, 1, &response) < 0) return false;
	return true;
}

bool STSServoDriver::setId(byte const &oldServoId, byte const &newServoId)
{
	if (servoType_[oldServoId] == ServoType::UNKNOWN)
	{
		if (!determineServoType(oldServoId)) return false;
	}

	if (oldServoId >= 254 || newServoId >= 254)
		return fail(oldServoId, STSRegisters::ID, "ID must be 0..253");
	if (ping(newServoId)) return fail(newServoId, STSRegisters::ID, "ID already in use");

	unsigned char lockRegister = STSRegisters::WRITE_LOCK;
	if (servoType_[oldServoId] == ServoType::SCS)
	{
		lockRegister = STSRegisters::TORQUE_LIMIT; // On SCS, this has been remapped.
	}
	// Unlock EEPROM
	if (!writeRegister(oldServoId, lockRegister, 0)) return false;
	delay(5);
	// Write new ID
	if (!writeRegister(oldServoId, STSRegisters::ID, newServoId)) return false;
	// Lock EEPROM
	delay(5);
	if (!writeRegister(newServoId, lockRegister, 1)) return false;
	// Give it some time to change id.
	bool hasPing = false;
	int nIter = 0;
	while (!hasPing && nIter < 10)
	{
		++nIter;
		delay(50);
		hasPing = ping(newServoId);
	}
	if (hasPing)
	{
		// Update servo type cache.
		servoType_[newServoId] = servoType_[oldServoId];
		servoType_[oldServoId] = ServoType::UNKNOWN;
	}
	return hasPing;
}

bool STSServoDriver::setPositionOffset(byte const &servoId, int const &offset, bool persist)
{
	if (!checkRange(servoId, STSRegisters::POSITION_CORRECTION, offset, -2047, 2047)) return false;
	if (servoType_[servoId] == UNKNOWN && !determineServoType(servoId)) return false;
	if (servoType_[servoId] != STS)
		return fail(servoId, STSRegisters::POSITION_CORRECTION, "Offset requires an STS servo");
	if (!setWriteLocked(servoId, false)) return false;
	const uint16_t encoded = offset < 0 ? uint16_t(-offset) | 0x0800 : uint16_t(offset);
	const byte parameters[] = {byte(encoded), byte(encoded >> 8)};
	if (!writeRegisters(servoId, STSRegisters::POSITION_CORRECTION, 2, parameters)) return false;
	delay(10);
	return !persist || setWriteLocked(servoId, true);
}

int STSServoDriver::getCurrentPosition(byte const &servoId)
{
	return readTwoBytesRegister(servoId, STSRegisters::CURRENT_POSITION);
}

int STSServoDriver::getCurrentSpeed(byte const &servoId)
{
	return readTwoBytesRegister(servoId, STSRegisters::CURRENT_SPEED);
}

int STSServoDriver::getCurrentTemperature(byte const &servoId)
{
	const int value = readRegister(servoId, STSRegisters::CURRENT_TEMPERATURE);
	return lastError_[0] ? -1 : value;
}

float STSServoDriver::getCurrentCurrent(byte const &servoId)
{
	int16_t current = readTwoBytesRegister(servoId, STSRegisters::CURRENT_CURRENT);
	return current * 0.0065;
}

bool STSServoDriver::isMoving(byte const &servoId)
{
	const byte value = readRegister(servoId, STSRegisters::MOVING_STATUS);
	return !lastError_[0] && value != 0;
}

bool STSServoDriver::setTargetPosition(
	byte const &servoId, int const &position, int const &speed, bool const &asynchronous)
{
	if (!checkRange(servoId, STSRegisters::TARGET_POSITION, position, -32766, 32766) ||
		!checkRange(servoId, STSRegisters::RUNNING_SPEED, speed, 0, 32766))
		return false;
	byte params[6] = {0, 0, // Position
		0, 0,									// Padding
		0, 0};								// Velocity
	if (!convertIntToBytes(servoId, position, &params[0]) ||
		!convertIntToBytes(servoId, speed, &params[4]))
		return false;
	return writeRegisters(
		servoId, STSRegisters::TARGET_POSITION, sizeof(params), params, asynchronous);
}

bool STSServoDriver::setTargetVelocity(
	byte const &servoId, int const &velocity, bool const &asynchronous)
{
	if (!checkRange(servoId, STSRegisters::RUNNING_SPEED, velocity, -32766, 32766)) return false;
	return writeTwoBytesRegister(servoId, STSRegisters::RUNNING_SPEED, velocity, asynchronous);
}

bool STSServoDriver::setTargetAcceleration(
	byte const &servoId, int const &acceleration, bool const &asynchronous)
{
	if (!checkRange(servoId, STSRegisters::TARGET_ACCELERATION, acceleration, 0, 254)) return false;
	return writeRegister(
		servoId, STSRegisters::TARGET_ACCELERATION, byte(acceleration), asynchronous);
}

bool STSServoDriver::setMode(unsigned char const &servoId, STSMode const &mode)
{
	if (mode != POSITION && mode != VELOCITY && mode != STEP)
		return fail(servoId, STSRegisters::OPERATION_MODE, "Mode must be position, velocity or step");
	return setByte(servoId, STSRegisters::OPERATION_MODE, int(mode), 0, 3);
}

bool STSServoDriver::trigerAction()
{
	byte noParam = 0;
	lastError_[0] = 0;
	int send = sendMessage(0xFE, instruction::ACTION, 0, &noParam);
	return send == 6 || fail(254, 0, "Failed to send action");
}

int STSServoDriver::sendMessage(
	byte const &servoId, byte const &commandID, byte const &paramLength, byte *parameters)
{
	if (!port_)
	{
		fail(servoId, 0, "Serial bus not initialized");
		return -1;
	}
	while (port_->available() > 0)
	{
		port_->read();
	}
	byte message[256];
	byte checksum = servoId + paramLength + 2 + commandID;
	message[0] = 0xFF;
	message[1] = 0xFF;
	message[2] = servoId;
	message[3] = paramLength + 2;
	message[4] = commandID;
	for (int i = 0; i < paramLength; i++)
	{
		message[5 + i] = parameters[i];
		checksum += parameters[i];
	}
	message[5 + paramLength] = ~checksum;
	if (this->dirPin_ < 255)
	{
		digitalWrite(dirPin_, HIGH);
	}
	int ret = port_->write(message, 6 + paramLength);
	port_->flush();
	if (this->dirPin_ < 255)
	{
		digitalWrite(dirPin_, LOW);
	}
	delayMicroseconds(200);
	return ret;
}

bool STSServoDriver::writeRegisters(byte const &servoId, byte const &startRegister,
	byte const &writeLength, byte const *parameters, bool const &asynchronous)
{
	lastError_[0] = 0;
	if (!port_) return fail(servoId, startRegister, "Serial bus not initialized");
	if (servoId > 254 || !writeLength || writeLength > 248)
		return fail(servoId, startRegister, "Invalid ID or packet length");
	byte packet[256];
	packet[0] = startRegister;
	for (unsigned index = 0; index < writeLength; ++index) packet[index + 1] = parameters[index];
	if (sendMessage(servoId, asynchronous ? instruction::REGWRITE : instruction::WRITE,
				writeLength + 1, packet) != writeLength + 7)
		return fail(servoId, startRegister, "Failed to send write");
	if (servoId == 254) return true;
	byte status = 0;
	register_ = startRegister;
	port_->setTimeout(startRegister < STSRegisters::TORQUE_SWITCH ? 20 : 2);
	const int result = receiveMessage(servoId, 1, &status);
	port_->setTimeout(2);
	return result == 0;
}

bool STSServoDriver::writeRegister(
	byte const &servoId, byte const &registerId, byte const &value, bool const &asynchronous)
{
	return writeRegisters(servoId, registerId, 1, &value, asynchronous);
}

bool STSServoDriver::writeTwoBytesRegister(
	byte const &servoId, byte const &registerId, int16_t const &value, bool const &asynchronous)
{
	byte params[2] = {0, 0};
	if (!convertIntToBytes(servoId, value, params)) return false;
	return writeRegisters(servoId, registerId, 2, params, asynchronous);
}

byte STSServoDriver::readRegister(byte const &servoId, byte const &registerId)
{
	byte result = 0;
	int rc = readRegisters(servoId, registerId, 1, &result);
	if (rc < 0) return 255;
	return result;
}

int16_t STSServoDriver::readTwoBytesRegister(byte const &servoId, byte const &registerId)
{
	lastError_[0] = 0;
	if (servoId >= 254)
	{
		fail(servoId, registerId, "Read ID must be 0..253");
		return 0;
	}
	if (servoType_[servoId] == ServoType::UNKNOWN)
	{
		if (!determineServoType(servoId)) return 0;
	}

	unsigned char result[2] = {0, 0};
	int16_t value = 0;
	int16_t signedValue = 0;
	int rc = readRegisters(servoId, registerId, 2, result);
	if (rc < 0) return 0;
	switch (servoType_[servoId])
	{
	case ServoType::SCS:
		value = static_cast<int16_t>(result[1] + (result[0] << 8));
		// Bit 10 is sign on SCS.
		signedValue = value & ~0x0400;
		if (value & 0x0400) signedValue = -signedValue;
		return signedValue;
	case ServoType::STS:
		value = static_cast<int16_t>(result[0] + (result[1] << 8));
		// Bit 15 is sign
		signedValue = value & ~0x8000;
		if (value & 0x8000) signedValue = -signedValue;
		return signedValue;
	default:
		return 0;
	}
}

int STSServoDriver::readRegisters(
	byte const &servoId, byte const &startRegister, byte const &readLength, byte *outputBuffer)
{
	lastError_[0] = 0;
	if (!port_)
	{
		fail(servoId, startRegister, "Serial bus not initialized");
		return -1;
	}
	if (servoId >= 254 || !readLength || readLength > 249)
	{
		fail(servoId, startRegister, "Invalid read ID or length");
		return -1;
	}
	byte parameters[] = {startRegister, readLength};
	if (sendMessage(servoId, instruction::READ, 2, parameters) != 8)
	{
		fail(servoId, startRegister, "Failed to send read");
		return -1;
	}
	byte response[256];
	register_ = startRegister;
	const int result = receiveMessage(servoId, readLength + 1, response);
	if (result < 0) return result;
	for (unsigned index = 0; index < readLength; ++index) outputBuffer[index] = response[index + 1];
	return 0;
}

int STSServoDriver::receiveMessage(byte const &servoId, byte const &readLength, byte *outputBuffer)
{
	if (this->dirPin_ < 255)
	{
		digitalWrite(dirPin_, LOW);
	}

	byte result[256];
	size_t rd = port_->readBytes(result, readLength + 5);
	if (rd != (unsigned short)(readLength + 5))
	{
		fail(servoId, register_, "Servo response timed out");
		return -1;
	}
	// Check message integrity
	if (result[0] != 0xFF || result[1] != 0xFF || result[2] != servoId ||
		result[3] != readLength + 1)
	{
		fail(servoId, register_, "Invalid response header");
		return -2;
	}
	byte checksum = 0;
	for (int i = 2; i < readLength + 4; i++) checksum += result[i];
	checksum = ~checksum;
	if (result[readLength + 4] != checksum)
	{
		fail(servoId, register_, "Invalid response checksum");
		return -3;
	}
	if (result[4])
	{
		failHardware(servoId, result[4]);
		return -4;
	}

	// Copy result to output buffer
	for (int i = 0; i < readLength; i++) outputBuffer[i] = result[i + 4];
	return 0;
}

bool STSServoDriver::convertIntToBytes(byte const &servoId, int const &value, byte result[2])
{
	uint16_t servoValue = 0;
	if (servoType_[servoId] == ServoType::UNKNOWN)
	{
		if (servoId != 254 && !determineServoType(servoId)) return false;
	}

	// Handle different servo type.
	switch (servoType_[servoId])
	{
	case ServoType::SCS:
		if (value < -1023 || value > 1023)
			return fail(servoId, STSRegisters::TARGET_POSITION, "SCS value outside -1023..1023");
		// Little endian ; byte 10 is sign.
		servoValue = abs(value);
		if (value < 0) servoValue = 0x0400 | servoValue;
		// Invert endianness
		servoValue = (servoValue >> 8) + ((servoValue & 0xFF) << 8);
		break;
	case ServoType::STS:
	default:
		servoValue = abs(value);
		if (value < 0) servoValue = 0x8000 | servoValue;
		break;
	}
	result[0] = static_cast<unsigned char>(servoValue & 0xFF);
	result[1] = static_cast<unsigned char>((servoValue >> 8) & 0xFF);
	return true;
}

bool STSServoDriver::setTargetPositions(
	byte const &numberOfServos, const byte servoIds[], const int positions[], const int speeds[])
{
	lastError_[0] = 0;
	if (!numberOfServos || numberOfServos > 35 || !servoIds || !positions || !speeds)
		return fail(254, STSRegisters::TARGET_POSITION, "Supply 1..35 servo targets");
	byte packet[247] = {STSRegisters::TARGET_POSITION, 6};
	for (unsigned index = 0; index < numberOfServos; ++index)
	{
		const byte id = servoIds[index];
		if (id >= 254) return fail(id, STSRegisters::ID, "Target ID must be 0..253");
		if (!checkRange(id, STSRegisters::TARGET_POSITION, positions[index], -32766, 32766) ||
			!checkRange(id, STSRegisters::RUNNING_SPEED, speeds[index], 0, 32766))
			return false;
		for (unsigned previous = 0; previous < index; ++previous)
			if (servoIds[previous] == id) return fail(id, STSRegisters::ID, "Duplicate target ID");
		byte *target = packet + 2 + index * 7;
		target[0] = id;
		if (!convertIntToBytes(id, positions[index], target + 1) ||
			!convertIntToBytes(id, speeds[index], target + 5))
			return false;
		target[3] = target[4] = 0;
	}
	const byte length = 2 + numberOfServos * 7;
	return sendMessage(254, instruction::SYNCWRITE, length, packet) == length + 6 ||
		fail(254, STSRegisters::TARGET_POSITION, "Failed to send targets");
}

bool STSServoDriver::determineServoType(byte const &servoId)
{
	const byte model = readRegister(servoId, STSRegisters::SERVO_MAJOR);
	if (lastError_[0]) return false;
	if (model != 9 && model != 5)
		return fail(servoId, STSRegisters::SERVO_MAJOR, "Unsupported servo model");
	servoType_[servoId] = model == 9 ? STS : SCS;
	return true;
}

static const char *registerName(byte address)
{
	switch (address)
	{
	case STSRegisters::ID:
		return "ID";
	case STSRegisters::RESPONSE_DELAY:
		return "response delay";
	case STSRegisters::MINIMUM_ANGLE:
		return "minimum angle";
	case STSRegisters::MAXIMUM_ANGLE:
		return "maximum angle";
	case STSRegisters::MAXIMUM_TEMPERATURE:
		return "maximum temperature";
	case STSRegisters::POS_PROPORTIONAL_GAIN:
		return "position proportional gain";
	case STSRegisters::POS_DERIVATIVE_GAIN:
		return "position derivative gain";
	case STSRegisters::CK_INSENSITIVE_AREA:
		return "clockwise dead band";
	case STSRegisters::CCK_INSENSITIVE_AREA:
		return "counterclockwise dead band";
	case STSRegisters::ANGULAR_RESOLUTION:
		return "angular resolution";
	case STSRegisters::POSITION_CORRECTION:
		return "position offset";
	case STSRegisters::OPERATION_MODE:
		return "operating mode";
	case STSRegisters::TORQUE_PROTECTION_TH:
		return "protection torque";
	case STSRegisters::TORQUE_PROTECTION_TIME:
		return "torque protection time";
	case STSRegisters::OVERLOAD_TORQUE:
		return "overload torque";
	case STSRegisters::OVERCURRENT_TIME:
		return "overcurrent time";
	case STSRegisters::TARGET_ACCELERATION:
		return "acceleration";
	case STSRegisters::TARGET_POSITION:
		return "target position";
	case STSRegisters::RUNNING_SPEED:
		return "velocity";
	case STSRegisters::TORQUE_LIMIT:
		return "torque limit";
	case STSRegisters::WRITE_LOCK:
		return "write lock";
	default:
		return "communication";
	}
}

bool STSServoDriver::fail(byte id, byte address, const char *reason)
{
	snprintf(lastError_, sizeof(lastError_), "Feetech servo %u %s: %s", unsigned(id),
		registerName(address), reason);
	return false;
}

void STSServoDriver::failHardware(byte id, byte flags)
{
	const char *names[] = {"voltage", "sensor", "temperature", "current", "angle", "overload"};
	snprintf(lastError_, sizeof(lastError_), "Feetech servo %u hardware fault:", unsigned(id));
	for (unsigned index = 0; index < 6; ++index)
		if (flags & (1 << index))
		{
			const size_t length = strlen(lastError_);
			snprintf(lastError_ + length, sizeof(lastError_) - length, " %s", names[index]);
		}
	if (flags & 0xC0)
	{
		const size_t length = strlen(lastError_);
		snprintf(lastError_ + length, sizeof(lastError_) - length, " unknown (0x%02X)", flags);
	}
}

bool STSServoDriver::checkRange(byte id, byte address, int value, int minimum, int maximum)
{
	lastError_[0] = 0;
	if (id > 254) return fail(id, address, "ID must be 0..254");
	if (value >= minimum && value <= maximum) return true;
	snprintf(lastError_, sizeof(lastError_), "Feetech servo %u %s: value %d outside %d..%d",
		unsigned(id), registerName(address), value, minimum, maximum);
	return false;
}

bool STSServoDriver::setByte(byte id, byte address, int value, int minimum, int maximum)
{
	if (!checkRange(id, address, value, minimum, maximum) ||
		!writeRegister(id, address, byte(value)))
		return false;
	if (address < STSRegisters::TORQUE_SWITCH) delay(10);
	return true;
}

bool STSServoDriver::setWord(byte id, byte address, int value, int minimum, int maximum)
{
	if (!checkRange(id, address, value, minimum, maximum) ||
		!writeTwoBytesRegister(id, address, int16_t(value)))
		return false;
	if (address < STSRegisters::TORQUE_SWITCH) delay(10);
	return true;
}

int STSServoDriver::getCurrentLoad(byte const &servoId)
{
	const int value = readTwoBytesRegister(servoId, STSRegisters::CURRENT_DRIVE_VOLTAGE);
	return lastError_[0] ? -1 : value & 1023;
}

int STSServoDriver::getTorqueLimit(byte const &servoId)
{
	const int value = readTwoBytesRegister(servoId, STSRegisters::TORQUE_LIMIT);
	if (lastError_[0]) return -1;
	if (!checkRange(servoId, STSRegisters::TORQUE_LIMIT, value, 0, 1000)) return -1;
	return value;
}

bool STSServoDriver::setTorqueLimit(byte const &servoId, int value)
{
	return setWord(servoId, STSRegisters::TORQUE_LIMIT, value, 0, 1000);
}

bool STSServoDriver::setMinimumAngle(byte const &servoId, int value)
{
	return setWord(servoId, STSRegisters::MINIMUM_ANGLE, value, 0, 4094);
}

bool STSServoDriver::setMaximumAngle(byte const &servoId, int value)
{
	return setWord(servoId, STSRegisters::MAXIMUM_ANGLE, value, 0, 4095);
}

bool STSServoDriver::setAngularResolution(byte const &servoId, int value)
{
	return setByte(servoId, STSRegisters::ANGULAR_RESOLUTION, value, 1, 100);
}

bool STSServoDriver::setResponseDelay(byte const &servoId, int value)
{
	return setByte(servoId, STSRegisters::RESPONSE_DELAY, value, 0, 254);
}

bool STSServoDriver::setMaximumTemperature(byte const &servoId, int value)
{
	return setByte(servoId, STSRegisters::MAXIMUM_TEMPERATURE, value, 0, 100);
}

bool STSServoDriver::setTorqueProtection(byte const &servoId, int value)
{
	return setByte(servoId, STSRegisters::TORQUE_PROTECTION_TH, value, 0, 100);
}

bool STSServoDriver::setOverloadTorque(byte const &servoId, int value)
{
	return setByte(servoId, STSRegisters::OVERLOAD_TORQUE, value, 0, 100);
}

bool STSServoDriver::setTorqueProtectionTime(byte const &servoId, int value)
{
	return setByte(servoId, STSRegisters::TORQUE_PROTECTION_TIME, value, 0, 254);
}

bool STSServoDriver::setOvercurrentTime(byte const &servoId, int value)
{
	return setByte(servoId, STSRegisters::OVERCURRENT_TIME, value, 0, 254);
}

bool STSServoDriver::setClockwiseDeadBand(byte const &servoId, int value)
{
	return setByte(servoId, STSRegisters::CK_INSENSITIVE_AREA, value, 0, 32);
}

bool STSServoDriver::setCounterclockwiseDeadBand(byte const &servoId, int value)
{
	return setByte(servoId, STSRegisters::CCK_INSENSITIVE_AREA, value, 0, 32);
}

bool STSServoDriver::setPositionProportionalGain(byte const &servoId, int value)
{
	return setByte(servoId, STSRegisters::POS_PROPORTIONAL_GAIN, value, 0, 254);
}

bool STSServoDriver::setPositionDerivativeGain(byte const &servoId, int value)
{
	return setByte(servoId, STSRegisters::POS_DERIVATIVE_GAIN, value, 0, 254);
}

bool STSServoDriver::setTorqueEnabled(byte const &servoId, bool enabled)
{
	return writeRegister(servoId, STSRegisters::TORQUE_SWITCH, enabled ? 1 : 0);
}

bool STSServoDriver::setWriteLocked(byte const &servoId, bool locked)
{
	return writeRegister(servoId, STSRegisters::WRITE_LOCK, locked ? 1 : 0);
}
