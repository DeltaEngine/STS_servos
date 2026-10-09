#pragma once
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <vector>
using byte = uint8_t;
constexpr int OUTPUT = 1, HIGH = 1, LOW = 0;
inline void pinMode(byte, int) {}
inline void digitalWrite(byte, int) {}
inline void delay(int) {}
inline void delayMicroseconds(int) {}
class HardwareSerial
{
public:
	std::vector<std::vector<byte>> packets;
	std::vector<byte> pending, response;
	bool timeout = false, corrupt = false, shortWrite = false;
	byte fault = 0, model = 9;
	int position = 0, speed = 0;
	void begin(long) {}
	void setTimeout(int) {}
	int available() { return int(response.size()); }
	int read()
	{
		if (response.empty()) return -1;
		byte value = response.front();
		response.erase(response.begin());
		return value;
	}
	size_t write(byte value)
	{
		pending.push_back(value);
		return 1;
	}
	size_t write(const byte *values, size_t count)
	{
		pending.insert(pending.end(), values, values + count);
		return shortWrite ? 0 : count;
	}
	void flush()
	{
		packets.push_back(pending);
		const auto packet = pending;
		pending.clear();
		response.clear();
		if (timeout || packet[2] == 254) return;
		std::vector<byte> values;
		if (packet[4] == 2)
		{
			int value = packet[5] == 3 ? model : packet[5] == 63 ? 70 : 0;
			if (packet[5] == 60) value = 0x0400 | 450;
			if (packet[5] == 48) value = 450;
			if (packet[5] == 56) value = position;
			if (packet[5] == 58) value = speed;
			if (model == 5 && packet[6] == 2) value = (value >> 8) | ((value & 255) << 8);
			for (byte index = 0; index < packet[6]; ++index)
				values.push_back(byte(value >> (index * 8)));
		}
		response = {255, 255, packet[2], byte(values.size() + 2), fault};
		response.insert(response.end(), values.begin(), values.end());
		byte checksum = 0;
		for (size_t index = 2; index < response.size(); ++index) checksum += response[index];
		response.push_back(byte(~checksum) ^ (corrupt ? 1 : 0));
	}
	size_t readBytes(byte *buffer, size_t count)
	{
		size_t readCount = response.size() < count ? response.size() : count;
		memcpy(buffer, response.data(), readCount);
		response.erase(response.begin(), response.begin() + readCount);
		return readCount;
	}
};
inline HardwareSerial Serial;
