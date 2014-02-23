#include "OneWire.h"
#include <Wire.h>

OneWire::OneWire()
{
	OneWire(0);
}

OneWire::OneWire(uint8_t address)
{
	mAddress = 0x18 | address;
	Wire.begin();
}

uint8_t OneWire::getAddress()
{
	return mAddress;
}

uint8_t OneWire::getError()
{
	return mError;
}

void OneWire::begin()
{
	Wire.beginTransmission(mAddress);
}

uint8_t OneWire::end()
{
	return Wire.endTransmission();
}

void OneWire::writeByte(uint8_t data)
{
	Wire.write(data); 
}

uint8_t OneWire::readByte()
{
	Wire.requestFrom(mAddress,1u);
	return Wire.read();
}

uint8_t OneWire::checkPresence()
{
	begin();
	return end();
}

void OneWire::deviceReset()
{
	begin();
	write(DS2482_COMMAND_RESET);
	end();
}

void OneWire::setReadPointer(uint8_t readPointer)
{
	begin();
	writeByte(DS2482_COMMAND_SRP);
	writeByte(readPointer);
	end();
}

uint8_t OneWire::readStatus()
{
	setReadPointer(DS2482_POINTER_STATUS);
	return readByte();
}

uint8_t OneWire::readData()
{
	setReadPointer(DS2482_POINTER_DATA);
	return readByte();
}

uint8_t OneWire::waitOnBusy()
{
	uint8_t status;

	int testCount = 2000;

	for(int i=1000; i>0; i--)
	{
		status = readStatus();
		if (!(status & DS2482_STATUS_BUSY))
			break;
		delayMicroseconds(20);
	}

	if (status & DS2482_STATUS_BUSY)
		mError = 1;

	return status;
}

uint8_t OneWire::readConfig()
{
	setReadPointer(DS2482_POINTER_CONFIG);
	return readByte();
}

void OneWire::writeConfig(uint8_t config)
{
	waitOnBusy();
	begin();
	writeByte(DS2482_COMMAND_WRITECONFIG);
	writeByte(config | (~config)<<4);
	end();
	
	if (readByte() != config)
		mError = 1;
}

uint8_t OneWire::wireReset()
{
	waitOnBusy();
	begin();
	writeByte(DS2482_COMMAND_RESETWIRE);
	end();

	uint8_t status = waitOnBusy();

	return status & DS2482_STATUS_PPD ? true : false;
}

void OneWire::wireWriteByte(uint8_t data)
{
	waitOnBusy();
	begin();
	writeByte(DS2482_COMMAND_WRITEBYTE);
	writeByte(data);
	end();
}

uint8_t OneWire::wireReadByte()
{
	waitOnBusy();
	begin();
	writeByte(DS2482_COMMAND_READBYTE);
	end();
	waitOnBusy();
	return readData();
}

void OneWire::wireWriteBit(uint8_t data)
{
	waitOnBusy();
	begin();
	writeByte(DS2482_COMMAND_SINGLEBIT);
	writeByte(data ? 0x80 : 0x00);
	end();
}

uint8_t OneWire::wireReadBit()
{
	wireWriteBit(1);
	uint8_t status = waitOnBusy();
	return status & DS2482_STATUS_SBR ? 1 : 0;
}

void OneWire::wireSkip()
{
	wireWriteByte(WIRE_COMMAND_SKIP);
}

void OneWire::wireSelect(const uint8_t rom[8])
{
	wireWriteByte(WIRE_COMMAND_SELECT);
	for (int i=0;i<8;i++)
		wireWriteByte(rom[i]);
}

void OneWire::wireResetSearch()
{
	searchLastDiscrepancy = 0;
	searchLastDeviceFlag = 0;

	for (int i = 0; i < 8; i++)
	{
		searchAddress[i] = 0;
	}

}

uint8_t OneWire::wireSearch(uint8_t *address)
{
	uint8_t direction;
	uint8_t last_zero=0;

	if (searchLastDeviceFlag)
		return 0;

	if (!wireReset())
		return 0;

	waitOnBusy();

	wireWriteByte(WIRE_COMMAND_SEARCH);

	for(uint8_t i=0;i<64;i++)
	{
		int searchByte = i / 8; 
		int searchBit = 1 << i % 8;

		if (i < searchLastDiscrepancy)
			direction = searchAddress[searchByte] & searchBit;
		else
			direction = i == searchLastDiscrepancy;

		waitOnBusy();
		begin();
		writeByte(DS2482_COMMAND_TRIPLET);
		writeByte(direction ? 0x80 : 0x00);
		end();

		uint8_t status = waitOnBusy();

		uint8_t id = status & DS2482_STATUS_SBR;
		uint8_t comp_id = status & DS2482_STATUS_TSB;
		direction = status & DS2482_STATUS_DIR;

		if (id && comp_id)
		{
			return 0;
		}
		else
		{
			if (!id && !comp_id && !direction)
			{
				last_zero = i;
			}
		}

		if (direction)
			searchAddress[searchByte] |= searchBit;
		else
			searchAddress[searchByte] &= ~searchBit;

	}

	searchLastDiscrepancy = last_zero;

	if (!last_zero)
		searchLastDeviceFlag = 1;

	for (uint8_t i=0; i<8; i++)
		address[i] = searchAddress[i];

	return 1;
}

uint8_t OneWire::crc8(const uint8_t *addr, uint8_t len)
{
	uint8_t crc = 0;
	
	while (len--) {
		uint8_t inbyte = *addr++;
		for (uint8_t i = 8; i; i--) {
			uint8_t mix = (crc ^ inbyte) & 0x01;
			crc >>= 1;
			if (mix) crc ^= 0x8C;
			inbyte >>= 1;
		}
	}
	return crc;
}

void OneWire::reset_search()
{
	wireResetSearch();
}

uint8_t OneWire::search(uint8_t *newAddr)
{
	return wireSearch(newAddr);
}

uint8_t OneWire::reset(void)
{
	return wireReset();
}

void OneWire::select(const uint8_t rom[8])
{
	wireSelect(rom);
}

void OneWire::skip(void)
{
	wireSkip();
}

void OneWire::write(uint8_t v, uint8_t power)
{
	wireWriteByte(v);	
}

uint8_t OneWire::read(void)
{
	return wireReadByte();
}

uint8_t OneWire::read_bit(void)
{
	return wireReadBit();
}

void OneWire::write_bit(uint8_t v)
{
	wireWriteBit(v);
}








