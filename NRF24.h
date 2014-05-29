#ifndef NRF24_H_
#define NRF24_H_

#include <Arduino.h>
#include <SPI.h>

#include "NRF24Reg.h"

typedef enum
{
	NRF24_NO_CRC = 0,
	NRF24_CRC_8BIT,
	NRF24_CRC_16BIT
} nrf24_crc_mode_e;

typedef enum
{
	NRF24_PA_MAX = 0,		//  0dBm	/	11.3mA transmit
	NRF24_PA_HIGH,			// -6dBm	/	9mA transmit
	NRF24_PA_MID,			// -12dBm	/	7.5mA transmit
	NRF24_PA_LOW			// -18dBm	/	7.0mA transmit
} nrf24_pa_level_e;

typedef enum
{
	NRF24_250KBPS = 0,
	NRF24_1MBPS,
	NRF24_2MBPS
} nrf24_datarate_e;

typedef enum
{
	NRF24_MODE_POWER_DOWN = 0,
	NRF24_MODE_STANDBY1,
	NRF24_MODE_STANDBY2,
	NRF24_MODE_RX,
	NRF24_MODE_TX
} nrf24_mode_e;

class NRF24
{
	public:
		// Netmask should be something "random"
		// All nodes that talk to eachother need to have the same netmask
		// 10101010.. can continue on preamble and cause missed packets
		// 11110000.. with only one logic transition can cause missed packets
		bool begin(uint8_t cePin, uint8_t csnPin, uint32_t netmask = 0xC2C2C2C2);

		// Logical RF channels
		void setAddress(uint8_t address);
		int8_t listenToAddress(uint8_t address);

		// Physical RF channel
		void setChannel(uint8_t channel);
		uint8_t getChannel();

		void setDataRate(nrf24_datarate_e dataRate);

		void setPowerAmplificationLevel(nrf24_pa_level_e level);
		nrf24_pa_level_e getPowerAmplificationLevel();

		bool broadcast(uint8_t *data, uint8_t length);
		bool broadcast(char *message);
		bool broadcast_P(const __FlashStringHelper *message);

		void transmit(uint8_t targetAddress, uint8_t *message, uint8_t length);
		void transmit(uint8_t targetAddress, char *message);

		uint8_t available(uint8_t *listener = NULL);
		uint8_t read(uint8_t *buf, uint8_t bufferSize);		// raw data
		uint8_t read(char *buf, uint8_t bufferSize);		// makes sure data is 0 terminated

		void setActive(bool active);
		bool getActive();

		nrf24_mode_e getCurrentMode();

		void startListening();
		void stopListening();

		void setRetries(uint8_t delay, uint8_t count);
		void setCRCMode(nrf24_crc_mode_e mode);

	private:
		void ceHigh()  { *cePort |= ceBitMask;    };
		void ceLow()   { *cePort &= ~ceBitMask;   };
		bool ceIsHigh(){ return *ceInput & ceBitMask; };
		void csnHigh() { *csnPort |= csnBitMask;  };
		void csnLow()  { *csnPort &= ~csnBitMask; };

		uint8_t readRegister(uint8_t reg);
		void writeRegister(uint8_t reg, uint8_t value);
		void writeRegister(uint8_t reg, uint8_t *value, uint8_t numBytes);

		bool transmit(uint8_t *data, uint8_t length);

		void assembleFullAddress(uint8_t address, uint8_t buf[5]);

		void setActiveTXPipe(uint8_t index);

		void flushTX();
		void flushRX();

		uint8_t pipeAddresses[5];
		bool listening;

		uint32_t netmask;
		uint8_t numPipes;
		int8_t previousPipe;

		volatile uint8_t *cePort;
		volatile uint8_t *ceInput;
		uint8_t ceBitMask;
		volatile uint8_t *csnPort;
		uint8_t csnBitMask;
};

extern NRF24 radio;

#endif // NRF24_H_