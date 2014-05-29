#include <SPI.h>
#include <NRF24.h>

NRF24 radio;

bool tx;

#define PACKET_SIZE 32

void setup()
{
	Serial.begin(115200);
	Serial.println(F("NRF24 Throughput example"));

	radio.begin(9, 10);

	// Pin 7 sets the mode (Sender or Receiver). Connect to GND on the sender
	pinMode(7, INPUT_PULLUP);
	tx = digitalRead(7);

	if (tx)
	{
		// transmitter doesn't need an address

		// This consumes more power but gives us 650% better throughput and is good for streaming applications
		radio.setActive(true);

		// Quicker retry time (250uS)
		radio.setRetries(0, 15);
	}
	else
	{
		// set our address
		radio.setAddress(0xAA);
		radio.startListening();
	}

	Serial.print(F("TX mode: "));
	Serial.println(tx);
}

void loop()
{
	if (tx)
	{
		// actual data is not important
		uint8_t buf[PACKET_SIZE];
		uint32_t packetIndex = 0;
		uint32_t time = micros();
		uint32_t stopTime = time + 1000000;
		while (true)
		{
			bool sent = radio.send(0xAA, buf, PACKET_SIZE);
			// bool sent = radio.broadcast(buf, PACKET_SIZE);
			if (sent) ++packetIndex;
			if (micros() >= stopTime) break;
		}

		Serial.print(packetIndex);
		Serial.print(F(" packets/s, "));
		Serial.print(packetIndex * PACKET_SIZE);
		Serial.print(F(" bytes, "));
		Serial.print((float)(packetIndex * PACKET_SIZE) / 1024.0);
		Serial.println(F("kb/s"));
	}
	else if (radio.available())
	{
		// Ping response sent automatically
		// Need to read out the data so we're able to receive more
		uint8_t buf[PACKET_SIZE];
		radio.read(buf, sizeof(buf));
	}

	// the receiver will respond automatically with ACK
}