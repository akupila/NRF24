#include <SPI.h>
#include <NRF24.h>

NRF24 radio;

bool tx;

uint8_t packetIndex = 0;

void setup()
{
	Serial.begin(115200);
	Serial.println(F("NRF24 Ping example"));

	radio.begin(9, 10);

	// Pin 7 sets the mode (Sender or Receiver). Connect to GND on the sender
	pinMode(7, INPUT_PULLUP);
	tx = !digitalRead(7);

	if (tx)
	{
		// transmitter doesn't need an address

		// We go to active mode (Standby-I) which makes the transmit about 10x faster
		// Otherwise we'll need to wait for the crystal to start up (and shut down) which takes about 4ms total
		radio.setActive(true);
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
		Serial.print(F("Ping.. "));
		uint8_t buf[1] = { packetIndex++ % 0xFF };
		uint32_t t = micros();
		bool sent = radio.send(0xAA, buf, sizeof(buf));
		t = micros() - t;
		if (sent)
		{
			Serial.print(t);
			Serial.println(F("uS"));
		}
		else
		{
			Serial.println(F("failed"));
		}
		delay(1000);
	}
	else if (radio.available())
	{
		// Ping response sent automatically
		uint8_t buf[3];
		uint8_t numBytes = radio.read(buf, sizeof(buf));
		Serial.print(F("Got packet "));
		Serial.println(buf[0]);
	}
}