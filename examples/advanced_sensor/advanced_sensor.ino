#include <SPI.h>
#include <NRF24.h>

NRF24 radio;

// If we have one sensor and multiple listeners
#define USE_BROADCAST

bool host;

uint16_t sensorDelay = 0;
unsigned long previousTransmission = 0;

void setup()
{
	Serial.begin(115200);
	Serial.println(F("NRF24 Mixed example"));

	radio.begin(9, 10);

	// Pin 7 sets the mode (Sender or Receiver). Connect to GND on the sender
	pinMode(7, INPUT_PULLUP);
	host = !digitalRead(7);

	if (host)
	{
		// host
#ifndef USE_BROADCAST
		// this could be enabled for broadcast too, just showing that it doesn't have to be
		// when broadcasting the client doesn't care what the address of the receiver(s) is
		radio.setAddress(0xEE);
#endif
		radio.listenToAddress(0xDD);
	}
	else
	{
		// sensor
		radio.setAddress(0xDD);
		radio.startListening();
	}

	Serial.print(F("Mode: "));
	Serial.println(host ? F("host") : F("sensor"));

	if (host)
	{
		Serial.println(F("Set transmission interval by using 0-9"));
	}
}

void loop()
{
	if (sensorDelay > 0 && millis() > previousTransmission + sensorDelay)
	{
		uint16_t v = analogRead(0);

		uint8_t buf[2];
		buf[0] = (v >> 8) & 0xFF;
		buf[1] = v & 0xFF;

		Serial.print(F("Sending sample "));
		Serial.println(v);

#ifdef USE_BROADCAST
		// the problem with this is that there's no confirmation they reached their target
		radio.broadcast(buf, sizeof(buf));
#else
		radio.send(0xEE, buf, sizeof(buf));
#endif

		previousTransmission = millis();
	}
	
	if (radio.available())
	{
		uint8_t buf[2];
		radio.read(buf, sizeof(buf));

		if (host)
		{
			// extract 16 bit value
			uint16_t v = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
			Serial.print(F("Received: "));
			Serial.println(v);
		}
		else
		{
			// received command from host
			sensorDelay = buf[0] * 100;

			Serial.print(F("New delay: "));
			Serial.print(sensorDelay);
			Serial.println(F("ms"));

			// allow host to transition to listener
			delay(1);
		}
	}

	if (host && Serial.available())
	{
		char c = Serial.read();
		if (host)
		{
			if (c >= '0' && c <= '9')
			{
				uint8_t numSamples = c - '0';

				if (numSamples > 0)
				{
					Serial.print(F("Setting sample delay "));
					Serial.print(numSamples * 100);
					Serial.println(F("ms"));
				}
				else
				{
					Serial.println(F("Turning off sensor"));
				}

				uint8_t buf[1] = {numSamples};
				bool sent = radio.send(0xDD, buf, sizeof(buf));
				if (!sent) Serial.println(F("Request failed"));
			}
		}
	}
}