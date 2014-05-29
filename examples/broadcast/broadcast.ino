#include <SPI.h>
#include <NRF24.h>

NRF24 radio;

bool tx;

void setup()
{
	Serial.begin(115200);
	Serial.println(F("NRF24 Broadcast example"));

	radio.begin(9, 10);

	// Pin 7 sets the mode (Sender or Receiver). Connect to GND on the sender
	pinMode(7, INPUT_PULLUP);
	tx = !digitalRead(7);

	if (tx)
	{
		radio.setAddress(0xD2);
	}
	else
	{
		radio.listenToAddress(0xD2);
	}

	Serial.print(F("TX mode: "));
	Serial.println(tx);
}

void loop()
{
	if (tx)
	{
		Serial.println(F("Broadcasting.. "));
		radio.broadcast("Hello world");
		delay(1000);
	}
	else if (radio.available())
	{
		char buf[32];
		uint8_t numBytes = radio.read(buf, sizeof(buf));
		Serial.print(F("Received "));
		Serial.print(numBytes);
		Serial.print(F(" bytes: "));
		Serial.println(buf);
	}
}