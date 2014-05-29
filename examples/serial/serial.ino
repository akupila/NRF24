#include <SPI.h>
#include <NRF24.h>

NRF24 radio;

bool tx;

void setup()
{
	Serial.begin(115200);
	Serial.println(F("NRF24 Serial init"));

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
		Serial.print(F("Sending.. "));
		bool ok = radio.broadcast("Hello world");
		Serial.println(ok ? "OK" : "failed");
		delay(1000);
	}
	else if (radio.available())
	{
		char buf[32];
		uint8_t bytes = radio.read(buf, sizeof(buf));
		Serial.println(buf);
	}
}