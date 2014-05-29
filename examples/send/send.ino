#include <SPI.h>
#include <NRF24.h>

NRF24 radio;

bool tx;

void setup()
{
	Serial.begin(115200);
	Serial.println(F("NRF24 Send example"));

	radio.begin(9, 10);

	// Pin 7 sets the mode (Sender or Receiver). Connect to GND on the sender
	pinMode(7, INPUT_PULLUP);
	tx = !digitalRead(7);

	if (tx)
	{
		// transmitter doesn't need an address
	}
	else
	{
		// set our address
		// note the address of the received, we'll need it later on!
		radio.setAddress(0xD2);
		radio.startListening();
	}

	Serial.print(F("TX mode: "));
	Serial.println(tx);
}

void loop()
{
	if (tx)
	{
		Serial.print(F("Sending.. "));
		bool sent = radio.send(0xD2, "Hello there receiver 0xD2!");	// make sure the first parameter matches the receiver's address
		Serial.println(sent ? "OK" : "failed");
		delay(1000);
	}
	else if (radio.available())
	{
		char buf[32];
		uint8_t numBytes = radio.read(buf, sizeof(buf));
		Serial.print(F("Received: "));
		Serial.println(buf);
	}
}