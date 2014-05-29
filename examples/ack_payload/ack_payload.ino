#include <SPI.h>
#include <NRF24.h>

NRF24 radio;

bool tx;

bool ackPayload = true;

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
		radio.setAddress(0xCC);
		radio.startListening();
	}

	Serial.print(F("TX mode: "));
	Serial.println(tx);
}

void loop()
{
	if (tx)
	{
		char *data = "What is the meaning of life?";
		uint8_t buf[32];
		Serial.print(F("Sending.. "));
		int8_t received = radio.send(0xCC, (uint8_t *)data, strlen(data) + 1, buf, sizeof(buf));
		if (received > 0)
		{
			Serial.print(F("got response ("));
			Serial.print(received);
			Serial.print(F(" bytes): "));
			for (uint8_t i = 0; i < received; i++)
			{
				Serial.print(buf[i]);
				Serial.print(F(" "));
			}
			Serial.println();
		}
		else if (received == 0)
		{
			Serial.println(F("ok but no payload in ACK"));
		}
		else
		{
			Serial.println(F("failed"));
		}
		delay(1000);
	}
	else if (radio.available())
	{
		// automatic response but we need to clear the RX buffer to receive new data
		char buf[32];
		radio.read(buf, sizeof(buf));

		if (ackPayload)
		{
			// next time we'll transmit back this
			uint8_t response[1] = { 42 };
			radio.queueResponse(response, sizeof(response));
		}
	}

	if (Serial.available())
	{
		ackPayload = !ackPayload;

		Serial.print(F("Attaching ACK payload: "));
		Serial.println(ackPayload);

		Serial.read();
	}
}