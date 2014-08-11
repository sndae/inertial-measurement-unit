#include <Wire.h>
#include <L3G.h>

// Gyroscope instance
L3G gyro;

// PB5 = 32
#define	LED	32					

// Turn-on and turn-off PB5 200 ms
#define BLINK_LED	PORTB |= LED, delay(200), PORTB &= ~LED, delay(200)

void setup() {
	
	Serial.begin(57600);				// Opens serial port, sets data rate to 57600 bps
	Wire.begin();
	DDRB |= (1 << PORTB5);				// Set PB5 as output
	
	// Check if gyroscope is working
	if (!gyro.init())
	{
		Serial.println("Failed to autodetect gyro type!");
		while (1);
	}
	gyro.enableDefault();
	
	// blink twice at startup
	BLINK_LED;
	BLINK_LED;
}

void loop() {
	
	
	gyro.read();					// read data from gyroscope
	Serial.print("X-Axis: ");
	Serial.print((int)gyro.g.x);
	Serial.print(" Y-Axis: ");
	Serial.print((int)gyro.g.y);
	Serial.print(" Z-Axis: ");
	Serial.print((int)gyro.g.z);
	Serial.write(10);				// send a line feed/new line
	
	delay(100);
	
}
