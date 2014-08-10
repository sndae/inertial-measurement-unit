#include <Wire.h>
#include <L3G.h>

L3G gyro;

void setup() {
	
	Serial.begin(9600);			// opens serial port, sets data rate to 9600 bps
	Wire.begin();
	pinMode(13, OUTPUT);
	
	if (!gyro.init())
	{
		Serial.println("Failed to autodetect gyro type!");
		while (1);
	}
	gyro.enableDefault();
	
	// blink twice at startup
	digitalWrite(13, LOW);
	delay(1000);
	
	digitalWrite(13, HIGH); // first blink
	delay(50);
	digitalWrite(13, LOW);
	delay(200);
	digitalWrite(13, HIGH); // second blink
	delay(50);
	digitalWrite(13, LOW);
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
