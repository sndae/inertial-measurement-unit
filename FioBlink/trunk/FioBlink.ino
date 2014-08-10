#include <Wire.h>
#include <L3G.h>

L3G gyro;

int incomingByte = 0;	// for incoming serial data

void setup() {
	Serial.begin(9600);	// opens serial port, sets data rate to 9600 bps
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
	
	// send data only when you receive data:
	if (Serial.available() > 0) {
		
		// read the incoming byte:
		incomingByte = Serial.read();
		
		// read data from gyroscope
		gyro.read();
		
		if(incomingByte == '0') {			
			digitalWrite(13, LOW);
		}
		else if(incomingByte == '1') {
			digitalWrite(13, HIGH);
		}
		
		// say what you got:
		Serial.print("Fio received: ");
		Serial.write(incomingByte);			// Arduino 1.0 compatibility
		Serial.write(10);					// send a line feed/new line, ascii 10
		Serial.print("X-Axis: ");
		Serial.print((int)gyro.g.x);
		Serial.write(10);					// send a line feed/new line
	}
}
