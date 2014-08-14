/************************************************************************/
/*						INERTIAL MEASUREMENT UNIT
The program performs readings from Accelerometer and Gyroscope to
perform calculations to get orientation and velocity.

The gyroscope needs to be calibrated at the beginning in order to avoid
zero-level rate. This is done by reading a specific number of samples and
performing an average of the data.

To avoid drifting from gyro data and noise from accelerometer data, is
necessary to perform a Complementary Filter.

/************************************************************************/
#include <Wire.h>
#include <L3G.h>

/************************************************************************/
/*  Variables                                                           */
/************************************************************************/
#define	LED	32						// PB5 as LED
#define BLINK_LED	PORTB |= LED, delay(200), PORTB &= ~LED, delay(200)
L3G gyro;							// Gyroscope instance
const byte sampleNum = 250;			// Number of samples to be read to perform calibration
const float gyroGain = 0.00875;		// Sensitivity factor for 250 dps
const float deltaTime = 0.010;		// deltaTime = 100Hz
long time;
signed int gyro_offsetX = 0;
signed int gyro_offsetY = 0;
signed int gyro_offsetZ = 0;
float gyroRateX = 0.0;
float gyroRateY = 0.0;
float gyroRateZ = 0.0;
float gyroAngleX = 0.0;
float gyroAngleY = 0.0;
float gyroAngleZ = 0.0;

/************************************************************************/
/*	Function Prototypes
/************************************************************************/
void calculateGyroOffset(void);

/************************************************************************/
/*	Setup Function
Initializes Serial communication and IMU
/************************************************************************/
void setup() {

Serial.begin(57600);				// Opens serial port, sets data rate to 57600 bps
Wire.begin();						// Initializes I2C
DDRB |= (1 << PORTB5);				// Set PB5 as output

// Check if gyroscope is working
if (!gyro.init()) {
Serial.println("Failed to autodetect gyro type!");
while (1);
}
gyro.enableDefault();				// Enable Gyroscope with default settings
calculateGyroOffset();				// Calculate gyroscope zero-rate level

// blink twice at startup
BLINK_LED;
BLINK_LED;
time = millis();
}

/************************************************************************/
/*	Loop Function
This functions perfoms the main loop of the program.
/************************************************************************/
void loop() {

	if (millis() - time > 10) {
		// Read raw data from Gyroscope
		gyro.read();

		// Calculate angular velocity from Gyro raw data
		gyroRateX = ((float)gyro.g.x - gyro_offsetX) * gyroGain;
		gyroRateY = ((float)gyro.g.y - gyro_offsetY) * gyroGain;
		gyroRateZ = ((float)gyro.g.z - gyro_offsetZ) * gyroGain;

		// Convert angular velocity to angles
		gyroAngleX += gyroRateX * deltaTime;
		gyroAngleY += gyroRateY * deltaTime;
		gyroAngleZ += gyroRateZ * deltaTime;
		
		Serial.print("AngleX: ");
		Serial.print(gyroAngleX);
		Serial.print(" AngleY: ");
		Serial.print(gyroAngleY);
		Serial.print(" AngleZ: ");
		Serial.print(gyroAngleZ);
		Serial.write(10);			//Carrier-return;
		time = millis();
	}
}

void calculateGyroOffset() {

for (byte n = 0; n < sampleNum; n++) {
gyro.read();
gyro_offsetX += (signed int)gyro.g.x;
gyro_offsetY += (signed int)gyro.g.y;
gyro_offsetZ += (signed int)gyro.g.z;
}
gyro_offsetX /= sampleNum;
gyro_offsetY /= sampleNum;
gyro_offsetZ /= sampleNum;

}


