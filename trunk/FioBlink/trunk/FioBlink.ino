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
#include <math.h>
#include <Wire.h>
#include <L3G.h>
#include <LSM303/LSM303.h>

//	Accelerometer and Gyroscope Instances
L3G gyro;
LSM303 compass;

//	Definitions and Constants
#define BLINK_LED	PORTB |= LED, delay(200), PORTB &= ~LED, delay(200)
#define	LED	32						// PB5 as LED
#define gyroGain	0.00875			// Gyroscope Sensitivity factor for 250 dps
#define accGain		0.05729			// Accelerometer Sensivity factor 1 mg/LSB * (180/Pi)
const byte sampleNum =	255;		// Number of samples to be read to perform calibration
const float deltaTime = 0.020;		// Sample Time = 20ms
unsigned long now = 0;

//	Gyroscope variables
int gyro_offsetX = 0;
int gyro_offsetY = 0;
int gyro_offsetZ = 0;
float gyroRateX = 0.0;
float gyroRateY = 0.0;
float gyroRateZ = 0.0;
float gyroAngleX = 0.0;
float gyroAngleY = 0.0;
float gyroAngleZ = 0.0;

// Accelerometer variables
int acc_offsetX = 0;
int acc_offsetY = 0;
int acc_offsetZ = 0;
float accX = 0.0;
float accY = 0.0;
float accZ = 0.0;
float roll = 0.0;
float pitch = 0.0;
float theta = 0.0;

//	Function Prototypes
unsigned char commandInput(void);
void idle(void);
void getGyroscopeReadings(void);
void calculateGyroOffset(void);
void getAccelerometerReadings(void);
void calculateAccOffset(void);
void getIMUData(void);

// FINITE STATE MACHINE DEFINITIONS

struct states_t {
	void (*fncPtr) ();		// Function pointer
	byte time;				// Wait time for each state
	states_t * next[4];		// Pointer to other states
	};
typedef struct states_t StateType;

#define IDLE			&fsm[0]
#define CALIBRATE_ACC	&fsm[1]
#define CALIBRATE_GYRO	&fsm[2]
#define OUTPUT_DATA		&fsm[3]

StateType fsm[4] = {
	{ &idle,				10,		{ IDLE, CALIBRATE_ACC, CALIBRATE_GYRO, OUTPUT_DATA }},
	{ &calculateAccOffset,	255,	{ IDLE, CALIBRATE_ACC, CALIBRATE_GYRO, OUTPUT_DATA }},
	{ &calculateGyroOffset,	255,	{ IDLE, CALIBRATE_ACC, CALIBRATE_GYRO, OUTPUT_DATA }},
	{ &getIMUData,			0,		{ IDLE, CALIBRATE_ACC, CALIBRATE_GYRO, OUTPUT_DATA }}
};

//	General use variables
StateType * Ptr;
unsigned char input;

//	SETUP FUNCTION
void setup() {

	Serial.begin(57600);				// Opens serial port, sets data rate to 57600 bps
	Wire.begin();						// Initializes I2C
	DDRB |= (1 << PORTB5);				// Set PB5 as output

	// Initialize Gyroscope
	if (!gyro.init()) {
		Serial.println("Failed to autodetect gyro type!");
		while (1);
	}
	gyro.enableDefault();				// Enable Gyroscope with default settings
	
	// Initialize Accelerometer
	compass.init();
	compass.enableDefault();
	
	// Wait until there's new command
	while( Serial.available() == 0);
	Serial.println("Ready!");

	// blink twice at startup
	BLINK_LED;
	BLINK_LED;
	
	// Set state to Idle
	Ptr = IDLE;	
}


//	MAIN LOOOP FUNCTION
void loop() {
	(*Ptr->fncPtr) ();			// Execute function of state
	delay(Ptr->time);			// Wait the specified amount of time for the function
	input = commandInput();		// Read incoming
	Ptr = Ptr->next[input];		// Go the next state, depending of the input
	
}

unsigned char commandInput() {
	static unsigned char command;
	if (Serial.available() > 0) {
		command = Serial.read();
	}
	return command;
}

void idle() {
	
}

void getGyroscopeReadings() {	
	
	// Read raw data from Gyroscope
	gyro.read();

	// Calculate angular velocity from Gyro raw data
	gyroRateX = (float)((int)gyro.g.x - gyro_offsetX) * gyroGain;
	gyroRateY = (float)((int)gyro.g.y - gyro_offsetY) * gyroGain;
	gyroRateZ = (float)((int)gyro.g.z - gyro_offsetZ) * gyroGain;

	// Convert angular velocity to angles
	gyroAngleX += gyroRateX * deltaTime;
	gyroAngleY += gyroRateY * deltaTime;
	gyroAngleZ += gyroRateZ * deltaTime;
	
	// Keep angles between -180 to 179
	if (gyroAngleX < -180)			gyroAngleX += 360;
	else if (gyroAngleX >= 180)		gyroAngleX -= 360;
	
	if (gyroAngleY < -180)			gyroAngleY += 360;
	else if (gyroAngleY >= 180)		gyroAngleY -= 360;
	
	if (gyroAngleZ < -180)			gyroAngleZ += 360;
	else if (gyroAngleZ >= 180)		gyroAngleZ -= 360;
	
}

void calculateGyroOffset() {
	
	// Set variables to 0's
	gyro_offsetX = 0;
	gyro_offsetY = 0;
	gyro_offsetZ = 0;
	
	// Perform averaging
	for (byte n = 0; n < sampleNum; n++) {
		gyro.read();
		gyro_offsetX += (int)gyro.g.x;
		gyro_offsetY += (int)gyro.g.y;
		gyro_offsetZ += (int)gyro.g.z;
	}
	gyro_offsetX /= sampleNum;
	gyro_offsetY /= sampleNum;
	gyro_offsetZ /= sampleNum;
	
	Serial.print("Gyro Bias: ");
	Serial.print(gyro_offsetX);
	Serial.print(", ");
	Serial.print(gyro_offsetY);
	Serial.print(", ");
	Serial.println(gyro_offsetZ);
	
}

void getAccelerometerReadings() {
	
	// Read raw values of accelerometer
	compass.readAcc();
	
	// Current value -  offset * Accelerometer gain
	accX = (float)(compass.a.x - acc_offsetX) * accGain;
	accY = (float)(compass.a.y - acc_offsetY) * accGain;
	accZ = (float)(compass.a.z - acc_offsetZ) * accGain;
	
	// Calculate roll, pitch and theta angles
	roll = (atan2(accZ, sqrt(accY*accY + accX*accX))) * RAD_TO_DEG;
	pitch = (atan2(accX, sqrt(accY*accY + accZ*accZ))) * RAD_TO_DEG;
	theta = (atan2(sqrt(accX*accX + accZ*accZ),accY)) * RAD_TO_DEG;
	
	// Keep angles between -180 to 179
	if (roll < -180)			roll += 360;
	else if (roll >= 180)		roll -= 360;
	
	if (pitch < -180)			pitch += 360;
	else if (pitch >= 180)		pitch -= 360;
	
	if (theta < -180)			theta += 360;
	else if (theta >= 180)		theta -= 360;
	
}
void calculateAccOffset() {
	
	// Set variables to 0's
	acc_offsetX = 0;
	acc_offsetY = 0;
	acc_offsetZ = 0;
	
	// Perform averaging
	for (byte n = 0; n < sampleNum; n++) {
		compass.readAcc();
		acc_offsetX += (compass.a.x >> 4);
		acc_offsetY += (compass.a.y >> 4);
		acc_offsetZ += (compass.a.z >> 4);
	}
	
	acc_offsetX /= sampleNum;
	acc_offsetY /= sampleNum;
	acc_offsetZ /= sampleNum;
	
	Serial.print("Accelerometer Bias: ");
	Serial.print(acc_offsetX);
	Serial.print(", ");
	Serial.print(acc_offsetY);
	Serial.print(", ");
	Serial.print(acc_offsetZ);
	Serial.write(10);
	
}

void getIMUData() {
	
	// Get current time of system
	now = millis();
	
	// Perform Gyroscope readings
	getGyroscopeReadings();
	
	// Perform Accelerometer readings
	getAccelerometerReadings();
	
	// Display Information
	Serial.write("Gx: ");
	Serial.print(gyroAngleX);
	Serial.write("	Gy: ");
	Serial.print(gyroAngleY);
	Serial.write("	Gz: ");
	Serial.print(gyroAngleZ);
	Serial.write(10);
	
	Serial.write("Roll: ");
	Serial.print(roll);
	Serial.write("	Pitch: ");
	Serial.print(pitch);
	Serial.write("	Theta: ");
	Serial.print(theta);
	Serial.write(10);
	
	// Wait 20ms
	while ( (millis() - now) < 20);
	
}
