#include <math.h>
#include <vector>
#include <string.h>
#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <time.h>
#include <pthread.h>
#include <stdint.h>

using namespace std;

#include "BMP085.h"
#include "HMC5883L.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "PruProxy.h"
#include "Receiver.h"
#include "pid.h"


// min/max values for each stick
#define RC_THR_MIN   0
#define RC_THR_MAX   1023
#define RC_YAW_MIN   0
#define RC_YAW_MAX   1023
#define RC_PIT_MIN   0
#define RC_PIT_MAX   1023
#define RC_ROL_MIN   0
#define RC_ROL_MAX   1023
/*
#define RC_THR_MIN   1070
#define RC_THR_MAX   2083
#define RC_YAW_MIN   1068
#define RC_YAW_MAX   1915
#define RC_PIT_MIN   1077
#define RC_PIT_MAX   1915
#define RC_ROL_MIN   1090
#define RC_ROL_MAX   1913
*/

// Motor numbers definitions
#define MOTOR_FL   2    // Front left
#define MOTOR_FR   0    // Front right
#define MOTOR_BL   1    // back left
#define MOTOR_BR   3    // back right


// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

// class default I2C address is 0x77
// specific I2C addresses may be passed as a parameter here
// (though the BMP085 supports only one address)
BMP085 bmp;

// class default I2C address is 0x1E
// specific I2C addresses may be passed as a parameter here
// this device only supports one I2C address (0x1E)
HMC5883L hmc;


static PruProxy Pru;
static Receiver Recv;

// MPU control/status vars
bool dmpReady = false;	// set true if DMP init was successful
uint8_t mpuIntStatus;	// holds actual interrupt status byte from MPU
uint8_t devStatus;	// return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;	// expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;	// count of all bytes currently in FIFO
uint8_t fifoBuffer[64];	// FIFO storage buffer
int16_t g[3];
float gyro[3];

// orientation/motion vars
Quaternion q;		// [w, x, y, z]         quaternion container
VectorInt16 aa;		// [x, y, z]            accel sensor measurements
VectorInt16 aaReal;	// [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;	// [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;	// [x, y, z]            gravity vector
float euler[3];		// [psi, theta, phi]    Euler angle container
float ypr[3];		// [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// BMP status vars
float temperature;
float pressure;
float altitude;
int32_t lastMicros;

//HMC status vars
int16_t magX, magY, magZ;

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00,
		'\r', '\n' };


// map value in range function
long map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))
#define ToDeg(x) ((x)*57.2957795131) // *180/pi
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

// PID array (6 pids, two for each axis)
PID pids[6];
#define PID_PITCH_RATE	0
#define PID_ROLL_RATE	1
#define PID_PITCH_STAB	2
#define PID_ROLL_STAB	3
#define PID_YAW_RATE	4
#define PID_YAW_STAB	5


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

	// PID Configuration
	pids[PID_PITCH_RATE].set_Kpid(0.7, 0.000, 0.00);
	pids[PID_ROLL_RATE].set_Kpid(0.7, 0.000, 0.00);
	//pids[PID_YAW_RATE].set_Kpid(2.7, 0, 0);

	pids[PID_PITCH_STAB].set_Kpid(4.5, 0.1, 1.2);
	pids[PID_ROLL_STAB].set_Kpid(4.5, 0.1, 1.2);
	//pids[PID_YAW_STAB].set_Kpid(10,0,0);

	////////////////////////////////// MPU INIT ////////////////////////////////////////
	// initialize device
	printf("Initializing I2C devices...\n");
	bmp.initialize();
	hmc.initialize();
	mpu.initialize();

	// verify connection
	printf("Testing device connections...\n");
	printf(bmp.testConnection() ? "BMP085 connection successful\n" : "BMP085 connection failed\n");
	printf(hmc.testConnection() ? "HMC5883L connection successful\n" : "HMC5883L connection failed\n");
	printf(mpu.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");

	// load and configure the DMP
	printf("Initializing DMP...\n");
	devStatus = mpu.dmpInitialize();

	// make sure it worked (returns 0 if so)
	if (devStatus == 0) {
		// turn on the DMP, now that it's ready
		printf("Enabling DMP...\n");
		mpu.setDMPEnabled(true);

		// enable interrupt detection
		mpuIntStatus = mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		printf("DMP ready!\n");
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
	} else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		printf("DMP Initialization failed (code %d)\n", devStatus);
	}

	/////////////////////////MPU OVER /////////////////////////////////
}

// ******** Main Loop *********
void mainLoop() {

	long rcthr, rcyaw, rcpit, rcroll;
	long yaw, pitch, roll;
	long gyroYaw, gyroPitch, gyroRoll;
	static float yaw_target = 0;

	uint16_t channels[4];
	float motor[4];

	// read raw heading measurements from mdevice
	hmc.getHeading(&magX, &magY, &magZ);
	//cout << "magX : " << magX << " magY : " << magY << " magZ : " << magZ << endl;

	float heading = atan2(magY, magX);
	if (heading < 0) heading += 2 * M_PI;
	//cout << " heading : " << ToDeg(heading) << endl;

	// get input data from Receiver
	pthread_mutex_lock(&mutex);
	channels[2] = Recv.InputThrottle;
	channels[3] = Recv.InputPitch;
	channels[0] = Recv.InputYaw;
	channels[1] = Recv.InputRoll;
	pthread_mutex_unlock(&mutex);

	//rcthr = channels[2] * 2;
	rcthr = map(channels[2], RC_THR_MIN, RC_THR_MAX, 1070, 2083);
	rcyaw = map(channels[0], RC_YAW_MIN, RC_YAW_MAX, -180, 180) + 10;
	rcpit = -map(channels[3], RC_PIT_MIN, RC_PIT_MAX, -45, 45) - 11;
	rcroll = -map(channels[1], RC_ROL_MIN, RC_ROL_MAX, -45, 45) - 7;

	cout << " rc-> " << rcthr << " " << rcyaw << " " << rcpit << " " << rcroll << endl;

	////////////////////////////////////// MPU START ///////////////////////////////////////

	if (!dmpReady)
		return;

	// wait for FIFO count > 42 bits
	do {
		fifoCount = mpu.getFIFOCount();
	} while (fifoCount < 42);

	if (fifoCount == 1024) {
		// reset so we can continue cleanly
		mpu.resetFIFO();
		printf("FIFO overflow!\n");

		// otherwise, check for DMP data ready interrupt
		//(this should happen frequently)
	} else {
		//read packet from fifo
		mpu.getFIFOBytes(fifoBuffer, packetSize);

		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

		//scaling for degrees output
		for (int i = 0; i < 3; i++) {
			ypr[i] *= 180 / M_PI;
		}

		yaw = ypr[0];
		pitch = ypr[1];
		roll = ypr[2];

		cout<< " ypr-> " << yaw << " " << pitch <<" "<< roll << endl;

		mpu.dmpGetGyro(g, fifoBuffer);

		//0=gyroX, 1=gyroY, 2=gyroZ
		//swapped to match Yaw,Pitch,Roll
		//Scaled from deg/s to get tr/s
		for (int i = 0; i < 3; i++) {
			gyro[i] = ToDeg(g[3-i-1]);
		}
		gyroYaw = -gyro[0];
		gyroPitch = -gyro[1];
		gyroRoll = gyro[2];
		cout<< " Gyroypr-> " << gyroYaw << " " << gyroPitch <<" "<< gyroRoll<<endl;
	}

	/////////////////////////// MPU ENDS ///////////////////////////////////

	// Do the magic
	//if (false) { // Throttle raised, turn on stablisation.
	if (rcthr > 1070 + 100) { // Throttle raised, turn on stablisation.
	//if (rcthr > RC_THR_MIN + 100) { // Throttle raised, turn on stablisation.
		// Stablise PIDS
		float pitch_stab_output =
				constrain(pids[PID_PITCH_STAB].update_pid_std((float)rcpit, pitch, 1), -250, 250);
		float roll_stab_output =
				constrain(pids[PID_ROLL_STAB].update_pid_std((float)rcroll, roll, 1), -250, 250);
		float yaw_stab_output =
				constrain(pids[PID_YAW_STAB].update_pid_std(wrap_180(yaw_target), wrap_180(yaw), 1), -360, 360);

		// is pilot asking for yaw change - if so feed directly to rate pid (overwriting yaw stab output)
		if (abs(rcyaw) > 5) {
			yaw_stab_output = rcyaw;
			yaw_target = yaw; // remember this yaw for when pilot stops
		}

		// rate PIDS
		long pitch_output =
				(long) constrain(pids[PID_PITCH_RATE].update_pid_std(pitch_stab_output, gyroPitch, 1), - 500, 500);
		long roll_output =
				(long) constrain(pids[PID_ROLL_RATE].update_pid_std(roll_stab_output, gyroRoll, 1), -500, 500);
		long yaw_output =
				(long) constrain(pids[PID_YAW_RATE].update_pid_std(yaw_stab_output, gyroYaw, 1), -500, 500);

		cout << "pitch out : " << pitch_output << ", roll_output : " << roll_output << ", yaw_output : " << yaw_output << endl;

		// mix pid outputs and send to the motors.
		motor[MOTOR_FL] = rcthr + roll_output + pitch_output - yaw_output;
		motor[MOTOR_BL] = rcthr + roll_output - pitch_output + yaw_output;
		motor[MOTOR_FR] = rcthr - roll_output + pitch_output + yaw_output;
		motor[MOTOR_BR] = rcthr - roll_output - pitch_output - yaw_output;

		Pru.Output1 = motor[MOTOR_BR] * 200;
		Pru.Output2 = motor[MOTOR_FR] * 200;
		Pru.Output3 = motor[MOTOR_BL] * 200;
		Pru.Output4 = motor[MOTOR_FL] * 200;
		Pru.UpdateOutput();

	} else {
		// motors off
		Pru.Output1 = 1000 * 200;
		Pru.Output2 = 1000 * 200;
		Pru.Output3 = 1000 * 200;
		Pru.Output4 = 1000 * 200;
		Pru.UpdateOutput();

		// reset yaw target so we maintain this on takeoff
		yaw_target = yaw;
	}
}

unsigned long micros() {
	struct timeval currentTime;
	gettimeofday(&currentTime, NULL);

	return (unsigned int) currentTime.tv_usec;
}

void subLoop() {
	bmp.setControl(BMP085_MODE_TEMPERATURE);

	// wait appropriate time for conversion (4.5m delay)
	lastMicros = micros();
	//cout << "micros : " << lastMicros << endl;
	while (micros() - lastMicros < bmp.getMeasureDelayMicroseconds());

	// read caliibrated temperature value in degrees Celsius
	temperature = bmp.getTemperatureC();

	// request pressure (3x oversampling mode, high detail, 23.5m delay)
	bmp.setControl(BMP085_MODE_PRESSURE_3);
	lastMicros = micros();
	while (micros() - lastMicros < bmp.getMeasureDelayMicroseconds());

	pressure = bmp.getPressure();

	// calculate absolute altitude in meters based on known pressure
	// (may pass a second "sea level pressure" parameter here,
	// otherwise uses the standard valu of 101325 Pa)
	altitude = bmp.getAltitude(pressure);

	cout << "Temperature : " << temperature << ", Pressure : " << pressure << ", Altitude : " << altitude << endl;
}

void* threadMainLoop(void* data) {
	while (1) {
		mainLoop();
		usleep(1);
	}
}

void* threadSubLoop(void* data) {
	while (1) {
		subLoop();
		usleep(1); // to reduce cpu load
	}
}

//int main(int argc, char **argv) {
void startControl() {
	cout << "start" << endl;

	cout << "setup " << endl;

	Pru.Init();
	Recv.Init();

	setup();

	int thr_id;
	int a = 1;
	pthread_t p_thread[2];
	int status;
	thr_id = pthread_create(&p_thread[0], NULL, threadMainLoop, (void *)&a);
	sleep(1);
	thr_id = pthread_create(&p_thread[1], NULL, threadSubLoop, (void *)&a);

	//pthread_join(p_thread[0], (void **)&status);
	//pthread_join(p_thread[1], (void **)&status);

	status = pthread_mutex_destroy(&mutex);

//	return 0;
}

void updateInput(unsigned int throttle, unsigned int rudder,
		unsigned int elevator, unsigned int aileron) {

	//cout << "updateInput " << throttle << "rudduer : " << rudder << endl;

	Recv.UpdateInput(throttle, rudder, elevator, aileron);
}
