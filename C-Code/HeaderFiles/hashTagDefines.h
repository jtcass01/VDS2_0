// hashTagDefines.h

//debug settings.
//Since print statements make things run slow and take up memory to store. choosing what to compile
//can be a good way to make code more efficient

#define DEBUG_EMERGENCY			true				//toggles print statements that are super detailed
#define DEBUG_FLIGHTMODE		false
#define DEBUG_KALMAN			false
#define DEBUG_VELOCITY			true
#define	DEBUG_RAWSTATE			false
#define DEBUG_READFROMFILE		false
#define DEBUG_GETACCELERATION	false
#define DEBUG_ALTITUDEPLZ		false

#define TEST_MODE				true				//print statement indicating test mode. Set to TRUE for ground testing. SET TO FALSE FOR FLIGHT!
#define TEST_FILENAME			"8_6_16_test.dat"

//miscallaneous constants
#define ENC_RANGE			400
#define BUFF_N				30					//Number of Data Points per accel and alt array. MUST BE EVEN
#define MAX_EXP_VEL			300

//physical constants. Used for Kalman filter and SPP
#define POST_BURN_MASS		20.1
#define RHO					1.225
#define CD_R				0.4
#define CD_B				0.54
#define A_R					0.020922539
#define A_B					A_R + 0.005938865542
#define AVG_MOTOR_THRUST	2200

//SPP constants
#define TARGET_ALTITUDE		1609
#define ON_TRACK_VELOCITY	125

//pins
#define LED 13
