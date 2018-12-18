/**
* EURECOM Operating Systems Robot Challenge 2017
* Delivery: 15/01/2018
* Group Number 5 SegFault
* @version 5.0 14/01/2017
* @author Berkay, Jermaine, Ariane, Yasmine
* Disclaimer: Since we all live together, all of our meetings were done with all of us involved.
*/

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"
#include "coroutine.h"
#include "ev3_servo.h"

#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
// WIN32 /////////////////////////////////////////
#ifdef __WIN32__
#include <windows.h>
// UNIX //////////////////////////////////////////
#else
#include <unistd.h>
#define Sleep( msec ) usleep(( msec ) * 1000 )
#define SERV_ADDR  "40:E2:30:8E:5D:DC"     /* Whatever the address of the server is */
#define TEAM_ID     5                       /* Your team ID */

#define MSG_ACK     0
#define MSG_START    1
#define MSG_STOP   2
#define MSG_KICK    3
#define MSG_POSITION 4
#define MSG_MAPDATA 5
#define MSG_MAPDONE 6
#define MSG_OBSTACLE 7

#define MAP_WIDTH 80
#define MAP_HEIGHT 80


//////////////////////////////////////////////////
#endif
const char const *color[] = { "?", "BLACK", "BLUE", "GREEN", "YELLOW", "RED", "WHITE", "BROWN" };
#define COLOR_COUNT  (( int )( sizeof( color ) / sizeof( color[ 0 ])))

// GLOBAL VARIABLES

time_t start, end; //Start and End times of the program
double dif;		   //Time passed since the start of the challenge

char ROBOT_DIRECTION = 'E'; //Default starting direction
int ROBOT_X = 10;//starting point X
int ROBOT_Y = 2;//starting point Y

bool releaseobject = true;//Can I release an obstacle? Or did I released before?
bool releaseobjectmessagesent = false;//Did I notify the server for the obstacle I dropped?

char map[256][256] = { ' ' };//The map array. 256x256 because that is the maximum size allowed for the server.

//Bluetooth Server veriables
char string[58];
int s;
uint16_t msgId = 0;

// WIN32 /////////////////////////////////////////
#ifdef __WIN32__
#include <windows.h>
// UNIX //////////////////////////////////////////
#else
#include <unistd.h>
#define Sleep( msec ) usleep(( msec ) * 1000 )
#endif
#define L_MOTOR_PORT      OUTPUT_C
#define L_MOTOR_EXT_PORT  EXT_PORT__NONE_
#define R_MOTOR_PORT      OUTPUT_B
#define R_MOTOR_EXT_PORT  EXT_PORT__NONE_
#define IR_CHANNEL        0
#define SPEED_LINEAR      75  /* Motor speed for linear motion, in percents */
#define SPEED_CIRCULAR    50  /* ... for circular motion */
int max_speed;  /* Motor maximal speed */
#define DEGREE_TO_COUNT( d )  (( d ) * 260 / 90 )
int app_alive;

//The type of movements the robot is capable of
enum {
	
	//Non Turning movements
	MOVE_NONE,
	MOVE_FORWARD,
	MOVE_BACKWARD,
	STEP_BACKWARD,

	//Turning movements
	TURN_LEFT,//90 degrees
	TURN_RIGHT,//90 degrees
	TURN_ANGLE,//xx degrees
	
	//Customised movements
	MOVE_BACKWARDS_RANDOMLY,//Moves backwards right or left(Undewr control of the touch sensor)
	RELEASE_OBJECT,//Releases an object using the engine
	TURN_TO_TARGET,//Scan 10 degrees in front, turn to the target, get close to it, read its color, save the map values, go backwards to original spot, fix orientation
};


int moving;   /* Current moving */
int command;  /* Command for the 'drive' coroutine */
int angle;    /* Angle of rotation */
enum { L, R };
uint8_t motor[3] = { DESC_LIMIT, DESC_LIMIT, DESC_LIMIT };  /* Sequence numbers of motors */

//Sensor variables
FLAGS_T state;
uint8_t sn;
uint8_t sn_touch;
uint8_t sn_color;
uint8_t sn_compass;
uint8_t sn_sonar;
uint8_t sn_mag;
uint8_t sn_gyro;

//Custom functions from tester.c and robotclient.c
int read_from_server(int sock, char *buffer, size_t maxSize) {
	int bytes_read = read(sock, buffer, maxSize);

	if (bytes_read <= 0) {
		fprintf(stderr, "Server unexpectedly closed connection...\n");
		close(s);
		exit(EXIT_FAILURE);
	}

	printf("[DEBUG] received %d bytes\n", bytes_read);

	return bytes_read;
}
static void _run_forever(int l_speed, int r_speed)
{
	set_tacho_speed_sp(motor[L], l_speed);
	set_tacho_speed_sp(motor[R], r_speed);
	multi_set_tacho_command_inx(motor, TACHO_RUN_FOREVER);

}
static void _run_to_rel_pos(int l_speed, int l_pos, int r_speed, int r_pos)
{
	set_tacho_speed_sp(motor[L], l_speed);
	set_tacho_speed_sp(motor[R], r_speed);
	set_tacho_position_sp(motor[L], l_pos);
	set_tacho_position_sp(motor[R], r_pos);
	multi_set_tacho_command_inx(motor, TACHO_RUN_TO_REL_POS);
}
static void _run_timed(int l_speed, int r_speed, int ms)
{
	set_tacho_speed_sp(motor[L], l_speed);
	set_tacho_speed_sp(motor[R], r_speed);
	multi_set_tacho_time_sp(motor, ms);
	multi_set_tacho_command_inx(motor, TACHO_RUN_TIMED);
}
static int _is_running(void)
{
	FLAGS_T state = TACHO_STATE__NONE_;
	get_tacho_state_flags(motor[L], &state);
	if (state != TACHO_STATE__NONE_) return (1);
	get_tacho_state_flags(motor[R], &state);
	if (state != TACHO_STATE__NONE_) return (1);
	return (0);
}
static void _stop(void)
{
	multi_set_tacho_command_inx(motor, TACHO_STOP);
}

/*--------------app_init -----
|  Function app_init
|  Purpose:  Initialises the tacho engines (right and left wheel) connected.
|  Returns:	 0 for success and 1 for failure
*------------------------------*/
int app_init(void)
{
	char s[16];
	if (ev3_search_tacho_plugged_in(L_MOTOR_PORT, L_MOTOR_EXT_PORT, motor + L, 0)) {
		get_tacho_max_speed(motor[L], &max_speed);
		/* Reset the motor */
		set_tacho_command_inx(motor[L], TACHO_RESET);
	}
	else {
		printf("LEFT motor (%s) is NOT found.\n", ev3_port_name(L_MOTOR_PORT, L_MOTOR_EXT_PORT, 0, s));
		/* Inoperative without left motor */
		return (0);
	}
	if (ev3_search_tacho_plugged_in(R_MOTOR_PORT, R_MOTOR_EXT_PORT, motor + R, 0)) {
		/* Reset the motor */
		set_tacho_command_inx(motor[R], TACHO_RESET);
	}
	else {
		printf("RIGHT motor (%s) is NOT found.\n", ev3_port_name(R_MOTOR_PORT, R_MOTOR_EXT_PORT, 0, s));
		/* Inoperative without right motor */
		return (0);
	}
	command = moving = MOVE_NONE;
	return(1);
}


/*
Turn right and Turn left Attempts:
1) Compass Failure: Compass doesnt work when close to the board. Compass was unable to operate well during movement. Removed compass (Berkay)
2) Timed Turns: Timed turns are not working properly when battery is low. Not always reliable. (Yasmine)
3) Gyro Turns: Gyrosensor works great on turns. With almost zero tolerance. Current reaction time 50ms. (Jermaine)
*/

/*--------------turnright -----
|  Function turnright
|  Purpose:  Turns the robot to 90 degrees right using the Gyro sensor value
*------------------------------*/
void turnright()
{
	printf("turning right\n");
	float current_pos;
	get_sensor_value0(sn_gyro, &current_pos);
	float final_pos = current_pos + 90.0;

	int diff = (int)(final_pos - current_pos);
	bool turn_right;
	//printf("CURRENTPOS:%f FINALPOS:%f DIFF:%d\n", current_pos, final_pos, diff);

	while (abs(current_pos - final_pos) != 0)
	{

		turn_right = (diff > 0);

		uint8_t right_wheel, left_wheel;


		int max_speed;
		ev3_search_tacho_plugged_in(67, 0, &right_wheel, 0);
		ev3_search_tacho_plugged_in(66, 0, &left_wheel, 0);

		get_tacho_max_speed(right_wheel, &max_speed);

		set_tacho_stop_action_inx(right_wheel, TACHO_COAST);
		set_tacho_stop_action_inx(left_wheel, TACHO_COAST);

		set_tacho_speed_sp(right_wheel, max_speed * (turn_right ? 1 : -1) * 0.1);
		set_tacho_speed_sp(left_wheel, max_speed * (turn_right ? -1 : 1) * 0.1);

		set_tacho_time_sp(right_wheel, 50);
		set_tacho_time_sp(left_wheel, 50);

		set_tacho_command_inx(right_wheel, TACHO_RUN_TIMED);
		set_tacho_command_inx(left_wheel, TACHO_RUN_TIMED);

		get_sensor_value0(sn_gyro, &current_pos);
		diff = (int)(final_pos - current_pos);
		//printf("CURRENTPOS:%f FINALPOS:%f DIFF:%d\n", current_pos, final_pos, diff);
	}

}

/*--------------turnleft -----
|  Function turnleft
|  Purpose:  Turns the robot to 90 degrees left using the Gyro sensor value
*------------------------------*/
void turnleft()
{
	printf("turning left\n");
	float current_pos;
	get_sensor_value0(sn_gyro, &current_pos);
	float final_pos = current_pos - 90.0;

	int diff = (int)(final_pos - current_pos);
	bool turn_right;
	//printf("CURRENTPOS:%f FINALPOS:%f DIFF:%d\n", current_pos, final_pos, diff);

	while (abs(current_pos - final_pos) != 0)
	{

		turn_right = (diff > 0);

		uint8_t right_wheel, left_wheel;

		int max_speed;
		ev3_search_tacho_plugged_in(67, 0, &right_wheel, 0);
		ev3_search_tacho_plugged_in(66, 0, &left_wheel, 0);

		get_tacho_max_speed(right_wheel, &max_speed);
		//		get_tacho_max_speed( left_wheel, &max_speed );

		set_tacho_stop_action_inx(right_wheel, TACHO_COAST);
		set_tacho_stop_action_inx(left_wheel, TACHO_COAST);

		set_tacho_speed_sp(right_wheel, max_speed * (turn_right ? 1 : -1) * 0.1);
		set_tacho_speed_sp(left_wheel, max_speed * (turn_right ? -1 : 1) * 0.1);

		set_tacho_time_sp(right_wheel, 50);
		set_tacho_time_sp(left_wheel, 50);
		//		set_tacho_ramp_up_sp( right_wheel, 100 );
		//		set_tacho_ramp_up_sp( left_wheel,100 );

		set_tacho_command_inx(right_wheel, TACHO_RUN_TIMED);
		set_tacho_command_inx(left_wheel, TACHO_RUN_TIMED);


		get_sensor_value0(sn_gyro, &current_pos);
		diff = (int)(final_pos - current_pos);
		//printf("CURRENTPOS:%f FINALPOS:%f DIFF:%d\n", current_pos, final_pos, diff);
	}

}





//CORO functions defined that will handle the movement decisions
CORO_CONTEXT(handle_ir_proximity);//handles proximity and color sensor
CORO_CONTEXT(drive);//drives the engines and executes the incoming commands from other CORO routines
CORO_CONTEXT(handle_brick_control);//handles brick board keyboard inputs. (i.e Back button exits the program)




/*--------------drive -----
|  CORO drive
|  Purpose:  Executes the incoming commands. Alloved commands: MOVE_NONE,MOVE_FORWARD,MOVE_BACKWARD,STEP_BACKWARD,TURN_LEFT,TURN_RIGHT,TURN_ANGLE,MOVE_BACKWARDS_RANDOMLY,RELEASE_OBJECT,TURN_TO_TARGET
*------------------------------*/
CORO_DEFINE(drive)
{
	//local variables of the coro routine for sensor reads
	CORO_LOCAL float value_gyro;
	CORO_LOCAL int speed_linear, speed_circular, speed_release, speed_linear2;
	CORO_LOCAL int _wait_stopped, value_color;
	CORO_LOCAL uint8_t release_engine;
	CORO_LOCAL uint8_t proximitysensor, gyrosensor, colorsensor;
	CORO_LOCAL float minimumdistance, currentdistance, currentpoint, targetpoint, startgyro;
	
	CORO_BEGIN();

	//initialises speed of engines for turning and moving
	speed_linear = max_speed * SPEED_LINEAR / 400;
	speed_circular = max_speed * SPEED_CIRCULAR / 400;
	speed_linear2 = max_speed * SPEED_LINEAR / 1600;

	for (; ; ) {
		/* Waiting new command */
		CORO_WAIT(moving != command);
		_wait_stopped = 0;


		//Execute command
		switch (command) {

			/*
			|  Do nothing
			*/
		case MOVE_NONE:
			_stop();
			_wait_stopped = 1;
			break;

			/*
			|  Moves engines to go forward forever
			*/
		case MOVE_FORWARD:
			_run_forever(-speed_linear, -speed_linear);
			break;

			/*
			|  Moves engines to go backwards forever
			*/
		case MOVE_BACKWARD:
			_run_forever(speed_linear, speed_linear);
			break;

			/*
			|  Moves engines to go backwards but turning one side slowly
			*/
		case MOVE_BACKWARDS_RANDOMLY:
			_run_forever(0.5*speed_linear, speed_linear);
			break;

			/*
			|  Turn left 90 degrees
			*/
		case TURN_LEFT:
			get_sensor_value0(sn_gyro, &value_gyro);
			turnleft();
			break;

			/*
			|  Turn right 90 degrees
			*/
		case TURN_RIGHT:
			get_sensor_value0(sn_gyro, &value_gyro);
			turnright();
			break;

			/*
			|  Turn to angle X
			*/
		case TURN_ANGLE:
			_run_to_rel_pos(speed_circular, DEGREE_TO_COUNT(-angle)
				, speed_circular, DEGREE_TO_COUNT(angle));
			_wait_stopped = 1;
			break;

			/*
			|  Move(step) backwards for 1 second
			*/
		case STEP_BACKWARD:
			_run_timed(speed_linear, speed_linear, 1000);
			_wait_stopped = 1;
			break;

			/*
			|  Release an obstacle. Lower the ramp, wait, bring it back up
			|  Author: Ariane
			*/
		case RELEASE_OBJECT:
			//initialise the releaser engine
			ev3_search_tacho_plugged_in(65, 0, &release_engine, 0);

			//release the ramp
			get_tacho_max_speed(release_engine, &speed_release);
			set_tacho_stop_action_inx(release_engine, TACHO_COAST);
			set_tacho_speed_sp(release_engine, -(0.05)*speed_release);
			set_tacho_time_sp(release_engine, 800);
			set_tacho_command_inx(release_engine, TACHO_RUN_TIMED);

			Sleep(800);

			//bring the ramp back up
			set_tacho_stop_action_inx(release_engine, TACHO_COAST);
			set_tacho_speed_sp(release_engine, (0.05)*speed_release);
			set_tacho_time_sp(release_engine, 800);
			set_tacho_command_inx(release_engine, TACHO_RUN_TIMED);

			break;

			/*
			|  Authors: Berkay, Ariane, Yasmine, Jermaine.
			|  We worked together on this one. Lots of movements and errors to fix
			*/

			/*
			|  1) Scan the -5/+5 degrees of gyro values and collect all distance values
			|  2) Turn towards the value of gyro where distance was minimum
			|  3) Get closer to the target till distance is 5cm
			|  4) Read the color of the target object
			|  5) Assign the values of the map with the obstacle being movable or not
			|  6) Go backwards to your old position
			|  7) (Optional) Fix back to your starting orientation
			*/
		case TURN_TO_TARGET:

			//initialise the sensors needed (One gyro, one proximity and one color)
			ev3_search_sensor(LEGO_EV3_GYRO, &gyrosensor, 0);
			ev3_search_sensor(LEGO_EV3_US, &proximitysensor, 0);
			ev3_search_sensor(LEGO_EV3_COLOR, &colorsensor, 0);

			get_sensor_value0(gyrosensor, &currentpoint);
			startgyro = currentpoint;
			Sleep(100);

			//Turn 5 degree left
			targetpoint = currentpoint - 5.0;
			while (currentpoint > targetpoint)
			{
				get_sensor_value0(gyrosensor, &currentpoint);
				_run_forever(-speed_linear2, speed_linear2);
			}

			//Turn 10 degree right and find the minimum distance
			get_sensor_value0(gyrosensor, &currentpoint);
			targetpoint = currentpoint + 10.0;

			minimumdistance = 99999;
			currentdistance = 99999;
			while (currentpoint < targetpoint)
			{
				get_sensor_value0(gyrosensor, &currentpoint);
				_run_forever(speed_linear2, -speed_linear2);

				get_sensor_value0(proximitysensor, &currentdistance);
				if (currentdistance < minimumdistance)
				{
					minimumdistance = currentdistance;
				}

			}

			printf("Minimum distance: %f \n", minimumdistance);

			//Turn left until the minimum distance is seen again
			printf("Turning until minimum distance is achieved\n");
			get_sensor_value0(gyrosensor, &targetpoint);
			get_sensor_value0(proximitysensor, &currentdistance);

			targetpoint = currentpoint - 10;

			//NOTE: there is a 0.2 cm tolerance on the distance. The distance does not measure the same everytime for the same object.
			while ((currentdistance > minimumdistance + 2))
			{
				//printf("CurrentDistance: %f DesiredDistance: %f Diff: %f\n", currentdistance, minimumdistance, currentdistance - minimumdistance);
				get_sensor_value0(gyrosensor, &currentpoint);
				get_sensor_value0(proximitysensor, &currentdistance);

				if (currentpoint < (startgyro - 5.0))
				{
					printf("distance not found. Break\n");
					break;
				}

				_run_forever(-(0.4)*speed_linear2, (0.4)*speed_linear2);


			}

			//Go forward to get closer.
			//NOTE: In case the object in front of you has disapeared (it might have been another robot passing by). The moment is disapears break the loop and skip this part!
			while (currentdistance > 50.0 && currentdistance < 120)
			{
				//printf("Forward\n");
				//printf("CurrentDistance: %f DesiredDistance: %f Diff: %f\n", currentdistance, minimumdistance, currentdistance - minimumdistance);
				get_sensor_value0(proximitysensor, &currentdistance);
				_run_forever(-(0.4)*speed_linear2, -(0.4)*speed_linear2);
			}

			//Read the color of the object in front of you
			if (!get_sensor_value(0, sn_color, &value_color) || (value_color < 0) || (value_color >= COLOR_COUNT)) {
				value_color = 0;
			}


			//If RED mark the map R
			if (value_color == 5)
			{
				printf("RED Movable Obstacle detected.\n");

				if (ROBOT_DIRECTION == 'E')
				{
					map[ROBOT_X + 1][ROBOT_Y] = 'R';
				}
				else if (ROBOT_DIRECTION == 'W')
				{
					if (ROBOT_X > 1) {
						map[ROBOT_X - 1][ROBOT_Y] = 'R';
					}
					else
					{
						map[0][ROBOT_Y] = 'R';
					}
				}
				else if (ROBOT_DIRECTION == 'N')
				{
					map[ROBOT_X][ROBOT_Y + 1] = 'R';
				}
				else if (ROBOT_DIRECTION == 'S')
				{
					if (ROBOT_Y > 1)
					{
						map[ROBOT_X][ROBOT_Y - 1] = 'R';
					}
					else
					{
						map[ROBOT_X][0] = 'R';
					}
				}

			}

			//If NOT Red mark the map B
			else
			{
				printf("NON Movable Obstacle detected.\n");

				if (ROBOT_DIRECTION == 'E')
				{
					map[ROBOT_X + 1][ROBOT_Y] = 'B';
				}
				else if (ROBOT_DIRECTION == 'W')
				{
					if (ROBOT_X > 1) {
						map[ROBOT_X - 1][ROBOT_Y] = 'B';
					}
					else
					{
						map[0][ROBOT_Y] = 'B';
					}
				}
				else if (ROBOT_DIRECTION == 'N')
				{
					map[ROBOT_X][ROBOT_Y + 1] = 'B';
				}
				else if (ROBOT_DIRECTION == 'S')
				{
					if (ROBOT_Y > 1)
					{
						map[ROBOT_X][ROBOT_Y - 1] = 'B';
					}
					else
					{
						map[ROBOT_X][0] = 'B';
					}
				}
			}


			//Go back to your previous distance before the engage
			while (currentdistance < minimumdistance)
			{
				//printf("Backwards\n");
				//printf("CurrentDistance: %f DesiredDistance: %f Diff: %f\n", currentdistance, minimumdistance, currentdistance - minimumdistance);
				get_sensor_value0(proximitysensor, &currentdistance);
				_run_forever((0.6)*speed_linear2, (0.6)*speed_linear2);
			}

			/*
			// Author: Yasmine. Fixing the orientation missorientates the robot when obstaclt is put with a weird angle. Not necessary
			//  (OPTIONAL) Fix your orientation back to the start up position.
			targetpoint = startgyro

			if(currentpoint > targetpoint)
			{
			while (currentpoint > targetpoint)
			{
				get_sensor_value0(gyrosensor, &currentpoint);
				_run_forever(-speed_linear2, speed_linear2);
			}
			}
			else{
			while (currentpoint < targetpoint)
			{
				get_sensor_value0(gyrosensor, &currentpoint);
				_run_forever(speed_linear2, -speed_linear2);
			}

			}
			*/

			//Stop moving
			_run_forever(0, 0);
			break;

		}

		moving = command;
		if (_wait_stopped) {
			/* Waiting the command is completed */
			CORO_WAIT(!_is_running());
			command = moving = MOVE_NONE;
		}
	}
	CORO_END();
}


/*--------------handle_brick_control -----
|  CORO handle_brick_control
|  Purpose:  Executes the commands coming to the keyboard of the EV3. Back button to kill the app without using the SSH
|  Author:  Jermaine
*------------------------------*/
CORO_DEFINE(handle_brick_control)
{
	CORO_LOCAL uint8_t keys, pressed = EV3_KEY__NONE_;
	CORO_BEGIN();
	for (; ; ) {

		/* Waiting any key is pressed or released */
		CORO_WAIT(ev3_read_keys(&keys) && (keys != pressed));
		pressed = keys;
		if (pressed & EV3_KEY_BACK) {
			printf("Brick Command: BACK BUTTON\n");
			/* Stop the vehicle */
			command = MOVE_NONE;
			/* Quit the program */
			app_alive = 0;
		}
		CORO_YIELD();
	}
	CORO_END();
}

/*
|  Authors: Berkay, Ariane, Yasmine, Jermaine. 
|  Again everyone worked on this CORO because it is the base of our robot. Details given below in the code
*/

/*--------------handle_ir_proximity -----
|  CORO handle_ir_proximity
|  Purpose:  Main decision making of the navigation of the robot across the map

|  1) Check the touch sensor
|		a) If touched go backwards a bit to fix it
|  2)  Check the proximity sensor
|		a) If nothing on the way check if we are on the edge of the platform (X and Y control for the open fence)
|			i) If on the edge turn and do not go forward
|			ii) if not just move forward as usual
|		b) If there is something on the way turn towards the target and identify its type (movable or not)
|			i) Check if we are at the edge of the map (topright, topleft, bottomright or bottomleft)
|				. if yes make logical decision to turn
|			ii) If not turn randomly to right or left
|
|  3)  With 10% probability decide to release an object. (Only if there is nothing infront of you to not get stuck)
*------------------------------*/
CORO_DEFINE(handle_ir_proximity)
{
	CORO_LOCAL float prox;
	CORO_LOCAL int touch_state, randoma, value_color;
	CORO_LOCAL const char const *colorarray[] = { "?", "BLACK", "BLUE", "GREEN", "YELLOW", "RED", "WHITE", "BROWN" };
	CORO_BEGIN();
	ev3_search_sensor(LEGO_EV3_TOUCH, &sn_touch, 0);
	for (; ; ) {
		get_sensor_value(0, sn_touch, &touch_state);

		//Touch Author: Jermaine
		//If something is touched go backwards
		if (touch_state != 0)
		{
			printf("Touched something I am going one step backwards.\n");
			command = MOVE_BACKWARDS_RANDOMLY;
		}


		//Nothing is touching me
		else
		{

			if (!get_sensor_value(0, sn_color, &value_color) || (value_color < 0) || (value_color >= COLOR_COUNT)) {
				value_color = 0;
			}


			get_sensor_value0(sn_sonar, &prox);

			printf("\r  Distance: %f Color: (%s) \n", prox, colorarray[value_color]);

			if (prox == 0) {
				command = MOVE_NONE;
			}

			//There is an object in front of me
			else if (prox < 120)
			{
				//Turn towards it read its color and all the work
				command = TURN_TO_TARGET;
				CORO_CALL(drive);

				//Position Frames Author: Yasmine
				//Depending on where you currently are either make a logical decision to turn or go full random
				if ((ROBOT_X < 6) && (ROBOT_Y < 6))
				{
					if (ROBOT_DIRECTION == 'E')
					{
						command = TURN_LEFT;
					}
					else if (ROBOT_DIRECTION == 'W')
					{
						command = TURN_RIGHT;
					}
					else if (ROBOT_DIRECTION == 'N')
					{
						command = TURN_RIGHT;
					}
					else if (ROBOT_DIRECTION == 'S')
					{
						command = TURN_LEFT;
					}

				}
				else if ((ROBOT_X < 6) && (ROBOT_Y > (MAP_HEIGHT-6)))
				{

					if (ROBOT_DIRECTION == 'E')
					{
						command = TURN_RIGHT;
					}
					else if (ROBOT_DIRECTION == 'W')
					{
						command = TURN_LEFT;
					}
					else if (ROBOT_DIRECTION == 'N')
					{
						command = TURN_RIGHT;
					}
					else if (ROBOT_DIRECTION == 'S')
					{
						command = TURN_LEFT;
					}

				}
				else if ((ROBOT_X >  (MAP_WIDTH - 6)) && (MAP_HEIGHT - 6))
				{


					if (ROBOT_DIRECTION == 'E')
					{
						command = TURN_LEFT;
					}
					else if (ROBOT_DIRECTION == 'W')
					{
						command = TURN_RIGHT;
					}
					else if (ROBOT_DIRECTION == 'N')
					{
						command = TURN_LEFT;
					}
					else if (ROBOT_DIRECTION == 'S')
					{
						command = TURN_RIGHT;
					}

				}
				else if ((ROBOT_X > (MAP_WIDTH - 6)) && (ROBOT_Y > (MAP_HEIGHT - 6)))
				{

					if (ROBOT_DIRECTION == 'E')
					{
						command = TURN_RIGHT;
					}
					else if (ROBOT_DIRECTION == 'W')
					{
						command = TURN_LEFT;
					}
					else if (ROBOT_DIRECTION == 'N')
					{
						command = TURN_LEFT;
					}
					else if (ROBOT_DIRECTION == 'S')
					{
						command = TURN_RIGHT;
					}
				}

				//Random Turns Author: Berkay
				//Somewhere else in the map. I can make a random turn! WOHOO
				else
				{
					randoma = rand() % 100;
					if (randoma < 50)
					{

						command = TURN_RIGHT;
					}
					else
					{
						command = TURN_LEFT;
					}

				}

			}

			//Road seems clear!
			else
			{	
				//Current location is green
				map[ROBOT_X][ROBOT_Y] = 'G';

				//No wall limitation author: Berkay
				//Stadium 2 Update: If on the edge of the stadium, even if there is nothing on the way DO NOT GO NEGATIVE Y
				if (ROBOT_Y < 2 && ROBOT_DIRECTION == 'S')
				{
					randoma = rand() % 100;
					if (randoma < 50)
					{
						command = TURN_RIGHT;
					}
					else
					{
						command = TURN_LEFT;
					}
				}

				//If not on the bototm edge of the map feel free to move forward
				else
				{
					command = MOVE_FORWARD;
				}

			}

			//Release obstacle author: Ariane
			//Can I release an obstacle please?
			if (releaseobject)
			{
				randoma = rand() % 100;
				//Release chance: 10%
				if (randoma < 10)
				{
					command = RELEASE_OBJECT;
					CORO_CALL(drive);
					Sleep(200);
					command = MOVE_NONE;
					CORO_CALL(drive);
					releaseobject = false;
				}
				else
				{
				//Okay maybe next time
				}

			}
		}

		CORO_YIELD();
	}
	CORO_END();
}




/*--------------setCoordinates -----
|  Function setCoordinates
|  Purpose:  Depending on the direction of the movement sents the new X and Y of the robot.
|  Update V2:Coordinates cannot overpass the height and width of the map. Or go negative. (Automatic recalibration to 0:0)
|  Main Author: Jermaine. 
|  Map Height Width Limits Author: Berkay
*------------------------------*/
void setCoordinates()
{
	if (command == MOVE_FORWARD)
	{

		if (ROBOT_DIRECTION == 'N')
		{
			ROBOT_Y++;
			if (ROBOT_Y > MAP_HEIGHT) { ROBOT_Y = MAP_HEIGHT; }
		}
		else if (ROBOT_DIRECTION == 'S')
		{
			ROBOT_Y--;
			if (ROBOT_Y < 1) { ROBOT_Y = 0; }
		}
		else if (ROBOT_DIRECTION == 'E')
		{
			ROBOT_X++;
			if (ROBOT_X > MAP_WIDTH) { ROBOT_X = MAP_WIDTH; }
		}
		else if (ROBOT_DIRECTION == 'W')
		{
			ROBOT_X--;
			if (ROBOT_X < 1) { ROBOT_X = 0; }
		}
	}
	else if (command == MOVE_BACKWARD || command == MOVE_BACKWARDS_RANDOMLY)
	{
		if (ROBOT_DIRECTION == 'N')
		{
			ROBOT_Y--;
			if (ROBOT_Y < 1) { ROBOT_Y = 0; }
		}
		else if (ROBOT_DIRECTION == 'S')
		{
			ROBOT_Y++;
			if (ROBOT_Y > MAP_HEIGHT) { ROBOT_Y = MAP_HEIGHT; }
		}
		else if (ROBOT_DIRECTION == 'E')
		{
			ROBOT_X--;
			if (ROBOT_X < 1) { ROBOT_X = 0; }
		}
		else if (ROBOT_DIRECTION == 'W')
		{
			ROBOT_X++;
			if (ROBOT_X > MAP_WIDTH) { ROBOT_X = MAP_WIDTH; }
		}
	}

}

/*--------------changeDirection -----
|  Function changeDirection
|  Purpose:  Depending on the command direction changes the robots direction
|  Note: Only left and right turns changes direction. The forward and backwards does not!
|  Author: Jermaine
*------------------------------*/
void changeDirection()
{
	if (command == TURN_LEFT) {
		if (ROBOT_DIRECTION == 'N')
		{
			ROBOT_DIRECTION = 'W';
		}
		else if (ROBOT_DIRECTION == 'S')
		{
			ROBOT_DIRECTION = 'E';
		}
		else if (ROBOT_DIRECTION == 'E')
		{
			ROBOT_DIRECTION = 'N';
		}
		else if (ROBOT_DIRECTION == 'W')
		{
			ROBOT_DIRECTION = 'S';
		}
	}
	if (command == TURN_RIGHT) {
		if (ROBOT_DIRECTION == 'N')
		{
			ROBOT_DIRECTION = 'E';
		}
		else if (ROBOT_DIRECTION == 'S')
		{
			ROBOT_DIRECTION = 'W';
		}
		else if (ROBOT_DIRECTION == 'E')
		{
			ROBOT_DIRECTION = 'S';
		}
		else if (ROBOT_DIRECTION == 'W')
		{
			ROBOT_DIRECTION = 'N';
		}
	}
}

int main(void)
{
	srand(time(NULL));
	int i;
	char s[256];
	int ss;
	int val;
	uint32_t n, ii;
	float value_compass, value_sonar, value_gyro;
	int val_color, val_touch;
#ifndef __ARM_ARCH_4T__
	/* Disable auto-detection of the brick (you have to set the correct address below) */
	ev3_brick_addr = "192.168.0.204";
#endif
	if (ev3_init() == -1) return (1);

#ifndef __ARM_ARCH_4T__
	printf("The EV3 brick auto-detection is DISABLED,\nwaiting %s online with plugged tacho...\n", ev3_brick_addr);
#else
	printf("Waiting tacho is plugged...\n");

#endif
	while (ev3_tacho_init() < 1) Sleep(1000);

	printf("*** ( EV3 ) Hello! ***\n");


	printf("Found tacho motors:\n");
	for (i = 0; i < DESC_LIMIT; i++) {
		if (ev3_tacho[i].type_inx != TACHO_TYPE__NONE_) {
			printf("  type = %s\n", ev3_tacho_type(ev3_tacho[i].type_inx));
			printf("  port = %s\n", ev3_tacho_port_name(i, s));
			printf("  port = %d %d\n", ev3_tacho_desc_port(i), ev3_tacho_desc_extport(i));
		}
	}

	//Run all sensors for tests
	ev3_sensor_init();

	printf("Found sensors:\n");
	for (i = 0; i < DESC_LIMIT; i++) {
		if (ev3_sensor[i].type_inx != SENSOR_TYPE__NONE_) {
			printf("  type = %s\n", ev3_sensor_type(ev3_sensor[i].type_inx));
			printf("  port = %s\n", ev3_sensor_port_name(i, s));
			if (get_sensor_mode(i, s, sizeof(s))) {
				printf("  mode = %s\n", s);
			}
			if (get_sensor_num_values(i, &n)) {
				for (ii = 0; ii < n; ii++) {
					if (get_sensor_value(ii, i, &val)) {
						printf("  value%d = %d\n", ii, val);
					}
				}
			}
		}
	}

	if (ev3_search_sensor(LEGO_EV3_TOUCH, &sn_touch, 0)) {
		//printf("TOUCH sensor is found, press BUTTON for EXIT...\n"); //NOT USED
	}


	if (ev3_search_sensor(LEGO_EV3_COLOR, &sn_color, 0)) {
		printf("COLOR sensor is found, reading COLOR...\n");
		if (!get_sensor_value(0, sn_color, &val_color) || (val_color < 0) || (val_color >= COLOR_COUNT)) {
			val_color = 0;
		}
		printf("\r(%s) \n", color[val_color]);
		fflush(stdout);
	}

	if (ev3_search_sensor(HT_NXT_COMPASS, &sn_compass, 0)) {
		printf("COMPASS found, reading compass...\n");
		if (!get_sensor_value0(sn_compass, &value_compass)) {
			value_compass = 0;
		}
		printf("\r(%f) \n", value_compass);
		fflush(stdout);
	}

	if (ev3_search_sensor(LEGO_EV3_US, &sn_sonar, 0)) {
		printf("SONAR found, reading sonar...\n");
		if (!get_sensor_value0(sn_sonar, &value_sonar)) {
			value_sonar = 0;
		}
		//printf( "\r(%f) \n", value_sonar);
		fflush(stdout);
	}

	if (ev3_search_sensor(LEGO_EV3_GYRO, &sn_gyro, 0)) {
		printf("SONAR found, reading gyro...\n");
		if (!get_sensor_value0(sn_gyro, &value_gyro)) {
			value_gyro = 0;
		}
		printf("\rvalue gyro: (%f) \n", value_gyro);
		fflush(stdout);
	}

	if (ev3_search_sensor(LEGO_EV3_TOUCH, &sn_touch, 0)) {
		printf("TOUCH sensor is found, reading COLOR...\n");
		if (!get_sensor_value(0, sn_touch, &val_touch)) {
			val_color = 0;
		}
		printf("\r(%d) \n", val_touch);
		fflush(stdout);
	}

	struct sockaddr_rc addr = { 0 };
	int status;

	ev3_tacho_init();

	ss = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
	addr.rc_family = AF_BLUETOOTH;
	addr.rc_channel = (uint8_t)1;
	str2ba(SERV_ADDR, &addr.rc_bdaddr);


	// Tries to connect to the server every 100ms until it is succeeded.
	status = connect(ss, (struct sockaddr *)&addr, sizeof(addr));
	while (status != 0)
	{
		status = connect(ss, (struct sockaddr *)&addr, sizeof(addr));
		printf("Connection Attempt\n");
		Sleep(100);
	}

	//Prints connected when server is reached and ready to send the commands
	printf("Connected!\n");

	//Taken from the robotclient.c
	//Wait for the start message from the server
	read_from_server(ss, string, 9);
	while (string[4] != MSG_START)
	{
		read_from_server(ss, string, 9);
	}
	//Start the timer and the program
	printf("Start message received!\n");
	app_alive = app_init();
	time(&start);

	//LETS PLAY
	while (app_alive) {
		
		CORO_CALL(handle_brick_control);
		CORO_CALL(handle_ir_proximity);
		CORO_CALL(drive);

		//Release message author: Ariane. Release position fix: Yasmine
		//If an obstacle is released notify the server where I dropped it
		//NOTE: the X and Y of the obstacle is one step away from the robot to the direction he is looking
		//NOTE: Our obstacle is NON-Movable
		if (!releaseobjectmessagesent && !releaseobject)
		{
			printf("Sending a release object message.\n");

			char obstaclemessage[58];
			*((uint16_t *)obstaclemessage) = msgId++;
			obstaclemessage[6] = ROBOT_X;
			obstaclemessage[8] = ROBOT_Y;

			if (ROBOT_DIRECTION == 'W')
			{
				map[ROBOT_X + 1][ROBOT_Y] = 'B';
				obstaclemessage[6] = ROBOT_X + 1;
			}
			else if (ROBOT_DIRECTION == 'E')
			{
				if (ROBOT_X > 1) {
					map[ROBOT_X - 1][ROBOT_Y] = 'B';
					obstaclemessage[6] = ROBOT_X - 1;
				}
				else
				{
					map[0][ROBOT_Y] = 'B';
					obstaclemessage[6] = 0;
				}
			}
			else if (ROBOT_DIRECTION == 'S')
			{
				map[ROBOT_X][ROBOT_Y + 1] = 'B';
				obstaclemessage[8] = ROBOT_Y + 1;
			}
			else if (ROBOT_DIRECTION == 'N')
			{
				if (ROBOT_Y > 1)
				{
					map[ROBOT_X][ROBOT_Y - 1] = 'B';
					obstaclemessage[8] = ROBOT_Y - 1;
				}
				else
				{
					obstaclemessage[8] = 0;
					map[ROBOT_X][0] = 'B';
				}
			}

			//I have released my only obstacle. No more messages for the server about obstacles
			releaseobjectmessagesent = true;

			obstaclemessage[2] = TEAM_ID;
			obstaclemessage[3] = 0xFF;
			obstaclemessage[4] = MSG_OBSTACLE;
			obstaclemessage[5] = 0;

			obstaclemessage[7] = 0x00;

			obstaclemessage[9] = 0x00;
			write(ss, obstaclemessage, 10);
		}

		//Update coordinates and the direction of the robot
		setCoordinates();
		changeDirection();

		//450ms with the given speed is a perfect match for 5cm between moves
		Sleep(450);

		//Stop and think what you are doing
		command = MOVE_NONE;
		CORO_CALL(drive);

		//Position messages author: Yasmine
		printf("Sending position: ROBOT_DIRECTION: %c, ROBOT_X: %d, ROBOT_Y: %d\n", ROBOT_DIRECTION, ROBOT_X, ROBOT_Y);
		char positionmessage[58];
		*((uint16_t *)positionmessage) = msgId++;
		positionmessage[2] = TEAM_ID;
		positionmessage[3] = 0xFF;
		positionmessage[4] = MSG_POSITION;
		positionmessage[5] = ROBOT_X;          /* x */
		positionmessage[6] = 0x00;
		positionmessage[7] = ROBOT_Y;		/* y */
		positionmessage[8] = 0x00;
		write(ss, positionmessage, 9);


		/*
		|  Authors: Berkay, Ariane, Yasmine, Jermaine. 
		|  Approach of map data sending was important. Everyone participated.
		|  Current approach: Send map every 20 steps (in case of app error occurence) and update every time
		|  v2: Only send the explored positions
		*/

		//Every once after 20 steps send the map to the server to keep it updates. In case the program dies or gets stuck before completing
		int temp;
		temp = msgId;
		if (msgId % 20 == 0)
		{
			msgId += 1000;
			//send mapdone

			printf("Sending the mapdata\n");
			char mapmessage[58];

			for (int i = 0; i < 256; i++)
			{
				for (int j = 0; j < 256; j++)
				{
					//Only send the coordinates that we have discovered
					if (map[i][j] == 'R' || map[i][j] == 'G' || map[i][j] == 'B')
					{
						printf("Sending one point x=%d y=%d and value= %c\n", i, j, map[i][j]);
						*((uint16_t *)mapmessage) = msgId++;
						mapmessage[2] = TEAM_ID;
						mapmessage[3] = 0xFF;
						mapmessage[4] = MSG_MAPDATA;
						mapmessage[5] = i;
						mapmessage[6] = 0x00;
						mapmessage[7] = j;
						mapmessage[8] = 0x00;

						mapmessage[9] = 0x00;
						mapmessage[10] = 0x00;
						mapmessage[11] = 0x00;

						if (map[i][j] == 'R')
						{
							mapmessage[9] = 254;
						}
						else if (map[i][j] == 'G')
						{
							mapmessage[10] = 254;
						}
						else if (map[i][j] == 'B')
						{
							mapmessage[11] = 254;
						}

						write(ss, mapmessage, 12);
						Sleep(1);
					}
				}

			}

			//Send the mapdone message
			char mapdonemessage[58];
			*((uint16_t *)mapdonemessage) = msgId++;
			mapdonemessage[2] = TEAM_ID;
			mapdonemessage[3] = 0xFF;
			mapdonemessage[4] = MSG_MAPDONE;
			write(ss, mapdonemessage, 5);


			//App 4 minute timeout author: Berkay
			//Every robot has 4 minutes in the game. So the app exits if the time is over
			time(&end);
			dif = difftime(end, start);
			if (dif > 235.0)
			{
				close(ss);
				ev3_uninit();
				printf("*** BYE: SegFault is out! ***\n");
				return (0);
			}
			else
			{
				printf("Seconds passed: %.2lf \n", dif);
			}

		}
		msgId = temp;

		//Kick Messages Author: Jermaine
		//IMPORTANT: Kick messages are ignored. Noone kicks segfault
		/*
		read_from_server(ss, string, 9);
		if (string[4] == MSG_KICK && string[5] == TEAM_ID)
		{
		printf("I am kicked :( \n");
		close(ss);
		ev3_uninit();
		printf("*** BYE SegFault is out! ***\n");
			return (0);
		}
		else
		{
			printf("I am NOT kicked!\n");
		}
		*/


	}


	close(ss);
	ev3_uninit();
	printf("*** BYE SegFault is out! ***\n");

	return (0);
}
