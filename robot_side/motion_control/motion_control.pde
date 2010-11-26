/*****************************************************
Project : Maximus
Version : 5
Date : 23/11/2010
Author : JBot
Company :
Comments:
-Change the 2 TICK_PER_MM constants with your own number (come from your encodeurs)
and the diameter of your wheels.
-Change the DIAMETER constant with the distance between the 2 wheels of your robot


Program type : Application
Clock frequency : 16,00 MHz
 *****************************************************/

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <math.h>

/* Wait function */
void delay_ms(uint16_t millis) {
	while(millis) {
		_delay_ms(1);
		millis--;
	}
}

/***********/
/* Defines */
/***********/
#define TICK_PER_MM_LEFT 	91.143671935
#define TICK_PER_MM_RIGHT 	91.143671935
#define DIAMETER 		155.0 // Distance between the 2 wheels

#define TWOPI 			6.2831853070
#define RAD2DEG 		57.2958 /* radians to degrees conversion */

// Types of motor
#define ALPHA_MOTOR 		0
#define DELTA_MOTOR 		1
#define LEFT_MOTOR 		2
#define RIGHT_MOTOR 		3

#define ALPHADELTA 		0
#define LEFTRIGHT 		1

#define COMMAND_DONE 		0
#define PROCESSING_COMMAND 	1
#define WAITING_BEGIN 		2
#define ERROR 			3

/***********************/
/* Specific structures */
/***********************/
struct motor {
	int type;
	signed long des_speed;
	signed long cur_speed;
	long last_error;
	long error_sum;
	int kP;
	int kI;
	int kD;
	signed long accel;
	signed long decel;
	signed long max_speed;
	float distance;
};

struct robot {
	double pos_X;
	double pos_Y;
	double theta;
	float yaw;
	float pitch;
	float roll;
	float yaw_offset;
};

struct RobotCommand {
	char state;
	double current_distance;
	double desired_distance;
};

/********************/
/* Global variables */
/********************/
struct motor left_motor;
struct motor right_motor;
struct motor alpha_motor;
struct motor delta_motor;

struct robot maximus;

struct RobotCommand bot_command_delta;
struct RobotCommand prev_bot_command_delta;
struct RobotCommand bot_command_alpha;

volatile long left_cnt = 0;
volatile long right_cnt = 0;

char output_ON = 0;
char motion_control_ON = 1;

int last_left = 0;
int last_right = 0;

int left_diff = 0;
int right_diff = 0;

float total_distance = 0.0;

unsigned int entier;
char display1, display2, display3, display4, display5, display6, display7;

char serial_command;


/***********************/
/* INTERRUPT FUNCTIONS */
/***********************/

// External Interrupt 4 service routine => PIN2
ISR(INT4_vect)
{
	//#asm("cli")
	if((PINB & 0x10) != 0) {
		if((PINE & 0x10) != 0)
			left_cnt--;
		else
			left_cnt++;
	}
	else {
		if((PINE & 0x10) == 0)
			left_cnt--;
		else
			left_cnt++;
	}

	//#asm("sei")
}

// External Interrupt 5 service routine => PIN3
ISR(INT5_vect)
{
	if((PINK & 0x80) != 0) {
		if((PINE & 0x20) != 0)
			right_cnt++;
		else
			right_cnt--;
	}
	else {
		if((PINE & 0x20) == 0)
			right_cnt++;
		else
			right_cnt--;
	}

}

// Pin change 0-7 interrupt service routine => PIN10
ISR(PCINT0_vect)
{
	if((PINE & 0x10) != 0) {
		if((PINB & 0x10) != 0){
			left_cnt++;
		}
		else
			left_cnt--;
	}
	else {
		if((PINB & 0x10) == 0){
			left_cnt++;
		}
		else
			left_cnt--;
	}

}

// Pin change 16-23 interrupt service routine => PIN-ADC15
ISR(PCINT2_vect)
{
	if((PINE & 0x20) != 0) {
		if((PINK & 0x80) != 0)
			right_cnt--;
		else
			right_cnt++;
	}
	else {
		if((PINK & 0x80) == 0)
			right_cnt--;
		else
			right_cnt++;
	}

}

// Timer 1 overflow interrupt service routine
ISR(TIMER1_OVF_vect)
{
	sei(); // enable interrupts
	get_Odometers();

	if(motion_control_ON == 1) {
		do_motion_control();
		if(output_ON == 1)
			move_motors(ALPHADELTA); // Update the motor speed
	}
	else {
		if(output_ON == 1)
			move_motors(LEFTRIGHT); // Update the motor speed
	}
}


/*************************/
/* SYSTEM INITIALIZATION */
/*************************/
void setup()
{
	// Crystal Oscillator division factor: 1
/*#pragma optsize-
	CLKPR=0x80;
	CLKPR=0x00;
#ifdef _OPTIMIZE_SIZE_
#pragma optsize+
#endif*/
         
        

	// Input/Output Ports initialization
	// Port A initialization
	// Func7=In Func6=In Func5=In Func4=In Func3=In Func2=In Func1=In Func0=In
	// State7=T State6=T State5=T State4=T State3=T State2=T State1=T State0=T
	PORTA=0x00;
	DDRA=0x00;

	// Port B initialization
	// Func7=In Func6=In Func5=In Func4=In Func3=In Func2=In Func1=In Func0=Out
	// State7=T State6=T State5=T State4=T State3=T State2=T State1=T State0=T
	PORTB=0x00;
	DDRB=0x00;

	// Port C initialization
	// Func7=In Func6=In Func5=In Func4=In Func3=In Func2=In Func1=In Func0=In
	// State7=T State6=T State5=T State4=T State3=T State2=T State1=T State0=T
	PORTC=0x00;
	DDRC=0x00;

	// Port D initialization
	// Func7=In Func6=In Func5=In Func4=In Func3=In Func2=In Func1=In Func0=In
	// State7=T State6=T State5=T State4=T State3=T State2=T State1=T State0=T
	PORTD=0x00;
	DDRD=0x00;

	// Port E initialization
	// Func2=In Func1=In Func0=In
	// State2=T State1=T State0=T
	PORTE=0x00;
	DDRE=0x00;

	PORTK=0x00;
	DDRK=0x00;

        pinMode(13, OUTPUT);
        

	// Timer/Counter 1 initialization
	// Clock source: System Clock
	// Clock value: 15,625 kHz
	// Mode: Ph. correct PWM top=00FFh
	// OC1A output: Discon.
	// OC1B output: Discon.
	// OC1C output: Discon.
	// Noise Canceler: Off
	// Input Capture on Falling Edge
	// Timer 1 Overflow Interrupt: On
	// Input Capture Interrupt: Off
	// Compare A Match Interrupt: Off
	// Compare B Match Interrupt: Off
	// Compare C Match Interrupt: Off
	TCCR1A=0x01;
	TCCR1B=0x05;
	TCNT1H=0x00;
	TCNT1L=0x00;
	ICR1H=0x00;
	ICR1L=0x00;
	OCR1AH=0x00;
	OCR1AL=0x00;
	OCR1BH=0x00;
	OCR1BL=0x00;
	OCR1CH=0x00;
	OCR1CL=0x00;

	// External Interrupt(s) initialization
	EICRA=0x00;
	EICRB=0x05;
	EIMSK=0x30;
	EIFR=0x30;
	// Interrupt on PCINT
	PCICR=0x05;
	PCIFR=0x05;
	PCMSK0=0x10;
	PCMSK1=0x00;
	PCMSK2=0x80;


	//ETIMSK=0x00;



	Serial.begin(9600);
	Serial1.begin(38400);

digitalWrite(13, HIGH);

	// Timer(s)/Counter(s) Interrupt(s) initialization
	TIMSK1 |= 0x01;
	TIFR1  |= 0x01;


	/******************************/
	/* Initialization of the code */
	/******************************/
	write_RoboClaw_speed_M1M2(128, 0, 0);

	init_motors(); // Init motors
	init_Robot(&maximus); // Init robot status

	init_Command(&bot_command_delta); // Init robot command
	init_Command(&bot_command_alpha); // Init robot command

	// Global enable interrupts
	sei();

	delay_ms(1000);

	// For ROS utilization
	output_ON = 1;
}

/******************/
/* MAIN CODE LOOP */
/******************/
void loop()
{
	// Place your code here

	delay_ms(50);

	if(output_ON == 1) {
//get_Odometers();
		/* Display for ROS */
		entier = (unsigned int) fabs(maximus.pos_X*10);
		display6 = (entier % 10) +48;
		entier = (unsigned int) (entier / 10);
		display5 = (entier % 10) +48;
		entier = (unsigned int) (entier / 10);
		display4 = (entier % 10) +48;
		entier = (unsigned int) (entier / 10);
		display3 = (entier % 10) +48;
		entier = (unsigned int) (entier / 10);
		display2 = (entier % 10) +48;
		entier = (unsigned int) (entier / 10);
		display1 = (entier % 10) +48;
		Serial.print('x');
		if(maximus.pos_X < 0)
			Serial.print('-');
		Serial.print(display1);
		Serial.print(display2);
		Serial.print(display3);
		Serial.print(display4);
		Serial.print(display5);
		Serial.print(display6);

		entier = (unsigned int) fabs(maximus.pos_Y*10);
		display6 = (entier % 10) +48;
		entier = (unsigned int) (entier / 10);
		display5 = (entier % 10) +48;
		entier = (unsigned int) (entier / 10);
		display4 = (entier % 10) +48;
		entier = (unsigned int) (entier / 10);
		display3 = (entier % 10) +48;
		entier = (unsigned int) (entier / 10);
		display2 = (entier % 10) +48;
		entier = (unsigned int) (entier / 10);
		display1 = (entier % 10) +48;
		Serial.print('y');
		if(maximus.pos_Y < 0)
			Serial.print('-');
		Serial.print(display1);
		Serial.print(display2);
		Serial.print(display3);
		Serial.print(display4);
		Serial.print(display5);
		Serial.print(display6);

		entier = (unsigned int) fabs(maximus.theta * 10000);
		//entier = (unsigned int) fabs(maximus.theta * RAD2DEG);
		display6 = (entier % 10) +48;
		entier = (unsigned int) (entier / 10);
		display5 = (entier % 10) +48;
		entier = (unsigned int) (entier / 10);
		display4 = (entier % 10) +48;
		entier = (unsigned int) (entier / 10);
		display3 = (entier % 10) +48;
		entier = (unsigned int) (entier / 10);
		display2 = (entier % 10) +48;
		entier = (unsigned int) (entier / 10);
		display1 = (entier % 10) +48;
		Serial.print('t');
		if(maximus.theta < 0)
			Serial.print('-');
		Serial.print(display1);
		Serial.print(display2);
		Serial.print(display3);
		Serial.print(display4);
		Serial.print(display5);
		Serial.print(display6);


		Serial.print('\n');
		Serial.print('\r');

	}

	if(Serial.available() >= (1+6)){
		serial_command = Serial.read();
		switch(serial_command) {
			/*
			   case 'a' :// positive
			   angular_from_ROS(1);
			   break;
			   case 'A' : // negative
			   angular_from_ROS(0);
			   break;
			   case 'l' :
			   linear_from_ROS(1);
			   break;
			   case 'L' :
			   linear_from_ROS(0);
			   break;*/
			// case 'T' :
			// start = 1;
			// mycounter = 0;
			// //compute_distance_ramp(&bot_command, &delta_motor, convert_dist2ticks(350), general_time_counter);
			// Serial.read();
			// Serial.read();
			// Serial.read();
			// Serial.read();
			// Serial.read();
			// Serial.read();
			// break;
			// case 'P' :
			// change_alphakP();
			// break;
			// case 'D' :
			// change_alphakD();
			// break;
			// case 'I' :
			// change_alphakI();
			// break;
			// case 'p' :
			// change_deltakP();
			// break;
			// case 'd' :
			// change_deltakD();
			// break;
			// case 'i' :
			// change_deltakI();
			// break;
			case 'G' :
				get_goal_from_ROS();
				break;
			case 'A' :
				set_alpha();
				break;
			case 'D' :
				set_delta();
				break;      
			case 'S' :
				stop_position_motion_control();
				break;
			case 's' :
				start_position_motion_control();
				break;
			case 'P' :
				stop_robot();
				break;
			case 'M' :
				set_maxspeed_delta();
				break;
			case 'm' :
				set_maxspeed_alpha();
				break;	
			case 'x' :
				set_X();
				break;
			case 'y' :
				set_Y();
				break;
			case 't' :
				set_theta();
				break;	

		}
	}

} 

/****************************/
/* INITIALIZATION FUNCTIONS */
/****************************/
void init_Robot(struct robot *my_robot) {
	my_robot->pos_X = -1459; // -700         
	my_robot->pos_Y = 192; // 700          
	my_robot->theta = 0; // PI/2
	my_robot->yaw = 0.0;
	my_robot->pitch = 0.0;
	my_robot->roll = 0.0;
	my_robot->yaw_offset = 0.0;
}

void init_Command(struct RobotCommand *cmd) {
	cmd->state = COMMAND_DONE;
	cmd->current_distance = 0;
	cmd->desired_distance = 0;
}

void init_motors(void) {
	/* Left motor initialization */
	left_motor.type = LEFT_MOTOR;
	left_motor.des_speed = 0;
	left_motor.cur_speed = 0;
	left_motor.last_error = 0;
	left_motor.error_sum = 0;
	left_motor.kP = 12;
	left_motor.kI = 6;
	left_motor.kD = 1;
	left_motor.accel = 5;
	left_motor.decel = 5;
	left_motor.max_speed = 30;
	left_motor.distance = 0.0;

	/* Right motor initialization */
	right_motor.type = RIGHT_MOTOR;
	right_motor.des_speed = 0;
	right_motor.cur_speed = 0;
	right_motor.last_error = 0;
	right_motor.error_sum = 0;
	right_motor.kP = 12;
	right_motor.kI = 6;
	right_motor.kD = 1;
	right_motor.accel = 5;
	right_motor.decel = 5;
	right_motor.max_speed = 30;
	right_motor.distance = 0.0;

	/* Alpha motor initialization */
	alpha_motor.type = ALPHA_MOTOR;
	alpha_motor.des_speed = 0;
	alpha_motor.cur_speed = 0;
	alpha_motor.last_error = 0;
	alpha_motor.error_sum = 0;
	alpha_motor.kP = 600;
	alpha_motor.kI = 0;
	alpha_motor.kD = 100;
	alpha_motor.accel = 300;
	alpha_motor.decel = 500;
	alpha_motor.max_speed = 8000; //12000
	alpha_motor.distance = 0.0;

	/* Delta motor initialization */
	delta_motor.type = DELTA_MOTOR;
	delta_motor.des_speed = 0;
	delta_motor.cur_speed = 0;
	delta_motor.last_error = 0;
	delta_motor.error_sum = 0;
	delta_motor.kP = 600;
	delta_motor.kI = 0;
	delta_motor.kD = 100;
	delta_motor.accel = 300;
	delta_motor.decel = 500;
	delta_motor.max_speed = 20000;
	delta_motor.distance = 0.0;
}


/*******************************/
/* ROBO CLAW FUNCTIONS */
/*******************************/
// Used to change the speed value of motor 1
void write_RoboClaw_speed_M1(char addr, signed long speed) {
	char checkSUM;
	checkSUM = (addr + 35 + ((char) ((speed >> 24) & 0xFF)) + ((char) ((speed >> 16) & 0xFF)) + ((char) ((speed >> 8) & 0xFF)) + ((char) (speed & 0xFF)) ) & 0x7F;
	Serial1.print(addr);
	Serial1.print(35);
	Serial1.print( ((char) ((speed >> 24) & 0xFF)) );
	Serial1.print( ((char) ((speed >> 16) & 0xFF)) );
	Serial1.print( ((char) ((speed >> 8) & 0xFF)) );
	Serial1.print( ((char) (speed & 0xFF)) );

	Serial1.print(checkSUM);
}

// Used to change the speed value of motor 2
void write_RoboClaw_speed_M2(char addr, signed long speed) {
	char checkSUM;
	checkSUM = (addr + 36 + ((char) ((speed >> 24) & 0xFF)) + ((char) ((speed >> 16) & 0xFF)) + ((char) ((speed >> 8) & 0xFF)) + ((char) (speed & 0xFF)) ) & 0x7F;
	Serial1.print(addr);
	Serial1.print(36);
	Serial1.print( ((char) ((speed >> 24) & 0xFF)) );
	Serial1.print( ((char) ((speed >> 16) & 0xFF)) );
	Serial1.print( ((char) ((speed >> 8) & 0xFF)) );
	Serial1.print( ((char) (speed & 0xFF)) );

	Serial1.print(checkSUM);
}

// Used to change the speed value of motors 1 and 2
void write_RoboClaw_speed_M1M2(char addr, signed long speedM1, signed long speedM2) {
	char checkSUM;
	checkSUM = (addr + 37 + ((char) ((speedM1 >> 24) & 0xFF)) + ((char) ((speedM1 >> 16) & 0xFF)) + ((char) ((speedM1 >> 8) & 0xFF)) + ((char) (speedM1 & 0xFF)) + ((char) ((speedM2 >> 24) & 0xFF)) + ((char) ((speedM2 >> 16) & 0xFF)) + ((char) ((speedM2 >> 8) & 0xFF)) + ((char) (speedM2 & 0xFF)) ) & 0x7F;
	Serial1.print(addr, BYTE);
	Serial1.print(37, BYTE);
	Serial1.print( ((char) ((speedM1 >> 24) & 0xFF)) , BYTE);
	Serial1.print( ((char) ((speedM1 >> 16) & 0xFF)) , BYTE);
	Serial1.print( ((char) ((speedM1 >> 8) & 0xFF)) , BYTE);
	Serial1.print( ((char) (speedM1 & 0xFF)) , BYTE);

	Serial1.print( ((char) ((speedM2 >> 24) & 0xFF)) , BYTE);
	Serial1.print( ((char) ((speedM2 >> 16) & 0xFF)) , BYTE);
	Serial1.print( ((char) ((speedM2 >> 8) & 0xFF)) , BYTE);
	Serial1.print( ((char) (speedM2 & 0xFF)) , BYTE);

	Serial1.print(checkSUM , BYTE);
}

// Used to change the speed value of motor 1 and 2 during a specific distance
void write_RoboClaw_speed_dist_M1M2(char addr, signed long speedM1, signed long distanceM1, signed long speedM2, signed long distanceM2) {
	char checkSUM;
	checkSUM = (addr + 43 + ((char) ((speedM1 >> 24) & 0xFF)) + ((char) ((speedM1 >> 16) & 0xFF)) + ((char) ((speedM1 >> 8) & 0xFF)) + ((char) (speedM1 & 0xFF)) + ((char) ((speedM2 >> 24) & 0xFF)) + ((char) ((speedM2 >> 16) & 0xFF)) + ((char) ((speedM2 >> 8) & 0xFF)) + ((char) (speedM2 & 0xFF)) + ((char) ((distanceM1 >> 24) & 0xFF)) + ((char) ((distanceM1 >> 16) & 0xFF)) + ((char) ((distanceM1 >> 8) & 0xFF)) + ((char) (distanceM1 & 0xFF)) + ((char) ((distanceM2 >> 24) & 0xFF)) + ((char) ((distanceM2 >> 16) & 0xFF)) + ((char) ((distanceM2 >> 8) & 0xFF)) + ((char) (distanceM2 & 0xFF)) + 1) & 0x7F;
	Serial1.print(addr);
	Serial1.print(43);
	Serial1.print( ((char) ((speedM1 >> 24) & 0xFF)) );
	Serial1.print( ((char) ((speedM1 >> 16) & 0xFF)) );
	Serial1.print( ((char) ((speedM1 >> 8) & 0xFF)) );
	Serial1.print( ((char) (speedM1 & 0xFF)) );

	Serial1.print( ((char) ((distanceM1 >> 24) & 0xFF)) );
	Serial1.print( ((char) ((distanceM1 >> 16) & 0xFF)) );
	Serial1.print( ((char) ((distanceM1 >> 8) & 0xFF)) );
	Serial1.print( ((char) (distanceM1 & 0xFF)) );

	Serial1.print( ((char) ((speedM2 >> 24) & 0xFF)) );
	Serial1.print( ((char) ((speedM2 >> 16) & 0xFF)) );
	Serial1.print( ((char) ((speedM2 >> 8) & 0xFF)) );
	Serial1.print( ((char) (speedM2 & 0xFF)) );

	Serial1.print( ((char) ((distanceM2 >> 24) & 0xFF)) );
	Serial1.print( ((char) ((distanceM2 >> 16) & 0xFF)) );
	Serial1.print( ((char) ((distanceM2 >> 8) & 0xFF)) );
	Serial1.print( ((char) (distanceM2 & 0xFF)) );

	Serial1.print(1);

	Serial1.print(checkSUM);
}

// Used to change the speed value of motor 1 and 2 during a specific distance with a specific acceleration
void write_RoboClaw_allcmd_M1M2(char addr, signed long accel, signed long speedM1, signed long distanceM1, signed long speedM2, signed long distanceM2) {
	char checkSUM;
	checkSUM = (addr + 46 + ((char) ((accel >> 24) & 0xFF)) + ((char) ((accel >> 16) & 0xFF)) + ((char) ((accel >> 8) & 0xFF)) + ((char) (accel & 0xFF)) + ((char) ((speedM1 >> 24) & 0xFF)) + ((char) ((speedM1 >> 16) & 0xFF)) + ((char) ((speedM1 >> 8) & 0xFF)) + ((char) (speedM1 & 0xFF)) + ((char) ((speedM2 >> 24) & 0xFF)) + ((char) ((speedM2 >> 16) & 0xFF)) + ((char) ((speedM2 >> 8) & 0xFF)) + ((char) (speedM2 & 0xFF)) + ((char) ((distanceM1 >> 24) & 0xFF)) + ((char) ((distanceM1 >> 16) & 0xFF)) + ((char) ((distanceM1 >> 8) & 0xFF)) + ((char) (distanceM1 & 0xFF)) + ((char) ((distanceM2 >> 24) & 0xFF)) + ((char) ((distanceM2 >> 16) & 0xFF)) + ((char) ((distanceM2 >> 8) & 0xFF)) + ((char) (distanceM2 & 0xFF)) + 1) & 0x7F;

	Serial1.print(addr);
	Serial1.print(46);

	Serial1.print( ((char) ((accel >> 24) & 0xFF)) );
	Serial1.print( ((char) ((accel >> 16) & 0xFF)) );
	Serial1.print( ((char) ((accel >> 8) & 0xFF)) );
	Serial1.print( ((char) (accel & 0xFF)) );

	Serial1.print( ((char) ((speedM1 >> 24) & 0xFF)) );
	Serial1.print( ((char) ((speedM1 >> 16) & 0xFF)) );
	Serial1.print( ((char) ((speedM1 >> 8) & 0xFF)) );
	Serial1.print( ((char) (speedM1 & 0xFF)) );

	Serial1.print( ((char) ((distanceM1 >> 24) & 0xFF)) );
	Serial1.print( ((char) ((distanceM1 >> 16) & 0xFF)) );
	Serial1.print( ((char) ((distanceM1 >> 8) & 0xFF)) );
	Serial1.print( ((char) (distanceM1 & 0xFF)) );

	Serial1.print( ((char) ((speedM2 >> 24) & 0xFF)) );
	Serial1.print( ((char) ((speedM2 >> 16) & 0xFF)) );
	Serial1.print( ((char) ((speedM2 >> 8) & 0xFF)) );
	Serial1.print( ((char) (speedM2 & 0xFF)) );

	Serial1.print( ((char) ((distanceM2 >> 24) & 0xFF)) );
	Serial1.print( ((char) ((distanceM2 >> 16) & 0xFF)) );
	Serial1.print( ((char) ((distanceM2 >> 8) & 0xFF)) );
	Serial1.print( ((char) (distanceM2 & 0xFF)) );

	Serial1.print(1);

	Serial1.print(checkSUM);
}

/*******************************/
/* MOTION CONTROL FUNCTIONS */
/*******************************/
void do_motion_control(void) {
	// PID distance
	if( (bot_command_alpha.state == WAITING_BEGIN) || (bot_command_alpha.state == PROCESSING_COMMAND) ) { // If alpha motor have not finished its movement 

	}
	else {
		if( (bot_command_delta.state != PROCESSING_COMMAND) && (prev_bot_command_delta.state == WAITING_BEGIN) ) {
			prev_bot_command_delta.state = PROCESSING_COMMAND;
			set_new_command(&bot_command_delta, prev_bot_command_delta.desired_distance);
		}
	}    
	delta_motor.des_speed = compute_position_PID(&bot_command_delta, &delta_motor);


	// PID angle
	alpha_motor.des_speed = compute_position_PID(&bot_command_alpha, &alpha_motor);
	if( bot_command_alpha.state == WAITING_BEGIN) {
		bot_command_alpha.state = PROCESSING_COMMAND;
	}

}

void set_new_command(struct RobotCommand *cmd, long distance) {
	cmd->state = WAITING_BEGIN;
	//#asm("cli") // disable interrupts
	cmd->current_distance = 0;
	cmd->desired_distance = distance;
	//#asm("sei") // enable interrupts
}

long compute_position_PID(struct RobotCommand *cmd, struct motor *used_motor) {
	long P,I,D;
	long errDif, err;
	long tmp = 0;

	if( cmd->state == WAITING_BEGIN) {
		cmd->state = PROCESSING_COMMAND;
	}

	if(used_motor->type == ALPHA_MOTOR)
		err = cmd->desired_distance*10 - cmd->current_distance*10*RAD2DEG;
	else    
		err = cmd->desired_distance - cmd->current_distance;

	used_motor->error_sum += err; //Somme les erreurs depuis le debut
	if(used_motor->error_sum > 10)
		used_motor->error_sum = 10;
	if(used_motor->error_sum < -10)
		used_motor->error_sum = -10;

	errDif = err - used_motor->last_error; //Calcule la variation de l'erreur

	used_motor->last_error = err;

	P = err * used_motor->kP; //Proportionnelle
	I = used_motor->error_sum * used_motor->kI; //Integrale
	D = errDif * used_motor->kD; //Derivee

	tmp = (P + I + D);

	if(tmp > (used_motor->des_speed + used_motor->accel))
		tmp = (used_motor->des_speed + used_motor->accel);
	else if(tmp < (used_motor->des_speed - used_motor->accel))
		tmp = (used_motor->des_speed - used_motor->accel);

	if(tmp > (used_motor->max_speed))
		tmp = (used_motor->max_speed);
	if(tmp < -(used_motor->max_speed))
		tmp = -(used_motor->max_speed);

	if( (cmd->state == PROCESSING_COMMAND) && (abs(err) < 8) &&  (abs(errDif) < 8) ) {
		cmd->state = COMMAND_DONE;           
	}

	return tmp;
}

double distance_coord(struct robot *my_robot, double x1, double y1) {
	double x = 0;
	x = sqrt( pow(fabs(x1-my_robot->pos_X),2) + pow(fabs(y1-my_robot->pos_Y),2) );        
	return x;
}

double angle_coord(struct robot *my_robot, double x1, double y1) {
	double angletodo =0;
	if( (x1 < my_robot->pos_X) && (y1 < my_robot->pos_Y) ) { 
		angletodo = -PI/2 - atan(fabs( (x1 - my_robot->pos_X) / (y1 - my_robot->pos_Y) ) );
	}
	else if( (x1 > my_robot->pos_X) && (y1 < my_robot->pos_Y) ) { 
		angletodo = - atan(fabs( (y1 - my_robot->pos_Y) / (x1 - my_robot->pos_X) ) );
	}
	else if( (x1 > my_robot->pos_X) && (y1 > my_robot->pos_Y) ) { 
		angletodo = atan(fabs( (y1 - my_robot->pos_Y) / (x1 - my_robot->pos_X) ) );
	}
	else if( (x1 < my_robot->pos_X) && (y1 > my_robot->pos_Y) ) { 
		angletodo = PI/2 + atan(fabs( (x1 - my_robot->pos_X) / (y1 - my_robot->pos_Y) ) );
	}
	else if( (x1 < my_robot->pos_X) && (y1 == my_robot->pos_Y) ) { // 
		angletodo = -PI;
	}
	else if( (x1 > my_robot->pos_X) && (y1 == my_robot->pos_Y) ) { // 
		angletodo = 0;
	}
	else if( (x1 == my_robot->pos_X) && (y1 < my_robot->pos_Y) ) { // 
		angletodo = -PI/2;
	}
	else if( (x1 == my_robot->pos_X) && (y1 > my_robot->pos_Y) ) { // 
		angletodo = PI/2;
	}
	else angletodo = 0;  

	angletodo = angletodo - my_robot->theta;

	if(angletodo > PI)
		angletodo = angletodo - 2*PI;
	if(angletodo < -PI)
		angletodo = 2*PI + angletodo;

	return angletodo;
}

void goto_xy(double x, double y) {  
	double ang, dist;

	ang = angle_coord(&maximus, x, y)*RAD2DEG;
	set_new_command(&bot_command_alpha, ang);

	dist = distance_coord(&maximus, x, y);
	set_new_command(&prev_bot_command_delta, dist);
}

/*******************************/
/* CONVERSION FUNCTIONS */
/*******************************/
signed long convert_dist2ticks(signed long distance) {
	return (distance * TICK_PER_MM_RIGHT);
}

signed long convert_ticks2dist(signed long ticks) {
	return (ticks / TICK_PER_MM_RIGHT);
}

/********************/
/* MOTORS FUNCTIONS */
/********************/
	void move_motors(char type) {
		if(type == ALPHADELTA)
			write_RoboClaw_speed_M1M2(128, delta_motor.des_speed - alpha_motor.des_speed, delta_motor.des_speed + alpha_motor.des_speed);
		else 
			write_RoboClaw_speed_M1M2(128, left_motor.des_speed, right_motor.des_speed);
	}

void update_motor(struct motor *used_motor) {
	switch(used_motor->type) {
		case LEFT_MOTOR :
			used_motor->cur_speed = left_diff;
			break;
		case RIGHT_MOTOR :
			used_motor->cur_speed = right_diff;
			break;
		case ALPHA_MOTOR :
			used_motor->cur_speed = left_diff - right_diff;
			break;
		case DELTA_MOTOR :
			used_motor->cur_speed = (left_diff + right_diff)/2;
			break;
		default : break;
	}
}


/********************************/
/* POSITION ESTIMATION FUNCTION */
/********************************/

/* Compute the position of the robot */
void get_Odometers(void) {
	long left_wheel = 0;
	long right_wheel = 0;

	double left_mm = 0.0;
	double right_mm = 0.0;

	double distance = 0.0;


	left_wheel = left_cnt;
	right_wheel = right_cnt;

	left_diff = last_left - left_wheel;
	right_diff = last_right - right_wheel;

	last_left = left_wheel;
	last_right = right_wheel;

	left_mm = ((double) left_diff) / TICK_PER_MM_LEFT;
	right_mm = ((double) right_diff) / TICK_PER_MM_RIGHT;

	distance = (left_mm + right_mm) / 2;
	total_distance += distance;
	bot_command_delta.current_distance += distance;

	maximus.theta += (right_mm - left_mm) / DIAMETER;
	bot_command_alpha.current_distance += (right_mm - left_mm) / DIAMETER;

	if( maximus.theta > PI )
		maximus.theta -= TWOPI;
	if( maximus.theta < (-PI) )
		maximus.theta += TWOPI;

	maximus.pos_Y += distance * sin(maximus.theta);
	maximus.pos_X += distance * cos(maximus.theta);

	update_motor(&left_motor);
	update_motor(&right_motor);
	update_motor(&alpha_motor);
	update_motor(&delta_motor);


}

/***********************/
/* Interface Functions */
/***********************/
void angular_from_ROS(char sign) {
	signed long tmp = 0;

	tmp = (tmp * 10) + (Serial.read() - 48);
	tmp = (tmp * 10) + (Serial.read() - 48);
	tmp = (tmp * 10) + (Serial.read() - 48);
	tmp = (tmp * 10) + (Serial.read() - 48);
	tmp = (tmp * 10) + (Serial.read() - 48);
	Serial.read();
	if(sign == 0) // negative
		alpha_motor.des_speed = -tmp;
	else
		alpha_motor.des_speed = tmp;
}

void linear_from_ROS(char sign) {
	signed long tmp = 0;

	tmp = (tmp * 10) + (Serial.read() - 48);
	tmp = (tmp * 10) + (Serial.read() - 48);
	tmp = (tmp * 10) + (Serial.read() - 48);
	tmp = (tmp * 10) + (Serial.read() - 48);
	tmp = (tmp * 10) + (Serial.read() - 48);
	Serial.read();
	if(sign == 0) // negative
		delta_motor.des_speed = -tmp;
	else
		delta_motor.des_speed = tmp;
}

void change_alphakP(void) {
	signed long tmp = 0;

	tmp = (tmp * 10) + (Serial.read() - 48);
	tmp = (tmp * 10) + (Serial.read() - 48);
	tmp = (tmp * 10) + (Serial.read() - 48);
	tmp = (tmp * 10) + (Serial.read() - 48);
	tmp = (tmp * 10) + (Serial.read() - 48);
	Serial.read();
	alpha_motor.kP = tmp;
}

void change_alphakD(void) {
	signed long tmp = 0;

	tmp = (tmp * 10) + (Serial.read() - 48);
	tmp = (tmp * 10) + (Serial.read() - 48);
	tmp = (tmp * 10) + (Serial.read() - 48);
	tmp = (tmp * 10) + (Serial.read() - 48);
	tmp = (tmp * 10) + (Serial.read() - 48);
	Serial.read();
	alpha_motor.kD = tmp;
}

void change_alphakI(void) {
	signed long tmp = 0;

	tmp = (tmp * 10) + (Serial.read() - 48);
	tmp = (tmp * 10) + (Serial.read() - 48);
	tmp = (tmp * 10) + (Serial.read() - 48);
	tmp = (tmp * 10) + (Serial.read() - 48);
	tmp = (tmp * 10) + (Serial.read() - 48);
	Serial.read();
	alpha_motor.kI = tmp;
}


void change_deltakP(void) {
	signed long tmp = 0;

	tmp = (tmp * 10) + (Serial.read() - 48);
	tmp = (tmp * 10) + (Serial.read() - 48);
	tmp = (tmp * 10) + (Serial.read() - 48);
	tmp = (tmp * 10) + (Serial.read() - 48);
	tmp = (tmp * 10) + (Serial.read() - 48);
	Serial.read();
	delta_motor.kP = tmp;
}

void change_deltakD(void) {
	signed long tmp = 0;

	tmp = (tmp * 10) + (Serial.read() - 48);
	tmp = (tmp * 10) + (Serial.read() - 48);
	tmp = (tmp * 10) + (Serial.read() - 48);
	tmp = (tmp * 10) + (Serial.read() - 48);
	tmp = (tmp * 10) + (Serial.read() - 48);
	Serial.read();
	delta_motor.kD = tmp;
}

void change_deltakI(void) {
	signed long tmp = 0;

	tmp = (tmp * 10) + (Serial.read() - 48);
	tmp = (tmp * 10) + (Serial.read() - 48);
	tmp = (tmp * 10) + (Serial.read() - 48);
	tmp = (tmp * 10) + (Serial.read() - 48);
	tmp = (tmp * 10) + (Serial.read() - 48);
	Serial.read();
	delta_motor.kI = tmp;
}   

void get_goal_from_ROS(void) {
	signed long x = 0, y = 0;

	x = (x * 10) + (Serial.read() - 48);
	x = (x * 10) + (Serial.read() - 48);
	x = (x * 10) + (Serial.read() - 48);
	x = x*(-10);

	y = (y * 10) + (Serial.read() - 48);
	y = (y * 10) + (Serial.read() - 48);
	y = (y * 10) + (Serial.read() - 48);
	y = y*10;

	goto_xy( ((double)x/2), ((double)y/2) );
}

void stop_position_motion_control(void) {
	left_motor.des_speed = 0;
	right_motor.des_speed = 0;
	motion_control_ON = 0;
}

void start_position_motion_control(void) {
	set_new_command(&bot_command_alpha, 0);
	set_new_command(&prev_bot_command_delta, 0);
	motion_control_ON = 1;
}

void stop_robot(void) {
	set_new_command(&bot_command_alpha, 0);
	set_new_command(&prev_bot_command_delta, 0);
	right_motor.des_speed = 0;
	left_motor.des_speed = 0;
	write_RoboClaw_speed_M1M2(128, 0, 0);
}

void set_alpha(void) {
	signed long tmp = 0;
	if(Serial.read() == '+') {
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);

		set_new_command(&bot_command_alpha, (float)tmp / (100));
	}
	else {
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);

		set_new_command(&bot_command_alpha, (float)tmp / (-100));
	}
}

void set_delta(void) {
	signed long tmp = 0;
	if(Serial.read() == '+') {
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);

		set_new_command(&bot_command_delta, (float)tmp / (10));
	}
	else {
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);

		set_new_command(&bot_command_delta, (float)tmp / (-10));
	}
}

void set_maxspeed_delta(void) {
	signed long tmp = 0;
	tmp = (tmp * 10) + (Serial.read() - 48);
	tmp = (tmp * 10) + (Serial.read() - 48);
	tmp = (tmp * 10) + (Serial.read() - 48);
	tmp = (tmp * 10) + (Serial.read() - 48);
	tmp = (tmp * 10) + (Serial.read() - 48);
	tmp = (tmp * 10) + (Serial.read() - 48);
	delta_motor.max_speed = tmp;	
}

void set_maxspeed_alpha(void) {
	signed long tmp = 0;
	tmp = (tmp * 10) + (Serial.read() - 48);
	tmp = (tmp * 10) + (Serial.read() - 48);
	tmp = (tmp * 10) + (Serial.read() - 48);
	tmp = (tmp * 10) + (Serial.read() - 48);
	tmp = (tmp * 10) + (Serial.read() - 48);
	tmp = (tmp * 10) + (Serial.read() - 48);
	alpha_motor.max_speed = tmp;	
}

void set_X(void) {
	signed long tmp = 0;
	if(Serial.read() == '+') {
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);
		maximus.pos_X = (double)tmp / 100;
	}
	else {
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);
		maximus.pos_X = -(double)tmp / 100;
	}
}

void set_Y(void) {
	signed long tmp = 0;
	if(Serial.read() == '+') {
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);
		maximus.pos_Y = (double)tmp / 100;
	}
	else {
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);
		maximus.pos_Y = -(double)tmp / 100;
	}
}

void set_theta(void) {
	signed long tmp = 0;
	if(Serial.read() == '+') {
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);
		maximus.theta = (double)tmp / 1000;
	}
	else {
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);
		tmp = (tmp * 10) + (Serial.read() - 48);
		maximus.theta = -(double)tmp / 1000;
	}
}


