/*****************************************************
Project : Maximus
Version : 1.0
Date : 21/03/2010
Author : JBot
Company : autonomouS Multi Application Robot Team (S.M.A.R.T)
Comments:
-Change the 2 TICK_PER_MM constants with your own number (come from your encodeurs)
and the diameter of your wheels.
-Change the DIAMETER constant with the distance between the 2 wheels of your robot


Program type : Application
Clock frequency : 16,00 MHz
 *****************************************************/
// Arduino specific includes
#include <Servo.h>

// Other includes
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <math.h>


/* Wait function */
void delay_ms(uint16_t millis)
{
    while (millis) {
        _delay_ms(1);
        millis--;
    }
}

/***********/
/* Defines */
/***********/
#define TICK_PER_MM_LEFT 	9.2628378129  // 90.9456817668
#define TICK_PER_MM_RIGHT 	9.2628378129  // 90.9456817668
#define DIAMETER 		272.0 //275.0 // 166.0         // Distance between the 2 wheels

#define TWOPI 			6.2831853070
#define RAD2DEG 		57.2958                    /* radians to degrees conversion */

#define DISTANCE_CENTER_PAWN    220.0

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



#define DISTANCE_REAR_WHEELS    45                         // Distance between the rear of the robot and the center of the 2 wheels

#define NO_PAWN                 0
#define TAKE_PAWN               1
#define GOTO_RELEASE            2
#define GO_BACK                 3
#define GRABBING                4
#define TURNING_DIRECTION       5

// I/Os definition
//#define INPUT_MOTION_PIN        2
#define RIGHT_SERVO             12
#define LEFT_SERVO              11
#define PAWN_SENSOR             25 // A DEFINIR // MICROSWITCH
#define PAWN_SENSOR_LEFT        0 // A DEFINIR
#define PAWN_SENSOR_MIDDLE      1 // A DEFINIR
#define PAWN_SENSOR_RIGHT       2 // A DEFINIR
#define OPPONENT_SENSOR_LEFT    3 // A DEFINIR
#define OPPONENT_SENSOR_RIGHT   4 // A DEFINIR
#define RIGHT_IR_SENSOR         10 // A DEFINIR
#define LEFT_IR_SENSOR          11 // A DEFINIR
#define LIFT_MOTOR_PWM		40 // A DEFINIR
#define LIFT_MOTOR_SENS		41 // A DEFINIR
#define LIFT_SWITCH_UP          22
#define LIFT_SWITCH_DOWN        23
#define BEACON_NORTH_PIN	46 // A DEFINIR
#define BEACON_SOUTH_PIN	47 // A DEFINIR
#define BEACON_EAST_PIN	        48 // A DEFINIR
#define BEACON_WEST_PIN	        49 // A DEFINIR
#define LEFT_REAR_SENSOR        30 // A DEFINIR
#define RIGHT_REAR_SENSOR       31 // A DEFINIR

#define RESET_ROBOCLAW          45


#define LIFT_GO_UP		HIGH
#define LIFT_GO_DOWN		LOW

#define SECURE_PAWN             0 // First phase to secure 2 pawn in the secure zones
#define CREATE_TOWER            1 // Seconde phase to build 2 tower and place it on the right color
#define PLACING_PAWN            2 // Last phase to moving on the table and take the enemy pawns

#define BEACON_NORTH            0
#define BEACON_SOUTH            1
#define BEACON_EAST             2
#define BEACON_WEST             3

#define ALPHA_MAX_SPEED         9000
#define DELTA_MAX_SPEED         37000

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
    double distance;
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

struct Point {
    int x;
    int y;
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
char roboclaw_ON = 0;
char motion_control_ON = 1;


int last_left = 0;
int last_right = 0;

int left_diff = 0;
int right_diff = 0;

double total_distance = 0.0;

unsigned int entier;
char display1, display2, display3, display4, display5, display6, display7;

char serial_command;


// Servo-motors
Servo gripServo_right;
Servo gripServo_left;

Servo lifter_servo;

// IR sensors
volatile int front_distance_down_left = 50;
volatile int front_distance_down_middle = 50;
volatile int front_distance_down_right = 50;
int front_distance_up_left = 50;
int front_distance_up_right = 50;

int prev_front_distance = 50;

unsigned long global_time_counter = 0;
char has_pawn = NO_PAWN;

struct Point red_points[18];
struct Point blue_points[18];

struct Point *my_color_points;

struct Point way_points[20];
int way_point_index = 0;

struct Point release_point;
struct Point my_test_point;

char start_MOTION = 0;
char turn_counter = 0;

char pawn_stack = 0; // used to know how many pawn we are stacking
char robot_mode = SECURE_PAWN; // used to switch between the different phases
char beacon_direction = BEACON_NORTH; // used to know where is the opponent
char ajusting_pawn = 0; // To know if we are trying to catch a pawn moving right and left to center it
char go_grab_pawn = 0; 
char nb_check = 0;

volatile char transmit_status = 1; // 1 if OK / 0 if not finished

Servo right_wheel;
Servo left_wheel;


/***********************/
/* INTERRUPT FUNCTIONS */
/***********************/

// External Interrupt 4 service routine => PIN2
ISR(INT4_vect)
{
    //#asm("cli")
    if ((PINB & 0x10) != 0) {
        if ((PINE & 0x10) != 0)
            left_cnt--;
        else
            left_cnt++;
    } else {
        if ((PINE & 0x10) == 0)
            left_cnt--;
        else
            left_cnt++;
    }

    //#asm("sei")
}

// External Interrupt 5 service routine => PIN3
ISR(INT5_vect)
{
    if ((PINK & 0x80) != 0) {
        if ((PINE & 0x20) != 0)
            right_cnt++;
        else
            right_cnt--;
    } else {
        if ((PINE & 0x20) == 0)
            right_cnt++;
        else
            right_cnt--;
    }

}

// Pin change 0-7 interrupt service routine => PIN10
ISR(PCINT0_vect)
{
    if ((PINE & 0x10) != 0) {
        if ((PINB & 0x10) != 0) {
            left_cnt++;
        } else
            left_cnt--;
    } else {
        if ((PINB & 0x10) == 0) {
            left_cnt++;
        } else
            left_cnt--;
    }

}

// Pin change 16-23 interrupt service routine => PIN-ADC15
ISR(PCINT2_vect)
{
    if ((PINE & 0x20) != 0) {
        if ((PINK & 0x80) != 0)
            right_cnt--;
        else
            right_cnt++;
    } else {
        if ((PINK & 0x80) == 0)
            right_cnt--;
        else
            right_cnt++;
    }

}

// Timer 1 overflow interrupt service routine
ISR(TIMER1_OVF_vect)
{
    sei();                                                 // enable interrupts
    get_Odometers();
    
    
    
    if (motion_control_ON == 1) {
        do_motion_control();
        if (roboclaw_ON == 1)
          if((transmit_status) == 1)
            move_motors(ALPHADELTA);                       // Update the motor speed
    } else {
        if (roboclaw_ON == 1)
            if((global_time_counter % 2) == 0)
              move_motors(LEFTRIGHT);                        // Update the motor speed
    }
}

// Timer 1 overflow interrupt service routine
/*ISR(TIMER3_OVF_vect)
{
	sei();                                                 // enable interrupts


	// Compute sensors
	BEACON_get_direction();

	int sensorValue = 0;

	sensorValue = analogRead(PAWN_SENSOR_MIDDLE);
	front_distance_down_middle = (front_distance_down_middle + ( convert_longIR_value(sensorValue) ) * 2) / 3;
	if( front_distance_down_middle > 430)
		front_distance_down_middle = 0;
	sensorValue = analogRead(PAWN_SENSOR_LEFT);
	front_distance_down_left = (front_distance_down_left + ( convert_longIR_value(sensorValue) ) * 2) / 3;
	sensorValue = analogRead(PAWN_SENSOR_RIGHT);
	front_distance_down_right = (front_distance_down_right + ( convert_longIR_value(sensorValue) ) * 2) / 3;
	sensorValue = analogRead(OPPONENT_SENSOR_LEFT);
	front_distance_up_left = (front_distance_up_left + ( convert_medIR_value(sensorValue) ) * 2) / 3;
	sensorValue = analogRead(OPPONENT_SENSOR_RIGHT);
	front_distance_up_right = (front_distance_up_right + ( convert_medIR_value(sensorValue) ) * 2) / 3;
	
	   Serial.print("left : ");
	   Serial.print(front_distance_down_left);
	   Serial.print(" middle : ");
	   Serial.print(front_distance_down_middle);
	   Serial.print(" right : ");
	   Serial.println(front_distance_down_right);
	 
	
//	   Serial.print(digitalRead(LIFT_SWITCH_DOWN));
//	   Serial.println(digitalRead(LIFT_SWITCH_UP));
	 
}*/


/*************************/
/* SYSTEM INITIALIZATION */
/*************************/
void setup()
{
    // Crystal Oscillator division factor: 1
#pragma optsize-
    CLKPR = 0x80;
    CLKPR = 0x00;
#ifdef _OPTIMIZE_SIZE_
#pragma optsize+
#endif



    // Input/Output Ports initialization
    // Port A initialization
    // Func7=In Func6=In Func5=In Func4=In Func3=In Func2=In Func1=In Func0=In
    // State7=T State6=T State5=T State4=T State3=T State2=T State1=T State0=T
    PORTA = 0x00;
    DDRA = 0x00;

    // Port B initialization
    // Func7=In Func6=In Func5=In Func4=In Func3=In Func2=In Func1=In Func0=Out
    // State7=T State6=T State5=T State4=T State3=T State2=T State1=T State0=T
    PORTB = 0x00;
    DDRB = 0x00;

    // Port C initialization
    // Func7=In Func6=In Func5=In Func4=In Func3=In Func2=In Func1=In Func0=In
    // State7=T State6=T State5=T State4=T State3=T State2=T State1=T State0=T
    PORTC = 0x00;
    DDRC = 0x00;

    // Port D initialization
    // Func7=In Func6=In Func5=In Func4=In Func3=In Func2=In Func1=In Func0=In
    // State7=T State6=T State5=T State4=T State3=T State2=T State1=T State0=T
    PORTD = 0x00;
    DDRD = 0x00;

    // Port E initialization
    // Func2=In Func1=In Func0=In
    // State2=T State1=T State0=T
    PORTE = 0x00;
    DDRE = 0x00;

    PORTK = 0x00;
    DDRK = 0x00;

    pinMode(13, OUTPUT);

        /**********************/
	/* I/O INITIALIZATION */
	/**********************/
	// OUTPUTS
//	pinMode(13, OUTPUT);
	pinMode(LIFT_MOTOR_PWM, OUTPUT);
	analogWrite(LIFT_MOTOR_PWM, 0);
	pinMode(LIFT_MOTOR_SENS, OUTPUT);
    
        pinMode(RESET_ROBOCLAW, OUTPUT);
        digitalWrite(RESET_ROBOCLAW, HIGH);

	// INPUTS
//	pinMode(INPUT_MOTION_PIN, INPUT);
	pinMode(PAWN_SENSOR, INPUT);
	pinMode(LIFT_SWITCH_UP, INPUT);
	pinMode(LIFT_SWITCH_DOWN, INPUT);
	pinMode(BEACON_NORTH_PIN, INPUT);
	pinMode(BEACON_SOUTH_PIN, INPUT);
	pinMode(BEACON_EAST_PIN, INPUT);
	pinMode(BEACON_WEST_PIN, INPUT);
	pinMode(LEFT_REAR_SENSOR, INPUT);
	pinMode(RIGHT_REAR_SENSOR, INPUT);



    // Timer/Counter 1 initialization
    // Clock source: System Clock
    // Clock value: 62,500 kHz
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
    TCCR1A = 0x01;
    TCCR1B = 0x04;
    TCNT1H = 0x00;
    TCNT1L = 0x00;
    ICR1H = 0x00;
    ICR1L = 0x00;
    OCR1AH = 0x00;
    OCR1AL = 0x00;
    OCR1BH = 0x00;
    OCR1BL = 0x00;
    OCR1CH = 0x00;
    OCR1CL = 0x00;

    // Timer/Counter 3 initialization
    // Clock source: System Clock
    // Clock value: 62,500 kHz
    // Mode: Ph. correct PWM top=00FFh
    // OC1A output: Discon.
    // OC1B output: Discon.
    // OC1C output: Discon.
    // Noise Canceler: Off
    // Input Capture on Falling Edge
    // Timer 3 Overflow Interrupt: On
    // Input Capture Interrupt: Off
    // Compare A Match Interrupt: Off
    // Compare B Match Interrupt: Off
    // Compare C Match Interrupt: Off
  /*  TCCR3A = 0x03;    // Interrupt every 16.4ms
    TCCR3B = 0x04;
    TCNT3H = 0x00;
    TCNT3L = 0x00;
    ICR3H = 0x00;
    ICR3L = 0x00;
    OCR3AH = 0x00;
    OCR3AL = 0x00;
    OCR3BH = 0x00;
    OCR3BL = 0x00;
    OCR3CH = 0x00;
    OCR3CL = 0x00;
*/

    // External Interrupt(s) initialization
    EICRA = 0x00;
    EICRB = 0x05;
    EIMSK = 0x30;
    EIFR = 0x30;
    // Interrupt on PCINT
    PCICR = 0x05;
    PCIFR = 0x05;
    PCMSK0 = 0x10;
    PCMSK1 = 0x00;
    PCMSK2 = 0x80;


    //ETIMSK=0x00;


    // Mode Debug
    Serial.begin(57600); // DEBUG
    //Serial.begin(19200);
    //Serial.begin(115200);
    Serial1.begin(38400); // ROBOCLAW
    // Bluetooth
    Serial2.begin(38400); 
    Serial3.begin(57600); // Color module

    // Timer(s)/Counter(s) Interrupt(s) initialization
    TIMSK1 |= 0x01;
    TIFR1 |= 0x01;
//    TIMSK3 |= 0x01;
//    TIFR3 |= 0x01;
    

    /******************************/
    /* Initialization of the code */
    /******************************/
    right_wheel.attach(6);
    left_wheel.attach(5);

    write_RoboClaw_speed_M1M2(128, 0, 0);

    init_motors();                                         // Init motors
    init_blue_Robot(&maximus);                             // Init robot status

    init_Command(&bot_command_delta);                      // Init robot command
    init_Command(&bot_command_alpha);                      // Init robot command
    init_Command(&prev_bot_command_delta);
    

    init_PAWN_status();

    init_color_points();                                   // Init the color field
    init_way_points();

    
    
    
    // Global enable interrupts
    sei();

    digitalWrite(13, LOW);

//    digitalWrite(13, HIGH);
    delay_ms(300);
    digitalWrite(13, LOW);
    delay_ms(300);
//    digitalWrite(13, HIGH);
    delay_ms(300);

    // enable communication with the roboclaw
    roboclaw_ON = 1;

    delay_ms(1000);
	Serial3.print('C');
	delay_ms(10);
	int color_serial_in = Serial3.read(); 
	while( (color_serial_in != 'B') && (color_serial_in != 'R') ) {
		delay_ms(100);
		Serial3.print('C');
		delay_ms(10);
		color_serial_in = Serial3.read();
		//Serial.print(color_serial_in, BYTE);
	}

	delay_ms(500);


	// WAIT START COMMAND

	PAWN_release_pawn();
	delay_ms(1000);

}

/******************/
/* MAIN CODE LOOP */
/******************/
void loop()
{
    // Place your code here

    if( global_time_counter == 8) {
      if(transmit_status == 1) {
        read_RoboClaw_voltage(128);
      }
    }
    if( global_time_counter == 9) {
      check_RoboClaw_response(128);
    }


    if(global_time_counter > 10) {
      //Serial.println("Alive");
      global_time_counter = 0;
    }

    delay_ms(30);

	BEACON_get_direction();

	int sensorValue = 0;

	sensorValue = analogRead(PAWN_SENSOR_MIDDLE);
	front_distance_down_middle = (front_distance_down_middle + ( convert_longIR_value(sensorValue) ) * 2) / 3;
	if( front_distance_down_middle > 430 || front_distance_down_middle < 0)
		front_distance_down_middle = 0;
	sensorValue = analogRead(PAWN_SENSOR_LEFT);
	front_distance_down_left = (front_distance_down_left + ( convert_longIR_value(sensorValue) ) * 2) / 3;
        if( front_distance_down_left < 0 )
          front_distance_down_left = 150;
	sensorValue = analogRead(PAWN_SENSOR_RIGHT);
	front_distance_down_right = (front_distance_down_right + ( convert_longIR_value(sensorValue) ) * 2) / 3;
	if( front_distance_down_right < 0 )
          front_distance_down_right = 150;
        sensorValue = analogRead(OPPONENT_SENSOR_LEFT);
	front_distance_up_left = (front_distance_up_left + ( convert_medIR_value(sensorValue) ) * 2) / 3;
	sensorValue = analogRead(OPPONENT_SENSOR_RIGHT);
	front_distance_up_right = (front_distance_up_right + ( convert_medIR_value(sensorValue) ) * 2) / 3;
	

    

    global_time_counter++;


    		if( ( (front_distance_down_right < 30) || ((front_distance_down_middle < 35) && (front_distance_down_middle > 5)) || (front_distance_down_left < 30) ) && (has_pawn == NO_PAWN) ) {
			// TODO : S'assurer que ce que l'on voit n'est pas un mur
			my_test_point = compute_pawn_position(&maximus, 210 + front_distance_down_middle * 10);
			if( (check_point_in_map(&my_test_point) != 0) ) {


				// TODO : Vérifier que ce n'est pas un pion qui est sur une case de notre couleur      
				if(find_pawn_in_our_color(&my_test_point, my_color_points, 18) != 0 ) {
					// Eviter le pion
                                        Serial.println("Pawn in our color");
				}
				else {
					// Prendre le pion


					if(nb_check == 2) {

						stop_robot();
						delta_motor.max_speed = 25000;
						alpha_motor.max_speed = 6000;
						has_pawn = GRABBING;
						ajusting_pawn = 1;


						Serial.print("left : ");
						Serial.print(front_distance_down_left);
						Serial.print(" middle : ");
						Serial.print(front_distance_down_middle);
						Serial.print(" right : ");
						Serial.println(front_distance_down_right);

						nb_check = 0;
					}
					else {
						nb_check++;
					}
				}
			}
			else {
				Serial.print("-Point NOT in the map ");
                                Serial.println(front_distance_down_middle);
				if(front_distance_down_middle < 26) {
					stop_robot();
				}
			}    
		}
		else {
			nb_check = 0;
		}


	if(has_pawn == GRABBING) {
		if(front_distance_down_right < 33) {
			// tourner sur la droite 
			set_new_command(&bot_command_alpha, (-10));
			ajusting_pawn = 1;
		} else if(front_distance_down_left < 33) {
			// tourner sur la gauche 
			set_new_command(&bot_command_alpha, (10));
			ajusting_pawn = 1;
		} else if( ajusting_pawn == 1){
			set_new_command(&bot_command_alpha, (0));
			//MOTION_stop_robot();
			go_grab_pawn = 1;
			ajusting_pawn = 0;
		}

	}



    if((bot_command_alpha.state == COMMAND_DONE) && (bot_command_delta.state == COMMAND_DONE)) {
	//if ((global_time_counter > 20)) {        // If all commands are done, go to the next step
		//global_time_counter = 21;

		double x_topawn;
		double y_topawn;
		int sens;

		switch (has_pawn) {
			case GRABBING:

				// Attrapper un pion
				if(front_distance_down_right < 33) {
					// tourner sur la droite 
					set_new_command(&bot_command_alpha, (-10));
					ajusting_pawn = 1;
				} else if(front_distance_down_left < 33) {
					// tourner sur la gauche 
					set_new_command(&bot_command_alpha, (10));
					ajusting_pawn = 1;
				} else if( ajusting_pawn == 1){
					set_new_command(&bot_command_alpha, (0));
					//MOTION_stop_robot();
					go_grab_pawn = 1;
					ajusting_pawn = 0;
				}

				if(go_grab_pawn >= 1) {

					if( (front_distance_down_middle < 2) ) { //|| (digitalRead(INPUT_MOTION_PIN) && (go_grab_pawn == 2)) ) {
						stop_robot();
						go_grab_pawn = 0;
						PAWN_release_pawn();
                                                PAWN_go_down();
						PAWN_grip_pawn();
						//PAWN_go_up();
						//delay_ms(1000);
						if(pawn_stack == 0) { // Stacking pawn
                                                  PAWN_go_up();
                                                  pawn_stack = 1;
                                                  delta_motor.max_speed = DELTA_MAX_SPEED;
					          alpha_motor.max_speed = ALPHA_MAX_SPEED;
                                                  has_pawn = TURNING_DIRECTION;
                                                }
                                                else { // Go put the pawn on the right space
                                                  has_pawn = TAKE_PAWN;
						}
                                                //PAWN_release_pawn();
						//delay_ms(3000);
					}
					else {
						if((front_distance_down_middle < 50)) {
							my_test_point = compute_pawn_position(&maximus, 210 + front_distance_down_middle * 10);
							/*Serial.print("X = ");
							  Serial.print(my_test_point.x);
							  Serial.print(" Y = ");
							  Serial.print(my_test_point.y);
							 */
							if( (check_point_in_map(&my_test_point) != 0) && (go_grab_pawn == 1) ) {
								Serial.println("Point in the map");
								//MOTION_set_maxspeed_delta(25000);
								set_new_command(&bot_command_delta, ((front_distance_down_middle + 2) * 10)); 
								go_grab_pawn = 2;
							}
							else {
								Serial.println("Point NOT in the map");
							}
						}
					}      
					}
					break;
					case TAKE_PAWN:                                   // The robot just take a pawn to put in his color
					release_point = find_nearest(&maximus, my_color_points, 18);
					x_topawn = release_point.x;
					y_topawn = release_point.y;

                                        delta_motor.max_speed = DELTA_MAX_SPEED;
					alpha_motor.max_speed = ALPHA_MAX_SPEED;

					sens = move_pawn_to_xy(&maximus, &x_topawn, &y_topawn);
					if (sens == 0) {                               // Front
						goto_xy(x_topawn, y_topawn);
					} else {                                       // Back
						goto_xy_back(x_topawn, y_topawn);
					}
					delay_ms(100);
					has_pawn = GOTO_RELEASE;
					break;

					case GOTO_RELEASE:                                // The robot have a pawn and is on his release point
					PAWN_release_pawn();
					PAWN_go_up();
                                        pawn_stack = 0;
					
					//MOTION_set_delta(-180);
					set_new_command(&bot_command_delta, (-200));
					delay_ms(100);
					has_pawn = GO_BACK;
					break;

					case GO_BACK:                                     // The robot release the pawn and go back to let it in position
					//MOTION_goto_xy(way_points[way_point_index - 1].x, way_points[way_point_index - 1].y);
					//MOTION_set_alpha(angle_coord(&maximus, way_points[way_point_index - 1].x, way_points[way_point_index - 1].y) * RAD2DEG);
					//delay_ms(300);
					struct Point the_point;
					the_point.x = maximus.pos_X;
					the_point.y = maximus.pos_Y;

					if(trajectory_intersection_pawn(&the_point, &way_points[way_point_index - 1], &release_point, 100+130) == 1) { // pawn is in the trajectory
						// Compute an intermediate way_point
						Serial.println("In trajectory");
						has_pawn = TURNING_DIRECTION;
					}
					else {
						Serial.println("Not in trajectory");
						set_new_command(&bot_command_alpha, (angle_coord(&maximus, way_points[way_point_index - 1].x, way_points[way_point_index - 1].y) * RAD2DEG));
						Serial.print("Turning : ");
						Serial.println(angle_coord(&maximus, way_points[way_point_index - 1].x, way_points[way_point_index - 1].y) * RAD2DEG);
						Serial.println(maximus.theta* RAD2DEG);
						has_pawn = TURNING_DIRECTION;
						//delay_ms(1000);
                                                //delay_ms(500);
					}

					//has_pawn = TURNING_DIRECTION;
					break;
					case TURNING_DIRECTION:
					// Look if we see the pawn we just put in place
					goto_xy(way_points[way_point_index - 1].x, way_points[way_point_index - 1].y);
					Serial.println(maximus.theta* RAD2DEG);
					Serial.print("!END Turning! ");
					Serial.print("Go to :");
					Serial.print(way_points[way_point_index - 1].x);
					Serial.print(" ");
					Serial.println(way_points[way_point_index - 1].y);
					has_pawn = NO_PAWN;
					break;        
					default:                                          // has no pawn => Move to the next position
					if(turn_counter < 3) {
						if (way_point_index >= 6) {
							way_point_index = 2;
							turn_counter++;
						}

						goto_xy(way_points[way_point_index].x, way_points[way_point_index].y);
						Serial.print("Go to :");
						Serial.print(way_points[way_point_index].x);
						Serial.print(" ");
						Serial.println(way_points[way_point_index].y);
						way_point_index++;
					}
					else {
						if (turn_counter == 3) {
							goto_xy_back(-700, 200);
							turn_counter++;
						}
					}
					delay_ms(100);
					//Serial.println('X');
					break;
				}

				delay_ms(100);
		//}

 
    }
    //else {
    //      digitalWrite(13, LOW);

    //}




}

/****************************/
/* INITIALIZATION FUNCTIONS */
/****************************/
void init_Robot(struct robot *my_robot)
{
    my_robot->pos_X = -1459;                               // -700         
    my_robot->pos_Y = 192;                                 // 700          
    my_robot->theta = 0;                                   // PI/2
    my_robot->yaw = 0.0;
    my_robot->pitch = 0.0;
    my_robot->roll = 0.0;
    my_robot->yaw_offset = 0.0;
}

void init_Command(struct RobotCommand *cmd)
{
    cmd->state = COMMAND_DONE;
    cmd->current_distance = 0;
    cmd->desired_distance = 0;
}

void init_motors(void)
{
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
    alpha_motor.kP = 230;//250//350                                  // 600
    alpha_motor.kI = 0;
    alpha_motor.kD = 340;//300 //180                                  // 200
    alpha_motor.accel = 300;//350//200;                               // 300
    alpha_motor.decel = 1300;//1200;//1100;//1200;                              // 500
    alpha_motor.max_speed = ALPHA_MAX_SPEED;//7000;                          //8000
    alpha_motor.distance = 0.0;

    /* Delta motor initialization */
    delta_motor.type = DELTA_MOTOR;
    delta_motor.des_speed = 0;
    delta_motor.cur_speed = 0;
    delta_motor.last_error = 0;
    delta_motor.error_sum = 0;
    delta_motor.kP = 600;                                  // 600
    delta_motor.kI = 0;
    delta_motor.kD = 200;                                  // 100 * 1.09
    delta_motor.accel = 600;//400;//500;
    delta_motor.decel = 1800;//1350;//1100;//1200;
    delta_motor.max_speed = DELTA_MAX_SPEED;//25000;//35000;
    delta_motor.distance = 0.0;
}


/***********************/
/* ROBO CLAW FUNCTIONS */
/***********************/
// Used to change the speed value of motor 1
void write_RoboClaw_speed_M1(char addr, signed long speed)
{
    char checkSUM;
    checkSUM =
        (addr + 35 + ((char) ((speed >> 24) & 0xFF)) +
         ((char) ((speed >> 16) & 0xFF)) + ((char) ((speed >> 8) & 0xFF)) + ((char) (speed & 0xFF))) & 0x7F;
    Serial2.print(addr);
    Serial2.print(35);
    Serial2.print(((char) ((speed >> 24) & 0xFF)));
    Serial2.print(((char) ((speed >> 16) & 0xFF)));
    Serial2.print(((char) ((speed >> 8) & 0xFF)));
    Serial2.print(((char) (speed & 0xFF)));

    Serial2.print(checkSUM);
}

// Used to change the speed value of motor 2
void write_RoboClaw_speed_M2(char addr, signed long speed)
{
    char checkSUM;
    checkSUM =
        (addr + 36 + ((char) ((speed >> 24) & 0xFF)) +
         ((char) ((speed >> 16) & 0xFF)) + ((char) ((speed >> 8) & 0xFF)) + ((char) (speed & 0xFF))) & 0x7F;
    Serial2.print(addr);
    Serial2.print(36);
    Serial2.print(((char) ((speed >> 24) & 0xFF)));
    Serial2.print(((char) ((speed >> 16) & 0xFF)));
    Serial2.print(((char) ((speed >> 8) & 0xFF)));
    Serial2.print(((char) (speed & 0xFF)));

    Serial2.print(checkSUM);
}

// Used to change the speed value of motors 1 and 2
void write_RoboClaw_speed_M1M2(char addr, signed long speedM1, signed long speedM2)
{
    char checkSUM;
    transmit_status = 0;
    
    checkSUM =
        (addr + 37 + ((char) ((speedM1 >> 24) & 0xFF)) +
         ((char) ((speedM1 >> 16) & 0xFF)) + ((char) ((speedM1 >> 8) & 0xFF)) +
         ((char) (speedM1 & 0xFF)) + ((char) ((speedM2 >> 24) & 0xFF)) +
         ((char) ((speedM2 >> 16) & 0xFF)) + ((char) ((speedM2 >> 8) & 0xFF)) + ((char) (speedM2 & 0xFF))) & 0x7F;

    

    Serial2.print(addr, BYTE);
    Serial2.print(37, BYTE);
    Serial2.print(((char) ((speedM1 >> 24) & 0xFF)), BYTE);
    Serial2.print(((char) ((speedM1 >> 16) & 0xFF)), BYTE);
    Serial2.print(((char) ((speedM1 >> 8) & 0xFF)), BYTE);
    Serial2.print(((char) (speedM1 & 0xFF)), BYTE);

    Serial2.print(((char) ((speedM2 >> 24) & 0xFF)), BYTE);
    Serial2.print(((char) ((speedM2 >> 16) & 0xFF)), BYTE);
    Serial2.print(((char) ((speedM2 >> 8) & 0xFF)), BYTE);
    Serial2.print(((char) (speedM2 & 0xFF)), BYTE);

    Serial2.print(checkSUM, BYTE);


/*
    Serial2.print((speedM1/625)+64, BYTE);
    Serial2.print((speedM2/625)+192, BYTE);
*/    


    
/*    
    right_wheel.write((speedM2/80)+1500);
    left_wheel.write((speedM1/80)+1500);
*/    
    

    transmit_status = 1;
}



// Used to change the speed value of motor 1 and 2 during a specific distance
void write_RoboClaw_speed_dist_M1M2(char addr, signed long speedM1, signed long distanceM1, signed long speedM2, signed long distanceM2)
{
    char checkSUM;
    checkSUM =
        (addr + 43 + ((char) ((speedM1 >> 24) & 0xFF)) +
         ((char) ((speedM1 >> 16) & 0xFF)) + ((char) ((speedM1 >> 8) & 0xFF)) +
         ((char) (speedM1 & 0xFF)) + ((char) ((speedM2 >> 24) & 0xFF)) +
         ((char) ((speedM2 >> 16) & 0xFF)) + ((char) ((speedM2 >> 8) & 0xFF)) +
         ((char) (speedM2 & 0xFF)) + ((char) ((distanceM1 >> 24) & 0xFF)) +
         ((char) ((distanceM1 >> 16) & 0xFF)) +
         ((char) ((distanceM1 >> 8) & 0xFF)) + ((char) (distanceM1 & 0xFF)) +
         ((char) ((distanceM2 >> 24) & 0xFF)) +
         ((char) ((distanceM2 >> 16) & 0xFF)) + ((char) ((distanceM2 >> 8) & 0xFF)) + ((char) (distanceM2 & 0xFF)) + 1) & 0x7F;
    Serial2.print(addr);
    Serial2.print(43);
    Serial2.print(((char) ((speedM1 >> 24) & 0xFF)));
    Serial2.print(((char) ((speedM1 >> 16) & 0xFF)));
    Serial2.print(((char) ((speedM1 >> 8) & 0xFF)));
    Serial2.print(((char) (speedM1 & 0xFF)));

    Serial2.print(((char) ((distanceM1 >> 24) & 0xFF)));
    Serial2.print(((char) ((distanceM1 >> 16) & 0xFF)));
    Serial2.print(((char) ((distanceM1 >> 8) & 0xFF)));
    Serial2.print(((char) (distanceM1 & 0xFF)));

    Serial2.print(((char) ((speedM2 >> 24) & 0xFF)));
    Serial2.print(((char) ((speedM2 >> 16) & 0xFF)));
    Serial2.print(((char) ((speedM2 >> 8) & 0xFF)));
    Serial2.print(((char) (speedM2 & 0xFF)));

    Serial2.print(((char) ((distanceM2 >> 24) & 0xFF)));
    Serial2.print(((char) ((distanceM2 >> 16) & 0xFF)));
    Serial2.print(((char) ((distanceM2 >> 8) & 0xFF)));
    Serial2.print(((char) (distanceM2 & 0xFF)));

    Serial2.print(1);

    Serial2.print(checkSUM);
}

// Used to change the speed value of motor 1 and 2 during a specific distance with a specific acceleration
void
write_RoboClaw_allcmd_M1M2(char addr, signed long accel, signed long speedM1, signed long distanceM1, signed long speedM2, signed long distanceM2)
{
    char checkSUM;
    checkSUM =
        (addr + 46 + ((char) ((accel >> 24) & 0xFF)) +
         ((char) ((accel >> 16) & 0xFF)) + ((char) ((accel >> 8) & 0xFF)) +
         ((char) (accel & 0xFF)) + ((char) ((speedM1 >> 24) & 0xFF)) +
         ((char) ((speedM1 >> 16) & 0xFF)) + ((char) ((speedM1 >> 8) & 0xFF)) +
         ((char) (speedM1 & 0xFF)) + ((char) ((speedM2 >> 24) & 0xFF)) +
         ((char) ((speedM2 >> 16) & 0xFF)) + ((char) ((speedM2 >> 8) & 0xFF)) +
         ((char) (speedM2 & 0xFF)) + ((char) ((distanceM1 >> 24) & 0xFF)) +
         ((char) ((distanceM1 >> 16) & 0xFF)) +
         ((char) ((distanceM1 >> 8) & 0xFF)) + ((char) (distanceM1 & 0xFF)) +
         ((char) ((distanceM2 >> 24) & 0xFF)) +
         ((char) ((distanceM2 >> 16) & 0xFF)) + ((char) ((distanceM2 >> 8) & 0xFF)) + ((char) (distanceM2 & 0xFF)) + 1) & 0x7F;

    Serial2.print(addr);
    Serial2.print(46);

    Serial2.print(((char) ((accel >> 24) & 0xFF)));
    Serial2.print(((char) ((accel >> 16) & 0xFF)));
    Serial2.print(((char) ((accel >> 8) & 0xFF)));
    Serial2.print(((char) (accel & 0xFF)));

    Serial2.print(((char) ((speedM1 >> 24) & 0xFF)));
    Serial2.print(((char) ((speedM1 >> 16) & 0xFF)));
    Serial2.print(((char) ((speedM1 >> 8) & 0xFF)));
    Serial2.print(((char) (speedM1 & 0xFF)));

    Serial2.print(((char) ((distanceM1 >> 24) & 0xFF)));
    Serial2.print(((char) ((distanceM1 >> 16) & 0xFF)));
    Serial2.print(((char) ((distanceM1 >> 8) & 0xFF)));
    Serial2.print(((char) (distanceM1 & 0xFF)));

    Serial2.print(((char) ((speedM2 >> 24) & 0xFF)));
    Serial2.print(((char) ((speedM2 >> 16) & 0xFF)));
    Serial2.print(((char) ((speedM2 >> 8) & 0xFF)));
    Serial2.print(((char) (speedM2 & 0xFF)));

    Serial2.print(((char) ((distanceM2 >> 24) & 0xFF)));
    Serial2.print(((char) ((distanceM2 >> 16) & 0xFF)));
    Serial2.print(((char) ((distanceM2 >> 8) & 0xFF)));
    Serial2.print(((char) (distanceM2 & 0xFF)));

    Serial2.print(1);

    Serial2.print(checkSUM);
}

// Used to read the battery voltage
void read_RoboClaw_voltage(char addr)
{
    char checkSUM;
    transmit_status = 0;

    Serial2.print(addr, BYTE);
    Serial2.print(24, BYTE);
    
    transmit_status = 1;
}

void check_RoboClaw_response(char addr) 
{
 
  if(Serial2.available() >= 3) {
   Serial.println("RoboClaw responding");
   Serial2.read();
   Serial2.read();
   Serial2.read(); 
  }
  else {
    digitalWrite(RESET_ROBOCLAW, LOW);
    Serial.println("RoboClaw not responding anymore");
    delay(1);
    digitalWrite(RESET_ROBOCLAW, HIGH);
  }
  
}



/*******************************/
/* MOTION CONTROL FUNCTIONS */
/*******************************/
void do_motion_control(void)
{

    // PID angle
    alpha_motor.des_speed = compute_position_PID(&bot_command_alpha, &alpha_motor);

  // PID distance
    if ((bot_command_alpha.state == WAITING_BEGIN) || (bot_command_alpha.state == PROCESSING_COMMAND)) {   // If alpha motor have not finished its movement 

    } else {
        if ((bot_command_delta.state != PROCESSING_COMMAND) && (prev_bot_command_delta.state == WAITING_BEGIN)) {
            prev_bot_command_delta.state = PROCESSING_COMMAND;
            set_new_command(&bot_command_delta, prev_bot_command_delta.desired_distance);
        }
    //delta_motor.des_speed = compute_position_PID(&bot_command_delta, &delta_motor);

    }
    delta_motor.des_speed = compute_position_PID(&bot_command_delta, &delta_motor);

    /*if (bot_command_alpha.state == WAITING_BEGIN) {
       bot_command_alpha.state = PROCESSING_COMMAND;
       } */

}

void set_new_command(struct RobotCommand *cmd, long distance)
{
    cmd->state = WAITING_BEGIN;
    //#asm("cli") // disable interrupts
    cmd->current_distance = 0;
    cmd->desired_distance = distance;
    //#asm("sei") // enable interrupts
}

long compute_position_PID(struct RobotCommand *cmd, struct motor *used_motor)
{
    long P, I, D;
    long errDif, err;
    long tmp = 0;

    if (cmd->state == WAITING_BEGIN) {
        cmd->state = PROCESSING_COMMAND;
    }

    if (used_motor->type == ALPHA_MOTOR)
        err = cmd->desired_distance * 10 - cmd->current_distance * 10 * RAD2DEG;
    else
        err = cmd->desired_distance - cmd->current_distance;

    used_motor->error_sum += err;                          // Error sum
    if (used_motor->error_sum > 10)
        used_motor->error_sum = 10;
    if (used_motor->error_sum < -10)
        used_motor->error_sum = -10;

    errDif = err - used_motor->last_error;                 // Compute the error variation

    used_motor->last_error = err;

    P = err * used_motor->kP;                              // Proportionnal
    I = used_motor->error_sum * used_motor->kI;            // Integral
    D = errDif * used_motor->kD;                           // Derivative

    tmp = (P + I + D);


    if (abs(tmp) < abs(used_motor->des_speed)) {           // Deceleration
        if (tmp > (used_motor->des_speed + used_motor->decel))
            tmp = (used_motor->des_speed + used_motor->decel);
        else if (tmp < (used_motor->des_speed - used_motor->decel))
            tmp = (used_motor->des_speed - used_motor->decel);
    } else {                                               // Acceleration
        if (tmp > (used_motor->des_speed + used_motor->accel))
            tmp = (used_motor->des_speed + used_motor->accel);
        else if (tmp < (used_motor->des_speed - used_motor->accel))
            tmp = (used_motor->des_speed - used_motor->accel);
    }

    if (tmp > (used_motor->max_speed))
        tmp = (used_motor->max_speed);
    if (tmp < -(used_motor->max_speed))
        tmp = -(used_motor->max_speed);

    if ((cmd->state == PROCESSING_COMMAND) && (abs(err) < 2)
        && (abs(errDif) < 2)) {
        cmd->state = COMMAND_DONE;
    }

    return tmp;
}

// Compute the distance to do to go to (x, y)
double distance_coord(struct robot *my_robot, double x1, double y1)
{
    double x = 0;
    x = sqrt(pow(fabs(x1 - my_robot->pos_X), 2) + pow(fabs(y1 - my_robot->pos_Y), 2));
    return x;
}

// Compute the angle to do to go to (x, y)
double angle_coord(struct robot *my_robot, double x1, double y1)
{
    double angletodo = 0;
    if ((x1 < my_robot->pos_X) && (y1 < my_robot->pos_Y)) {
        angletodo = -PI / 2 - atan(fabs((x1 - my_robot->pos_X) / (y1 - my_robot->pos_Y)));
    } else if ((x1 > my_robot->pos_X) && (y1 < my_robot->pos_Y)) {
        angletodo = -atan(fabs((y1 - my_robot->pos_Y) / (x1 - my_robot->pos_X)));
    } else if ((x1 > my_robot->pos_X) && (y1 > my_robot->pos_Y)) {
        angletodo = atan(fabs((y1 - my_robot->pos_Y) / (x1 - my_robot->pos_X)));
    } else if ((x1 < my_robot->pos_X) && (y1 > my_robot->pos_Y)) {
        angletodo = PI / 2 + atan(fabs((x1 - my_robot->pos_X) / (y1 - my_robot->pos_Y)));
    } else if ((x1 < my_robot->pos_X) && (y1 == my_robot->pos_Y)) {     // 
        angletodo = -PI;
    } else if ((x1 > my_robot->pos_X) && (y1 == my_robot->pos_Y)) {     // 
        angletodo = 0;
    } else if ((x1 == my_robot->pos_X) && (y1 < my_robot->pos_Y)) {     // 
        angletodo = -PI / 2;
    } else if ((x1 == my_robot->pos_X) && (y1 > my_robot->pos_Y)) {     // 
        angletodo = PI / 2;
    } else
        angletodo = 0;

    angletodo = angletodo - my_robot->theta;

    if (angletodo > PI)
        angletodo = angletodo - 2 * PI;
    if (angletodo < -PI)
        angletodo = 2 * PI + angletodo;

    return angletodo;
}

void goto_xy(double x, double y)
{
    double ang, dist;

    ang = angle_coord(&maximus, x, y) * RAD2DEG;
    set_new_command(&bot_command_alpha, ang);

    dist = distance_coord(&maximus, x, y);
    set_new_command(&prev_bot_command_delta, dist);
    bot_command_delta.state = WAITING_BEGIN;
}

void goto_xy_back(double x, double y)
{
    double ang, dist;

    ang = angle_coord(&maximus, x, y);
    if (ang < 0)
        ang = (ang + PI) * RAD2DEG;
    else
        ang = (ang - PI) * RAD2DEG;
    set_new_command(&bot_command_alpha, ang);

    dist = -distance_coord(&maximus, x, y);
    set_new_command(&prev_bot_command_delta, dist);
    bot_command_delta.state = WAITING_BEGIN;
}

// Compute the coordinate to put a pawn to (x, y)
int move_pawn_to_xy(struct robot *my_robot, double *x1, double *y1)
{
    double distance = distance_coord(my_robot, *x1, *y1);
    int sens = 0;
    double angletodo = 0;
    double x_todo = 0, y_todo = 0;

    if (distance < DISTANCE_CENTER_PAWN)
        sens = 1;

    if ((*x1 < my_robot->pos_X) && (*y1 < my_robot->pos_Y)) {
        angletodo = atan(fabs((*x1 - my_robot->pos_X) / (*y1 - my_robot->pos_Y)));
        x_todo = *x1 + (sin(angletodo) * DISTANCE_CENTER_PAWN);
        y_todo = *y1 + (cos(angletodo) * DISTANCE_CENTER_PAWN);
    } else if ((*x1 > my_robot->pos_X) && (*y1 < my_robot->pos_Y)) {
        angletodo = atan(fabs((*y1 - my_robot->pos_Y) / (*x1 - my_robot->pos_X)));
        x_todo = *x1 - (cos(angletodo) * DISTANCE_CENTER_PAWN);
        y_todo = *y1 + (sin(angletodo) * DISTANCE_CENTER_PAWN);
    } else if ((*x1 > my_robot->pos_X) && (*y1 > my_robot->pos_Y)) {
        angletodo = atan(fabs((*y1 - my_robot->pos_Y) / (*x1 - my_robot->pos_X)));
        x_todo = *x1 - (cos(angletodo) * DISTANCE_CENTER_PAWN);
        y_todo = *y1 - (sin(angletodo) * DISTANCE_CENTER_PAWN);
    } else if ((*x1 < my_robot->pos_X) && (*y1 > my_robot->pos_Y)) {
        angletodo = atan(fabs((*x1 - my_robot->pos_X) / (*y1 - my_robot->pos_Y)));
        x_todo = *x1 + (sin(angletodo) * DISTANCE_CENTER_PAWN);
        y_todo = *y1 - (cos(angletodo) * DISTANCE_CENTER_PAWN);
    } else if ((*x1 < my_robot->pos_X) && (*y1 == my_robot->pos_Y)) {   // 
        x_todo = *x1 + DISTANCE_CENTER_PAWN;
    } else if ((*x1 > my_robot->pos_X) && (*y1 == my_robot->pos_Y)) {   // 
        x_todo = *x1 - DISTANCE_CENTER_PAWN;
    } else if ((*x1 == my_robot->pos_X) && (*y1 < my_robot->pos_Y)) {   // 
        y_todo = *y1 + DISTANCE_CENTER_PAWN;
    } else if ((*x1 == my_robot->pos_X) && (*y1 > my_robot->pos_Y)) {   // 
        y_todo = *y1 - DISTANCE_CENTER_PAWN;
    } else {
        y_todo = *y1;
        x_todo = *x1;
    }

    *y1 = y_todo;
    *x1 = x_todo;

    return sens;
}


/************************/
/* CONVERSION FUNCTIONS */
/************************/
signed long convert_dist2ticks(signed long distance)
{
    return (distance * TICK_PER_MM_RIGHT);
}

signed long convert_ticks2dist(signed long ticks)
{
    return (ticks / TICK_PER_MM_RIGHT);
}

/********************/
/* MOTORS FUNCTIONS */
/********************/
void move_motors(char type)
{
    if (type == ALPHADELTA)
        write_RoboClaw_speed_M1M2(128, delta_motor.des_speed - alpha_motor.des_speed, delta_motor.des_speed + alpha_motor.des_speed);
    else
        write_RoboClaw_speed_M1M2(128, left_motor.des_speed, right_motor.des_speed);
}

void update_motor(struct motor *used_motor)
{
    switch (used_motor->type) {
    case LEFT_MOTOR:
        used_motor->cur_speed = left_diff;
        break;
    case RIGHT_MOTOR:
        used_motor->cur_speed = right_diff;
        break;
    case ALPHA_MOTOR:
        used_motor->cur_speed = left_diff - right_diff;
        break;
    case DELTA_MOTOR:
        used_motor->cur_speed = (left_diff + right_diff) / 2;
        break;
    default:
        break;
    }
}


/********************************/
/* POSITION ESTIMATION FUNCTION */
/********************************/

/* Compute the position of the robot */
void get_Odometers(void)
{
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

    if (maximus.theta > PI)
        maximus.theta -= TWOPI;
    if (maximus.theta < (-PI))
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
void stop_position_motion_control(void)
{
    left_motor.des_speed = 0;
    right_motor.des_speed = 0;
    motion_control_ON = 0;
}

void start_position_motion_control(void)
{
    set_new_command(&bot_command_alpha, 0);
    set_new_command(&prev_bot_command_delta, 0);
    motion_control_ON = 1;
}

void stop_robot(void)
{
    set_new_command(&bot_command_alpha, 0);
    set_new_command(&prev_bot_command_delta, delta_motor.des_speed / 800); // depends on current speed
    set_new_command(&bot_command_delta, delta_motor.des_speed / 800);
    //right_motor.des_speed = 0;
    //left_motor.des_speed = 0;
    //write_RoboClaw_speed_M1M2(128, 0, 0);
}




	/****************************/
	/* INITIALIZATION FUNCTIONS */
	/****************************/
	void init_blue_Robot(struct robot *my_robot)
	{
		my_robot->pos_X = -1500 + DISTANCE_REAR_WHEELS;//-1459;                               // -700         
		my_robot->pos_Y = 192;                                 // 700          
		my_robot->theta = 0;                                   // PI/2

		my_color_points = blue_points;
	}

	void init_red_Robot(struct robot *my_robot)
	{
		my_robot->pos_X = 1500 - DISTANCE_REAR_WHEELS;//1459;                                // -700         
		my_robot->pos_Y = 192;                                 // 700          
		my_robot->theta = PI;                                  // PI/2

		my_color_points = red_points;
	}


	void init_color_points(void)
	{
		/* Red points */
		red_points[0].x = -175 + 350 + 350;
		red_points[1].x = -175;
		red_points[2].x = -175 - 350 - 350;
		red_points[0].y = 175;
		red_points[1].y = 175;
		red_points[2].y = 175;

		red_points[3].x = 175 + 350 + 350;
		red_points[4].x = 175;
		red_points[5].x = 175 - 350 - 350;
		red_points[3].y = 175 + 350;
		red_points[4].y = 175 + 350;
		red_points[5].y = 175 + 350;

		red_points[6].x = -175 + 350 + 350;
		red_points[7].x = -175;
		red_points[8].x = -175 - 350 - 350;
		red_points[6].y = 175 + 350 + 350;
		red_points[7].y = 175 + 350 + 350;
		red_points[8].y = 175 + 350 + 350;

		red_points[9].x = 175 + 350 + 350;
		red_points[10].x = 175;
		red_points[11].x = 175 - 350 - 350;
		red_points[9].y = 175 + 350 + 350 + 350;
		red_points[10].y = 175 + 350 + 350 + 350;
		red_points[11].y = 175 + 350 + 350 + 350;

		red_points[12].x = -175 + 350 + 350;
		red_points[13].x = -175;
		red_points[14].x = -175 - 350 - 350;
		red_points[12].y = 175 + 350 + 350 + 350 + 350;
		red_points[13].y = 175 + 350 + 350 + 350 + 350;
		red_points[14].y = 175 + 350 + 350 + 350 + 350;

		red_points[15].x = 175 + 350 + 350;
		red_points[16].x = 175;
		red_points[17].x = 175 - 350 - 350;
		red_points[15].y = 175 + 350 + 350 + 350 + 350 + (350 / 2 + (350 - 120) / 2);
		red_points[16].y = 175 + 350 + 350 + 350 + 350 + 350;
		red_points[17].y = 175 + 350 + 350 + 350 + 350 + (350 / 2 + (350 - 120) / 2);

		/* Blue points */
		blue_points[0].x = 175 + 350 + 350;
		blue_points[1].x = 175;
		blue_points[2].x = 175 - 350 - 350;
		blue_points[0].y = 175;
		blue_points[1].y = 175;
		blue_points[2].y = 175;

		blue_points[3].x = -175 + 350 + 350;
		blue_points[4].x = -175;
		blue_points[5].x = -175 - 350 - 350;
		blue_points[3].y = 175 + 350;
		blue_points[4].y = 175 + 350;
		blue_points[5].y = 175 + 350;

		blue_points[6].x = 175 + 350 + 350;
		blue_points[7].x = 175;
		blue_points[8].x = 175 - 350 - 350;
		blue_points[6].y = 175 + 350 + 350;
		blue_points[7].y = 175 + 350 + 350;
		blue_points[8].y = 175 + 350 + 350;

		blue_points[9].x = -175 + 350 + 350;
		blue_points[10].x = -175;
		blue_points[11].x = -175 - 350 - 350;
		blue_points[9].y = 175 + 350 + 350 + 350;
		blue_points[10].y = 175 + 350 + 350 + 350;
		blue_points[11].y = 175 + 350 + 350 + 350;

		blue_points[12].x = 175 + 350 + 350;
		blue_points[13].x = 175;
		blue_points[14].x = 175 - 350 - 350;
		blue_points[12].y = 175 + 350 + 350 + 350 + 350;
		blue_points[13].y = 175 + 350 + 350 + 350 + 350;
		blue_points[14].y = 175 + 350 + 350 + 350 + 350;

		blue_points[15].x = -175 + 350 + 350;
		blue_points[16].x = -175;
		blue_points[17].x = -175 - 350 - 350;
		blue_points[15].y = 175 + 350 + 350 + 350 + 350 + (350 / 2 + (350 - 120) / 2);
		blue_points[16].y = 175 + 350 + 350 + 350 + 350 + 350;
		blue_points[17].y = 175 + 350 + 350 + 350 + 350 + (350 / 2 + (350 - 120) / 2);

	}

	void init_way_points(void)
	{
		way_points[0].x = -1100;
		way_points[0].y = 192;
		way_points[1].x = -700;
		way_points[1].y = 350;
		way_points[2].x = -700;
		way_points[2].y = 700;
		way_points[3].x = -700;
		way_points[3].y = 1600;

		way_points[4].x = -350;
		way_points[4].y = 1600;//1400;
		way_points[5].x = -350;
		way_points[5].y = 700;
		way_points[6].x = -700;
		way_points[6].y = 350;

		way_points[7].x = -700;
		way_points[7].y = 350;
		way_points[7].x = -700;
		way_points[7].y = 350;
		way_points[8].x = -700;
		way_points[8].y = 350;
		way_points[9].x = -700;
		way_points[9].y = 350;

	}

	void init_PAWN_status(void)
	{
		//pinMode(LIFT_MOTOR_PWM, OUTPUT);
		pinMode(LIFT_MOTOR_SENS, OUTPUT);

		pinMode(LIFT_SWITCH_UP, INPUT);
		pinMode(LIFT_SWITCH_DOWN, INPUT);

		gripServo_right.attach(RIGHT_SERVO);
		gripServo_left.attach(LEFT_SERVO);

		lifter_servo.attach(LIFT_MOTOR_PWM);
		lifter_servo.write(90);

		// PAWN_go_down();
		PAWN_go_up();
		PAWN_grip_pawn();

	}





	/******************/
	/* COMPUTING Functions */
	/******************/
	int convert_longIR_value(int value) 
	{
		return 65 * pow((value * 0.0048828125), -1.10);
	}

	int convert_medIR_value(int value) 
	{
		return (6787 / (value - 3)) - 4; 
	}

	struct Point compute_pawn_position(struct robot *my_robot, int distance) 
	{
		struct Point result;

		result.x = my_robot->pos_X + distance * cos(my_robot->theta);
		result.y = my_robot->pos_Y + distance * sin(my_robot->theta);

		return result;
	}

	char check_point_in_map(struct Point *my_point)
	{
		char error = 0; // Outside the map
		if( (my_point->x > 1420) || (my_point->x < -1420) || (my_point->y > 2020) || (my_point->y < 80) )
			error = 0;
		else 
			error = 1;

		return error; 
	}

	/* Return 1 if the trajectory and the pawn have an intersection, else 0 */
	int trajectory_intersection_pawn(struct Point *start, struct Point *end, struct Point *center, int radius)
	{
		int result = 0;  
		double delta = 0;
                double x1 = start->x /10;
                double y1 = start->y /10;
		double x2 = end->x /10;
                double y2 = end->y /10;
		double x3 = center->x /10;
                double y3 = center->y /10;
		/*  
		    Serial.print("s_x : ");
		    Serial.print(start->x);
		    Serial.print("s_y : ");
		    Serial.println(start->y);
		    Serial.print("e_x : ");
		    Serial.print(end->x);
		    Serial.print("e_y : ");
		    Serial.println(end->y);
		    Serial.print("c_x : ");
		    Serial.print(center->x);
		    Serial.print("c_y : ");
		    Serial.println(center->y);
		    Serial.print("radius : ");
		    Serial.println(radius);
		 */
/*
		double a = (end->x-start->x)*(end->x-start->x) + (end->y-start->y)*(end->y-start->y);
		double b = 2 * ( (end->x-start->x)*(start->x-center->x) + (end->y-start->y)*(start->y-center->y) );
		double c = (start->x-center->x)*(start->x-center->x) + (start->y-center->y)*(start->y-center->y) - radius*radius;
*/
		double a = (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1);
		double b = 2 * ( (x2-x1)*(x1-x3) + (y2-y1)*(y1-y3) );
		double c = (x1-x3)*(x1-x3) + (y1-y3)*(y1-y3) - (radius/10)*(radius/10);

		delta = b * b - 4 * a * c;
		/*
		   delta = pow(2*( (end->x-start->x)*(start->x-center->x) + (end->y-start->y)*(start->y-center->y) ),2) - 4*(pow(end->x-start->x, 2) + pow(end->y-start->y, 2))*(pow((start->x-center->x), 2)+pow((start->y-center->y),2)-pow(radius,2));
		 */
		if( delta >= 0)
			result = 1;
		Serial.println(delta);
		return result;  
	}


	/******************/
	/* A.I. Functions */
	/******************/
	struct Point find_nearest(struct robot *my_robot, struct Point tab[], int size)
	{
		struct Point result;
		double best_distance = 99999.9;
		for (int i = 0; i < size; i++) {
			double distance = distance_coord(my_robot, tab[i].x, tab[i].y);
			if (distance < best_distance) {
				best_distance = distance;
				result.x = tab[i].x;
				result.y = tab[i].y;
			}
		}
		return result;
	}

	/* Returns 0 if not in our color, 1 else */
	int find_pawn_in_our_color(struct Point *pawn, struct Point tab[], int size)
	{
		//int result = 0;
		//double best_distance = 99999.9;
		for (int i = 0; i < size; i++) {
			if( (pawn->x < (tab[i].x + 60)) && (pawn->x > (tab[i].x - 60)) && (pawn->y < (tab[i].y + 60)) && (pawn->y > (tab[i].y - 60)) ) {
				return 1;
			}
		}
		return 0;
	}

	void reinit_y_axis(void)
	{

	}




	/*************************************/
	/* COMMUNICATION WITH MOTION CONTROL */
	/*           SERIAL 1                */
	/*************************************/




	/*********************************/
	/* COMMUNICATION WITH GUI MODULE */
	/*********************************/




	/**********************************/
	/* COMMUNICATION WITH PAWN MODULE */
	/**********************************/
	void PAWN_grip_pawn(void)
	{
		gripServo_right.write(60);
		gripServo_left.write(130);
	}

	void PAWN_release_pawn(void)
	{
		//gripServo_right.write(91);
		//gripServo_left.write(100);
		gripServo_right.write(111);
		gripServo_left.write(80);
	}

	void PAWN_go_up(void)
	{
		int buttonState = 0;
		// Start going up
		//digitalWrite(LIFT_MOTOR_SENS, LIFT_GO_UP);
		//analogWrite(LIFT_MOTOR_PWM, 100);
		lifter_servo.write(118);
		// Stop when the microswitch in activated
		buttonState = digitalRead(LIFT_SWITCH_UP);
		while(buttonState == 1) {
			delay_ms(15);
			buttonState = digitalRead(LIFT_SWITCH_UP);
		}
		// Stop
		lifter_servo.write(90);
		//analogWrite(LIFT_MOTOR_PWM, 0);

	}

	void PAWN_go_down(void)
	{
		int buttonState = 0;
		// Start going up
		//digitalWrite(LIFT_MOTOR_SENS, LIFT_GO_DOWN);
		//analogWrite(LIFT_MOTOR_PWM, 100);
		lifter_servo.write(58);
		// Stop when the microswitch in activated
		buttonState = digitalRead(LIFT_SWITCH_DOWN);
		while(buttonState == 1) {
			delay_ms(15);
			buttonState = digitalRead(LIFT_SWITCH_DOWN);
		}
		// Stop
		lifter_servo.write(90);
		//analogWrite(LIFT_MOTOR_PWM, 0);

	}

	/************************************/
	/* COMMUNICATION WITH BEACON MODULE */
	/************************************/
	void BEACON_get_direction(void)
	{
		if(BEACON_NORTH_PIN) {
			beacon_direction = BEACON_NORTH;
		}
		else if(BEACON_SOUTH_PIN) {
			beacon_direction = BEACON_SOUTH;
		}
		else if(BEACON_EAST_PIN) {
			beacon_direction = BEACON_EAST;
		}
		else if(BEACON_WEST_PIN) {
			beacon_direction = BEACON_WEST;
		}
	}