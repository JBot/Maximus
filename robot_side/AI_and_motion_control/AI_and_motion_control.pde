/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * JBot wrote this file. As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return.
 * ----------------------------------------------------------------------------
 */

/*****************************************************
Project : Maximus
Version : 2.0
Date : 27/04/2010
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
#include <Wire.h>

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
#define TICK_PER_MM_LEFT 	9.2628378129               // 90.9456817668
#define TICK_PER_MM_RIGHT 	9.2628378129               // 90.9456817668
#define DIAMETER 		270.4                      //275.0 // 166.0         // Distance between the 2 wheels

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



#define DISTANCE_REAR_WHEELS    53                         // Distance between the rear of the robot and the center of the 2 wheels

#define NO_PAWN                 0
#define TAKE_PAWN               1
#define GOTO_RELEASE            2
#define GO_BACK                 3
#define GRABBING                4
#define TURNING_DIRECTION       5
#define AVOIDING1               6
#define AVOIDING2               7
#define FIN_MATCH               8
#define INTERMEDIATE_RELEASE    9
#define TAKE_KING1              10
#define TAKE_KING2              11
#define TAKE_KING3              12
#define AVOIDING0               13
#define AVOIDING_OPP1           14
#define AVOIDING_OPP2           15
#define FIND_PAWN               16
#define BACK                    17
#define TMP_PAWN                18
#define RELEASE_BONUS           19
#define STACK                   20


// I/Os definition
//#define INPUT_MOTION_PIN        2
#define RIGHT_SERVO             12
#define LEFT_SERVO              11
//#define PAWN_SENSOR             3                          // A DEFINIR // IR
#define PAWN_SENSOR             46                         // MICROSWITCH
#define PAWN_SENSOR_LEFT        0
#define PAWN_SENSOR_MIDDLE      1
#define PAWN_SENSOR_RIGHT       2
#define OPPONENT_SENSOR_LEFT    4                          // A DEFINIR
#define OPPONENT_SENSOR_RIGHT   5                          // A DEFINIR
#define RIGHT_IR_SENSOR         6                          // A DEFINIR
#define LEFT_IR_SENSOR          7                          // A DEFINIR
#define LIFT_MOTOR_PWM		40
#define LIFT_MOTOR_SENS		41                         // A DEFINIR
#define LIFT_SWITCH_UP          22
#define LIFT_SWITCH_DOWN        23
#define BEACON_NORTH_PIN	46                         // A DEFINIR
#define BEACON_SOUTH_PIN	47                         // A DEFINIR
#define BEACON_EAST_PIN	        48                         // A DEFINIR
#define BEACON_WEST_PIN	        49                         // A DEFINIR
#define LEFT_REAR_SENSOR        43
#define RIGHT_REAR_SENSOR       42

#define RESET_ROBOCLAW          45


#define LIFT_GO_UP		HIGH
#define LIFT_GO_DOWN		LOW

#define SECURE_PAWN             0                          // First phase to secure 2 pawn in the secure zones
#define CREATE_TOWER            1                          // Seconde phase to build 2 tower and place it on the right color
#define PLACING_PAWN            2                          // Last phase to moving on the table and take the enemy pawns

#define BEACON_NORTH            0
#define BEACON_SOUTH            1
#define BEACON_EAST             2
#define BEACON_WEST             3

#define ALPHA_MAX_SPEED         25000                      //24000                      //25000//13000                      // 9000
#define ALPHA_MAX_ACCEL         300
#define ALPHA_MAX_DECEL         3500                       //2500
#define DELTA_MAX_SPEED         50000                      //45000                      //50000//37000
#define DELTA_MAX_SPEED_BACK    35000                      //45000                      //50000//37000
#define DELTA_MAX_SPEED_BACK_PAWN    45000
#define DELTA_MAX_ACCEL         1000                       //900                        //600
#define DELTA_MAX_DECEL         10000                      //4000                       //1800


#define NO_SENSORS
#define HOME                    -170
//#define HOME                    0
//#define SERIAL_COMMANDS         0
//#define OPPONENT_DETECTION      0

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
volatile int pawn_distance = 30;
int front_distance_up_left = 50;
int front_distance_up_right = 50;
int side_king_sensor = 60;
int side_king_sensor2 = 0;
int opponent_sensor = 60;

int side_sensor_on = 1;

int prev_front_distance = 50;

unsigned long global_time_counter = 0;
char has_pawn = NO_PAWN;
char prev_has_pawn = NO_PAWN;


struct Point red_points[18];
struct Point blue_points[18];
struct Point green_points[10];
int green_point_index = 0;

struct Point *my_color_points;
int release_priorities[18];

int nearest_index = 0;

struct Point way_points[20];
int way_point_index = 0;

struct Point placed_pawn[10];
int placed_pawn_index = 0;

struct Point release_point;
struct Point my_test_point;
struct Point the_point;

char start_MOTION = 0;
char turn_counter = 0;

char pawn_stack = 0;                                       // used to know how many pawn we are stacking
char robot_mode = SECURE_PAWN;                             // used to switch between the different phases
char beacon_direction = BEACON_NORTH;                      // used to know where is the opponent
char ajusting_pawn = 0;                                    // To know if we are trying to catch a pawn moving right and left to center it
char go_grab_pawn = 0;
char nb_check = 0;
char nb_check_color = 0;
char sensor_off = 0;                                       // To put the sensor in offmode

volatile char transmit_status = 1;                         // 1 if OK / 0 if not finished

Servo right_wheel;
Servo left_wheel;

int color_serial_in = 'a';
int color = -1;                                            // -1 = blue ; 1 = red

int time_in_match = 0;                                     // To count the time => 21900 ticks = 90 secondes

int avoid_radius = 0;
int nb_pawn_in_case = 0;

double x_topawn = 0;
double y_topawn = 0;
int sens = 0;
int stack_to_do = 0;

int number_of_king_detected = 0;
struct Point king_points[10];                              // Only 2 values are needed, put to 10 to be sure not to make bug if we write to much values

double current_delta_speed = 0;

struct Point prev_position;

double my_angle = 0.0;
int have_king = 0;
int store_n_stack = 0;

int barCode_timer = 0;

int working_side = 1;                                      // 1 = working on our side / -1 = working on the opponent side
int y_king = 0;                                            // 0 if king have not been found
int y_queen = 0;                                           // 0 if queen have not been found

struct Point taken_king1;
struct Point taken_king2;
struct Point taken_queen1;
struct Point taken_queen2;
int king_taken_our = 0;
int king_taken_opponent = 0;
int queen_taken_our = 0;
int queen_taken_opponent = 0;

int release_pawn = 1;
int pawn_found = 0;

int our_green_zone_empty = 0;                              // To know if our green zone is empty
int opp_green_zone_empty = 0;                              // To know if the opponent's green zone is empty

struct Point subzones[6];
struct Point avoid_points[2];
int opponent_subzone = 0;

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
            if ((transmit_status) == 1)
                move_motors(ALPHADELTA);                   // Update the motor speed
    } else {
        if (roboclaw_ON == 1)
            move_motors(LEFTRIGHT);                        // Update the motor speed
    }

    if (color_serial_in == 'O')
        time_in_match++;

    if (time_in_match > 10970) {                           // End of the match
        if (time_in_match < 21900) {
            PAWN_release_pawn();
            Serial.println("STOP ROBOT");
            Serial3.print("e");
        }
        time_in_match = 21901;

        stop_robot();
        has_pawn = FIN_MATCH;
    }

}

// Timer 3 overflow interrupt service routine
/*ISR(TIMER3_OVF_vect)
{
    sei();                                                 // enable interrupts
    current_delta_speed = distance_coord(&maximus, prev_position.x, prev_position.y);
    Serial.println(current_delta_speed);
    if( (delta_motor.cur_speed > 6000) && (fabs(current_delta_speed) < 5) ) { // Si on veut avancer mais que notre vitesse est nulle 
      Serial.println("Block");      

    }

    prev_position.x = maximus.pos_X;
    prev_position.y = maximus.pos_Y;

    if() { // Si le compteur est atteint
      // On recule et on va au milieu du terrain
      
    }
	// Compute sensors
	BEACON_get_direction();

	int sensorValue = 0;

    sensorValue = analogRead(PAWN_SENSOR_MIDDLE);
    front_distance_down_middle = (front_distance_down_middle + (convert_longIR_value(sensorValue)) * 2) / 3;
    if (front_distance_down_middle > 330 || front_distance_down_middle < 0)
        front_distance_down_middle = 0;
    sensorValue = analogRead(PAWN_SENSOR_LEFT);
    front_distance_down_left = (front_distance_down_left + (convert_longIR_value(sensorValue)) * 2) / 3;
    if (front_distance_down_left < 0)
        front_distance_down_left = 150;
    if (front_distance_down_left > 250)
        front_distance_down_left = 0;    
    sensorValue = analogRead(PAWN_SENSOR_RIGHT);
    front_distance_down_right = (front_distance_down_right + (convert_longIR_value(sensorValue)) * 2) / 3;
    if (front_distance_down_right < 0)
        front_distance_down_right = 150;
    if (front_distance_down_right > 250)
        front_distance_down_right = 0;
    sensorValue = analogRead(OPPONENT_SENSOR_LEFT);
    front_distance_up_left = (front_distance_up_left + (convert_medIR_value(sensorValue)) * 2) / 3;
    sensorValue = analogRead(OPPONENT_SENSOR_RIGHT);
    front_distance_up_right = (front_distance_up_right + (convert_medIR_value(sensorValue)) * 2) / 3;
	
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
//      pinMode(13, OUTPUT);
    pinMode(LIFT_MOTOR_PWM, OUTPUT);
    analogWrite(LIFT_MOTOR_PWM, 0);
    pinMode(LIFT_MOTOR_SENS, OUTPUT);

    pinMode(RESET_ROBOCLAW, OUTPUT);
    digitalWrite(RESET_ROBOCLAW, HIGH);

    // INPUTS
//      pinMode(INPUT_MOTION_PIN, INPUT);
    //pinMode(PAWN_SENSOR, INPUT);
    pinMode(LIFT_SWITCH_UP, INPUT);
    pinMode(LIFT_SWITCH_DOWN, INPUT);
    pinMode(BEACON_NORTH_PIN, INPUT);
    pinMode(BEACON_SOUTH_PIN, INPUT);
    pinMode(BEACON_EAST_PIN, INPUT);
    pinMode(BEACON_WEST_PIN, INPUT);
    pinMode(LEFT_REAR_SENSOR, INPUT);
    pinMode(RIGHT_REAR_SENSOR, INPUT);
    pinMode(PAWN_SENSOR, INPUT);


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
/*       TCCR3A = 0x03;    // Interrupt every 16.4ms
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
    Serial.begin(57600);                                   // DEBUG
    //Serial.begin(19200);
    //Serial.begin(115200);
    Serial1.begin(9600);                                   // BARCODE SCANNER
    // Bluetooth
    Serial2.begin(38400);                                  // ROBOCLAW
    Serial3.begin(57600);                                  // Color module

    Wire.begin();

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

    taken_king1.x = 0;
    taken_king1.y = 0;
    taken_king2.x = 0;
    taken_king2.y = 0;
    taken_queen1.x = 0;
    taken_queen1.y = 0;
    taken_queen2.x = 0;
    taken_queen2.y = 0;

//    init_placed_pawns_for_test();

    //PAWN_go_down();

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
    //PAWN_go_up();

//lifter_servo.attach(LIFT_MOTOR_PWM);
//    lifter_servo.write(90);

#ifdef SERIAL_COMMANDS
    while (1) {



        if (Serial.available()) {
            char blabla = Serial.read();
            switch (blabla) {
            case 'A':
                set_new_command(&bot_command_alpha, 90);
                break;
            case 'Z':
                set_new_command(&bot_command_alpha, -180);
                break;
            case 'E':
                set_new_command(&bot_command_alpha, -90);
                break;
            case 'W':
                set_new_command(&bot_command_delta, 1000);
                break;
            case 'X':
                set_new_command(&bot_command_delta, -1000);
                break;
            case 'p':
                PAWN_release_pawn();
                break;
            case 'o':
                PAWN_grip_pawn();
                break;
            case 'm':
                PAWN_go_down();
                break;
            case 'l':
                PAWN_go_up();
                break;

            }
        }

        Wire.beginTransmission(0x70);
        Wire.send(0x00);
        Wire.send(0x51);
        Wire.endTransmission();


        //set_new_command(&bot_command_delta, 1256);
        delay(30);



//    Serial.print(maximus.pos_X);
//    Serial.print(" ");
//    Serial.print(maximus.pos_Y);
//    Serial.print(" ");
//    Serial.println(maximus.theta * RAD2DEG);


        int sensorValue = 0;

        Wire.beginTransmission(0x70);
        Wire.send(0x02);
        Wire.endTransmission();

        Wire.requestFrom(0x70, 2);

        if (2 <= Wire.available()) {
            side_king_sensor2 = Wire.receive();
            side_king_sensor2 = side_king_sensor2 << 8;
            side_king_sensor2 |= Wire.receive();
        }
//        read_down_IR();

        sense_opponent_ir();

        sensorValue = analogRead(PAWN_SENSOR);
        pawn_distance = (pawn_distance + (convert_shortIR_value(sensorValue)) * 2) / 3;
        if (pawn_distance > 30 || pawn_distance < 0)
            pawn_distance = 30;


        //sensorValue = analogRead(LEFT_IR_SENSOR);
        //side_king_sensor2 = (side_king_sensor2 + (convert_longIR_value(sensorValue)) * 4) / 5;
        //if (side_king_sensor2 > 150 || side_king_sensor2 < 0)
        //      side_king_sensor2 = 150;


        if (barCode_timer == 0) {
            barCode_scan();
            barCode_timer = 25;
        } else {
            barCode_timer--;
        }

        if (barCode_checkifdata() == 1) {
            have_king = 1;
            Serial.println("CODEBAR DETECTED");
        }
//        Serial.print("left : ");
//        Serial.print(front_distance_down_left);
//        Serial.print(" middle : ");
//        Serial.print(front_distance_down_middle);
//        Serial.print(" right : ");
//        Serial.print(front_distance_down_right);
        Serial.print(" PAWN : ");
        Serial.print(pawn_distance);
        Serial.print(" SIDE2 : ");
        Serial.print(side_king_sensor2);

        Serial.print(" OPP_RIGHT : ");
        Serial.print(front_distance_up_right);
        Serial.print(" OPP_LEFT : ");
        Serial.println(front_distance_up_left);



//        if (front_distance_down_right < 30)
//            Serial.println("RIGHT");

//        if (front_distance_down_left < 30)
//            Serial.println("LEFT");

//        if (front_distance_down_middle < 30)
//            Serial.println("MIDDLE");

//delay(2);

    }
#endif



    Serial3.print('C');
    delay_ms(10);
    color_serial_in = Serial3.read();
    while ((color_serial_in != 'B') && (color_serial_in != 'R')) {
        delay_ms(100);
        Serial3.print('C');
        delay_ms(10);
        color_serial_in = Serial3.read();
        //Serial.print(color_serial_in, BYTE);
    }
    if (color_serial_in == 'R') {                          // Red selected
        color = 1;
        init_red_Robot(&maximus);

    } else {                                               // Blue selected
        color = -1;
        init_blue_Robot(&maximus);

    }

    init_color_points();                                   // Init the color field
    init_way_points();

    // Do the auto initialization
    init_first_position(&maximus);

    Serial.print(maximus.pos_X);
    Serial.print(" ");
    Serial.print(maximus.pos_Y);
    Serial.print(" ");
    Serial.println(maximus.theta * RAD2DEG);


    delay_ms(500);


    // WAIT START COMMAND

    Serial3.print('S');
    delay_ms(10);
    color_serial_in = Serial3.read();
    while ((color_serial_in != 'O')) {
        delay_ms(100);
        Serial3.print('S');
        delay_ms(10);
        color_serial_in = Serial3.read();
        //Serial.print(color_serial_in, BYTE);
    }

    time_in_match = 0;

    Serial.println("START");

    Wire.beginTransmission(0x70);
    Wire.send(0x00);
    Wire.send(0x51);
    Wire.endTransmission();


    PAWN_release_pawn();
    delay_ms(10);

}

/******************/
/* MAIN CODE LOOP */
/******************/
void loop()
{
    // Place your code here
    int sensorValue = 0;


    if (global_time_counter == 8) {
        if (transmit_status == 1) {
            read_RoboClaw_voltage(128);
        }
    }
    if (global_time_counter == 9) {
        if (transmit_status == 1) {
            check_RoboClaw_response(128);
        }
    }


    if (global_time_counter > 50) {
        //Serial.println("Alive");
        global_time_counter = 0;
    }

    delay_ms(20);

    global_time_counter++;





    /* Main switch between the modes */
    switch (robot_mode) {
        /* Static phase to secure the first 2 pawns */
    case SECURE_PAWN:


        /* For pawn detection */
        if ((has_pawn == NO_PAWN)) {

/*            sensorValue = analogRead(PAWN_SENSOR);
            pawn_distance = (pawn_distance + (convert_shortIR_value(sensorValue)) * 2) / 3;
            if (pawn_distance > 30 || pawn_distance < 0)
                pawn_distance = 30;
*/
            sensorValue = digitalRead(PAWN_SENSOR);

            if ((sensorValue == HIGH) && (pawn_found == 0)) {

                if (nb_check == 0) {

                    stop_robot();

                    delta_motor.max_speed = 20000;

                    Serial.println("Take pawn");
                    go_grab_pawn = 0;

                    delay(100);
                    set_new_command(&bot_command_delta, 50);

                    PAWN_release_pawn();
                    PAWN_go_down();
                    PAWN_grip_pawn();

                    delta_motor.max_speed = DELTA_MAX_SPEED;

                    // Go put the pawn on the right space
                    delay(100);
                    PAWN_mini_go_up();

                    if (release_priorities[0] != 20) {     // It's the first pawn we take

                        x_topawn = my_color_points[0].x;
                        y_topawn = my_color_points[0].y;
                        release_point.x = my_color_points[0].x;
                        release_point.y = my_color_points[0].y;
                        nearest_index = 0;

                        sens = move_pawn_to_xy(&maximus, &x_topawn, &y_topawn);
                        if (sens == 0) {                   // Front
                            goto_xy(x_topawn, y_topawn);
                        } else {                           // Back
                            goto_xy_back(x_topawn, y_topawn);
                        }
                        Serial.println("Premier pion");
                        has_pawn = GOTO_RELEASE;
                    } else {                               // Pawn fot the secured zone
                        x_topawn = my_color_points[16].x;
                        y_topawn = my_color_points[16].y;
                        release_point.x = my_color_points[16].x;
                        release_point.y = my_color_points[16].y;
                        nearest_index = 16;

                        //goto_xy(0, 1600);
                        goto_xy(HOME, 1600);               // POUR LES TESTS A LA MAISON

                        Serial.println("Second pion");
                        has_pawn = INTERMEDIATE_RELEASE;
                    }


                    pawn_stack = 0;

                    nb_check = 0;
                } else {
                    nb_check++;
                }


            }

        }



        if ((bot_command_alpha.state == COMMAND_DONE) && (bot_command_delta.state == COMMAND_DONE)) {
            switch (has_pawn) {

            case NO_PAWN:

                goto_xy(way_points[way_point_index].x, way_points[way_point_index].y);

                Serial.print(".Go to :");
                Serial.print(way_points[way_point_index].x);
                Serial.print(" ");
                Serial.println(way_points[way_point_index].y);
                way_point_index++;



                break;

            case INTERMEDIATE_RELEASE:
                sens = move_pawn_to_xy(&maximus, &x_topawn, &y_topawn);
                if (sens == 0) {                           // Front
                    goto_xy(x_topawn, y_topawn);
                } else {                                   // Back
                    goto_xy_back(x_topawn, y_topawn);
                }

                has_pawn = GOTO_RELEASE;
                break;

            case GOTO_RELEASE:                            // The robot have a pawn and is on his release point
                PAWN_release_for_greenzone();
                delta_motor.max_speed = DELTA_MAX_SPEED_BACK;
                set_new_command(&bot_command_delta, (-150));    // TO ADJUST

                PAWN_go_up();

                pawn_stack = 0;

                release_priorities[nearest_index] = 20;
                placed_pawn[placed_pawn_index].x = my_color_points[nearest_index].x;
                placed_pawn[placed_pawn_index].y = my_color_points[nearest_index].y;
                placed_pawn_index++;

                have_king = 0;

                has_pawn = GO_BACK;
                break;

            case GO_BACK:
                PAWN_release_pawn();
                delta_motor.max_speed = DELTA_MAX_SPEED;
                if (nearest_index == 16) {                 // Phase 1 done
                    green_point_index = 3;
                    my_test_point.y = green_points[green_point_index].y;
                    if (green_points[green_point_index].x < 0)
                        goto_xy(-800, green_points[green_point_index].y);
                    else
                        goto_xy(800, green_points[green_point_index].y);
                    //goto_xy(green_points[way_point_index].x, green_points[way_point_index].y);
                    robot_mode = CREATE_TOWER;
                    has_pawn = NO_PAWN;
                } else {                                   // Go take the second pawn
                    goto_xy(way_points[2].x, way_points[2].y);

                    Serial.print(".Go to :");
                    Serial.print(way_points[2].x);
                    Serial.print(" ");
                    Serial.println(way_points[2].y);

                    has_pawn = NO_PAWN;
                }


                break;





            }

        }



        break;

    case CREATE_TOWER:




        Wire.beginTransmission(0x70);
        Wire.send(0x02);
        Wire.endTransmission();

        Wire.requestFrom(0x70, 2);

        if (2 <= Wire.available()) {
            opponent_sensor = Wire.receive();
            opponent_sensor = opponent_sensor << 8;
            opponent_sensor |= Wire.receive();
        }

        Wire.beginTransmission(0x70);
        Wire.send(0x00);
        Wire.send(0x51);
        Wire.endTransmission();

        sense_opponent_ir();

#ifdef OPPONENT_DETECTION
        /* OPPONENT DETECTION */
        if (((opponent_sensor < 47 && have_king == 0) || (front_distance_up_right < 35) || (front_distance_up_left < 35)) && (has_pawn != TAKE_PAWN)
            && (has_pawn != GO_BACK) && (has_pawn != BACK) && (has_pawn != TURNING_DIRECTION) && (bot_command_alpha.state == COMMAND_DONE)) {
            prev_has_pawn = has_pawn;

            struct Point test_point;
            test_point = estimate_center(&maximus);

            if ((check_point_in_map(&test_point) != 0)) {
                Serial.print("Opponent detected ");
                Serial.print(opponent_sensor);
                Serial.print(" ");
                Serial.print(front_distance_up_right);
                Serial.print(" ");
                Serial.println(front_distance_up_left);

                if (color == 1) {
                    if ((test_point.x > 350) && (test_point.y < 1050)) {
                        opponent_subzone = 4;
                    } else if ((test_point.x > 350) && (test_point.y > 1050)) {
                        opponent_subzone = 5;
                    } else if ((abs(test_point.x) < 350) && (test_point.y < 1050)) {
                        opponent_subzone = 2;
                    } else if ((abs(test_point.x) < 350) && (test_point.y > 1050)) {
                        opponent_subzone = 3;
                    } else if ((test_point.x < -350) && (test_point.y < 1050)) {
                        opponent_subzone = 1;
                    } else if ((test_point.x < -350) && (test_point.y > 1050)) {
                        opponent_subzone = 2;
                    }
                } else {
                    if ((test_point.x > 350) && (test_point.y < 1050)) {
                        opponent_subzone = 0;
                    } else if ((test_point.x > 350) && (test_point.y > 1050)) {
                        opponent_subzone = 1;
                    } else if ((abs(test_point.x) < 350) && (test_point.y < 1050)) {
                        opponent_subzone = 2;
                    } else if ((abs(test_point.x) < 350) && (test_point.y > 1050)) {
                        opponent_subzone = 3;
                    } else if ((test_point.x < -350) && (test_point.y < 1050)) {
                        opponent_subzone = 4;
                    } else if ((test_point.x < -350) && (test_point.y > 1050)) {
                        opponent_subzone = 5;
                    }
                }




                if ((is_in_our_side(&maximus) == 1) && (working_side == -1) && (opponent_subzone == 2)) {
                    // We want to go on opponent side, but opponent is in the middle
                    stop_robot();
                    delay(200);
                    goto (color * 350, 1400);
                    my_test_point.x = (-1) * color * 300;
                    my_test_point.y = 1400;
                    has_pawn = AVOIDING_OPP1;
                } else if ((is_in_our_side(&maximus) == 1) && (working_side == -1) && (opponent_subzone == 3)) {
                    // We want to go on opponent side, but opponent is in the middle
                    stop_robot();
                    delay(200);
                    goto (color * 350, 350);
                    my_test_point.x = (-1) * color * 300;
                    my_test_point.y = 350;
                    has_pawn = AVOIDING_OPP1;
                } else if ((is_in_our_side(&maximus) == 0) && (working_side == 1) && (opponent_subzone == 3)) {
                    // We want to go on our side, but opponent is in the middle
                    stop_robot();
                    delay(200);
                    goto ((-1) * color * 300, 350);
                    my_test_point.x = color * 350;
                    my_test_point.y = 350;
                    has_pawn = AVOIDING_OPP1;
                } else if ((is_in_our_side(&maximus) == 0) && (working_side == 1) && (opponent_subzone == 2)) {
                    // We want to go on our side, but opponent is in the middle
                    stop_robot();
                    delay(200);
                    goto ((-1) * color * 300, 1400);
                    my_test_point.x = color * 350;
                    my_test_point.y = 1400;
                    has_pawn = AVOIDING_OPP1;
                } else if ((is_in_our_side(&maximus) == 1) && (working_side == 1) && (opponent_subzone == 4 || opponent_subzone == 5)) {
                    // We are in our side, and the opponent too
                    stop_robot();
                    delay(200);
                    if (pawn_stack == 0) {                 // We don't carry anything


                    } else {                               // We are carrying something
                        switch (have_king) {
                        case 0:                           // Pawn

                            break;
                        case 1:                           // King

                            break;
                        case 2:                           // Queen

                            break;
                        }
                    }
                    goto ((-1) * color * 300, 1400);
                    my_test_point.x = color * 350;
                    my_test_point.y = 1400;
                    has_pawn = AVOIDING_OPP1;
                }











                if (pawn_stack == 0) {                     // We don't carry anything


                } else {                                   // We are carrying something
                    switch (have_king) {
                    case 0:                               // Pawn

                        break;
                    case 1:                               // King

                        break;
                    case 2:                               // Queen

                        break;
                    }
                }

                if (is_in_our_side(&maximus) == 1) {

                }

            }

        }
#endif










        /* For pawn detection */
        if ((has_pawn == FIND_PAWN)) {

            sensorValue = digitalRead(PAWN_SENSOR);

            if ((sensorValue == HIGH)) {

                if (nb_check == 0) {

                    stop_robot();

                    delta_motor.max_speed = 20000;

                    Serial.println("Take pawn");
                    go_grab_pawn = 0;

                    delay(100);
                    set_new_command(&bot_command_delta, 20);

                    PAWN_release_pawn();
                    delay(100);
                    set_new_command(&bot_command_delta, -20);
                    delay(100);
                    PAWN_go_down();
                    PAWN_grip_pawn();

                    delta_motor.max_speed = DELTA_MAX_SPEED;

                    // Go put the pawn on the right space
                    delay(200);
                    PAWN_go_up();

                    nearest_index = 16;
                    x_topawn = my_color_points[nearest_index].x;
                    y_topawn = my_color_points[nearest_index].y;
                    release_point.x = my_color_points[nearest_index].x;
                    release_point.y = my_color_points[nearest_index].y;

                    //goto_xy(0, 1600);
                    goto_xy(HOME, 1600);                   // POUR LES TESTS A LA MAISON

                    has_pawn = RELEASE_BONUS;
                    Serial.println("Tour de ROI");

                    //has_pawn = GOTO_RELEASE;
                    pawn_stack++;

                    nb_check = 0;
                } else {
                    nb_check++;
                }


            }

        }


        /* For pawn detection when we cary a QUEEN in the bonus zone */
        if ((has_pawn == RELEASE_BONUS)) {

            sensorValue = digitalRead(PAWN_SENSOR);

            if ((sensorValue == HIGH)) {

                if (nb_check == 0) {

                    stop_robot();

                    delta_motor.max_speed = 20000;

                    Serial.println("Take pawn");
                    go_grab_pawn = 0;

                    delay(100);
                    set_new_command(&bot_command_delta, 20);

                    PAWN_release_pawn();
                    delay(100);
                    set_new_command(&bot_command_delta, -20);
                    delay(100);
                    PAWN_go_down();
                    PAWN_grip_pawn();

                    delta_motor.max_speed = DELTA_MAX_SPEED;

                    // Go put the pawn on the right space
                    delay(200);
                    PAWN_go_up();

                    nearest_index = 16;
                    x_topawn = my_color_points[nearest_index].x;
                    y_topawn = my_color_points[nearest_index].y;
                    release_point.x = my_color_points[nearest_index].x;
                    release_point.y = my_color_points[nearest_index].y;

                    //goto_xy(0, 1600);
                    goto_xy(HOME, 1600);                   // POUR LES TESTS A LA MAISON

                    delay(200);

                    pawn_stack = 0;

                    nb_check = 0;
                } else {
                    nb_check++;
                }


            }

        }


        /* For pawn detection when we cary a PAWN to drop anywhere */
        if ((has_pawn == GOTO_RELEASE)) {

            sensorValue = digitalRead(PAWN_SENSOR);

            if ((sensorValue == HIGH)) {

                if (nb_check == 0) {

                    Serial.println("PAWN SEEN");

                    if (have_king == 0) {                  // Carriyng PAWN
                        release_pawn = 0;
                    } else {                               // King or queen

                    }


                    nb_check = 0;
                } else {
                    nb_check++;
                }


            }

        }


        /* For KING detection */
        if ((has_pawn == TAKE_PAWN)) {
            //if ((has_pawn == TAKE_PAWN) && (bot_command_alpha.state == COMMAND_DONE)) {
            if (barCode_timer == 0) {
                barCode_scan();
                barCode_timer = 25;
            } else {
                barCode_timer--;
            }

            if (have_king == 0) {
                int test_barcode = barCode_checkifdata();
                have_king = test_barcode;
            }


            sensorValue = digitalRead(PAWN_SENSOR);

            if ((sensorValue == HIGH) && (pawn_found == 0)) {

                if (nb_check == 0) {

                    direct_stop_robot();

                    Serial.println("Take pawn");
                    //set_new_command(&bot_command_delta, -5);

                    nb_check = 0;
                    pawn_found = 1;
                } else {
                    nb_check++;
                }

            }


        }




        if (has_pawn == AVOIDING2) {
            // Check if the trajectory become available

            the_point.x = maximus.pos_X;
            the_point.y = maximus.pos_Y;
            if (trajectory_intersection_pawn(&the_point, &my_test_point, &release_point, 270) == 1) {   // pawn is in the trajectory
                // Compute an intermediate way_point
                //Serial.print("In trajectory ");

            } else {
                Serial.println("Not anymore in trajectory");
                //stop_robot();

                delta_motor.max_speed = DELTA_MAX_SPEED;
                alpha_motor.max_speed = ALPHA_MAX_SPEED;
                set_new_command(&bot_command_delta, 10);
                //goto_xy(way_points[way_point_index - 1].x, way_points[way_point_index - 1].y);
                has_pawn = prev_has_pawn;
                goto_avoiding_placed_point(&maximus, placed_pawn, placed_pawn_index, &my_test_point);
                //delay(400);


            }

        }













        if ((bot_command_alpha.state == COMMAND_DONE) && (bot_command_delta.state == COMMAND_DONE)) {
            switch (has_pawn) {

            case NO_PAWN:
                Serial.println("NO_PAWN");
                barCode_flush();
                pawn_found = 0;
                delta_motor.max_speed = DELTA_MAX_SPEED_BACK;

/*                if (green_points[green_point_index].x < 0)
                    goto_xy(-900, green_points[green_point_index].y);
                else
                    goto_xy(900, green_points[green_point_index].y);
*/
                goto_xy(working_side * green_points[green_point_index].x, my_test_point.y);

                PAWN_release_for_greenzone();
                PAWN_go_down();

                //has_pawn = TURNING_DIRECTION;
                has_pawn = TAKE_PAWN;
                break;

            case TURNING_DIRECTION:


                break;

            case TAKE_PAWN:
                delta_motor.max_speed = DELTA_MAX_SPEED;

                if (pawn_found == 1) {
                    pawn_stack++;
                } else {                                   // Nothing has been take
                    sensorValue = digitalRead(PAWN_SENSOR);

                    if ((sensorValue == HIGH)) {
                        pawn_stack++;
                    }

                }
                Serial.print(pawn_distance);
                Serial.println("TAKE_PAWN");
                PAWN_grip_pawn();
                delay(250);

                PAWN_mini_go_up();

                delta_motor.max_speed = DELTA_MAX_SPEED_BACK_PAWN;
                set_new_command(&bot_command_delta, (-330));    //350

                // To store the king and queen position
                if (have_king == 1) {                      // I have a king
                    if (working_side == 1) {               // On our side
                        king_taken_our = 1;

                        if (taken_king1.x == 0) {
                            taken_king1.x = green_points[green_point_index].x;
                            taken_king1.y = green_points[green_point_index].y;
                        } else {
                            taken_king2.x = green_points[green_point_index].x;
                            taken_king2.y = green_points[green_point_index].y;
                        }
                    } else {
                        king_taken_opponent = 1;

                        if (taken_king1.x == 0) {
                            taken_king1.x = green_points[green_point_index].x;
                            taken_king1.y = green_points[green_point_index].y;
                        } else {
                            taken_king2.x = green_points[green_point_index].x;
                            taken_king2.y = green_points[green_point_index].y;
                        }
                    }
                } else if (have_king == 2) {               // I have a queen
                    if (working_side == 1) {               // On our side
                        queen_taken_our = 1;

                        if (taken_queen1.x == 0) {
                            taken_queen1.x = green_points[green_point_index].x;
                            taken_queen1.y = green_points[green_point_index].y;
                        } else {
                            taken_queen2.x = green_points[green_point_index].x;
                            taken_queen2.y = green_points[green_point_index].y;
                        }
                    } else {
                        queen_taken_opponent = 1;

                        if (taken_queen1.x == 0) {
                            taken_queen1.x = green_points[green_point_index].x;
                            taken_queen1.y = green_points[green_point_index].y;
                        } else {
                            taken_queen2.x = green_points[green_point_index].x;
                            taken_queen2.y = green_points[green_point_index].y;
                        }
                    }
                }



                PAWN_go_up();

                has_pawn = GO_BACK;


                break;

            case GO_BACK:
                Serial.println("GO_BACK");
                delta_motor.max_speed = DELTA_MAX_SPEED;
                release_pawn = 1;

                if (pawn_stack == 0) {                     // Nothing has been take
                    has_pawn = BACK;
                } else {

                    // MULTIPLE POSSIBILITIES
                    if (have_king == 1) {                  // I have a king


                        if (working_side == 1) {           // On our side
                            way_point_index = 3;
                            goto_xy(way_points[way_point_index].x, way_points[way_point_index].y);

                            has_pawn = FIND_PAWN;
                        } else {

                            // TODO
                            if (release_priorities[14] >= 10) { // if a pawn is stored
                                nearest_index = 14;
                                x_topawn = my_color_points[nearest_index].x;
                                y_topawn = my_color_points[nearest_index].y;
                                release_point.x = my_color_points[nearest_index].x;
                                release_point.y = my_color_points[nearest_index].y;


                                sens = move_pawn_to_xy(&maximus, &x_topawn, &y_topawn);
                                if (sens == 0) {           // Front
                                    goto_xy(x_topawn, y_topawn);
                                } else {                   // Back
                                    goto_xy_back(x_topawn, y_topawn);
                                }
                                nearest_index = 11;
                                x_topawn = my_color_points[nearest_index].x;
                                y_topawn = my_color_points[nearest_index].y;
                                release_point.x = my_color_points[nearest_index].x;
                                release_point.y = my_color_points[nearest_index].y;

                                has_pawn = STACK;
                            } else {
                                nearest_index = 11;
                                x_topawn = my_color_points[nearest_index].x;
                                y_topawn = my_color_points[nearest_index].y;
                                release_point.x = my_color_points[nearest_index].x;
                                release_point.y = my_color_points[nearest_index].y;


                                sens = move_pawn_to_xy(&maximus, &x_topawn, &y_topawn);
                                if (sens == 0) {           // Front
                                    goto_xy(x_topawn, y_topawn);
                                } else {                   // Back
                                    goto_xy_back(x_topawn, y_topawn);
                                }


                                has_pawn = GOTO_RELEASE;
                            }
                        }
                    } else if (have_king == 2) {           // I have a queen


                        if (working_side == 1) {           // On our side

                            // TODO
                            if (release_priorities[12] >= 10) { // if a pawn is stored
                                nearest_index = 12;
                                x_topawn = my_color_points[nearest_index].x;
                                y_topawn = my_color_points[nearest_index].y;
                                release_point.x = my_color_points[nearest_index].x;
                                release_point.y = my_color_points[nearest_index].y;


                                sens = move_pawn_to_xy(&maximus, &x_topawn, &y_topawn);
                                if (sens == 0) {           // Front
                                    goto_xy(x_topawn, y_topawn);
                                } else {                   // Back
                                    goto_xy_back(x_topawn, y_topawn);
                                }
                                nearest_index = 0;
                                x_topawn = my_color_points[nearest_index].x;
                                y_topawn = my_color_points[nearest_index].y;
                                release_point.x = my_color_points[nearest_index].x;
                                release_point.y = my_color_points[nearest_index].y;
                                has_pawn = STACK;

                            } else {
                                nearest_index = 0;
                                x_topawn = my_color_points[nearest_index].x;
                                y_topawn = my_color_points[nearest_index].y;
                                release_point.x = my_color_points[nearest_index].x;
                                release_point.y = my_color_points[nearest_index].y;

                                sens = move_pawn_to_xy(&maximus, &x_topawn, &y_topawn);
                                if (sens == 0) {           // Front
                                    goto_xy(x_topawn, y_topawn);
                                } else {                   // Back
                                    goto_xy_back(x_topawn, y_topawn);
                                }

                                has_pawn = GOTO_RELEASE;
                            }
                        } else {

                            // TODO
                            if (release_priorities[14] >= 10) { // if a pawn is stored
                                nearest_index = 14;
                                x_topawn = my_color_points[nearest_index].x;
                                y_topawn = my_color_points[nearest_index].y;
                                release_point.x = my_color_points[nearest_index].x;
                                release_point.y = my_color_points[nearest_index].y;


                                sens = move_pawn_to_xy(&maximus, &x_topawn, &y_topawn);
                                if (sens == 0) {           // Front
                                    goto_xy(x_topawn, y_topawn);
                                } else {                   // Back
                                    goto_xy_back(x_topawn, y_topawn);
                                }
                                has_pawn = STACK;
                            } else {
                                nearest_index = 5;
                                x_topawn = my_color_points[nearest_index].x;
                                y_topawn = my_color_points[nearest_index].y;
                                release_point.x = my_color_points[nearest_index].x;
                                release_point.y = my_color_points[nearest_index].y;


                                sens = move_pawn_to_xy(&maximus, &x_topawn, &y_topawn);
                                if (sens == 0) {           // Front
                                    goto_xy(x_topawn, y_topawn);
                                } else {                   // Back
                                    goto_xy_back(x_topawn, y_topawn);
                                }

                                has_pawn = GOTO_RELEASE;
                            }
                        }
                    } else {                               // I have a pawn
                        if (working_side == 1) {           // On our side
                            // First on the store zone, then on bonus zone in opponent's side
                            if (release_priorities[12] < 6) {
                                nearest_index = 12;
                                x_topawn = my_color_points[nearest_index].x;
                                y_topawn = my_color_points[nearest_index].y;
                                release_point.x = my_color_points[nearest_index].x;
                                release_point.y = my_color_points[nearest_index].y;

                                sens = move_pawn_to_xy(&maximus, &x_topawn, &y_topawn);
                                if (sens == 0) {           // Front
                                    goto_xy(x_topawn, y_topawn);
                                } else {                   // Back
                                    goto_xy_back(x_topawn, y_topawn);
                                }
                            } else if (release_priorities[11] < 6) {    // Nothing on it //11
                                nearest_index = 11;
                                x_topawn = my_color_points[nearest_index].x;
                                y_topawn = my_color_points[nearest_index].y;
                                release_point.x = my_color_points[nearest_index].x;
                                release_point.y = my_color_points[nearest_index].y;

                                sens = move_pawn_to_xy(&maximus, &x_topawn, &y_topawn);
                                if (sens == 0) {           // Front
                                    goto_xy(x_topawn, y_topawn);
                                } else {                   // Back
                                    goto_xy_back(x_topawn, y_topawn);
                                }
                                working_side = -1;
                            } else if (release_priorities[5] < 6) {     // Nothing on it //5
                                nearest_index = 5;
                                x_topawn = my_color_points[nearest_index].x;
                                y_topawn = my_color_points[nearest_index].y;
                                release_point.x = my_color_points[nearest_index].x;
                                release_point.y = my_color_points[nearest_index].y;

                                sens = move_pawn_to_xy(&maximus, &x_topawn, &y_topawn);
                                if (sens == 0) {           // Front
                                    goto_xy(x_topawn, y_topawn);
                                } else {                   // Back
                                    goto_xy_back(x_topawn, y_topawn);
                                }
                                working_side = -1;
                            } else {
                                // Find another place



                                nearest_index = 12;
                                x_topawn = my_color_points[nearest_index].x;
                                y_topawn = my_color_points[nearest_index].y;
                                release_point.x = my_color_points[nearest_index].x;
                                release_point.y = my_color_points[nearest_index].y;

                                sens = move_pawn_to_xy(&maximus, &x_topawn, &y_topawn);
                                if (sens == 0) {           // Front
                                    goto_xy(x_topawn, y_topawn);
                                } else {                   // Back
                                    goto_xy_back(x_topawn, y_topawn);
                                }




                            }

                            has_pawn = GOTO_RELEASE;
                        } else {
                            if (release_priorities[11] == 1) {  // Nothing on it
                                nearest_index = 11;
                                x_topawn = my_color_points[nearest_index].x;
                                y_topawn = my_color_points[nearest_index].y;
                                release_point.x = my_color_points[nearest_index].x;
                                release_point.y = my_color_points[nearest_index].y;



                                sens = move_pawn_to_xy(&maximus, &x_topawn, &y_topawn);
                                my_test_point.x = x_topawn;
                                my_test_point.y = y_topawn;
                                if (sens == 0) {           // Front
                                    //goto_xy(x_topawn, y_topawn);
                                    goto_avoiding_placed_point(&maximus, placed_pawn, placed_pawn_index, &my_test_point);
                                } else {                   // Back
                                    goto_xy_back(x_topawn, y_topawn);
                                }
                            } else if (release_priorities[5] == 1) {    // Nothing on it
                                nearest_index = 5;
                                x_topawn = my_color_points[nearest_index].x;
                                y_topawn = my_color_points[nearest_index].y;
                                release_point.x = my_color_points[nearest_index].x;
                                release_point.y = my_color_points[nearest_index].y;

                                sens = move_pawn_to_xy(&maximus, &x_topawn, &y_topawn);
                                my_test_point.x = x_topawn;
                                my_test_point.y = y_topawn;
                                if (sens == 0) {           // Front
                                    //goto_xy(x_topawn, y_topawn);
                                    goto_avoiding_placed_point(&maximus, placed_pawn, placed_pawn_index, &my_test_point);
                                } else {                   // Back
                                    goto_xy_back(x_topawn, y_topawn);
                                }
                            } else {
                                // Find another place



                                nearest_index = 14;
                                x_topawn = my_color_points[nearest_index].x;
                                y_topawn = my_color_points[nearest_index].y;
                                release_point.x = my_color_points[nearest_index].x;
                                release_point.y = my_color_points[nearest_index].y;

                                sens = move_pawn_to_xy(&maximus, &x_topawn, &y_topawn);
                                if (sens == 0) {           // Front
                                    goto_xy(x_topawn, y_topawn);
                                } else {                   // Back
                                    goto_xy_back(x_topawn, y_topawn);
                                }




                            }

                            has_pawn = GOTO_RELEASE;
                        }
                    }
                }

                break;

            case FIND_PAWN:
                Serial.println("FIND_PAWN");
                way_point_index++;
                goto_xy(way_points[way_point_index].x, way_points[way_point_index].y);
//                nearest_index = 0;

                break;

            case GOTO_RELEASE:
                Serial.println("GOTO_RELEASE");
                if (release_pawn == 1) {
                    PAWN_release_for_greenzone();
                    delta_motor.max_speed = DELTA_MAX_SPEED_BACK;
                    delay(100);
                    set_new_command(&bot_command_delta, (-200));        // TO ADJUST

                    PAWN_go_up();
                    pawn_stack = 0;
                    if (have_king >= 1) {
                        release_priorities[nearest_index] = 99;
                        Serial3.print("h");
                    } else {
                        release_priorities[nearest_index] = 20;
                    }
                    have_king = 0;
                    has_pawn = BACK;
                } else {
                    delta_motor.max_speed = DELTA_MAX_SPEED_BACK;
                    delay(100);
                    set_new_command(&bot_command_delta, (-200));        // TO ADJUST

                    release_priorities[nearest_index] = 20;
                    has_pawn = GO_BACK;
                }
                placed_pawn[placed_pawn_index].x = my_color_points[nearest_index].x;
                placed_pawn[placed_pawn_index].y = my_color_points[nearest_index].y;
                placed_pawn_index++;


                break;

            case BACK:
                Serial.println("BACK");
                PAWN_release_pawn();
                delta_motor.max_speed = DELTA_MAX_SPEED;

                has_pawn = NO_PAWN;
                have_king = 0;

                if (green_point_index == 0) {
                    // CHercher les rois/reines que l'on aurait pas prit

                    if ((king_taken_our == 0) && (king_taken_opponent == 1)) {
                        working_side = 1;
                        have_king = 1;
                        my_test_point.x = color * 800;
                        my_test_point.y = taken_king1.y;
                        goto_avoiding_placed_point(&maximus, placed_pawn, placed_pawn_index, &my_test_point);
                    } else if ((king_taken_our == 1) && (king_taken_opponent == 0)) {
                        working_side = -1;
                        have_king = 1;
                        my_test_point.x = (-1) * color * 800;
                        my_test_point.y = taken_king1.y;
                        goto_avoiding_placed_point(&maximus, placed_pawn, placed_pawn_index, &my_test_point);
                    } else if ((queen_taken_our == 0) && (queen_taken_opponent == 1)) {
                        working_side = 1;
                        have_king = 2;
                        my_test_point.x = color * 800;
                        my_test_point.y = taken_queen1.y;
                        goto_avoiding_placed_point(&maximus, placed_pawn, placed_pawn_index, &my_test_point);
                    } else if ((queen_taken_our == 1) && (queen_taken_opponent == 0)) {
                        working_side = -1;
                        have_king = 2;
                        my_test_point.x = (-1) * color * 800;
                        my_test_point.y = taken_queen1.y;
                        goto_avoiding_placed_point(&maximus, placed_pawn, placed_pawn_index, &my_test_point);
                    } else {
                        our_green_zone_empty = 1;          // To know if our green zone is empty
                        opp_green_zone_empty = 1;          // To know if the opponent's green zone is empty

                        has_pawn = GO_BACK;
                        robot_mode = PLACING_PAWN;
                    }



                } else {


                    if (working_side == 1) {
                        if (king_taken_our == 1) {         // Already take it
                            if (queen_taken_our == 1) {    // Already take it
// TODO
                                green_point_index--;
                                my_test_point.x = working_side * color * 800;
                                my_test_point.y = green_points[green_point_index].y;
                                goto_avoiding_placed_point(&maximus, placed_pawn, placed_pawn_index, &my_test_point);
                                //goto_xy(-800, green_points[green_point_index].y);
// TODO
                            } else if (queen_taken_opponent == 1) {     // Already take the opponent's
                                my_test_point.x = working_side * color * 800;
                                my_test_point.y = taken_queen1.y;
                                goto_avoiding_placed_point(&maximus, placed_pawn, placed_pawn_index, &my_test_point);
                                //goto_xy(-800, green_points[green_point_index].y);
                            } else {
                                green_point_index--;
                                my_test_point.x = working_side * color * 800;
                                my_test_point.y = green_points[green_point_index].y;
                                goto_avoiding_placed_point(&maximus, placed_pawn, placed_pawn_index, &my_test_point);
                                //goto_xy(-800, green_points[green_point_index].y);
                            }
                        } else {                           // King not taken yet
                            if (king_taken_opponent == 1) {
                                my_test_point.x = working_side * color * 800;
                                my_test_point.y = taken_king1.y;
                                goto_avoiding_placed_point(&maximus, placed_pawn, placed_pawn_index, &my_test_point);
                                //goto_xy(-800, green_points[green_point_index].y);
                            } else if (queen_taken_our == 1) {
                                green_point_index--;
                                my_test_point.x = working_side * color * 800;
                                my_test_point.y = green_points[green_point_index].y;
                                goto_avoiding_placed_point(&maximus, placed_pawn, placed_pawn_index, &my_test_point);
                                //goto_xy(-800, green_points[green_point_index].y);
                            } else if (queen_taken_opponent == 1) {
                                my_test_point.x = working_side * color * 800;
                                my_test_point.y = taken_queen1.y;
                                goto_avoiding_placed_point(&maximus, placed_pawn, placed_pawn_index, &my_test_point);
                                //goto_xy(-800, green_points[green_point_index].y);
                            } else {
                                green_point_index--;
                                my_test_point.x = working_side * color * 800;
                                my_test_point.y = green_points[green_point_index].y;
                                goto_avoiding_placed_point(&maximus, placed_pawn, placed_pawn_index, &my_test_point);
                                //goto_xy(-800, green_points[green_point_index].y);
                            }

                        }

                    } else {
                        if (king_taken_opponent == 1) {    // Already take it
                            if (queen_taken_opponent == 1) {    // Already take it
// TODO
                                green_point_index--;
                                my_test_point.x = working_side * color * 800;
                                my_test_point.y = green_points[green_point_index].y;
                                goto_avoiding_placed_point(&maximus, placed_pawn, placed_pawn_index, &my_test_point);
                                //goto_xy(-800, green_points[green_point_index].y);
// TODO
                            } else if (queen_taken_our == 1) {  // Already take the opponent's
                                my_test_point.x = working_side * color * 800;
                                my_test_point.y = taken_queen1.y;
                                goto_avoiding_placed_point(&maximus, placed_pawn, placed_pawn_index, &my_test_point);
                                //goto_xy(-800, green_points[green_point_index].y);
                            } else {
                                green_point_index--;
                                my_test_point.x = working_side * color * 800;
                                my_test_point.y = green_points[green_point_index].y;
                                goto_avoiding_placed_point(&maximus, placed_pawn, placed_pawn_index, &my_test_point);
                                //goto_xy(-800, green_points[green_point_index].y);
                            }
                        } else {                           // King not taken yet
                            if (king_taken_our == 1) {
                                my_test_point.x = working_side * color * 800;
                                my_test_point.y = taken_king1.y;
                                goto_avoiding_placed_point(&maximus, placed_pawn, placed_pawn_index, &my_test_point);
                                //goto_xy(-800, green_points[green_point_index].y);
                            } else if (queen_taken_opponent == 1) {
                                green_point_index--;
                                my_test_point.x = working_side * color * 800;
                                my_test_point.y = green_points[green_point_index].y;
                                goto_avoiding_placed_point(&maximus, placed_pawn, placed_pawn_index, &my_test_point);
                                //goto_xy(-800, green_points[green_point_index].y);
                            } else if (queen_taken_our == 1) {
                                my_test_point.x = working_side * color * 800;
                                my_test_point.y = taken_queen1.y;
                                goto_avoiding_placed_point(&maximus, placed_pawn, placed_pawn_index, &my_test_point);
                                //goto_xy(-800, green_points[green_point_index].y);
                            } else {
                                green_point_index--;
                                my_test_point.x = working_side * color * 800;
                                my_test_point.y = green_points[green_point_index].y;
                                goto_avoiding_placed_point(&maximus, placed_pawn, placed_pawn_index, &my_test_point);
                                //goto_xy(-800, green_points[green_point_index].y);
                            }

                        }


                    }

/*
                    if (green_points[green_point_index].x < 0) {
                        my_test_point.x = -800;
                        my_test_point.y = green_points[green_point_index].y;
                        goto_avoiding_placed_point(&maximus, placed_pawn, placed_pawn_index, &my_test_point);
                        //goto_xy(-800, green_points[green_point_index].y);
                    } else {
                        my_test_point.x = 800;
                        my_test_point.y = green_points[green_point_index].y;
                        goto_avoiding_placed_point(&maximus, placed_pawn, placed_pawn_index, &my_test_point);
                        //goto_xy(800, green_points[green_point_index].y);
                    }
*/
                }

                break;

            case RELEASE_BONUS:
                Serial.println("RELEASE_BONUS");
                x_topawn = my_color_points[16].x;
                y_topawn = my_color_points[16].y;
                release_point.x = my_color_points[16].x;
                release_point.y = my_color_points[16].y;
                nearest_index = 16;

                sens = move_pawn_to_xy(&maximus, &x_topawn, &y_topawn);
                if (sens == 0) {                           // Front
                    goto_xy(x_topawn, y_topawn);
                } else {                                   // Back
                    goto_xy_back(x_topawn, y_topawn);
                }

                has_pawn = GOTO_RELEASE;
                break;

            case STACK:
                Serial.println("STACK");
                PAWN_release_pawn();
                delay(100);
                PAWN_go_down();
                PAWN_grip_pawn();
                delay(100);

                sens = move_pawn_to_xy(&maximus, &x_topawn, &y_topawn);
                if (sens == 0) {                           // Front
                    goto_xy(x_topawn, y_topawn);
                } else {                                   // Back
                    goto_xy_back(x_topawn, y_topawn);
                }

                PAWN_go_up();

                has_pawn = GOTO_RELEASE;

                break;



            case AVOIDING0:
                Serial.println("AVOIDING0");
                avoid_radius = distance_coord(&maximus, release_point.x, release_point.y);
                my_angle = angle_coord(&maximus, release_point.x, release_point.y);

                if (my_angle < 0) {                        // Avoid by left
                    set_new_command(&bot_command_alpha, (my_angle + PI / 2) * RAD2DEG);
                } else {                                   // Avoid by right
                    set_new_command(&bot_command_alpha, (my_angle - PI / 2) * RAD2DEG);
                }

                has_pawn = AVOIDING1;
                break;
            case AVOIDING1:
                Serial.println("AVOIDING1");
                avoid_object(&maximus, &release_point, avoid_radius + 50);

                has_pawn = AVOIDING2;
                break;
            case AVOIDING2:
                Serial.println("AVOIDING2");
                delta_motor.max_speed = DELTA_MAX_SPEED;
                alpha_motor.max_speed = ALPHA_MAX_SPEED;

                goto_xy(my_test_point.x, my_test_point.y);
                has_pawn = prev_has_pawn;
                break;

            case AVOIDING_OPP1:
                Serial.println("AVOIDING_OPP1");
                delta_motor.max_speed = DELTA_MAX_SPEED;
                alpha_motor.max_speed = ALPHA_MAX_SPEED;

                goto_xy(my_test_point.x, my_test_point.y);
                has_pawn = AVOIDING_OPP2;
                break;

            case AVOIDING_OPP2:
                Serial.println("AVOIDING_OPP2");
                delta_motor.max_speed = DELTA_MAX_SPEED;
                alpha_motor.max_speed = ALPHA_MAX_SPEED;

                goto_xy(my_test_point.x, my_test_point.y);
                has_pawn = prev_has_pawn;
                break;



            }

        }
















        break;

    case PLACING_PAWN:























        break;
    }





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
    alpha_motor.kP = 230;                                  //250//350                                  // 600
    alpha_motor.kI = 0;
    alpha_motor.kD = 340;                                  //300 //180                                  // 200
    alpha_motor.accel = ALPHA_MAX_ACCEL;                   //300;                               //350//200;                               // 300
    alpha_motor.decel = ALPHA_MAX_DECEL;                   //1300;                              //1200;//1100;//1200;                              // 500
    alpha_motor.max_speed = ALPHA_MAX_SPEED;               //7000;                          //8000
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
    delta_motor.accel = DELTA_MAX_ACCEL;                   //600;                               //400;//500;
    delta_motor.decel = DELTA_MAX_DECEL;                   //1800;                              //1350;//1100;//1200;
    delta_motor.max_speed = DELTA_MAX_SPEED;               //25000;//35000;
    delta_motor.distance = 0.0;
}

        /****************************/
        /* INITIALIZATION FUNCTIONS */
        /****************************/
void init_blue_Robot(struct robot *my_robot)
{
    my_robot->pos_X = color * (1500 - DISTANCE_REAR_WHEELS);    //-1459;                               // -700         
    my_robot->pos_Y = 192;                                 // 700          
    my_robot->theta = 0;                                   // PI/2

    my_color_points = blue_points;
}

void init_red_Robot(struct robot *my_robot)
{
    my_robot->pos_X = color * (1500 - DISTANCE_REAR_WHEELS);    //1459;                                // -700         
    my_robot->pos_Y = 192;                                 // 700          
    my_robot->theta = PI;                                  // PI/2

    my_color_points = red_points;
}

void init_first_position(struct robot *my_robot)
{
    // Put the robot in low speed mode
    delta_motor.max_speed = 12000;
    alpha_motor.max_speed = 5000;
    // go back to touch the wall
    set_new_command(&bot_command_delta, -1000);

    while ((digitalRead(LEFT_REAR_SENSOR) == 0) || (digitalRead(RIGHT_REAR_SENSOR) == 0)) {
        delay(100);

    }
    delay(300);
    // Set the Y position and theta
    my_robot->theta = PI / 2;
    my_robot->pos_Y = DISTANCE_REAR_WHEELS;
    my_robot->pos_X = 0;

    delay(100);
    // Stop the motors
    set_new_command(&bot_command_alpha, 0);
    set_new_command(&bot_command_delta, 0);
    // Go forward, turn, and go bachward to touch the other wall
    set_new_command(&bot_command_delta, 150);
    delay(2000);
    set_new_command(&bot_command_alpha, (color * PI / 2 * RAD2DEG));
    delay(4000);
    set_new_command(&bot_command_delta, -1000);

    while ((digitalRead(LEFT_REAR_SENSOR) == 0) || (digitalRead(RIGHT_REAR_SENSOR) == 0)) {
        delay(100);

    }
    delay(300);
    // Set the X and theta values
    my_robot->pos_X = color * (1500 - DISTANCE_REAR_WHEELS);
    if (color == 1) {
        my_robot->theta = PI;
    } else {
        my_robot->theta = 0;
    }

    delay(100);

    // Stop the motors
    set_new_command(&bot_command_alpha, 0);
    set_new_command(&bot_command_delta, 0);

    delay(1000);
    // Go in the middle of the starting area
    set_new_command(&bot_command_delta, 80);

    gripServo_right.write(50);
    gripServo_left.write(130);

    delay(2000);
    // Set the speed to the maximum
    delta_motor.max_speed = DELTA_MAX_SPEED;
    alpha_motor.max_speed = ALPHA_MAX_SPEED;

}

void init_color_points(void)
{
    /* Red points */
    red_points[0].x = 515;
    red_points[1].x = -175;
    red_points[2].x = -175 - 350 - 350;
    red_points[0].y = 165;
    red_points[1].y = 170;
    red_points[2].y = 170;

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

    red_points[15].x = 875;
    red_points[16].x = 175;
    red_points[17].x = 175 - 350 - 350;
    red_points[15].y = 1865;                               //175 + 350 + 350 + 350 + 350 + (350 / 2 + (350 - 120) / 2);
    red_points[16].y = 175 + 350 + 350 + 350 + 350 + 350;
    red_points[17].y = 175 + 350 + 350 + 350 + 350 + (350 / 2 + (350 - 120) / 2);

    /* Blue points */
    blue_points[2].x = 175 + 350 + 350;
    blue_points[1].x = 175;
    blue_points[0].x = -515;
    blue_points[0].y = 165;
    blue_points[1].y = 170;
    blue_points[2].y = 170;

    blue_points[5].x = -175 + 350 + 350;
    blue_points[4].x = -175;
    blue_points[3].x = -175 - 350 - 350;
    blue_points[3].y = 175 + 350;
    blue_points[4].y = 175 + 350;
    blue_points[5].y = 175 + 350;

    blue_points[8].x = 175 + 350 + 350;
    blue_points[7].x = 175;
    blue_points[6].x = 175 - 350 - 350;
    blue_points[6].y = 175 + 350 + 350;
    blue_points[7].y = 175 + 350 + 350;
    blue_points[8].y = 175 + 350 + 350;

    blue_points[11].x = -175 + 350 + 350;
    blue_points[10].x = -175;
    blue_points[9].x = -175 - 350 - 350;
    blue_points[9].y = 175 + 350 + 350 + 350;
    blue_points[10].y = 175 + 350 + 350 + 350;
    blue_points[11].y = 175 + 350 + 350 + 350;

    blue_points[14].x = 175 + 350 + 350;
    blue_points[13].x = 175;
    blue_points[12].x = 175 - 350 - 350;
    blue_points[12].y = 175 + 350 + 350 + 350 + 350;
    blue_points[13].y = 175 + 350 + 350 + 350 + 350;
    blue_points[14].y = 175 + 350 + 350 + 350 + 350;

    blue_points[17].x = -175 + 350 + 350;
    blue_points[16].x = -175;
    blue_points[15].x = -875;
    blue_points[15].y = 1865;                              //175 + 350 + 350 + 350 + 350 + (350 / 2 + (350 - 120) / 2); // 1865
    blue_points[16].y = 175 + 350 + 350 + 350 + 350 + 350;
    blue_points[17].y = 175 + 350 + 350 + 350 + 350 + (350 / 2 + (350 - 120) / 2);

    /* Green points */
    green_points[0].x = (color) * 1160;
    green_points[1].x = (color) * 1160;
    green_points[2].x = (color) * 1160;
    green_points[0].y = 690;
    green_points[1].y = 970;
    green_points[2].y = 1250;

    green_points[3].x = (color) * 1160;
    green_points[4].x = (color) * 1160;
    green_points[5].x = (-color) * 1160;
    green_points[3].y = 1530;
    green_points[4].y = 1810;
    green_points[5].y = 690;

    green_points[6].x = (-color) * 1160;
    green_points[7].x = (-color) * 1160;
    green_points[8].x = (-color) * 1160;
    green_points[6].y = 970;
    green_points[7].y = 1250;
    green_points[8].y = 1530;

    green_points[9].x = (-color) * 1170;
    green_points[9].y = 1810;

    release_priorities[0] = 2;
    release_priorities[1] = 2;
    release_priorities[2] = 2;

    release_priorities[3] = 5;
    release_priorities[4] = 3;
    release_priorities[5] = 1;

    release_priorities[6] = 3;
    release_priorities[7] = 3;
    release_priorities[8] = 5;

    release_priorities[9] = 3;
    release_priorities[10] = 3;
    release_priorities[11] = 1;

    release_priorities[12] = 3;
    release_priorities[13] = 3;
    release_priorities[14] = 3;

    release_priorities[15] = 3;
    release_priorities[16] = 2;
    release_priorities[17] = 3;

    subzones[0].x = (-color) * 700;
    subzones[0].y = 350 + (350 / 2);
    subzones[1].x = (-color) * 700;
    subzones[1].y = 1400 + (350 / 2);

    subzones[2].x = 0;
    subzones[2].y = 350 + (350 / 2);
    subzones[3].x = 0;
    subzones[3].y = 1400 + (350 / 2);

    subzones[4].x = (color) * 700;
    subzones[4].y = 350 + (350 / 2);
    subzones[5].x = (color) * 700;
    subzones[5].y = 1400 + (350 / 2);

    avoid_points[0].x = 350;
    avoid_points[0].y = 350;
    avoid_points[1].x = 350;
    avoid_points[1].y = 1400;
}

void init_way_points(void)
{
#ifdef NO_SENSORS

    way_points[0].x = color * 1050;
    way_points[0].y = 200;
    way_points[1].x = color * 700;
    way_points[1].y = 350;
    way_points[2].x = color * 700;
    way_points[2].y = 1550;

    way_points[3].x = color * 350;
    way_points[3].y = 1400;                                //1400;
    way_points[4].x = color * 350;
    way_points[4].y = 500;
    way_points[5].x = 0;
    way_points[5].y = 1050;

    way_points[6].x = (-1) * color * 350;
    way_points[6].y = 350;
    way_points[7].x = (-1) * color * 350;
    way_points[7].y = 1600;
    way_points[8].x = (-1) * color * 700;
    way_points[8].y = 1600;
    way_points[9].x = (-1) * color * 700;
    way_points[9].y = 350;

    way_points[10].x = 0;
    way_points[10].y = 350;
    way_points[11].x = color * 525;
    way_points[11].y = 525;
    way_points[12].x = color * 525;
    way_points[12].y = 1225;

    way_points[13].x = (-1) * color * 750;
    way_points[13].y = 350;

/*
    way_points[0].x = color * 1050;
    way_points[0].y = 200;
    way_points[1].x = color * 525;
    way_points[1].y = 525;
    way_points[2].x = color * 875;
    way_points[2].y = 1575;

    way_points[3].x = color * 175;
    way_points[3].y = 1575;                                //1400;
    way_points[4].x = color * 525;
    way_points[4].y = 525;
    way_points[5].x = 0;
    way_points[5].y = 1050;
*/

#else
    way_points[0].x = color * 1050;
    way_points[0].y = 200;
    way_points[1].x = color * 800;
    way_points[1].y = 450;
    way_points[2].x = color * 780;
    way_points[2].y = 1500;

    way_points[3].x = color * 350;
    way_points[3].y = 1600;                                //1400;
    way_points[4].x = color * 350;
    way_points[4].y = 700;
    way_points[5].x = 0;
    way_points[5].y = 700;

    way_points[6].x = (-1) * color * 350;
    way_points[6].y = 350;
    way_points[7].x = (-1) * color * 350;
    way_points[7].y = 1600;
    way_points[8].x = (-1) * color * 750;
    way_points[8].y = 1600;
    way_points[9].x = (-1) * color * 750;
    way_points[9].y = 350;

    way_points[10].x = 0;
    way_points[10].y = 350;
    way_points[11].x = color * 525;
    way_points[11].y = 525;
    way_points[12].x = color * 525;
    way_points[12].y = 1225;

    way_points[13].x = (-1) * color * 750;
    way_points[13].y = 350;
#endif


/*
    way_points[0].x = color * 1050;
    way_points[0].y = 200;
    way_points[1].x = color * 700;
    way_points[1].y = 450;
    way_points[2].x = color * 700;
    way_points[2].y = 1500;

    way_points[3].x = color * 350;
    way_points[3].y = 1600;                                //1400;
    way_points[4].x = color * 350;
    way_points[4].y = 700;
    way_points[5].x = 0;
    way_points[5].y = 700;

    way_points[6].x = (-1) * color * 350;
    way_points[6].y = 350;
    way_points[7].x = (-1) * color * 350;
    way_points[7].y = 1600;
    way_points[8].x = (-1) * color * 700;
    way_points[8].y = 1600;
    way_points[9].x = (-1) * color * 700;
    way_points[9].y = 350;
    
    way_points[10].x = 0;
    way_points[10].y = 350;
    way_points[11].x = color * 525;
    way_points[11].y = 525;
    way_points[12].x = color * 525;
    way_points[12].y = 1225;
    
    way_points[13].x = (-1) * color * 750;
    way_points[13].y = 350;
*/

}

void init_PAWN_status(void)
{
    pinMode(LIFT_MOTOR_SENS, OUTPUT);

    pinMode(LIFT_SWITCH_UP, INPUT);
    pinMode(LIFT_SWITCH_DOWN, INPUT);

    gripServo_right.attach(RIGHT_SERVO);
    gripServo_left.attach(LEFT_SERVO);

    lifter_servo.attach(LIFT_MOTOR_PWM);
    lifter_servo.write(90);

    //PAWN_go_down();
    PAWN_go_up();
    PAWN_grip_pawn();

}

void init_placed_pawns_for_test(void)
{
    placed_pawn[0].x = -525;
    placed_pawn[0].y = 875;

    placed_pawn[1].x = -525;
    placed_pawn[1].y = 1575;

    placed_pawn_index = 2;
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
    //Serial.println("Voltage from RoboClaw");

    Serial2.print(addr, BYTE);
    Serial2.print(24, BYTE);

    transmit_status = 1;
}

void check_RoboClaw_response(char addr)
{
    transmit_status = 0;
    if (Serial2.available() >= 3) {
        //Serial.println("RoboClaw responding");
        Serial2.read();
        Serial2.read();
        Serial2.read();
    } else {
        digitalWrite(RESET_ROBOCLAW, LOW);
        Serial.println("RoboClaw not responding anymore");
        delay(5);
        digitalWrite(RESET_ROBOCLAW, HIGH);
        delay(500);
    }
    transmit_status = 1;

}



/*******************************/
/* MOTION CONTROL FUNCTIONS */
/*******************************/
void do_motion_control(void)
{

    // PID angle
    alpha_motor.des_speed = compute_position_PID(&bot_command_alpha, &alpha_motor);

    // PID distance
    if ((bot_command_alpha.state == WAITING_BEGIN) || (bot_command_alpha.state == PROCESSING_COMMAND)) {        // If alpha motor have not finished its movement 

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

    if ((cmd->state == PROCESSING_COMMAND) && (abs(err) < 3)
        && (abs(errDif) < 3)) {                            // 2 before
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

    Serial.print(".Go to :");
    Serial.print((int) x);
    Serial.print(" ");
    Serial.println((int) y);

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

void avoid_object(struct robot *my_robot, struct Point *center, double radius)
{
    double ratio = 0;
    double max1 = 40000;
    double max2 = 0;
    double diff = 0;

    Serial.println(radius);
    // Compute max possible speed
    ratio = (radius + (166 / 2)) / (radius - (166 / 2));
    //Serial.println(ratio);
    max2 = max1 / ratio;
    //Serial.println(max2);
    diff = fabs(max1 - max2) / 2;
    //Serial.println(diff);
    delta_motor.max_speed = (max1 - diff);
    alpha_motor.max_speed = diff;
    //Serial.println(delta_motor.max_speed);
    //Serial.println(alpha_motor.max_speed);
    // Set the angle and distance to do 
    if (angle_coord(my_robot, center->x, center->y) < 0) {
        set_new_command(&bot_command_alpha, -100);
    } else {
        set_new_command(&bot_command_alpha, 100);
    }

    set_new_command(&bot_command_delta, 1256);
    set_new_command(&prev_bot_command_delta, 0);
    bot_command_delta.state = COMMAND_DONE;

}

struct Point estimate_center(struct robot *my_robot)
{
#define DETECTION_LIMIT 35
#define DETECTION_LIMIT_MIDDLE 47

    struct Point result;

    //read_down_IR();


    if ((front_distance_up_right < DETECTION_LIMIT) && (opponent_sensor > DETECTION_LIMIT_MIDDLE)
        && (front_distance_up_left > DETECTION_LIMIT)) {

        result.x = my_robot->pos_X + 170.0 * sin(my_robot->theta) + 10.0 * (front_distance_up_right + 15) * cos(my_robot->theta);
        result.y = my_robot->pos_Y - 170.0 * cos(my_robot->theta) + 10.0 * (front_distance_up_right + 15) * sin(my_robot->theta);

    } else if ((front_distance_up_right < DETECTION_LIMIT) && (opponent_sensor < DETECTION_LIMIT_MIDDLE)
               && (front_distance_up_left > DETECTION_LIMIT)) {

        result.x = my_robot->pos_X + 90.0 * sin(my_robot->theta) + 10.0 * (opponent_sensor) * cos(my_robot->theta);
        result.y = my_robot->pos_Y - 90.0 * cos(my_robot->theta) + 10.0 * (opponent_sensor) * sin(my_robot->theta);

    } else if ((front_distance_up_right > DETECTION_LIMIT) && (opponent_sensor < DETECTION_LIMIT_MIDDLE)
               && (front_distance_up_left > DETECTION_LIMIT)) {

        result.x = my_robot->pos_X + 10.0 * (opponent_sensor + 5) * cos(my_robot->theta);
        result.y = my_robot->pos_Y + 10.0 * (opponent_sensor + 5) * sin(my_robot->theta);

    } else if ((front_distance_up_right > DETECTION_LIMIT) && (opponent_sensor < DETECTION_LIMIT_MIDDLE)
               && (front_distance_up_left < DETECTION_LIMIT)) {

        result.x = my_robot->pos_X - 90.0 * sin(my_robot->theta) + 10.0 * (opponent_sensor) * cos(my_robot->theta);
        result.y = my_robot->pos_Y + 90.0 * cos(my_robot->theta) + 10.0 * (opponent_sensor) * sin(my_robot->theta);

    } else if ((front_distance_up_right > DETECTION_LIMIT) && (opponent_sensor > DETECTION_LIMIT_MIDDLE)
               && (front_distance_up_left < DETECTION_LIMIT)) {

        result.x = my_robot->pos_X - 170.0 * sin(my_robot->theta) + 10.0 * (front_distance_up_left + 15) * cos(my_robot->theta);
        result.y = my_robot->pos_Y + 170.0 * cos(my_robot->theta) + 10.0 * (front_distance_up_left + 15) * sin(my_robot->theta);

    } else {

        result.x = my_robot->pos_X + 10 * (opponent_sensor + 5) * cos(my_robot->theta);
        result.y = my_robot->pos_Y + 10 * (opponent_sensor + 5) * sin(my_robot->theta);

    }

    return result;
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

void read_down_IR(void)
{
    int sensorValue = 0;

    sensorValue = analogRead(PAWN_SENSOR_MIDDLE);
    //front_distance_down_middle = (front_distance_down_middle + (convert_longIR_value(sensorValue)) * 2) / 3;
    front_distance_down_middle = (front_distance_down_middle * 2 + (convert_medIR_value(sensorValue)) * 1) / 3;
    if (front_distance_down_middle > 80 || front_distance_down_middle < 0)
        front_distance_down_middle = 80;

    sensorValue = analogRead(PAWN_SENSOR_LEFT);
    //front_distance_down_left = (front_distance_down_left + (convert_longIR_value(sensorValue)) * 2) / 3;
    front_distance_down_left = (front_distance_down_left * 2 + (convert_medIR_value(sensorValue)) * 1) / 3;
    if (front_distance_down_left < 0)
        front_distance_down_left = 80;
    if (front_distance_down_left > 80)
        front_distance_down_left = 80;
    sensorValue = analogRead(PAWN_SENSOR_RIGHT);
    //front_distance_down_right = (front_distance_down_right + (convert_longIR_value(sensorValue)) * 2) / 3;
    front_distance_down_right = (front_distance_down_right * 2 + (convert_medIR_value(sensorValue)) * 1) / 3;
    if (front_distance_down_right < 0)
        front_distance_down_right = 80;
    if (front_distance_down_right > 80)
        front_distance_down_right = 80;
}

void sense_opponent_ir(void)
{
    int sensorValue = 0;

    sensorValue = analogRead(OPPONENT_SENSOR_LEFT);
    front_distance_up_left = (front_distance_up_left * 2 + (convert_longIR_value(sensorValue)) * 1) / 3;
    if (front_distance_up_left > 150 || front_distance_up_left <= 0)
        front_distance_up_left = 150;

    sensorValue = analogRead(OPPONENT_SENSOR_RIGHT);
    front_distance_up_right = (front_distance_up_right * 2 + (convert_longIR_value(sensorValue)) * 1) / 3;
    if (front_distance_up_right > 150 || front_distance_up_right <= 0)
        front_distance_up_right = 150;
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
    set_new_command(&prev_bot_command_delta, delta_motor.des_speed / 800);      // depends on current speed
    set_new_command(&bot_command_delta, delta_motor.des_speed / 800);   // depends on current speed
    //right_motor.des_speed = 0;
    //left_motor.des_speed = 0;
    //write_RoboClaw_speed_M1M2(128, 0, 0);
}

void direct_stop_robot(void)
{
    //set_new_command(&bot_command_alpha, 0);
    set_new_command(&prev_bot_command_delta, delta_motor.des_speed / 2000);     // depends on current speed
    set_new_command(&bot_command_delta, delta_motor.des_speed / 2000);  // depends on current speed
    //right_motor.des_speed = 0;
    //left_motor.des_speed = 0;
    //write_RoboClaw_speed_M1M2(128, 0, 0);
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
    return (int) ((6787.0 / ((double) value - 3.0)) - 4.0);
}

int convert_shortIR_value(int value)
{
    return (2914 / (value + 5)) - 1;
}

/* return 1 if true, else 0 */
int is_in_our_side(struct robot *my_robot)
{
    int result = 0;                                        // No
    if ((my_robot->pos_X * color) > 0)
        result = 1;

    return result;
}

struct Point compute_pawn_position(struct robot *my_robot, int distance)
{
    struct Point result;

    result.x = my_robot->pos_X + distance * cos(my_robot->theta);
    result.y = my_robot->pos_Y + distance * sin(my_robot->theta);

    return result;
}

int check_point_in_map(struct Point *my_point)
{
#define BORDER_LIMIT 100
    int error = 0;                                         // Outside the map
    if ((my_point->x > (1500 - BORDER_LIMIT - 300)) || (my_point->x < (-1500 + BORDER_LIMIT + 300)) || (my_point->y > (2100 - BORDER_LIMIT))
        || (my_point->y < BORDER_LIMIT)
        || ((my_point->y > 1800) && (((my_point->x > 350) && (my_point->x < 1450)) || ((my_point->x < -350) && (my_point->x > -1450))))
        || ((my_point->y < 450) && ((my_point->x > 950) || (my_point->x < -950)))
        || ((my_point->y > 1800) && (my_point->x > (color * 50))))
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
    double result_t0 = 0;
    double result_t1 = 0;
    double x1 = (double) start->x / 10.0;
    double y1 = (double) start->y / 10.0;
    double x2 = (double) end->x / 10.0;
    double y2 = (double) end->y / 10.0;
    double x3 = (double) center->x / 10.0;
    double y3 = (double) center->y / 10.0;
/*      
       Serial.print("s_x : ");
       Serial.print(x1);
       Serial.print(" s_y : ");
       Serial.print(y1);
       Serial.print(" e_x : ");
       Serial.print(x2);
       Serial.print(" e_y : ");
       Serial.print(y2);
       Serial.print(" c_x : ");
       Serial.print(x3);
       Serial.print(" c_y : ");
       Serial.print(y3);
       Serial.print(" radius : ");
       Serial.println(radius);
*/
/*
		double a = (end->x-start->x)*(end->x-start->x) + (end->y-start->y)*(end->y-start->y);
		double b = 2 * ( (end->x-start->x)*(start->x-center->x) + (end->y-start->y)*(start->y-center->y) );
		double c = (start->x-center->x)*(start->x-center->x) + (start->y-center->y)*(start->y-center->y) - radius*radius;
*/
    double a = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
    double b = 2.0 * ((x2 - x1) * (x1 - x3) + (y2 - y1) * (y1 - y3));
    double c = (x1 - x3) * (x1 - x3) + (y1 - y3) * (y1 - y3) - ((double) radius / 10.0) * ((double) radius / 10.0);
/*
       Serial.print(a);
       Serial.print(" ");
       Serial.print(b);
       Serial.print(" ");
       Serial.println(c);
*/
    //result_t0 = c;
    //result_t1 = a + b + c;

    delta = b * b - 4 * a * c;
    /*
       delta = pow(2*( (end->x-start->x)*(start->x-center->x) + (end->y-start->y)*(start->y-center->y) ),2) - 4*(pow(end->x-start->x, 2) + pow(end->y-start->y, 2))*(pow((start->x-center->x), 2)+pow((start->y-center->y),2)-pow(radius,2));
     */
    if (delta >= 0) {
        result_t0 = ((-1) * b + sqrt(delta)) / (2 * a);
        result_t1 = ((-1) * b - sqrt(delta)) / (2 * a);
        if ((result_t1 >= 0 && result_t1 <= 1) || (result_t0 >= 0 && result_t0 <= 1))
            result = 1;
    }
    //Serial.println(delta);
    return result;
}


        /******************/
        /* A.I. Functions */
        /******************/
struct Point find_nearest(struct robot *my_robot, struct Point tab[], int size)
{
    struct Point result;
    result.x = color * 875;
    result.y = 525;
    double best_distance = 999999.9;
    nearest_index = 8;
    for (int i = 0; i < size; i++) {
        if (color == 1) {
            if (my_robot->pos_X < tab[i].x) {
                double distance = distance_coord(my_robot, tab[i].x, tab[i].y) * release_priorities[i];
                if (distance < best_distance) {
                    best_distance = distance;
                    result.x = tab[i].x;
                    result.y = tab[i].y;
                    nearest_index = i;
                }
            }
        } else {
            if (my_robot->pos_X > tab[i].x) {
                double distance = distance_coord(my_robot, tab[i].x, tab[i].y) * release_priorities[i];
                if (distance < best_distance) {
                    best_distance = distance;
                    result.x = tab[i].x;
                    result.y = tab[i].y;
                    nearest_index = i;
                }
            }
        }
    }
    return result;
}

void goto_avoiding_placed_point(struct robot *my_robot, struct Point tab[], int size, struct Point *dest)
{
    struct Point result, tmp;
    int result2 = 0;
    result.x = color * 875;
    result.y = 875;
    tmp.x = my_robot->pos_X;
    tmp.y = my_robot->pos_Y;
    double best_distance = 99999.9;
    for (int i = 0; i < size; i++) {
        if (trajectory_intersection_pawn(&tmp, dest, &tab[i], 270) == 1) {
            double distance = distance_coord(my_robot, tab[i].x, tab[i].y);
            if (distance < best_distance) {
                best_distance = distance;
                result.x = tab[i].x;
                result.y = tab[i].y;
                result2 = 1;
            }
        }

    }

    if (result2 == 1) {
        double ang, dist;
        Serial.println("Must avoid pawn");
        release_point.x = result.x;
        release_point.y = result.y;

        ang = angle_coord(my_robot, dest->x, dest->y) * RAD2DEG;
        set_new_command(&bot_command_alpha, ang);

        dist = distance_coord(&maximus, result.x, result.y) - 280;      // TO REDUCE LATER
        set_new_command(&prev_bot_command_delta, dist);
        bot_command_delta.state = WAITING_BEGIN;
        //goto_xy(way_points[way_point_index - 1].x, way_points[way_point_index - 1].y);
        prev_has_pawn = has_pawn;
        has_pawn = AVOIDING0;
    } else {
        goto_xy(dest->x, dest->y);
        //has_pawn = ;
    }

}

        /* Returns 0 if not in our color, 1 else */
int find_pawn_in_our_color(struct Point *pawn, struct Point tab[], int size)
{
#define COLOR_LIMIT 65
    //int result = 0;
    //double best_distance = 99999.9;
    for (int i = 0; i < size; i++) {
        if ((pawn->x < (tab[i].x + COLOR_LIMIT)) && (pawn->x > (tab[i].x - COLOR_LIMIT)) && (pawn->y < (tab[i].y + COLOR_LIMIT))
            && (pawn->y > (tab[i].y - COLOR_LIMIT))) {
            return 1;
        }
    }
    return 0;
}

void reinit_y_axis(struct robot *my_robot)
{
    // Put the robot in low speed mode
    delta_motor.max_speed = 12000;
    alpha_motor.max_speed = 5000;
    // go back to touch the wall
    set_new_command(&bot_command_delta, -1000);

    while ((digitalRead(LEFT_REAR_SENSOR) == 0) || (digitalRead(RIGHT_REAR_SENSOR) == 0)) {
        delay(100);

    }
    delay(300);
    // Set the Y position and theta
    my_robot->theta = PI / 2;
    my_robot->pos_Y = DISTANCE_REAR_WHEELS;

    delay(100);
    // Stop the motors
    set_new_command(&bot_command_alpha, 0);
    set_new_command(&bot_command_delta, 0);

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
    //gripServo_right.write(55);
    //gripServo_left.write(135);
    gripServo_right.write(65);
    gripServo_left.write(115);
}

void PAWN_release_pawn(void)
{
    //gripServo_right.write(91);
    //gripServo_left.write(100);
    gripServo_right.write(111);
    gripServo_left.write(70);
}

void PAWN_release_for_greenzone(void)
{
    //gripServo_right.write(91);
    //gripServo_left.write(100);
    gripServo_right.write(96);
    gripServo_left.write(85);
}

void PAWN_go_up(void)
{
    int buttonState = 0;
    // Start going up
    //digitalWrite(LIFT_MOTOR_SENS, LIFT_GO_UP);
    //analogWrite(LIFT_MOTOR_PWM, 100);
    lifter_servo.write(125);
    // Stop when the microswitch in activated
    buttonState = digitalRead(LIFT_SWITCH_UP);
    while (buttonState == 1) {
        delay_ms(5);
        buttonState = digitalRead(LIFT_SWITCH_UP);
    }
    //delay(50);
    // Stop
    lifter_servo.write(90);
    //analogWrite(LIFT_MOTOR_PWM, 0);

}

void PAWN_mini_go_up(void)
{
    lifter_servo.write(117);
    delay_ms(220);
    lifter_servo.write(90);
    //analogWrite(LIFT_MOTOR_PWM, 0);

}


void PAWN_go_down(void)
{
    int buttonState = 0;
    int error = 0;
    // Start going up
    //digitalWrite(LIFT_MOTOR_SENS, LIFT_GO_DOWN);
    //analogWrite(LIFT_MOTOR_PWM, 100);

    if (digitalRead(LIFT_SWITCH_UP) == 0) {
        lifter_servo.write(47);
        delay(200);
    } else {
        lifter_servo.write(47);
        delay(20);
    }
    // Stop when the microswitch in activated
    buttonState = digitalRead(LIFT_SWITCH_DOWN);
    while (buttonState == 1) {
        delay_ms(5);
        buttonState = digitalRead(LIFT_SWITCH_UP);
        if ((buttonState == 0) && (error == 0)) {
            set_new_command(&bot_command_delta, -25);
            lifter_servo.write(117);
            delay(300);
            error = 1;
        } else if ((buttonState == 0) && (error == 1)) {
            set_new_command(&bot_command_delta, -20);
            lifter_servo.write(75);
            delay(300);
            error = 0;
        }

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
    if (BEACON_NORTH_PIN) {
        beacon_direction = BEACON_NORTH;
    } else if (BEACON_SOUTH_PIN) {
        beacon_direction = BEACON_SOUTH;
    } else if (BEACON_EAST_PIN) {
        beacon_direction = BEACON_EAST;
    } else if (BEACON_WEST_PIN) {
        beacon_direction = BEACON_WEST;
    }
}


/**************************************/
/* COMMUNICATION WITH BARCODE SCANNER */
/**************************************/
void barCode_scan(void)
{
    Serial1.print(0x1B, BYTE);
    Serial1.print("Z");
    Serial1.print(0x0D, BYTE);
}

/* Returns 0 if nothing, 1 if king, 2 if queen */
int barCode_checkifdata(void)
{
    int result = 0;
    if (Serial1.available()) {
        if (Serial1.read() == 'Q')
            result = 2;
        else
            result = 1;
    }
    while (Serial1.available()) {

        Serial.print(Serial1.read());                      // DEBUG
        //Serial1.read();
    }
    return result;
}

void barCode_flush(void)
{
    while (Serial1.available()) {
        Serial1.read();
    }
}
