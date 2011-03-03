/*****************************************************
Project : Maximus
Version : 1.0
Date : 10/12/2010
Author : JBot
Company :
Comments:

Program type : Application
Clock frequency : 16,00 MHz
 *****************************************************/
// Arduino specific includes
#include <Servo.h>
#include <SoftwareSerial.h>

// Other includes
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <math.h>
#include <ctype.h>

#define bit9600Delay 84
#define halfBit9600Delay 42
#define bit4800Delay 188
#define halfBit4800Delay 94


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
#define TWOPI 			6.2831853070
#define RAD2DEG 		57.29577951                    /* radians to degrees conversion */

#define DISTANCE_CENTER_PAWN    240.0

#define NO_PAWN                 0
#define TAKE_PAWN               1
#define GOTO_RELEASE            2
#define GO_BACK                 3

// I/Os definition
#define INPUT_MOTION_PIN        2
#define RIGHT_SERVO             12
#define LEFT_SERVO              11
#define PAWN_SENSOR             9
#define RIGHT_IR_SENSOR         10
#define LEFT_IR_SENSOR          11


/***********************/
/* Specific structures */
/***********************/
struct robot {
    double pos_X;
    double pos_Y;
    double theta;
};

struct Point {
    int x;
    int y;
};




/********************/
/* Global variables */
/********************/
struct robot maximus;

unsigned int entier;
char display1, display2, display3, display4, display5, display6, display7;

Servo gripServo_right;
Servo gripServo_left;

int front_distance = 50;
int prev_front_distance = 50;

long global_time_counter = 0;
char has_pawn = NO_PAWN;

struct Point red_points[18];
struct Point blue_points[18];

struct Point *my_color_points;

struct Point way_points[20];
int way_point_index = 0;

char start_MOTION = 0;


/***********************/
/* INTERRUPT FUNCTIONS */
/***********************/
// Timer 1 overflow interrupt service routine
ISR(TIMER1_OVF_vect)
{
    sei();                                                 // enable interrupts

    // Get the robot's position
    if (Serial.available() >= 10) {
      double x = 0, y = 0, t = 0;

    Serial.read(); // x
    if(Serial.read() == '+') {
    x = (x * 10) + (Serial.read() - 48);
    x = (x * 10) + (Serial.read() - 48);
    x = (x * 10) + (Serial.read() - 48);
    x = (x * 10) + (Serial.read() - 48);
    x = (x * 10) + (Serial.read() - 48);
    x = (x * 10) + (Serial.read() - 48);
    x = x / (10);
    }
    else {
    x = (x * 10) + (Serial.read() - 48);
    x = (x * 10) + (Serial.read() - 48);
    x = (x * 10) + (Serial.read() - 48);
    x = (x * 10) + (Serial.read() - 48);
    x = (x * 10) + (Serial.read() - 48);
    x = (x * 10) + (Serial.read() - 48);
    x = x / (-10);
    }
    
    Serial.read(); // y
    if(Serial.read() == '+') {
    y = (y * 10) + (Serial.read() - 48);
    y = (y * 10) + (Serial.read() - 48);
    y = (y * 10) + (Serial.read() - 48);
    y = (y * 10) + (Serial.read() - 48);
    y = (y * 10) + (Serial.read() - 48);
    y = (y * 10) + (Serial.read() - 48);
    y = y / 10;
    }
    else {
    y = (y * 10) + (Serial.read() - 48);
    y = (y * 10) + (Serial.read() - 48);
    y = (y * 10) + (Serial.read() - 48);
    y = (y * 10) + (Serial.read() - 48);
    y = (y * 10) + (Serial.read() - 48);
    y = (y * 10) + (Serial.read() - 48);
    y = y / (-10);  
    }

    Serial.read(); // theta in RAD
    if(Serial.read() == '+') {
    t = (t * 10) + (Serial.read() - 48);
    t = (t * 10) + (Serial.read() - 48);
    t = (t * 10) + (Serial.read() - 48);
    t = (t * 10) + (Serial.read() - 48);
    t = (t * 10) + (Serial.read() - 48);
    t = (t * 10) + (Serial.read() - 48);
    t = t / 10000; 
    }
    else {
    t = (t * 10) + (Serial.read() - 48);
    t = (t * 10) + (Serial.read() - 48);
    t = (t * 10) + (Serial.read() - 48);
    t = (t * 10) + (Serial.read() - 48);
    t = (t * 10) + (Serial.read() - 48);
    t = (t * 10) + (Serial.read() - 48);
    t = t / (-10000);  
    }
      
      
    }


    // Compute sensors
    int sensorValue = analogRead(PAWN_SENSOR);
    front_distance = (front_distance + ((6787 / (sensorValue - 3)) - 4) * 2) / 3;



}


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
/*    PORTA = 0x00;
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
*/
    pinMode(13, OUTPUT);

    pinMode(INPUT_MOTION_PIN, INPUT);     


    // Initialize Timer interrupts
    // Timer/Counter 1 initialization
    // Clock source: System Clock
    // Clock value: 62,500 kHz
    // Mode: Ph. correct PWM top=03FFh
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
    TCCR1A = 0x03;                                         // Interrupt every 16.4ms
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
/*
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
  */
    // Timer(s)/Counter(s) Interrupt(s) initialization
    TIMSK1 |= 0x01;
    TIFR1 |= 0x01;



    //List of modules to connect :
    // - Motion control
    // - GUI
    // - Pawn module
    // - Power module
    // - Bluetooth debug  
    // Initialize all serial ports:
//    Serial.begin(115200);                                  // Bluetooth serial port
    Serial.begin(9600);                                  // Bluetooth serial port
    Serial1.begin(115200);                                 // Motion control serial port
    Serial2.begin(115200);                                 // Pawn module
    Serial3.begin(115200);                                 // Optionnal module

    // POWER MODULE SERIAL CONNEXION
    /*pinMode(22,INPUT); // RX
       pinMode(23,OUTPUT);// TX
       digitalWrite(23,HIGH);// TX to 1 */



    gripServo_right.attach(RIGHT_SERVO);
    gripServo_left.attach(LEFT_SERVO);
    
    release_pawn();



    /******************************/
    /* Initialization of the code */
    /******************************/
    init_blue_Robot(&maximus);                             // Init robot status

    init_color_points();                                   // Init the color field
    init_way_points();

    // Global enable interrupts
    sei();

    digitalWrite(13, HIGH);
    delay_ms(300);
    digitalWrite(13, LOW);
    delay_ms(300);
    digitalWrite(13, HIGH);
    delay_ms(300);


    Serial1.print('Z');
    Serial1.print(0);
    Serial1.print(0);
    Serial1.print(0);
    Serial1.print(0);
    Serial1.print(0);
    Serial1.print(0);

}

void loop()
{

    delay_ms(50);
    
    global_time_counter++;

    /*if ((front_distance < 8) && (has_pawn == NO_PAWN)) {
//        grip_pawn();
        has_pawn = TAKE_PAWN;
        MOTION_stop_robot();
        delay_ms(400);
    }*/

    entier = (unsigned int) abs(front_distance);
    display6 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display5 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display4 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display3 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display2 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display1 = (entier % 10) + 48;

    if (front_distance < 0)
        Serial.print('-');
    else
        Serial.print('+');
    Serial.print(display1);
    Serial.print(display2);
    Serial.print(display3);
    Serial.print(display4);
    Serial.print(display5);
    Serial.print(display6);

  Serial.print('\n');
  
  
     if ((global_time_counter > 90) && (digitalRead(INPUT_MOTION_PIN))) { // If all commands are done, go to the next step
        struct Point release_point;
        double x_topawn;
        double y_topawn;
        int sens;

        switch (has_pawn) {
        case TAKE_PAWN:                                   // The robot just take a pawn to put in his color
            release_point = find_nearest(&maximus, my_color_points, 18);
            x_topawn = release_point.x;
            y_topawn = release_point.y;

            sens = move_pawn_to_xy(&maximus, &x_topawn, &y_topawn);
            if (sens == 0) {                               // Front
                MOTION_goto_xy(x_topawn, y_topawn);
            } else {                                       // Back
                MOTION_goto_xy_back(x_topawn, y_topawn);
            }

            has_pawn = GOTO_RELEASE;
            break;

        case GOTO_RELEASE:                                // The robot have a pawn and is on his release point
            release_pawn();
            //set_new_command(&bot_command_delta, -180);
            has_pawn = GO_BACK;
            break;

        case GO_BACK:                                     // The robot release the pawn and go back to let it in position
            MOTION_goto_xy(way_points[way_point_index - 1].x, way_points[way_point_index - 1].y);
            has_pawn = NO_PAWN;
            break;

        default:                                          // has no pawn => Move to the next position
            if (way_point_index == 5)
                way_point_index = 1;
            MOTION_goto_xy(way_points[way_point_index].x, way_points[way_point_index].y);
            way_point_index++;
            break;
        }

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
}

void init_blue_Robot(struct robot *my_robot)
{
    my_robot->pos_X = -1459;                               // -700         
    my_robot->pos_Y = 192;                                 // 700          
    my_robot->theta = 0;                                   // PI/2

    my_color_points = blue_points;
}

void init_red_Robot(struct robot *my_robot)
{
    my_robot->pos_X = 1459;                                // -700         
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
    way_points[2].y = 1600;

    way_points[3].x = -350;
    way_points[3].y = 1400;
    way_points[4].x = -350;
    way_points[4].y = 350;
    way_points[5].x = -700;
    way_points[5].y = 350;

}


// Compute the distance to do to go to (x, y)
double distance_coord(struct robot *my_robot, double x1, double y1)
{
    double x = 0;
    x = sqrt(pow(fabs(x1 - my_robot->pos_X), 2) + pow(fabs(y1 - my_robot->pos_Y), 2));
    return x;
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


/*************************************/
/* COMMUNICATION WITH MOTION CONTROL */
/*           SERIAL 1                */
/*************************************/
void MOTION_goto_xy(double x, double y)
{
    Serial1.print('G');


    entier = (unsigned int) fabs(x * 10);
    display6 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display5 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display4 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display3 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display2 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display1 = (entier % 10) + 48;

    if (x < 0)
        Serial1.print('-');
    else
        Serial1.print('+');
    Serial1.print(display1);
    Serial1.print(display2);
    Serial1.print(display3);
    Serial1.print(display4);
    Serial1.print(display5);
    Serial1.print(display6);

    entier = (unsigned int) fabs(y * 10);
    display6 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display5 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display4 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display3 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display2 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display1 = (entier % 10) + 48;

    if (y < 0)
        Serial1.print('-');
    else
        Serial1.print('+');
    Serial1.print(display1);
    Serial1.print(display2);
    Serial1.print(display3);
    Serial1.print(display4);
    Serial1.print(display5);
    Serial1.print(display6);
}

void MOTION_goto_xy_back(double x, double y)
{
    Serial1.print('g');                                    


    entier = (unsigned int) fabs(x * 10);
    display6 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display5 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display4 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display3 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display2 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display1 = (entier % 10) + 48;

    if (x < 0)
        Serial1.print('-');
    else
        Serial1.print('+');
    Serial1.print(display1);
    Serial1.print(display2);
    Serial1.print(display3);
    Serial1.print(display4);
    Serial1.print(display5);
    Serial1.print(display6);

    entier = (unsigned int) fabs(y * 10);
    display6 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display5 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display4 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display3 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display2 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display1 = (entier % 10) + 48;

    if (y < 0)
        Serial1.print('-');
    else
        Serial1.print('+');
    Serial1.print(display1);
    Serial1.print(display2);
    Serial1.print(display3);
    Serial1.print(display4);
    Serial1.print(display5);
    Serial1.print(display6);
}

// Compute the coordinate to put a pawn to (x, y)
int MOTION_move_pawn_to_xy(double x, double y)
{
    Serial1.print('p');                                    


    entier = (unsigned int) fabs(x * 10);
    display6 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display5 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display4 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display3 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display2 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display1 = (entier % 10) + 48;

    if (x < 0)
        Serial1.print('-');
    else
        Serial1.print('+');
    Serial1.print(display1);
    Serial1.print(display2);
    Serial1.print(display3);
    Serial1.print(display4);
    Serial1.print(display5);
    Serial1.print(display6);

    entier = (unsigned int) fabs(y * 10);
    display6 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display5 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display4 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display3 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display2 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display1 = (entier % 10) + 48;

    if (y < 0)
        Serial1.print('-');
    else
        Serial1.print('+');
    Serial1.print(display1);
    Serial1.print(display2);
    Serial1.print(display3);
    Serial1.print(display4);
    Serial1.print(display5);
    Serial1.print(display6);
}

void MOTION_set_X(double x)
{
    Serial1.print('x');

    entier = (unsigned int) fabs(x * 10);
    display6 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display5 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display4 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display3 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display2 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display1 = (entier % 10) + 48;

    if (x < 0)
        Serial1.print('-');
    else
        Serial1.print('+');
    Serial1.print(display1);
    Serial1.print(display2);
    Serial1.print(display3);
    Serial1.print(display4);
    Serial1.print(display5);
    Serial1.print(display6);
}

void MOTION_set_Y(double y)
{
    Serial1.print('y');

    entier = (unsigned int) fabs(y * 10);
    display6 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display5 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display4 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display3 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display2 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display1 = (entier % 10) + 48;

    if (y < 0)
        Serial1.print('-');
    else
        Serial1.print('+');
    Serial1.print(display1);
    Serial1.print(display2);
    Serial1.print(display3);
    Serial1.print(display4);
    Serial1.print(display5);
    Serial1.print(display6);
}

void MOTION_set_theta(double t)
{
    Serial1.print('t');

    entier = (unsigned int) fabs(t * 1000);
    display6 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display5 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display4 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display3 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display2 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display1 = (entier % 10) + 48;

    if (t < 0)
        Serial1.print('-');
    else
        Serial1.print('+');
    Serial1.print(display1);
    Serial1.print(display2);
    Serial1.print(display3);
    Serial1.print(display4);
    Serial1.print(display5);
    Serial1.print(display6);
}


void MOTION_stop_position_motion_control(void)
{
    Serial1.print('S');
    Serial1.print(0);
    Serial1.print(0);
    Serial1.print(0);
    Serial1.print(0);
    Serial1.print(0);
    Serial1.print(0);
}


void MOTION_start_position_motion_control(void)
{
    Serial1.print('s');
    Serial1.print(0);
    Serial1.print(0);
    Serial1.print(0);
    Serial1.print(0);
    Serial1.print(0);
    Serial1.print(0);
}


void MOTION_stop_robot(void)
{
    Serial1.print('P');
    Serial1.print(0);
    Serial1.print(0);
    Serial1.print(0);
    Serial1.print(0);
    Serial1.print(0);
    Serial1.print(0);
}

void MOTION_set_delta(double delta) {
  Serial1.print('D');
  
    entier = (unsigned int) fabs(delta * 10);
    display6 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display5 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display4 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display3 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display2 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display1 = (entier % 10) + 48;

    if (delta < 0)
        Serial1.print('-');
    else
        Serial1.print('+');
    Serial1.print(display1);
    Serial1.print(display2);
    Serial1.print(display3);
    Serial1.print(display4);
    Serial1.print(display5);
    Serial1.print(display6);
  
}

void MOTION_set_alpha(double alpha) {
  Serial1.print('A');
  
    entier = (unsigned int) fabs(alpha * 1000);
    display6 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display5 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display4 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display3 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display2 = (entier % 10) + 48;
    entier = (unsigned int) (entier / 10);
    display1 = (entier % 10) + 48;

    if (alpha < 0)
        Serial1.print('-');
    else
        Serial1.print('+');
    Serial1.print(display1);
    Serial1.print(display2);
    Serial1.print(display3);
    Serial1.print(display4);
    Serial1.print(display5);
    Serial1.print(display6);
  
}


/*********************************/
/* COMMUNICATION WITH GUI MODULE */
/*********************************/




/**********************************/
/* COMMUNICATION WITH PAWN MODULE */
/**********************************/
void grip_pawn(void)
{
    gripServo_right.write(90);
    gripServo_left.write(90);
}

void release_pawn(void)
{
    gripServo_right.write(170);
    gripServo_left.write(10);
}
