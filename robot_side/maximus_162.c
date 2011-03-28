/*****************************************************
Project : Maximus
Version : 4.2
Date : 01/11/2010
Author : JBot
Company :
Comments:


Chip type : ATmega162
Program type : Application
Clock frequency : 14,745600 MHz
Memory model : Small
External SRAM size : 0
Data Stack size : 256
 *****************************************************/  
    
#include <mega162.h>
#include <delay.h>
#include <math.h>
    
#define DEF
// #define USE_IMU
    
#define TICK_PER_MM_LEFT 91.143671935
#define TICK_PER_MM_RIGHT 91.143671935
#define DIAMETER 328.0
    
#define TWOPI 6.2831853070
#define RAD2DEG 57.2958 /* radians to degrees conversion */
    
// Types of motor
#define ALPHA_MOTOR 0
#define DELTA_MOTOR 1
#define LEFT_MOTOR 2
#define RIGHT_MOTOR 3
    typedef struct {
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
} motor;
typedef struct {
    double pos_X;
     double pos_Y;
     double theta;
     float yaw;
     float pitch;
     float roll;
     float yaw_offset;
} robot;
typedef struct {
    char state;
     double current_distance;
     double desired_distance;
} RobotCommand;

#define COMMAND_DONE 0
#define PROCESSING_COMMAND 1
#define WAITING_BEGIN 2
#define ERROR 3
long general_time_counter = 0;
motor left_motor;
motor right_motor;
motor alpha_motor;
motor delta_motor;
robot maximus;
RobotCommand bot_command_delta;
RobotCommand prev_bot_command_delta;
RobotCommand bot_command_alpha;
volatile long left_cnt = 0;
volatile long right_cnt = 0;

/***********************/ 
/* Odometry interrupts */ 
/***********************/ 
// External Interrupt 0 service routine
    interrupt[EXT_INT0]
void ext_int0_isr(void) 
{
    
        //#asm("cli")
        if (PINA .0 == 1) {
        if (PIND .2 == 1)
            left_cnt--;
        
        else
            left_cnt++;
    }
    
    else {
        if (PIND .2 == 0)
            left_cnt--;
        
        else
            left_cnt++;
    }
    
        //#asm("sei")
}


// External Interrupt 1 service routine
    interrupt[EXT_INT1]
void ext_int1_isr(void) 
{
    if (PINC .0 == 1) {
        if (PIND .3 == 1)
            right_cnt++;
        
        else
            right_cnt--;
    }
    
    else {
        if (PIND .3 == 0)
            right_cnt++;
        
        else
            right_cnt--;
    }
}


// Pin change 0-7 interrupt service routine
    interrupt[PCINT0]
void pin_change_isr0(void) 
{
    if (PIND .2 == 1) {
        if (PINA .0 == 1) {
            left_cnt++;
        }
        
        else
            left_cnt--;
    }
    
    else {
        if (PINA .0 == 0) {
            left_cnt++;
        }
        
        else
            left_cnt--;
    }
}


// Pin change 8-15 interrupt service routine
    interrupt[PCINT1]
void pin_change_isr1(void) 
{
    if (PIND .3 == 1) {
        if (PINC .0 == 1)
            right_cnt--;
        
        else
            right_cnt++;
    }
    
    else {
        if (PINC .0 == 0)
            right_cnt--;
        
        else
            right_cnt++;
    }
}


#define RXB8 1
#define TXB8 0
#define UPE 2
#define OVR 3
#define FE 4
#define UDRE 5
#define RXC 7
    
#define FRAMING_ERROR (1<<FE)
#define PARITY_ERROR (1<<UPE)
#define DATA_OVERRUN (1<<OVR)
#define DATA_REGISTER_EMPTY (1<<UDRE)
#define RX_COMPLETE (1<<RXC)
    
// USART0 Receiver buffer
#define RX_BUFFER_SIZE0 64
char rx_buffer0[RX_BUFFER_SIZE0];

#if RX_BUFFER_SIZE0<256
unsigned char rx_wr_index0, rx_rd_index0, rx_counter0;

#else   /*  */
unsigned int rx_wr_index0, rx_rd_index0, rx_counter0;

#endif  /*  */
    
// This flag is set on USART0 Receiver buffer overflow
    bit rx_buffer_overflow0;

// USART0 Receiver interrupt service routine
    interrupt[USART0_RXC]
void usart0_rx_isr(void) 
{
    char status, data;
    status = UCSR0A;
    data = UDR0;
    if ((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN)) == 0)
         {
        rx_buffer0[rx_wr_index0] = data;
        if (++rx_wr_index0 == RX_BUFFER_SIZE0)
            rx_wr_index0 = 0;
        if (++rx_counter0 == RX_BUFFER_SIZE0)
             {
            rx_counter0 = 0;
            rx_buffer_overflow0 = 1;
            };
        };
}


#ifndef _DEBUG_TERMINAL_IO_
// Get a character from the USART0 Receiver buffer
#define _ALTERNATE_GETCHAR_
#pragma used+
char getchar(void) 
{
    char data;
    while (rx_counter0 == 0);
    data = rx_buffer0[rx_rd_index0];
    if (++rx_rd_index0 == RX_BUFFER_SIZE0)
        rx_rd_index0 = 0;
    
#asm("cli")
        --rx_counter0;
    
#asm("sei")
        return data;
}


#pragma used-
#endif  /*  */
    
// USART0 Transmitter buffer
#define TX_BUFFER_SIZE0 32
char tx_buffer0[TX_BUFFER_SIZE0];

#if TX_BUFFER_SIZE0<256
unsigned char tx_wr_index0, tx_rd_index0, tx_counter0;

#else   /*  */
unsigned int tx_wr_index0, tx_rd_index0, tx_counter0;

#endif  /*  */
    
// USART0 Transmitter interrupt service routine
    interrupt[USART0_TXC]
void usart0_tx_isr(void) 
{
    if (tx_counter0)
         {
        --tx_counter0;
        UDR0 = tx_buffer0[tx_rd_index0];
        if (++tx_rd_index0 == TX_BUFFER_SIZE0)
            tx_rd_index0 = 0;
        };
}


#ifndef _DEBUG_TERMINAL_IO_
// Write a character to the USART0 Transmitter buffer
#define _ALTERNATE_PUTCHAR_
#pragma used+
void putchar(char c) 
{
    while (tx_counter0 == TX_BUFFER_SIZE0);
    
#asm("cli")
        if (tx_counter0 || ((UCSR0A & DATA_REGISTER_EMPTY) == 0))
         {
        tx_buffer0[tx_wr_index0] = c;
        if (++tx_wr_index0 == TX_BUFFER_SIZE0)
            tx_wr_index0 = 0;
        ++tx_counter0;
        }
    
    else
        UDR0 = c;
    
#asm("sei")
}


#pragma used-
#endif  /*  */
    
// USART1 Receiver buffer
#define RX_BUFFER_SIZE1 32
char rx_buffer1[RX_BUFFER_SIZE1];

#if RX_BUFFER_SIZE1<256
unsigned char rx_wr_index1, rx_rd_index1, rx_counter1;

#else   /*  */
unsigned int rx_wr_index1, rx_rd_index1, rx_counter1;

#endif  /*  */
    
// This flag is set on USART1 Receiver buffer overflow
    bit rx_buffer_overflow1;

// USART1 Receiver interrupt service routine
    interrupt[USART1_RXC]
void usart1_rx_isr(void) 
{
    char status, data;
    status = UCSR1A;
    data = UDR1;
    if ((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN)) == 0)
         {
        rx_buffer1[rx_wr_index1] = data;
        if (++rx_wr_index1 == RX_BUFFER_SIZE1)
            rx_wr_index1 = 0;
        if (++rx_counter1 == RX_BUFFER_SIZE1)
             {
            rx_counter1 = 0;
            rx_buffer_overflow1 = 1;
            };
        };
}


// Get a character from the USART1 Receiver buffer
#pragma used+
char getchar1(void) 
{
    char data;
    while (rx_counter1 == 0);
    data = rx_buffer1[rx_rd_index1];
    if (++rx_rd_index1 == RX_BUFFER_SIZE1)
        rx_rd_index1 = 0;
    
#asm("cli")
        --rx_counter1;
    
#asm("sei")
        return data;
}


#pragma used-
// USART1 Transmitter buffer
#define TX_BUFFER_SIZE1 64
char tx_buffer1[TX_BUFFER_SIZE1];

#if TX_BUFFER_SIZE1<256
unsigned char tx_wr_index1, tx_rd_index1, tx_counter1;

#else   /*  */
unsigned int tx_wr_index1, tx_rd_index1, tx_counter1;

#endif  /*  */
    
// USART1 Transmitter interrupt service routine
    interrupt[USART1_TXC]
void usart1_tx_isr(void) 
{
    if (tx_counter1)
         {
        --tx_counter1;
        UDR1 = tx_buffer1[tx_rd_index1];
        if (++tx_rd_index1 == TX_BUFFER_SIZE1)
            tx_rd_index1 = 0;
        };
}


// Write a character to the USART1 Transmitter buffer
#pragma used+
void putchar1(char c) 
{
    while (tx_counter1 == TX_BUFFER_SIZE1);
    
#asm("cli")
        if (tx_counter1 || ((UCSR1A & DATA_REGISTER_EMPTY) == 0))
         {
        tx_buffer1[tx_wr_index1] = c;
        if (++tx_wr_index1 == TX_BUFFER_SIZE1)
            tx_wr_index1 = 0;
        ++tx_counter1;
        }
    
    else
        UDR1 = c;
    
#asm("sei")
}


#pragma used-
    
// Standard Input/Output functions
#include <stdio.h>
char output_ON = 0;
char asserv_ON = 1;
int last_left = 0;
int last_right = 0;
int left_diff = 0;
int right_diff = 0;
float total_distance = 0.0;
unsigned int entier;
char display1, display2, display3, display4, display5, display6, display7;

/*************************/ 
/* Global Init functions */ 
/*************************/ 
void init_Robot(robot * my_robot)
{
    my_robot->pos_X = -700.0;                             // -700
    my_robot->pos_Y = 700.0;                               // 700
    my_robot->theta = PI / 2;                              // PI/2
    my_robot->yaw = 0.0;
    my_robot->pitch = 0.0;
    my_robot->roll = 0.0;
    my_robot->yaw_offset = 0.0;
} void init_Command(RobotCommand * cmd)
{
    cmd->state = COMMAND_DONE;
    cmd->current_distance = 0;
    cmd->desired_distance = 0;
} void init_motors(void)
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
    alpha_motor.kP = 600;
    alpha_motor.kI = 0;
    alpha_motor.kD = 100;
    alpha_motor.accel = 300;
    alpha_motor.decel = 500;
    alpha_motor.max_speed = 12000;
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
#ifdef DEF
// Used to change the speed value of motor 1
void write_RoboClaw_speed_M1(char addr, signed long speed)
{
    char checkSUM;
    checkSUM =
        (addr + 35 + ((char) ((speed >> 24) & 0xFF)) + ((char) ((speed >> 16) & 0xFF)) + ((char) ((speed >> 8) & 0xFF)) +
         ((char) (speed & 0xFF))) & 0x7F;
    putchar1(addr);
    putchar1(35);
    putchar1(((char) ((speed >> 24) & 0xFF)));
    putchar1(((char) ((speed >> 16) & 0xFF)));
    putchar1(((char) ((speed >> 8) & 0xFF)));
    putchar1(((char) (speed & 0xFF)));
    putchar1(checkSUM);
} 

// Used to change the speed value of motor 2
void write_RoboClaw_speed_M2(char addr, signed long speed)
{
    char checkSUM;
    checkSUM =
        (addr + 36 + ((char) ((speed >> 24) & 0xFF)) + ((char) ((speed >> 16) & 0xFF)) + ((char) ((speed >> 8) & 0xFF)) +
         ((char) (speed & 0xFF))) & 0x7F;
    putchar1(addr);
    putchar1(36);
    putchar1(((char) ((speed >> 24) & 0xFF)));
    putchar1(((char) ((speed >> 16) & 0xFF)));
    putchar1(((char) ((speed >> 8) & 0xFF)));
    putchar1(((char) (speed & 0xFF)));
    putchar1(checkSUM);
} 

// Used to change the speed value of motors 1 and 2
void write_RoboClaw_speed_M1M2(char addr, signed long speedM1, signed long speedM2)
{
    char checkSUM;
    checkSUM =
        (addr + 37 + ((char) ((speedM1 >> 24) & 0xFF)) + ((char) ((speedM1 >> 16) & 0xFF)) + ((char) ((speedM1 >> 8) & 0xFF)) +
         ((char) (speedM1 & 0xFF)) + ((char) ((speedM2 >> 24) & 0xFF)) + ((char) ((speedM2 >> 16) & 0xFF)) + ((char) ((speedM2 >> 8) & 0xFF)) +
         ((char) (speedM2 & 0xFF))) & 0x7F;
    putchar1(addr);
    putchar1(37);
    putchar1(((char) ((speedM1 >> 24) & 0xFF)));
    putchar1(((char) ((speedM1 >> 16) & 0xFF)));
    putchar1(((char) ((speedM1 >> 8) & 0xFF)));
    putchar1(((char) (speedM1 & 0xFF)));
    putchar1(((char) ((speedM2 >> 24) & 0xFF)));
    putchar1(((char) ((speedM2 >> 16) & 0xFF)));
    putchar1(((char) ((speedM2 >> 8) & 0xFF)));
    putchar1(((char) (speedM2 & 0xFF)));
    putchar1(checkSUM);
} 

// Used to change the speed value of motor 1 and 2 during a specific distance
void write_RoboClaw_speed_dist_M1M2(char addr, signed long speedM1, signed long distanceM1, signed long speedM2, signed long distanceM2)
{
    char checkSUM;
    checkSUM =
        (addr + 43 + ((char) ((speedM1 >> 24) & 0xFF)) + ((char) ((speedM1 >> 16) & 0xFF)) + ((char) ((speedM1 >> 8) & 0xFF)) +
         ((char) (speedM1 & 0xFF)) + ((char) ((speedM2 >> 24) & 0xFF)) + ((char) ((speedM2 >> 16) & 0xFF)) + ((char) ((speedM2 >> 8) & 0xFF)) +
         ((char) (speedM2 & 0xFF)) + ((char) ((distanceM1 >> 24) & 0xFF)) + ((char) ((distanceM1 >> 16) & 0xFF)) +
         ((char) ((distanceM1 >> 8) & 0xFF)) + ((char) (distanceM1 & 0xFF)) + ((char) ((distanceM2 >> 24) & 0xFF)) +
         ((char) ((distanceM2 >> 16) & 0xFF)) + ((char) ((distanceM2 >> 8) & 0xFF)) + ((char) (distanceM2 & 0xFF)) + 1) & 0x7F;
    putchar1(addr);
    putchar1(43);
    putchar1(((char) ((speedM1 >> 24) & 0xFF)));
    putchar1(((char) ((speedM1 >> 16) & 0xFF)));
    putchar1(((char) ((speedM1 >> 8) & 0xFF)));
    putchar1(((char) (speedM1 & 0xFF)));
    putchar1(((char) ((distanceM1 >> 24) & 0xFF)));
    putchar1(((char) ((distanceM1 >> 16) & 0xFF)));
    putchar1(((char) ((distanceM1 >> 8) & 0xFF)));
    putchar1(((char) (distanceM1 & 0xFF)));
    putchar1(((char) ((speedM2 >> 24) & 0xFF)));
    putchar1(((char) ((speedM2 >> 16) & 0xFF)));
    putchar1(((char) ((speedM2 >> 8) & 0xFF)));
    putchar1(((char) (speedM2 & 0xFF)));
    putchar1(((char) ((distanceM2 >> 24) & 0xFF)));
    putchar1(((char) ((distanceM2 >> 16) & 0xFF)));
    putchar1(((char) ((distanceM2 >> 8) & 0xFF)));
    putchar1(((char) (distanceM2 & 0xFF)));
    putchar1(1);
    putchar1(checkSUM);
} 

// Used to change the speed value of motor 1 and 2 during a specific distance with a specific acceleration
void write_RoboClaw_allcmd_M1M2(char addr, signed long accel, signed long speedM1, signed long distanceM1, signed long speedM2,
                                signed long distanceM2)
{
    char checkSUM;
    checkSUM =
        (addr + 46 + ((char) ((accel >> 24) & 0xFF)) + ((char) ((accel >> 16) & 0xFF)) + ((char) ((accel >> 8) & 0xFF)) + ((char) (accel & 0xFF)) +
         ((char) ((speedM1 >> 24) & 0xFF)) + ((char) ((speedM1 >> 16) & 0xFF)) + ((char) ((speedM1 >> 8) & 0xFF)) + ((char) (speedM1 & 0xFF)) +
         ((char) ((speedM2 >> 24) & 0xFF)) + ((char) ((speedM2 >> 16) & 0xFF)) + ((char) ((speedM2 >> 8) & 0xFF)) + ((char) (speedM2 & 0xFF)) +
         ((char) ((distanceM1 >> 24) & 0xFF)) + ((char) ((distanceM1 >> 16) & 0xFF)) + ((char) ((distanceM1 >> 8) & 0xFF)) +
         ((char) (distanceM1 & 0xFF)) + ((char) ((distanceM2 >> 24) & 0xFF)) + ((char) ((distanceM2 >> 16) & 0xFF)) +
         ((char) ((distanceM2 >> 8) & 0xFF)) + ((char) (distanceM2 & 0xFF)) + 1) & 0x7F;
    putchar1(addr);
    putchar1(46);
    putchar1(((char) ((accel >> 24) & 0xFF)));
    putchar1(((char) ((accel >> 16) & 0xFF)));
    putchar1(((char) ((accel >> 8) & 0xFF)));
    putchar1(((char) (accel & 0xFF)));
    putchar1(((char) ((speedM1 >> 24) & 0xFF)));
    putchar1(((char) ((speedM1 >> 16) & 0xFF)));
    putchar1(((char) ((speedM1 >> 8) & 0xFF)));
    putchar1(((char) (speedM1 & 0xFF)));
    putchar1(((char) ((distanceM1 >> 24) & 0xFF)));
    putchar1(((char) ((distanceM1 >> 16) & 0xFF)));
    putchar1(((char) ((distanceM1 >> 8) & 0xFF)));
    putchar1(((char) (distanceM1 & 0xFF)));
    putchar1(((char) ((speedM2 >> 24) & 0xFF)));
    putchar1(((char) ((speedM2 >> 16) & 0xFF)));
    putchar1(((char) ((speedM2 >> 8) & 0xFF)));
    putchar1(((char) (speedM2 & 0xFF)));
    putchar1(((char) ((distanceM2 >> 24) & 0xFF)));
    putchar1(((char) ((distanceM2 >> 16) & 0xFF)));
    putchar1(((char) ((distanceM2 >> 8) & 0xFF)));
    putchar1(((char) (distanceM2 & 0xFF)));
    putchar1(1);
    putchar1(checkSUM);
} 
#endif  /*  */
    
/*******************************/ 
/* CONVERSION FUNCTIONS */ 
/*******************************/ 
signed long convert_dist2ticks(signed long distance)
{
    return (distance * TICK_PER_MM_RIGHT);
}

signed long convert_ticks2dist(signed long ticks)
{
    return (ticks / TICK_PER_MM_RIGHT);
}

void move_motors(void)
{
    write_RoboClaw_speed_M1M2(128, delta_motor.des_speed - alpha_motor.des_speed, delta_motor.des_speed + alpha_motor.des_speed);
} 

/*******************************/ 
/* MOTION CONTROL FUNCTIONS */ 
/*******************************/ 
void set_new_command(RobotCommand * cmd, long distance)
{
    cmd->state = WAITING_BEGIN;
    
        //#asm("cli") // disable interrupts
        cmd->current_distance = 0;
    cmd->desired_distance = distance;
    
        //#asm("sei") // enable interrupts
} long compute_position_PID(RobotCommand * cmd, motor * used_motor)
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
    used_motor->error_sum += err;                        //Somme les erreurs depuis le debut
    if (used_motor->error_sum > 10)
        used_motor->error_sum = 10;
    if (used_motor->error_sum < -10)
        used_motor->error_sum = -10;
    errDif = err - used_motor->last_error;               //Calcule la variation de l'erreur
    used_motor->last_error = err;
    P = err * used_motor->kP;                            //Proportionnelle
    I = used_motor->error_sum * used_motor->kI;            //Integrale
    D = errDif * used_motor->kD;                           //Derivee
    tmp = (P + I + D);
    if (tmp > (used_motor->des_speed + used_motor->accel))
        tmp = (used_motor->des_speed + used_motor->accel);
    
    else if (tmp < (used_motor->des_speed - used_motor->accel))
        tmp = (used_motor->des_speed - used_motor->accel);
    if (tmp > (used_motor->max_speed))
        tmp = (used_motor->max_speed);
    if (tmp < -(used_motor->max_speed))
        tmp = -(used_motor->max_speed);
    if ((cmd->state == PROCESSING_COMMAND) && (abs(err) < 8) && (abs(errDif) < 8)) {
        cmd->state = COMMAND_DONE;
    }
    return tmp;
}

double distance_coord(robot * my_robot, double x1, double y1)
{
    double x = 0;
    x = sqrt(pow(fabs(x1 - my_robot->pos_X), 2) + pow(fabs(y1 - my_robot->pos_Y), 2));
    return x;
}

double angle_coord(robot * my_robot, double x1, double y1)
{
    double angletodo = 0;
    if ((x1 < my_robot->pos_X) && (y1 < my_robot->pos_Y)) {
        angletodo = -PI / 2 - atan(fabs((x1 - my_robot->pos_X) / (y1 - my_robot->pos_Y)));
    }
    
    else if ((x1 > my_robot->pos_X) && (y1 < my_robot->pos_Y)) {
        angletodo = -atan(fabs((y1 - my_robot->pos_Y) / (x1 - my_robot->pos_X)));
    }
    
    else if ((x1 > my_robot->pos_X) && (y1 > my_robot->pos_Y)) {
        angletodo = atan(fabs((y1 - my_robot->pos_Y) / (x1 - my_robot->pos_X)));
    }
    
    else if ((x1 < my_robot->pos_X) && (y1 > my_robot->pos_Y)) {
        angletodo = PI / 2 + atan(fabs((x1 - my_robot->pos_X) / (y1 - my_robot->pos_Y)));
    }
    
    else if ((x1 < my_robot->pos_X) && (y1 == my_robot->pos_Y)) {       // 
        angletodo = -PI;
    }
    
    else if ((x1 > my_robot->pos_X) && (y1 == my_robot->pos_Y)) {       // 
        angletodo = 0;
    }
    
    else if ((x1 == my_robot->pos_X) && (y1 < my_robot->pos_Y)) {       // 
        angletodo = -PI / 2;
    }
    
    else if ((x1 == my_robot->pos_X) && (y1 > my_robot->pos_Y)) {       // 
        angletodo = PI / 2;
    }
    
    else
        angletodo = 0;
    angletodo = angletodo - my_robot->theta;
    if (angletodo > PI)
        angletodo = angletodo - 2 * PI;
    if (angletodo < -PI)
        angletodo = 2 * PI + angletodo;
    return angletodo;
}


/* New version */ 
void goto_xy(double x, double y)
{
    double ang, dist;
    ang = angle_coord(&maximus, x, y) * RAD2DEG;
    set_new_command(&bot_command_alpha, ang);
    dist = distance_coord(&maximus, x, y);
    set_new_command(&prev_bot_command_delta, dist);
} void update_motor(motor * used_motor)
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


/*************************************/ 
/* Compute the position of the robot */ 
/*************************************/ 
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

void do_motion_control(void)
{
    
        // PID distance
        if ((bot_command_alpha.state == WAITING_BEGIN) || (bot_command_alpha.state == PROCESSING_COMMAND)) {    // If alpha motor have not finished its movement 
    }
    
    else {
        if ((bot_command_delta.state != PROCESSING_COMMAND) && (prev_bot_command_delta.state == WAITING_BEGIN)) {
            prev_bot_command_delta.state = PROCESSING_COMMAND;
            set_new_command(&bot_command_delta, prev_bot_command_delta.desired_distance);
        }
    }
    delta_motor.des_speed = compute_position_PID(&bot_command_delta, &delta_motor);
    
        // PID angle
        alpha_motor.des_speed = compute_position_PID(&bot_command_alpha, &alpha_motor);
    if (bot_command_alpha.state == WAITING_BEGIN) {
        bot_command_alpha.state = PROCESSING_COMMAND;
    }
}


/***************************/ 
/* ROS Interface Functions */ 
/***************************/ 
void angular_from_ROS(char sign)
{
    signed long tmp = 0;
    tmp = (tmp * 10) + (getchar() - 48);
    tmp = (tmp * 10) + (getchar() - 48);
    tmp = (tmp * 10) + (getchar() - 48);
    tmp = (tmp * 10) + (getchar() - 48);
    tmp = (tmp * 10) + (getchar() - 48);
    getchar();
    if (sign == 0)                                        // negative
        alpha_motor.des_speed = -tmp;
    
    else
        alpha_motor.des_speed = tmp;
}

void linear_from_ROS(char sign)
{
    signed long tmp = 0;
    tmp = (tmp * 10) + (getchar() - 48);
    tmp = (tmp * 10) + (getchar() - 48);
    tmp = (tmp * 10) + (getchar() - 48);
    tmp = (tmp * 10) + (getchar() - 48);
    tmp = (tmp * 10) + (getchar() - 48);
    getchar();
    if (sign == 0)                                        // negative
        delta_motor.des_speed = -tmp;
    
    else
        delta_motor.des_speed = tmp;
}

void change_alphakP(void)
{
    signed long tmp = 0;
    tmp = (tmp * 10) + (getchar() - 48);
    tmp = (tmp * 10) + (getchar() - 48);
    tmp = (tmp * 10) + (getchar() - 48);
    tmp = (tmp * 10) + (getchar() - 48);
    tmp = (tmp * 10) + (getchar() - 48);
    getchar();
    alpha_motor.kP = tmp;
} void change_alphakD(void)
{
    signed long tmp = 0;
    tmp = (tmp * 10) + (getchar() - 48);
    tmp = (tmp * 10) + (getchar() - 48);
    tmp = (tmp * 10) + (getchar() - 48);
    tmp = (tmp * 10) + (getchar() - 48);
    tmp = (tmp * 10) + (getchar() - 48);
    getchar();
    alpha_motor.kD = tmp;
} void change_alphakI(void)
{
    signed long tmp = 0;
    tmp = (tmp * 10) + (getchar() - 48);
    tmp = (tmp * 10) + (getchar() - 48);
    tmp = (tmp * 10) + (getchar() - 48);
    tmp = (tmp * 10) + (getchar() - 48);
    tmp = (tmp * 10) + (getchar() - 48);
    getchar();
    alpha_motor.kI = tmp;
} void change_deltakP(void)
{
    signed long tmp = 0;
    tmp = (tmp * 10) + (getchar() - 48);
    tmp = (tmp * 10) + (getchar() - 48);
    tmp = (tmp * 10) + (getchar() - 48);
    tmp = (tmp * 10) + (getchar() - 48);
    tmp = (tmp * 10) + (getchar() - 48);
    getchar();
    delta_motor.kP = tmp;
} void change_deltakD(void)
{
    signed long tmp = 0;
    tmp = (tmp * 10) + (getchar() - 48);
    tmp = (tmp * 10) + (getchar() - 48);
    tmp = (tmp * 10) + (getchar() - 48);
    tmp = (tmp * 10) + (getchar() - 48);
    tmp = (tmp * 10) + (getchar() - 48);
    getchar();
    delta_motor.kD = tmp;
} void change_deltakI(void)
{
    signed long tmp = 0;
    tmp = (tmp * 10) + (getchar() - 48);
    tmp = (tmp * 10) + (getchar() - 48);
    tmp = (tmp * 10) + (getchar() - 48);
    tmp = (tmp * 10) + (getchar() - 48);
    tmp = (tmp * 10) + (getchar() - 48);
    getchar();
    delta_motor.kI = tmp;
} void get_goal_from_ROS(void)
{
    signed long x = 0, y = 0;
    x = (x * 10) + (getchar() - 48);
    x = (x * 10) + (getchar() - 48);
    x = (x * 10) + (getchar() - 48);
    x = x * (-10);
    y = (y * 10) + (getchar() - 48);
    y = (y * 10) + (getchar() - 48);
    y = (y * 10) + (getchar() - 48);
    y = y * 10;
    goto_xy(((double) x / 2), ((double) y / 2));
} 

/*************************************/ 
/* Basic movement functions for test */ 
/*************************************/ 
void tourner_droite(void)
{
    
        /*
         */ 
} void tourner_gauche(void)
{
    
        /*
         */ 
} void avancer(void)
{
    
        /*
         */ 
} void reculer(void)
{
    
        /*
         */ 
} void stop(void)
{
    
        /*
         */ 
} 

// Timer 0 overflow interrupt service routine
    interrupt[TIM0_OVF]
void timer0_ovf_isr(void) 
{
    
#asm("sei")                                                // enable interrupts
        get_Odometers();
    do_motion_control();
    if (output_ON == 1)
        move_motors();                                    // Update the motor speed
}


// Timer 1 overflow interrupt service routine
    interrupt[TIM1_OVF]
void timer1_ovf_isr(void) 
{
    
#asm("sei")                                                // enable interrupts
} 

// Declare your global variables here
void main(void) 
{
    
        // Declare your local variables here
    char serial_command;
    volatile int mycounter = 0;
    char start = 0;
    
        // Crystal Oscillator division factor: 1
#pragma optsize-
        CLKPR = 0x80;
    CLKPR = 0x00;
    
#ifdef _OPTIMIZE_SIZE_
#pragma optsize+
#endif  /*  */
#ifdef DEF
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
    DDRB = 0x01;
    
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
    
        // Timer/Counter 0 initialization
        // Clock source: System Clock
        // Clock value: 57,600 kHz
        // Mode: Normal top=FFh
        // OC0 output: Disconnected
        TCCR0 = 0x04;
    TCNT0 = 0x00;
    OCR0 = 0x00;
    
        // Timer/Counter 1 initialization
        // Clock source: System Clock
        // Clock value: 14,400 kHz // 35.4861 ms => 21.180039 fois dans 1 sec
        // Mode: Ph. correct PWM top=01FFh
        // OC1A output: Discon.
        // OC1B output: Discon.
        // Noise Canceler: Off
        // Input Capture on Falling Edge
        // Timer 1 Overflow Interrupt: On
        // Input Capture Interrupt: Off
        // Compare A Match Interrupt: Off
        // Compare B Match Interrupt: Off
        TCCR1A = 0x02;
    TCCR1B = 0x05;
    TCNT1H = 0x00;
    TCNT1L = 0x00;
    ICR1H = 0x00;
    ICR1L = 0x00;
    OCR1AH = 0x00;
    OCR1AL = 0x00;
    OCR1BH = 0x00;
    OCR1BL = 0x00;
    
        // Timer/Counter 2 initialization
        // Clock source: System Clock
        // Clock value: Timer 2 Stopped
        // Mode: Normal top=FFh
        // OC2 output: Disconnected
        ASSR = 0x00;
    TCCR2 = 0x00;
    TCNT2 = 0x00;
    OCR2 = 0x00;
    
        // Timer/Counter 3 initialization
        // Clock value: Timer 3 Stopped
        // Mode: Normal top=FFFFh
        // Noise Canceler: Off
        // Input Capture on Falling Edge
        // OC3A output: Discon.
        // OC3B output: Discon.
        // Timer 3 Overflow Interrupt: Off
        // Input Capture Interrupt: Off
        // Compare A Match Interrupt: Off
        // Compare B Match Interrupt: Off
        TCCR3A = 0x00;
    TCCR3B = 0x00;
    TCNT3H = 0x00;
    TCNT3L = 0x00;
    ICR3H = 0x00;
    ICR3L = 0x00;
    OCR3AH = 0x00;
    OCR3AL = 0x00;
    OCR3BH = 0x00;
    OCR3BL = 0x00;
    
        // External Interrupt(s) initialization
        // INT0: On
        // INT0 Mode: Any change
        // INT1: On
        // INT1 Mode: Any change
        // INT2: Off
        // Interrupt on any change on pins PCINT0-7: On
        // Interrupt on any change on pins PCINT8-15: On
        GICR |= 0xD8;
    PCMSK0 = 0x01;
    PCMSK1 = 0x01;
    MCUCR = 0x05;
    EMCUCR = 0x00;
    GIFR = 0xD8;
    
        // Timer(s)/Counter(s) Interrupt(s) initialization
        TIMSK = 0x82;
    ETIMSK = 0x00;
    
        // USART0 initialization
        // Communication Parameters: 8 Data, 1 Stop, No Parity
        // USART0 Receiver: On
        // USART0 Transmitter: On
        // USART0 Mode: Asynchronous
        // USART0 Baud rate: 115200
        UCSR0A = 0x00;
    UCSR0B = 0xD8;
    UCSR0C = 0x86;
    UBRR0H = 0x00;
    UBRR0L = 0x07;
    
        // USART1 initialization
        // Communication Parameters: 8 Data, 1 Stop, No Parity
        // USART1 Receiver: On
        // USART1 Transmitter: On
        // USART1 Mode: Asynchronous
        // USART1 Baud rate: 38400
        UCSR1A = 0x00;
    UCSR1B = 0xD8;
    UCSR1C = 0x86;
    UBRR1H = 0x00;
    UBRR1L = 0x17;
    
        // Analog Comparator initialization
        // Analog Comparator: Off
        // Analog Comparator Input Capture by Timer/Counter 1: Off
        ACSR = 0x80;
    
#endif  /*  */
        write_RoboClaw_speed_M1M2(128, 0, 0);
    init_motors();                                       // Init motors
    init_Robot(&maximus);                                  // Init robot status
    init_Command(&bot_command_delta);                     // Init robot command
    init_Command(&bot_command_alpha);                      // Init robot command
    
        // Global enable interrupts
#asm("sei")
        delay_ms(1000);
    
        // For ROS utilization
        output_ON = 1;
    while (1)
         {
        
            // Place your code here
            delay_ms(50);
        if (output_ON == 1) {
            
                /* Display for ROS */ 
                entier = (unsigned int) fabs(maximus.pos_X * 10);
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
            putchar('x');
            if (maximus.pos_X < 0)
                putchar('-');
            putchar(display1);
            putchar(display2);
            putchar(display3);
            putchar(display4);
            putchar(display5);
            putchar(display6);
            entier = (unsigned int) fabs(maximus.pos_Y * 10);
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
            putchar('y');
            if (maximus.pos_Y < 0)
                putchar('-');
            putchar(display1);
            putchar(display2);
            putchar(display3);
            putchar(display4);
            putchar(display5);
            putchar(display6);
            entier = (unsigned int) fabs(maximus.theta * 10000);
            
                //entier = (unsigned int) fabs(maximus.theta * RAD2DEG);
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
            putchar('t');
            if (maximus.theta < 0)
                putchar('-');
            putchar(display1);
            putchar(display2);
            putchar(display3);
            putchar(display4);
            putchar(display5);
            putchar(display6);
            putchar('\n');
            putchar('\r');
        }
        if (rx_counter0 >= (1 + 6)) {
            serial_command = getchar();
            switch (serial_command) {
            case 'a':                                    // positive
                angular_from_ROS(1);
                break;
            case 'A':                                    // negative
                angular_from_ROS(0);
                break;
            case 'l':
                linear_from_ROS(1);
                break;
            case 'L':
                linear_from_ROS(0);
                break;
                
                    // case 'T' :
                    // start = 1;
                    // mycounter = 0;
                    // //compute_distance_ramp(&bot_command, &delta_motor, convert_dist2ticks(350), general_time_counter);
                    // getchar();
                    // getchar();
                    // getchar();
                    // getchar();
                    // getchar();
                    // getchar();
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
            case 'G':
                get_goal_from_ROS();;
                break;
            }
        }
        };
}


