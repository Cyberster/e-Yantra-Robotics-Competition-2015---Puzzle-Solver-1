/*
* Team Id:			eYRCPlus-PS1#2678
* Author List:		Avick Dutta (team leader), Arpan Das, Subhendu Hazra, Asesh Basu
* Filename:			Puzzle_Solver.c
* Theme:			Puzzle Solver Robot (GLCD) - eYRCPlus
* Functions:		left_led_on, left_led_off, right_led_on, right_led_off,
*					get_point_cost, get_nearest_point, follow_black_line, follow_black_line_mm,
*					turn_robot, change_direction, move_one_cell, match_column, match_row,
*					go_to_coordinate, go_to_cell_no, get_pickup_direction, pickup, deposit
* Global Variables:	ADC_Value, flag, Left_white_line, Center_white_line, Right_white_line,
*					ShaftCountLeft, ShaftCountRight, Degrees, data, input_str, d1_position_map, 
*					d2_position_map, current_velocity, current_direction,
*					pickup_direction, current_grid, current_cell_no, current_coordinate,
*					BNW_Thresh, left_velocity_float, right_velocity_float,
*					left_velocity, right_velocity
*/

// Note: that predefined function definitions are written in Allman indent style
//		and our function definitions are written in K&R indent style
//		reference: https://en.wikipedia.org/wiki/Indent_style

#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>	//included to support power function
#include "lcd.h"
#include "glcd.h"	//User defined LCD library which contains the lcd routines
#include "glcd.c"

#include "glcd_big_font_msg.h"	// big ubuntu font stored as image
#include "glcd_big_font_msg.c"

#include <string.h>

void port_init();
void timer5_init();
void velocity(unsigned char, unsigned char);
void motors_delay();

unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char flag = 0;
unsigned char Left_white_line = 0;
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;

volatile unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder
volatile unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
volatile unsigned int Degrees; //to accept angle in degrees for turning

char data; //to store received data from UDR1
char input_str[100] = ""; // stores the raw input string

//Function to configure LCD port
void lcd_port_config (void)
{
	DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
	PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

//ADC pin configuration
void adc_pin_config (void)
{
	DDRF = 0x00;
	PORTF = 0x00;
	DDRK = 0x00;
	PORTK = 0x00;
}

//Function to configure ports to enable robot's motion
void motion_pin_config (void) 
{
	DDRA = DDRA | 0x0F;
	PORTA = PORTA & 0xF0;
	DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
	PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}


//Function to configure INT4 (PORTE 4) pin as input for the left position encoder
void left_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}

//Function to initialize Buzzer
void buzzer_pin_config (void)
{
	DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
	PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}

//Function to configure Interrupt switch
void interrupt_switch_config (void)
{
	DDRE = DDRE & 0x7F;  //PORTE 7 pin set as input
	PORTE = PORTE | 0x80; //PORTE7 internal pull-up enabled
}

//Function to configure LED port i.e. pg0 and pg1 for left and right LEDs respectively
void LED_pin_config (void) {
	DDRG |= 0b00000011; // all the LED pin's direction set as output
	PORTG &= 0b11111100; // all the LED pins are set to logic 0
}

//Function to Initialize PORTS
void port_init()
{
	lcd_port_config();
	adc_pin_config();
	motion_pin_config();	
	left_encoder_pin_config(); //left encoder pin config
	right_encoder_pin_config(); //right encoder pin config
	buzzer_pin_config();
	LED_pin_config();
	interrupt_switch_config();
}

void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
	EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
	sei();   // Enables the global interrupt
}

void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
	EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
	sei();   // Enables the global interrupt
}


void buzzer_on (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore | 0x08;
	PORTC = port_restore;
}

void buzzer_off (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore & 0xF7;
	PORTC = port_restore;
}


//ISR for right position encoder
ISR(INT5_vect)
{
	ShaftCountRight++;  //increment right shaft position count
}


//ISR for left position encoder
ISR(INT4_vect)
{
	ShaftCountLeft++;  //increment left shaft position count
}


// Timer 5 initialized in PWM mode for velocity control
// Prescale:256
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//Function For ADC Conversion
unsigned char ADC_Conversion(unsigned char Ch) 
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;  			
	ADMUX= 0x20| Ch;	   		
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}

//Function To Print Sensor Values At Desired Row And Column Location on LCD
void print_sensor(char row, char coloumn,unsigned char channel)
{
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}

//Function for velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
	unsigned char PortARestore = 0;

	Direction &= 0x0F; 		// removing upper nibbel for the protection
	PortARestore = PORTA; 		// reading the PORTA original status
	PortARestore &= 0xF0; 		// making lower direction nibbel to 0
	PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
	PORTA = PortARestore; 		// executing the command
}

void forward (void) //both wheels forward
{
	motion_set(0x06);
}

void back (void) //both wheels backward
{
	motion_set(0x09);
}

void left (void) //Left wheel backward, Right wheel forward
{
	motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
	motion_set(0x0A);
}

void soft_left (void) //Left wheel stationary, Right wheel forward
{
	motion_set(0x04);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
	motion_set(0x02);
}

void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
	motion_set(0x01);
}

void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
	motion_set(0x08);
}

void stop (void)
{
	motion_set(0x00);
}


//Function used for turning robot by specified degrees
void angle_rotate(unsigned int Degrees)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;

	while (1)
	{
		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
		break;
	}
	stop(); //Stop robot
}

//Function used for moving robot forward by specified distance

void linear_distance_mm(unsigned int DistanceInMM)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
	
	ShaftCountRight = 0;
	while(1)
	{
		if(ShaftCountRight > ReqdShaftCountInt)
		{
			break;
		}
	}
	stop(); //Stop robot
}


void forward_mm(unsigned int DistanceInMM)
{
	forward();
	linear_distance_mm(DistanceInMM);
}

void back_mm(unsigned int DistanceInMM)
{
	back();
	linear_distance_mm(DistanceInMM);
}

void left_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	left(); //Turn left
	angle_rotate(Degrees);
}


void right_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	right(); //Turn right
	angle_rotate(Degrees);
}


void soft_left_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_left(); //Turn soft left
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void soft_right_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_right();  //Turn soft right
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void soft_left_2_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_left_2(); //Turn reverse soft left
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void soft_right_2_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_right_2();  //Turn reverse soft right
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}


//Function To Initialize UART2
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char size: 8 bit
// parity: Disabled
void uart2_init(void)
{
	UCSR2B = 0x00; //disable while setting baud rate
	UCSR2A = 0x00;
	UCSR2C = 0x06;
	UBRR2L = 0x5F; //set baud rate lo
	UBRR2H = 0x00; //set baud rate hi
	UCSR2B = 0x98;
}

SIGNAL(SIG_USART2_RECV) {		// ISR for receive complete interrupt
	data = UDR2; 				// making copy of data from UDR2 in 'data' variable
	UDR2 = data; 				// echo data back to PC
	strcat(input_str, &data);	// concatenate each ascii character received to string input_str
	//GLCD_DisplayChar(data);
}

void init_devices (void)
{
 	cli(); //Clears the global interrupts
	port_init();
	lcd_set_4bit();
	lcd_init();
	GLCD_Init();
	uart2_init(); //Initialize UART2 for serial communication
	adc_init();
	timer5_init();
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	sei();   //Enables the global interrupts
}
// #########################################################################################################
// ################################ my functions and variables start #######################################
// #########################################################################################################


// ################################ declaring global variables start #######################################

// mapping / setting cell co-ordinates in D1
//	12 cells, 4 points in each cell, 2 index for row-column in each point
//	4 points for each cell are sorted in this order: top-left, top-right, bottom-right, bottom-left
int d1_position_map[12][4][2] = {
	{{0, 0}, {0, 1}, {1, 1}, {1, 0}}, {{0, 1}, {0, 2}, {1, 2}, {1, 1}}, {{0, 2}, {0, 3}, {1, 3}, {1, 2}}, {{0, 3}, {0, 4}, {1, 4}, {1, 3}},
	{{1, 0}, {1, 1}, {2, 1}, {2, 0}}, {{1, 1}, {1, 2}, {2, 2}, {2, 1}}, {{1, 2}, {1, 3}, {2, 3}, {2, 2}}, {{1, 3}, {1, 4}, {2, 4}, {2, 3}},
	{{2, 0}, {2, 1}, {3, 1}, {3, 0}}, {{2, 1}, {2, 2}, {3, 2}, {3, 1}}, {{2, 2}, {2, 3}, {3, 3}, {3, 2}}, {{2, 3}, {2, 4}, {3, 4}, {3, 3}}
};

// mapping / setting cell co-ordinates in D2
//	24 cells, 4 points in each cell, 2 index for row-column in each point
//	4 points for each cell are sorted in this order: top-left, top-right, bottom-right, bottom-left
int d2_position_map[24][4][2] = {
	{{0, 0}, {0, 1}, {1, 1}, {1, 0}}, {{0, 1}, {0, 2}, {1, 2}, {1, 1}}, {{0, 2}, {0, 3}, {1, 3}, {1, 2}}, {{0, 3}, {0, 4}, {1, 4}, {1, 3}}, {{0, 4}, {0, 5}, {1, 5}, {1, 4}}, {{0, 5}, {0, 6}, {1, 6}, {1, 5}},
	{{1, 0}, {1, 1}, {2, 1}, {2, 0}}, {{1, 1}, {1, 2}, {2, 2}, {2, 1}}, {{1, 2}, {1, 3}, {2, 3}, {2, 2}}, {{1, 3}, {1, 4}, {2, 4}, {2, 3}}, {{1, 4}, {1, 5}, {2, 5}, {2, 4}}, {{1, 5}, {1, 6}, {2, 6}, {2, 5}},
	{{2, 0}, {2, 1}, {3, 1}, {3, 0}}, {{2, 1}, {2, 2}, {3, 2}, {3, 1}}, {{2, 2}, {2, 3}, {3, 3}, {3, 2}}, {{2, 3}, {2, 4}, {3, 4}, {3, 3}}, {{2, 4}, {2, 5}, {3, 5}, {3, 4}}, {{2, 5}, {2, 6}, {3, 6}, {3, 5}},
	{{3, 0}, {3, 1}, {4, 1}, {4, 0}}, {{3, 1}, {3, 2}, {4, 2}, {4, 1}}, {{3, 2}, {3, 3}, {4, 3}, {4, 2}}, {{3, 3}, {3, 4}, {4, 4}, {4, 3}}, {{3, 4}, {3, 5}, {4, 5}, {4, 4}}, {{3, 5}, {3, 6}, {4, 6}, {4, 5}}
};

unsigned char current_velocity = 127;	// value can be 0-255, default velocity 127
char current_direction = 'N';			// value can be E/W/N/S i.e. east, west, north or south respectively, Default N
char pickup_direction = '\0';			// value can be L or R i.e. left or right respectively
int current_grid = -1;					// value can be 1 or 2 i.e. D1 or D2, initially -1 as invalid
int current_cell_no = -1;				// value can be 0 to 11 for D1, 0-23 for D2, initially -1 as invalid
int current_coordinate[2] = {-1, -1};	// co-ordinate of the cell i.e. row-column number from position map, initially -1, -1 as invalid
int BNW_Thresh = 40;					// black and white threshold value, default 28 [formula: ((W+B)/2)-(B/3)]

float left_velocity_float, right_velocity_float;	// stores velocity of left and right wheel respectively as float
unsigned char left_velocity, right_velocity;		// stores velocity of left and right wheel respectively as unsigned char
// ################################ declaring global variables end #########################################

/*
 * Function Name:	left_led_on
 * Input:			None	
 * Output:			It turns on the left RGB LED
 * Logic:			PORT G0 is configured for left RGB LED, writing logic high to PG0
 * Example Call:	left_led_on()
 */
void left_led_on (void) {
	unsigned char port_restore = 0;
	port_restore = PING;
	port_restore |= 0b00000001;
	PORTG = port_restore;
}

/*
 * Function Name:	left_led_off
 * Input:			None		
 * Output:			It turns of the left RGB LED
 * Logic:			PORT G0 is configured for left RGB LED, writing logic low to PG0		
 * Example Call:	left_led_on()
 */
void left_led_off (void) {
	unsigned char port_restore = 0;
	port_restore = PING;
	port_restore &= 0b11111110;
	PORTG = port_restore;
}

/*
 * Function Name:	right_led_on
 * Input:			None	
 * Output:			It turns on the right RGB LED
 * Logic:			PORT G1 is configured for right RGB LED, writing logic high to PG1			
 * Example Call:	right_led_on()
 */
void right_led_on (void) {
	unsigned char port_restore = 0;
	port_restore = PING;
	port_restore |= 0b00000010;
	PORTG = port_restore;
}

/*
 * Function Name:	right_led_off
 * Input:			None		
 * Output:			It turns on the right RGB LED
 * Logic:			PORT G1 is configured for right RGB LED, writing logic low to PG1			
 * Example Call:	right_led_off()
 */
void right_led_off (void) {
	unsigned char port_restore = 0;
	port_restore = PING;
	port_restore &= 0b11111101;
	PORTG = port_restore;
}

/*
 * Function Name:	get_point_cost
 * Input:			current_point - int array containing row & column number of current coordinate point,
 *					target_point - int array containing row & column number of target coordinate point
 * Output:			total_cost - int which is the calculated cost between current and target coordinate point
 * Logic:			It calculates the cost of traveling between two points i.e. calculating row difference and  
 *					column difference and make sum of them
 * Example Call:	get_point_cost(current_point, target_point)
 */
int get_point_cost (int current_point[2], int target_point[2]) {
	int total_cost;	
	total_cost = abs(current_point[0] - target_point[0]) + abs(current_point[1] - target_point[1]);
	return total_cost;
}

// It receives current point and target cell which contains 4 points and returns nearest point from current point among those 4 points

/*
 * Function Name:	get_nearest_point
 * Input:			current_point - int array containing row & column number of current coordinate point,
 *					target_cell - 2D int array contains target cell's 4 points with coordinate based on row & column
 * Output:			nearest_point - int array containing row & column number of nearest coordinate point among 4 points of target_cell
 * Logic:			We are calculating traveling cost i.e. distance between current point and each of target cell's 4 points (top-left,
 *					top-right, bottom-right, bottom-left) and returning that point with lowest cost
 * Example Call:	get_nearest_point(current_coordinate, d1_position_map[target_cell_no])
 */
int * get_nearest_point (int current_point[2], int target_cell[4][2]) {
	int * nearest_point = malloc(2 * sizeof(int));
	
	int i, current_cost, lowest_cost = 100;
	
	for (i=0; i<4; i++) {
		current_cost = get_point_cost(current_point, target_cell[i]);
		if (current_cost < lowest_cost) {
			nearest_point[0] = target_cell[i][0];
			nearest_point[1] = target_cell[i][1];
			lowest_cost = current_cost;
		}
	}
	
	return nearest_point;
}

// It follows a 1cm thick black line on white surface

/*
 * Function Name:	read_wl_sensor_values
 * Input:			Global variables Left_white_line, Center_white_line, Right_white_line
 * Output:			It updates global variables Left_white_line, Center_white_line, Right_white_line
 * Logic:			It reads all 3 white line sensor values by using function ADC_Conversion()
 * Example Call:	read_wl_sensor_values()
 */
void read_wl_sensor_values () {
	Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
	Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
	Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
}

/*
 * Function Name:	follow_black_line
 * Input:			Left_white_line, Center_white_line, Right_white_line are global variables,
 *					direction can be 'F' or 'B' as forward or backward respectively
 * Output:			It follows a 1cm thick black line on white surface
 * Logic:			To follow a 1cm black line, we can linearly forward the robot if all three white line sensors
 *					are on white surface or the center white line sensor is on the black surface. We decrease velocity of the
 *					right wheel and increase velocity of the left wheel if right white line sensor is on the black surface and
 *					other twos are on the white surface. Similarly we decrease velocity of the left wheel and increase velocity
 *					of the right wheel if left white line sensor is on the black surface and other twos are on the white surface.
 *					This way the robot follows a 1cm thick black line.
 *					
 * Example Call:	follow_black_line('F')
 */
void follow_black_line (char direction) {
	flag = 0;
		
	if (((Left_white_line <= BNW_Thresh) && (Center_white_line <= BNW_Thresh) && (Right_white_line <= BNW_Thresh)) || (Center_white_line > BNW_Thresh)) {
		flag = 1;
		if (direction == 'F') forward();
		else back();
		velocity(left_velocity, right_velocity);
	}

	if((Left_white_line <= BNW_Thresh) && (Center_white_line <= BNW_Thresh) && (Right_white_line > BNW_Thresh) && (flag == 0)) {
		flag = 1;
		if (direction == 'F') {
			forward();
			velocity(left_velocity+30, right_velocity-50);
		} else {
			back();
			velocity(left_velocity-50, right_velocity+30);
		}
	}

	if((Left_white_line > BNW_Thresh) && (Center_white_line <= BNW_Thresh) && (Right_white_line <= BNW_Thresh) && (flag == 0)) {
		flag = 1;
		if (direction == 'F') {
			forward();
			velocity(left_velocity-50, right_velocity+30);
		} else {
			back();
			velocity(left_velocity+30, right_velocity-50);
		}
	}
}


// It follows a black line up to a fixed distance

/*
 * Function Name:	follow_black_line_mm
 * Input:			DistanceInMM - is a unsigned integer, 
 *					direction - is a char variable
 * Output:			It follows a 1cm thick black line up to a specific distance
 * Logic:			It calculates required shaft count to travel up to DistanceInMM
 *					and follows the black line. ShaftCountRight and ShaftCountLeft are initially
 *					set to 0. While moving the robot, we increase them and When the ShaftCountRight
 *					or ShaftCountLeft reaches as required, we stop the robot
 * Example Call:	follow_black_line_mm(100, 'F')
 */
void follow_black_line_mm (unsigned int DistanceInMM, char direction) {
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
	
	ShaftCountRight = 0;
	ShaftCountLeft = 0;
	while(1) {
		read_wl_sensor_values();
		
		if(ShaftCountRight > ReqdShaftCountInt || ShaftCountLeft > ReqdShaftCountInt) {
			break;
		} else {
			follow_black_line(direction);
		}
	}
	
	stop(); //Stop robot	
}

// It turns the robot left/right by 5 degrees until encounters a black line.

/*
 * Function Name:	turn_robot
 * Input:			direction is a char variable which can be 'L' or 'R' as left or right respectively
 * Output:			This function rotates the robot perfect 90 degrees left or right
 * Logic:			At first it reads all white line sensor values and increase velocity to current velocity + 10
 *					left_degrees(90)3is static and does not always as per hardware and physical limitations.
 *					So, we have to rotate the robot dynamically. We rotate the robot by 5 degrees until
 *					the center white line sensor detects a black line. But to do that we have to cross/overcome 
 *					current black line and the thick black number inside a cell. If direction is 'L' then
 *					we rotate the robot left by 75 degrees using left_degrees(75) and if direction is 'R'
 *					then rotate the robot right by 75 degrees using right_degrees(75). After that, we rotate
 *					the robot by 5 degrees in mentioned direction until the center white line sensor detects
 *					a black line.		
 * Example Call:	turn_robot('L')
 */
void turn_robot (char direction) {
	read_wl_sensor_values();
	velocity(current_velocity+10, current_velocity+10);
	
	if (direction == 'L') {
		left_degrees(75);
		read_wl_sensor_values();
		while (Center_white_line <= BNW_Thresh) {
			read_wl_sensor_values();
			left_degrees(5);
		}
	} else {		
		right_degrees(75);
		read_wl_sensor_values();
		while (Center_white_line <= BNW_Thresh) {
			read_wl_sensor_values();
			right_degrees(5);
		}
	}
	velocity(current_velocity-10 ,current_velocity-10);
	_delay_ms(500);
}

// It changes the direction of the robot i.e. east/west/north/south.

/*
 * Function Name:	change_direction
 * Input:			desired_direction is a unsigned char variable which can be 'E'/'W'/'N'/'S' as
 *					ease, west, north or south respectively
 * Output:			It changes the robots direction to ease, west, north or south and updates its current 
 *					direction which is stored in global variable current_direction
 * Logic:			if current direction is north and desired direction is west then
 *						we turn the robot left by 90 degrees using turn_robot('L') and updates current direction to west
 *					if current direction is north and desired direction is east then
 *						we turn the robot right by 90 degrees using turn_robot('R') and updates current direction to east
 *					if current direction is north and desired direction is south then
 *						if current column is 0 then we turn the robot right by 90 degrees using turn_robot('R') two times
 *						else we turn the robot left by 90 degrees using turn_robot('L') two times
 *						and update current direction to south
 *					if current direction is south and desired direction is north then
 *						if current column is 0 then we turn the robot left by 90 degrees using turn_robot('L') two times
 *						else we turn the robot right by 90 degrees using turn_robot('R') two times
 *						and update current direction to north
 *					if current direction is south and desired direction is east then
 *						we turn the robot left by 90 degrees using turn_robot('L') and updates current direction to east
 *					if current direction is south and desired direction is west then
 *						we turn the robot right by 90 degrees using turn_robot('R') and updates current direction to west
 *					if current direction is east and desired direction is north then
 *						we turn the robot left by 90 degrees using turn_robot('L') and updates current direction to north	
 *					if current direction is east and desired direction is west then
 *						if current row is 0 then we turn the robot right by 90 degrees using turn_robot('R') two times
 *						else we turn the robot left by 90 degrees using turn_robot('L') two times
 *						and update current direction to west
 *					if current direction is east and desired direction is south then
 *						we turn the robot right by 90 degrees using turn_robot('R') and updates current direction to south
 *					if current direction is west and desired direction is north then
 *						we turn the robot right by 90 degrees using turn_robot('R') and updates current direction to north
 *					if current direction is west and desired direction is east then
 *						if current row is 0 then we turn the robot left by 90 degrees using turn_robot('L') two times
 *						else we turn the robot right by 90 degrees using turn_robot('R') two times
 *						and update current direction to east
 *					if current direction is west and desired direction is south then
 *						we turn the robot left by 90 degrees using turn_robot('L') and updates current direction to south
 * Example Call:	change_direction('N')
 */
void change_direction (unsigned char desired_direction) {
	if (current_direction == desired_direction) return;
	
	if (current_direction == 'N' && desired_direction == 'W') { // north
		turn_robot('L');
		current_direction = 'W';
	} else if (current_direction == 'N' && desired_direction == 'E') { // north
		turn_robot('R');
		current_direction = 'E';
	} else if (current_direction == 'N' && desired_direction == 'S') { // north
		if (current_coordinate[1] == 0) {
			turn_robot('R');
			turn_robot('R');
		} else {
			turn_robot('L');
			turn_robot('L');
		}

		current_direction = 'S';
	} else if (current_direction == 'S' && desired_direction == 'N') { //south
		if (current_coordinate[1] == 0) {
			turn_robot('L');
			turn_robot('L');
		} else {
			turn_robot('R');
			turn_robot('R');
		}
		
		current_direction = 'N';
	} else if (current_direction == 'S' && desired_direction == 'E') { //south
		turn_robot('L');
		current_direction = 'E';
	} else if (current_direction == 'S' && desired_direction == 'W') { //south
		turn_robot('R');
		current_direction = 'W';
	} else if (current_direction == 'E' && desired_direction == 'N') { //east
		turn_robot('L');
		current_direction = 'N';
	} else if (current_direction == 'E' && desired_direction == 'W') { //east
		if (current_coordinate[0] == 0) {
			turn_robot('R');
			turn_robot('R');	
		} else {
			turn_robot('L');
			turn_robot('L');
		}
		
		current_direction = 'W';
	} else if (current_direction == 'E' && desired_direction == 'S') { //east
		turn_robot('R');	
		current_direction = 'S';
	} else if (current_direction == 'W' && desired_direction == 'N') { //west
		turn_robot('R');	
		current_direction = 'N';
	} else if (current_direction == 'W' && desired_direction == 'E') { //west
		if (current_coordinate[0] == 0) {
			turn_robot('L');
			turn_robot('L');
		} else {
			turn_robot('R');	
			turn_robot('R');
		}
		
		current_direction = 'E';
	} else if (current_direction == 'W' && desired_direction == 'S') { //west
		turn_robot('L');
		current_direction = 'S';
	}
}

// It follows a black line until encounter a 3x3 cm black square.

/*
 * Function Name:	move_one_cell
 * Input:			Reads global variables Left_white_line, Center_white_line, Right_white_line, BNW_Thresh		
 * Output:			It moved the robot 1 cell in forward direction i.e. robot forwards until get a 3x3 black square
 * Logic:			If right and center white line sensors are on black surface and left white line sensor is on white
 *						surface, we also detect it as a 3x3cm black square.
 *					If left and center white line sensors are on the black surface and right white line sensor is on
 *						white surface, we detect it as a 3x3cm black square.
 *					Again, if all three white line sensors are on black surface, we also detect it as a 3x3cm black square.
 *					After all, forward the robot 5 cm to make the wheels on the 3x3cm black squares.					
 * Example Call:	move_one_cell()
 */
void move_one_cell () {
	read_wl_sensor_values();

	while (!(((Left_white_line > BNW_Thresh) && (Center_white_line > BNW_Thresh)) || ((Center_white_line > BNW_Thresh) && (Right_white_line > BNW_Thresh)) // 1-2 or 3-2 on white
			|| ((Left_white_line > BNW_Thresh) && (Center_white_line > BNW_Thresh) && (Right_white_line > BNW_Thresh)))) { // center on black
		read_wl_sensor_values();
		follow_black_line('F');
	}
		
	// this delay is important: if disabled, the bot may get our of line
	_delay_ms(200);	// delay
	
	// adjust 10/2 cm forward to make the wheels on the 3x3cm black squares.
	follow_black_line_mm(50, 'F');
}

// It is a helper function, used to match columns.

/*
 * Function Name:	match_column
 * Input:			target_coordinate is a int array which holds a co-ordinate i.e. row, column number
 * Output:			It updates current co-ordinate's column number i.e. the global variable current_coordinate[1]
 * Logic:			Until current co-ordinate's column > target co-ordinates's column, we repeat the process indented below
 *						change direction to west by using change_direction('W') and then move the robot by one cell 
 *						forward by using move_one_cell(). After that we update current co-ordinate's column number by decreasing 1
 *					All after above, Until current co-ordinate's column < target co-ordinates's column, we repeat the process indented below
 *						change direction to east by using change_direction('E') and then move the robot by one cell
 *						forward by using move_one_cell(). After that we update current co-ordinate's column number by increase 1
 * Example Call:	match_column((int){1, 3})
 */
void match_column (int target_coordinate[]) {
	while (current_coordinate[1] > target_coordinate[1]) {
		change_direction('W');
		move_one_cell();
		current_coordinate[1]--;
	}
		
	while (current_coordinate[1] < target_coordinate[1]) {// go east/west until both position on same column
		change_direction('E');
		move_one_cell();
		current_coordinate[1]++;
	}
}

// It is a helper function, used to match rows.

/*
 * Function Name:	match_row
 * Input:			target_coordinate is a int array which holds a co-ordinate i.e. row, column number
 * Output:			It updates current co-ordinate's row number i.e. the global variable current_coordinate[0]
 * Logic:			Until current co-ordinate's row > target co-ordinates's row, we repeat the process indented below
 *						change direction to north by using change_direction('N') and then move the robot by one cell 
 *						forward by using move_one_cell(). After that we update current co-ordinate's column number by decreasing 1
 *					All after above, Until current co-ordinate's row < target co-ordinates's row, we repeat the process indented below
 *						change direction to south by using change_direction('S') and then move the robot by one cell
 *						forward by using move_one_cell(). After that we update current co-ordinate's column number by increase 1
 * Example Call:	match_row((int){1, 3})
 */
void match_row (int target_coordinate[]) {
	while (current_coordinate[0] > target_coordinate[0]) {// go north/south until both position on same row
		change_direction('N');
		move_one_cell();
		current_coordinate[0]--;
	}
	
	while (current_coordinate[0] < target_coordinate[0]) {// go north/south until both position on same row
		change_direction('S');
		move_one_cell();
		current_coordinate[0]++;
	}
}

// It is used to move the robot to a particular coordinate.
// constraint: when we are in D1, then we cannot put co-ordinate of D2 and vice-versa. To do that, first cross the bridge.

/*
 * Function Name:	go_to_coordinate
 * Input:			target_coordinate is a int array which holds a co-ordinate i.e. row, column number
 * Output:			It moves the robot to a specific co-ordinate by using match_column and match_row functions
 * Logic:			If the robot is currently in D1 then
 *						If current direction is east or west then
 *							move the robot and match robot's and target co-ordinate's column
 *							move the robot and match robot's and target co-ordinate's row
 *						else
 *							move the robot and match robot's and target co-ordinate's row
 *							move the robot and match robot's and target co-ordinate's column
 *					else
 *						If target co-ordinate's row number is 2 and column number is 0 then
 *							move the robot and match robot's and target co-ordinate's row
 *							move the robot and match robot's and target co-ordinate's column
 *						else
 *							move the robot and match robot's and target co-ordinate's column
 *							move the robot and match robot's and target co-ordinate's row
 * Example Call:	go_to_coordinate((int){1, 3})
 */
void go_to_coordinate (int target_coordinate[]) {
	if (current_grid == 1) {
		if (current_direction == 'E' || current_direction == 'W') { // match column then row
			match_column(target_coordinate);
			match_row(target_coordinate);		
		} else { // current_direction = N/S  // match row then column
			match_row(target_coordinate);
			match_column(target_coordinate);		
		}		
	} else { // if robot is in d2, and target is bridge point, match row, then column
		if (target_coordinate[0] == 2 && target_coordinate[1] == 0) {
			match_row(target_coordinate);
			match_column(target_coordinate);			
		} else {
			match_column(target_coordinate);
			match_row(target_coordinate);
		}
	}
}

// It is used to go to nearest co-ordinate point of a cell from current point's co-ordinate.

/*
 * Function Name:	go_to_cell_no
 * Input:			target_division - is an int variable which can be 1 or 2 as D1 or D2 respectively,
 *					target_cell_no - is an int variable which can be 0-11 in D1 and 0-23 in D2
 * Output:			It moves the robot to nearest among 4 points (top-left, top-right, bottom-right, bottom-left)
 *					of a given cell number with target division
 * Logic:			It creates an integer pointer to store a co-ordinate. Then if the target division is 1 i.e. D1
 *					then copy returned data from get_nearest_point(current_coordinate, d1_position_map[target_cell_no])
 *						to nearest_point. Note that we are using d1_position_map here
 *					else copy returned data from get_nearest_point(current_coordinate, d2_position_map[target_cell_no])
 *						to nearest_point. Note that we are using d2_position_map here this time			
 * Example Call:	go_to_cell_no(2, 5)
 */
void go_to_cell_no (int target_division, int target_cell_no) {
	int * nearest_point = (int *) malloc(2 * sizeof(int));
	
	if (target_division == 1) { // go to cell no in D1
		memcpy(nearest_point, get_nearest_point(current_coordinate, d1_position_map[target_cell_no]), 2 * sizeof(int));
	} else { // go to cell no in D2	
		memcpy(nearest_point, get_nearest_point(current_coordinate, d2_position_map[target_cell_no]), 2 * sizeof(int));
	}
	
	go_to_coordinate(nearest_point);
	
	// after reaching, update current_cell_no
	current_cell_no = target_cell_no;
}

// It is used to get the pickup direction i.e. L or R i.e. left or right respectively.

/*
 * Function Name:	get_pickup_direction
 * Input:			Global variable current_direction.
 * Output:			It updates the global variable pickup_direction which can be 'L' or 'R' as left or right respectively
 * Logic:			Integer left and right is initialized with 0
 *					if current direction of the robot is north then
 *						i = 0
 *						if i < 4 then
 *							if current co-ordinate's column number > d1_position_map[current_cell_no][i][1]	then left = left + 1
 *							else current co-ordinate's column number < d1_position_map[current_cell_no][i][1] then right = right + 1
 *							i = i + 1
 *					if current direction of the robot is south then
 *						i = 0
 *						if i < 4 then
 *							if current co-ordinate's column number > d1_position_map[current_cell_no][i][1]	then right = right + 1
 *							else current co-ordinate's column number < d1_position_map[current_cell_no][i][1] then left = left + 1
 *							i = i + 1
 *					if current direction of the robot is east then
 *						i = 0
 *						if i < 4 then
 *							if current co-ordinate's row number > d1_position_map[current_cell_no][i][0]	then left = left + 1
 *							else current co-ordinate's row number < d1_position_map[current_cell_no][i][0] then right = right + 1
 *							i = i + 1
 *					if current direction of the robot is west then
 *						i = 0
 *						if i < 4 then
 *							if current co-ordinate's row number > d1_position_map[current_cell_no][i][0] then right = right + 1
 *							else current co-ordinate's row number < d1_position_map[current_cell_no][i][0] then left = left + 1
 *							i = i + 1
 *					if left > right then set pickup direction to 'L' as left
 *					else set pickup direction to 'R' as right
 * Example Call:	get_pickup_direction()
 */
void get_pickup_direction () {
	int i, left = 0, right = 0;	
	
	if (current_direction == 'N') { // if north, just compare columns
		for (i=0; i<4; i++) {
			if (current_coordinate[1] > d1_position_map[current_cell_no][i][1]) left++;
			else if (current_coordinate[1] < d1_position_map[current_cell_no][i][1]) right++;
		}
	} else if (current_direction == 'S') { // if south, just compare columns
		for (i=0; i<4; i++) {
			if (current_coordinate[1] > d1_position_map[current_cell_no][i][1]) right++;
			else if (current_coordinate[1] < d1_position_map[current_cell_no][i][1]) left++;
		}		
	} else if (current_direction == 'E') { // if east, just compare rows
		for (i=0; i<4; i++) {
			if (current_coordinate[0] > d1_position_map[current_cell_no][i][0]) left++;
			else if (current_coordinate[0] < d1_position_map[current_cell_no][i][0]) right++;
		}		
	} else { // if west, just compare rows
		for (i=0; i<4; i++) {
			if (current_coordinate[0] > d1_position_map[current_cell_no][i][0]) right++;
			else if (current_coordinate[0] < d1_position_map[current_cell_no][i][0]) left++;
		}		
	}
	
	if (left > right) pickup_direction = 'L';
	else pickup_direction = 'R';
}

// It is used to pickup a number from D1.

/*
 * Function Name:	pickup
 * Input:			num - is an integer number which can be 0-9 present in D1,
 *					skip_over - is an integer number which can be 0 or 1. It decides move forward of backward after pickup.
 * Output:			It picks up a number from D1 by forwarding the robot half of the cell and then pickup by 
 *					glowing left or right RGB LEDs and showing the number in the GLCD and after pickup the 
 *					robot forward and skip over the cell or backwards and returns to previous node as per travel cost
 * Logic:			Then we forward the robot by 50 mm to get to the center of the cell before pickup a number
 *					the we calculate pickup direction.
 *					If pickup direction is left, then turn on left RGB LED
 *					Else, turn on right RGB LED
 *					After that clear GLCD screen and display the number we have picked up and delay for 500 ms
 *					Then, if skip_over is equals to 1 then call move_one_cell()
 *					else follow black line forward for 25 mm by follow_black_line_mm(25, 'B')
 * Example Call:	pickup(9, 1)
 */
void pickup (int num, int skip_over) {
	follow_black_line_mm(50, 'F');
	
	get_pickup_direction();
	
	// turn on the left/right led
	if (pickup_direction == 'L') left_led_on();
	else right_led_on();
	
	GLCD_Clear();
	GLCD_DisplayBitmap(num); // show the num in big ubuntu font
	
	_delay_ms(500);
	
	if (skip_over == 1) {
		move_one_cell();
	} else {
		follow_black_line_mm(25, 'B');
	}
}

// It is used to deposit a number in D2.

/*
 * Function Name:	deposit
 * Input:			completed - is an integer variable which can be 0 or 1. It determines if a number in D2 is completed or not,
 *					isEnd - is an integer variable which can be 0 or 1. It determines if robot has ended the whole task or not.
 * Output:			It deposits the number that picked from D1 to a cell in D2 by maintaining pickup and deposit direction equal.
 *					While depositing a number, the robot turns the RGB LED off, shows a deposit message on GLCD and a buzzes for
 *					for 1000ms if a number in D2 is completed, and continuous buzzer if whole task is completed.
 * Logic:			If current_coordinate is on top left of the cell then
 *						If pickup direction is left then change robot's direction to south
 *						else change robot's direction to east
 *					If current_coordinate is on top right of the cell then
 *						If pickup direction is left then change robot's direction to west
 *						else change robot's direction to south	
 *					If current_coordinate is on bottom right of the cell then
 *						If pickup direction is left then change robot's direction to north
 *						else change robot's direction to west		
 *					If current_coordinate is on bottom left of the cell then
 *						If pickup direction is left then change robot's direction to east
 *						else change robot's direction to north
 *					After that, follow black line for 50 mm to get to the center of the cell
 *					If pickup direction is left, then turn off the left RGB LED, else turn off the right RGB LED
 *					Clear the GLCD and show the deposit message
 *					If completed is equal to 1 and isEnd is equal to 0 then turn on buzzer for 1000 ms
 *					else, if completed is equal to 1 and isEnd is equal to 1 turn on buzzer for lifetime
 *					else delay for 1000 ms
 *					After all these steps, clear the GLCD
 *					If isEnd is equal to 0 then follow black line in backward direction for 25 mm
 * Example Call:	deposit(1, 0)
 */
void deposit (int completed, int isEnd) {	
	if ((current_coordinate[0] == d2_position_map[current_cell_no][0][0]) && // current_coordinate is on top left of the cell
	(current_coordinate[1] == d2_position_map[current_cell_no][0][1])) {
		if (pickup_direction == 'L') change_direction('S');
		else change_direction('E');
	} else if ((current_coordinate[0] == d2_position_map[current_cell_no][1][0]) && // current_coordinate is on top right of the cell
	(current_coordinate[1] == d2_position_map[current_cell_no][1][1])) {
		if (pickup_direction == 'L') change_direction('W');
		else change_direction('S');
	} else if ((current_coordinate[0] == d2_position_map[current_cell_no][2][0]) && // current_coordinate is on bottom right of the cell
	(current_coordinate[1] == d2_position_map[current_cell_no][2][1])) {
		if (pickup_direction == 'L') change_direction('N');
		else change_direction('W');
	} else if ((current_coordinate[0] == d2_position_map[current_cell_no][3][0]) && // current_coordinate is on bottom left of the cell
	(current_coordinate[1] == d2_position_map[current_cell_no][3][1])) {
		if (pickup_direction == 'L') change_direction('E');
		else change_direction('N');
	}
	
	follow_black_line_mm(50, 'F');	
	
	//turn off led
	if (pickup_direction == 'L') left_led_off();
	else right_led_off();
	
	GLCD_Clear();
	GLCD_DisplayBitmap(-1); // show DEPOSIT message in big font
	
	if (completed == 1 && isEnd == 0) { // 1000ms buzzer if number in D2 completed but task is not
		buzzer_on();
		_delay_ms(1000);
		buzzer_off();
	} else if (completed == 1 && isEnd == 1) { // the robot has finished the task
		// continuous buzzer
		buzzer_on();
		while(1);
	} else {
		_delay_ms(1000);
	}
	GLCD_Clear();
	
	if (isEnd == 0) { // the robot has not finished the task yet
		follow_black_line_mm(25, 'B');
	}
}

// my functions and variables end ##########################################################################

//Main Function start ######################################################################################
/*
 * Function Name:	main
 * Input:			input_str - is a string which is a global variable. It contains comma separated numbers.
 * Output:			int to inform the caller that the program exited correctly or incorrectly (C code standard)
 * Logic:			At first, we initialize devices. Then we make the robot busy until pressing interrupt key.
 *					If interrupt key is pressed then initialize integer array path_points -1 set to all elements.
 *					Then we convert the comma separated string input_str and store into integer array path_points.
 *					Set current_velocity = 135 and synchronize wheel speed if needed and set synchronized velocity.
 *					Now, follow from the SRART position, follow black line for 50 mm to skip thick black line of START.
 *					Then move one cell and update robots current grid to 1, direction to north, and position 
 *					coordinate to [3, 2]. After that start traversal on the grids. Now set variable j = 0,
 *					and sum = 0 initially. Now iterate through i in range from 0 to 99, follow the indented steps below:
 *						If i th element in path_points is not equal to -1, then do the following indented steps below:
 *							If j is even, then do the following indented steps below:
 *								sum = sum + (i+1) th element in path_points. (target cell is in D1 i.e. pickup operation)
 *								if current_grid is 1 i.e. robot is already in D1, then do the following indented steps below:
 *									Go to the cell no which is the i th element of path_points.
 *									if current direction is east of (current direction is north and current row is 3) then:
 *										Pickup i+1 th element of path_points i.e. number in D1 and skip over the cell.
 *										If current direction is north, then update current row = current row - 1
 *										If current direction is east, then update current column = current column + 1
 *									Else, pickup i+1 th element of path_points i.e. number in D1 and move back to previous node.
 *								Else, do as the following indention:
 *									Move to bridge point (2, 0) in D2.
 *									Go to west one cell.
 *									Update current grid to 1 i.e. D1 and current coordinate = (2, 4) (D1 bridge point)
 *									Update current coordinate to (2, 4).
 *									Go to cell no in D1 which is i th element of path_points.
 *									Pickup i+1 th element of path_points i.e. number in D1 and move back to previous node.
 *							Else (j is odd i.e. position is in D2), as as the following indention:
 *								Move to bridge point (2, 4) in D1.
 *								Go east one cell.
 *								Update current_grid=2 and current_coordinate=(2, 0) (D2 bridge point).
 *								Update current_coordinate to (2, 0).
 *								Go to cell no in D2 which is i th element of path_points.
 *								If sum is equal to i+1 th element of path_points, then do as following indention:
 *									If i+2 th element of path_points is equal to -1 then do as following indention:
 *										Deposit as number in D2 is completed and whole task is completed.
 *									Else, do as following indention:
 *										Deposit as number in D2 is completed but whole task is not completed.
 *									sum = 0
 *								Else, deposit as number in D2 is not completed and whole task is not completed as well.
 *								If i+2 th element in path_points is -1, then continuous buzzer as robot has finished the task.
 * Example Call:	Called automatically by the Operating System
 */
int main() {
	init_devices();
	
	// make the robot busy until detecting boot switch i.e. interrupt is pressed
	while (1) {
		if((PINE | 0x7F) == 0x7F) { // interrupt switch is pressed
			//GLCD_Clear();
			break;
		}
	}
	
	/*************************** converting input string to int array start ****************************/
	//	it must be done after pressing the interrupt key else data will be lost
	int i, j;
	int path_points[100]; // -1 refers to invalid/null point; odd index = the point is in D1, even index = the point is in D2
	for (i=0; i<100; i++) path_points[i] = -1;
	
    char * token;
	token = strtok(input_str, ",");
	i = 0;
	while (token != '\0') {
		path_points[i++] = atoi((char *)token);
		token = strtok('\0', ",");
	}
	/*************************** converting input string to int array end ******************************/
	
	// set velocity
	current_velocity = 135; // 127 on full charge
	
	// synchronize wheels
	// suppose left wheel is physically 7.18% slower than the right wheel, so increase velocity of left wheel
	left_velocity_float = current_velocity + current_velocity * 0/100.0; // replace 0 with patch
	right_velocity_float = current_velocity + current_velocity * 0/100.0; // replace 0 with patch
	left_velocity = (unsigned char) left_velocity_float;
	right_velocity = (unsigned char) right_velocity_float;
	
	// go to 9th cell from start
	velocity(left_velocity, right_velocity);
	
	follow_black_line_mm(50, 'F');
	move_one_cell();
	_delay_ms(500);
	current_grid = 1;
	current_direction = 'N';
	current_coordinate[0] = 3;
	current_coordinate[1] = 2;

	
	// start traversal	
	//	iterate through all positions
	//	to get value, add 1 to i, e.g.: path_points[i+1]
	//	use j to count iteration number, even value of j = position is in D1, odd value of j = position is in D2
	j = 0; // iteration counter
	int sum = 0; // used to track when a number in D2 get completed
	
	for (i=0; i<100; i+=2) {
		if (path_points[i] != -1) {			
			if (j%2 == 0) { // j is even i.e. position is in D1
				// target cell is in D1 i.e. pickup operation
				sum += path_points[i+1]; // add D1 number's value to sum
				
				if (current_grid == 1) { // if robot is already in D1
					go_to_cell_no(1, path_points[i]);
					
					// decides just pickup ad back or pickup and forward
					if (current_direction == 'E' || (current_direction == 'N' && current_coordinate[0] == 3)) {
						pickup(path_points[i+1], 1);
						
						if (current_direction == 'N') current_coordinate[0]--; // current row = current row - 1
						if (current_direction == 'E') current_coordinate[1]++; // current column = current column + 1
					} else {
						pickup(path_points[i+1], 0);
					}		
				} else { // robot is in D2, need to cross the bridge to go to D1
					// 1. move to bridge point (2, 0) in D2
					go_to_coordinate((int[]){2, 0});
					
					// 2. go to west one cell
					change_direction('W');
					move_one_cell();
					
					// 3. update current_grid=1 and current_coordinate=(2, 4) (D1 bridge point)
					current_grid = 1;
					// update current_coordinate to (2, 4)
					current_coordinate[0] = 2; 
					current_coordinate[1] = 4;
					
					// 4. go_to_cell_no()
					go_to_cell_no(1, path_points[i]);
					
					// 5. pickup
					pickup(path_points[i+1], 0);
				}
			} else { // j is odd i.e. position is in D2
				// target cell is in D2 i.e. deposit operation
				
				// 1. move to bridge point (2, 4) in D1
				go_to_coordinate((int[]){2, 4});
				
				// 2. go east one cell
				change_direction('E');
				move_one_cell();
				
				// 3. update current_grid=2 and current_coordinate=(2, 0) (D2 bridge point)
				current_grid = 2;
				// update current_coordinate to (2, 0)
				current_coordinate[0] = 2;
				current_coordinate[1] = 0;
				
				// 4. go_to_cell_no()
				go_to_cell_no(2, path_points[i]);
				
				// 5. deposit
				if (sum == path_points[i+1]) {
					if (path_points[i+2] == -1) {
						deposit(1, 1); // 1 = number in D2 is completed, 1 = task completed respectively
					} else {
						deposit(1, 0); // 1 = number in D2 is completed, 0 = task not completed respectively
					}
					sum = 0;
				} else {
					deposit(0, 0); // 0 = number in D2 is not completed, 0 = task not completed respectively
				}
				
				// it should be deleted before video submission/final
				if (path_points[i+2] == -1) {
					// continuous buzzer on when finished the task
					buzzer_on();
					while(1);
				}
			}
			
			j++;
		}
	}
}
//Main Function end ########################################################################################