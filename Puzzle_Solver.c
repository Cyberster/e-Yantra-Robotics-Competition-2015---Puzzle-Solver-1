/*
* Team Id :			eYRCPlus-PS1#2678
* Author List :		Avick Dutta (team leader), Arpan Das, Subhendu Hazra, Asesh Basu
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
//		and out function definitions are written in K&R indent style
//		reference: https://en.wikipedia.org/wiki/Indent_style

#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h> //included to support power function
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

/*void equal_velocity_forward () {
	ShaftCountRight = 0;
	ShaftCountLeft = 0;
	
	forward();
	
	while (1) {
		while (ShaftCountLeft < ShaftCountRight) {
			soft_right();
		}
		while (ShaftCountLeft > ShaftCountRight) {
			soft_left();
		}
	}
}*/

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


// helper/debugger functions start #########################################################################
/** Prints a given string to terminal software in PC
 *
 * @param str is a char array.
 */
void print_str_to_pc (char str[]) {
	int i;
	UDR2 = 0x0D;
	for (i=0; i<strlen(str); i++) {
		UDR2 = (unsigned char) str[i];
		_delay_ms(10);
	}
}

/** Prints a given integer to terminal software in PC
 *
 * @param num is an int.
 */
void print_int_to_pc (int num) {
	int i;
	char str[10];
	snprintf(str, 10, "%d", num);
	
	UDR2 = 0x0D;
	for (i=0; i<strlen(str); i++) {
		UDR2 = (unsigned char) str[i];
		_delay_ms(10);
	}
}

/** It prints current co-ordinate and direction to LCD, and it can also pause execution
 *
 * @param id is an int.
 * @param pause is an int.
 */
void debug (int id, int pause) {
	lcd_print(2, 1, current_coordinate[0], 2);
	lcd_print(2, 4, current_coordinate[1], 2);
	lcd_cursor(2, 7);
	lcd_wr_char(current_direction);
	lcd_print(1, 14, id, 2);
	//GLCD_Clear();
	//GLCD_Printf("\n\n%2d", id);
	
	if (pause == 1) {
		// make the robot busy until detecting boot switch i.e. interrupt is pressed
		while (1) {
			if((PINE | 0x7F) == 0x7F) { // interrupt switch is pressed
				break;
			}
		}
	}
}
// helper/debugger functions end ###########################################################################

/*
 * Function Name:	left_led_on
 * Input :			
 *					
 * Output :			It turns on the left RGB LED
 * Logic:			PORT G0 is configured for left RGB LED, writing logic high to PG0
 *					
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
 * Input :			
 *					
 * Output :			It turns of the left RGB LED
 * Logic:			PORT G0 is configured for left RGB LED, writing logic low to PG0
 *					
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
 * Input :			
 *					
 * Output :			It turns on the right RGB LED
 * Logic:			PORT G1 is configured for right RGB LED, writing logic high to PG1
 *					
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
 * Input :			
 *					
 * Output :			It turns on the right RGB LED
 * Logic:			PORT G1 is configured for right RGB LED, writing logic low to PG1
 *					
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
 * Input :			current_point - int array containing row & column number of current coordinate point,
 *					target_point - int array containing row & column number of target coordinate point
 * Output :			total_cost - int which is the calculated cost between current and target coordinate point
 * Logic:			It calculates the cost of traveling between two points i.e. calculating row difference and  
 *					column difference and make sum of them
 * Example Call:	get_point_cost(current_point, target_point)
 */
int get_point_cost (int current_point[2], int target_point[2]) {
	int total_cost;	
	total_cost = abs(current_point[0] - target_point[0]) + abs(current_point[1] - target_point[1]);
	return total_cost;
}

/** It receives current point and target cell which contains 4 points
 *	and returns nearest point from current point among those 4 points
 *
 * @param current_point is an int *.
 * @param target_cell is an int **.
 * @returns nearest_point which is an int *.
 */

/*
 * Function Name:	get_nearest_point
 * Input :			current_point - int array containing row & column number of current coordinate point,
 *					target_cell - 2D int array contains target cell's 4 points with coordinate based on row & column
 * Output :			nearest_point - int array containing row & column number of nearest coordinate point among 4 points of target_cell
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

/** It follows a 1cm thick black line on white surface
 *
 * @param Left_white_line is an unsigned char.
 * @param Center_white_line is an unsigned char.
 * @param Right_white_line is an unsigned char.
 */

/*
 * Function Name:	read_wl_sensor_values
 * Input :			
 *					
 * Output :			It updates global variables Left_white_line, Center_white_line, Right_white_line
 * Logic:			It reads all 3 white line sensor values by using function ADC_Conversion()
 *					
 * Example Call:	read_wl_sensor_values()
 */
void read_wl_sensor_values () {
	Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
	Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
	Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
}

/*
 * Function Name:	follow_black_line
 * Input :			Left_white_line, Center_white_line, Right_white_line
 *					
 * Output :			It follows a 1cm thick black line on white surface
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
//void follow_black_line (unsigned char Left_white_line, unsigned char Center_white_line, unsigned char Right_white_line, char direction) {
	flag = 0;
		
	if (((Left_white_line <= BNW_Thresh) && (Center_white_line <= BNW_Thresh) && (Right_white_line <= BNW_Thresh)) || (Center_white_line > BNW_Thresh)) {
		flag=1;
		if (direction == 'F') forward();
		else back();
		velocity(left_velocity, right_velocity);
	}

	if((Left_white_line <= BNW_Thresh) && (Center_white_line <= BNW_Thresh) && (Right_white_line > BNW_Thresh) && (flag == 0)) {
		flag=1;
		if (direction == 'F') forward();
		else back();
		velocity(left_velocity+30, right_velocity-50);
	}

	if((Left_white_line > BNW_Thresh) && (Center_white_line <= BNW_Thresh) && (Right_white_line <= BNW_Thresh) && (flag == 0)) {
		flag=1;
		if (direction == 'F') forward();
		else back();
		velocity(left_velocity-50, right_velocity+30);
	}
	
	//GLCD_Clear();
	//GLCD_Printf("%3d %3d %3d ", Left_white_line, Center_white_line, Right_white_line);
}


/** It follows a black line up to a fixed distance
 *
 * @param DistanceInMM is an unsigned int.
 * @param direction is a char.
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
			//follow_black_line(Left_white_line, Center_white_line, Right_white_line, direction);
			follow_black_line(direction);
		}
	}
	
	stop(); //Stop robot	
}

/** It turns the robot left/right until encounters a black line.
 *
 * @param direction is a char.
 */
//void turn_robot (char direction, int digit_count) {
void turn_robot (char direction) {
	read_wl_sensor_values();
	velocity(current_velocity+15, current_velocity+15);
	
	if (direction == 'L') {
		left_degrees(75);
		//debug(1, 1);
		read_wl_sensor_values();
		while (Center_white_line <= BNW_Thresh) {
			read_wl_sensor_values();
			left_degrees(5);
			//_delay_ms(50);
		}
	} else {		
		right_degrees(75);
		//debug(2, 1);
		read_wl_sensor_values();
		while (Center_white_line <= BNW_Thresh) {
			read_wl_sensor_values();
			right_degrees(5);
			//_delay_ms(50);
		}
	}
	velocity(current_velocity-15 ,current_velocity-15);
	_delay_ms(500);
}

/** It changes the direction of the robot i.e. east/west/north/south.
 *
 * @param desired_direction is an unsigned char.
 */
void change_direction (unsigned char desired_direction) {
	if (current_direction == desired_direction) return;
	
	if (current_direction == 'N' && desired_direction == 'W') { // north
		//turn_left();
		turn_robot('L');
		current_direction = 'W';
	} else if (current_direction == 'N' && desired_direction == 'E') { // north
		//turn_right();
		turn_robot('R');
		current_direction = 'E';
	} else if (current_direction == 'N' && desired_direction == 'S') { // north
		if ((current_grid == 1 && current_coordinate[1] == 0) ||
			(current_grid == 2 && current_coordinate[1] == 0)) {
			turn_robot('R');
			turn_robot('R');
		} else {
			turn_robot('L');
			turn_robot('L');
		}

		current_direction = 'S';
	} else if (current_direction == 'S' && desired_direction == 'N') { //south
		if ((current_grid == 1 && current_coordinate[1] == 0) ||
			(current_grid == 2 && current_coordinate[1] == 0)) {
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
		if ((current_grid == 1 && current_coordinate[0] == 0) ||
			(current_grid == 2 && current_coordinate[0] == 0 )) {
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
		if ((current_grid == 1 && current_coordinate[0] == 0) ||
			(current_grid == 2 && current_coordinate[0] == 0 )) {
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

/** It follows a black line until encounter a 3x3 cm black square.
 *
 */
void move_one_cell () {
	Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
	Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
	Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
	
	/*if (current_cell_no == -1) { // if the robot is in start position
		// forward until detecting 1cm black line if there is no previous rotation
		//while (!((Left_white_line > 16) || (Center_white_line > 16) || (Right_white_line > 16))) { // center on black	
		while (!((Left_white_line <= 16) && (Center_white_line > 16) && (Right_white_line <= 16))) { // center on black
			Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
			Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
			Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
		
			print_sensor(1,1,3);	//Prints value of White Line Sensor1
			print_sensor(1,5,2);	//Prints Value of White Line Sensor2
			print_sensor(1,9,1);	//Prints Value of White Line Sensor3		
		
			follow_black_line(Left_white_line, Center_white_line, Right_white_line);
		}
		//velocity(current_velocity, current_velocity);
		//forward_mm(20);
		
		buzzer_on();
		_delay_ms(50);		//delay
		buzzer_off();
	}*/

	// forward until detecting next 3x3 black box
	//while (!((Left_white_line > 20) && (Center_white_line > 20) && (Right_white_line > 20))) { // all on black
	while (!(((Left_white_line > BNW_Thresh) && (Center_white_line > BNW_Thresh)) || ((Center_white_line > BNW_Thresh) && (Right_white_line > BNW_Thresh)) // 1-2 or 3-2 on white
			|| ((Left_white_line > BNW_Thresh) && (Center_white_line > BNW_Thresh) && (Right_white_line > BNW_Thresh)))) { // center on black
		read_wl_sensor_values();
		
		/*print_sensor(1,1,3);	//Prints value of White Line Sensor1
		print_sensor(1,5,2);	//Prints Value of White Line Sensor2
		print_sensor(1,9,1);	//Prints Value of White Line Sensor3*/
		//GLCD_Clear();
		//GLCD_Printf("%3d %3d %3d ", Left_white_line, Center_white_line, Right_white_line);
		
		//follow_black_line(Left_white_line, Center_white_line, Right_white_line, 'F');
		follow_black_line('F');
	}
		
	//buzzer_on();
	// this delay is important: if disabled, the bot may get our of line
	_delay_ms(200);		//delay
	//buzzer_off();
	
	//stop();
	//_delay_ms(500);
	
	// adjust 11 cm forward
	forward_mm(50);
	//follow_black_line_mm(Left_white_line, Center_white_line, Right_white_line, 50, 'F'); // old robot 110
}

/** It is a helper function, used to match columns
 *
 * @param target_coordinate is an int *.
 */	
void match_column (int target_coordinate[]) {
	while (current_coordinate[1] > target_coordinate[1]) {
		change_direction('W');
		move_one_cell();
		//_delay_ms(500);
		//current_cell_no--; // 1, 2, 3; 4, 5, 6; ...
		current_coordinate[1] = current_coordinate[1] - 1;
		//debug(1, 0);
		// move one cell and update robot's status			
	}
		
	while (current_coordinate[1] < target_coordinate[1]) {// go east/west until both position on same column
		change_direction('E');
		move_one_cell();
		//_delay_ms(500);
		//current_cell_no++; // 1, 2, 3; 4, 5, 6; ...
		current_coordinate[1] = current_coordinate[1] + 1;
		//debug(2, 0);
	}
}

/** It is a helper function, used to match rows
 *
 * @param target_coordinate is an int *.
 */	
void match_row (int target_coordinate[]) {
	while (current_coordinate[0] > target_coordinate[0]) {// go north/south until both position on same row
		change_direction('N');
		move_one_cell();
		//_delay_ms(500);
		//current_cell_no -= 4; // 8, 4, 0; 9, 5, 1; ...
		current_coordinate[0] = current_coordinate[0] - 1;
		//debug(3, 0);
		// move one cell and update robot's status
	}
	
	while (current_coordinate[0] < target_coordinate[0]) {// go north/south until both position on same row
		change_direction('S');
		move_one_cell();
		//_delay_ms(500);
		//current_cell_no += 4; // 8, 4, 0; 9, 5, 1; ...
		current_coordinate[0] = current_coordinate[0] + 1;
		//debug(4, 0);
		// move one cell and update robot's status
	}
}

/** It is used to move the robot to a particular coordinate
 *	constraint: when we are in D1, then we cannot put co-ordinate of D2 and vice-versa
 *	to do that, first cross the bridge
 *
 * @param target_coordinate is an int *.
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

/** It is used to go to nearest co-ordinate point of a cell from current point's co-ordinate
 *
 * @param target_division is an int.
 * @param target_cell_no is an int.
 */	
void go_to_cell_no (int target_division, int target_cell_no) {
	int * nearest_point;
	nearest_point = (int *) malloc(2 * sizeof(int));
	
	if (target_division == 1) { // go to cell no in D1
		memcpy(nearest_point, get_nearest_point(current_coordinate, d1_position_map[target_cell_no]), 2 * sizeof(int));
	} else { // go to cell no in D2	
		memcpy(nearest_point, get_nearest_point(current_coordinate, d2_position_map[target_cell_no]), 2 * sizeof(int));
	}
	
	go_to_coordinate(nearest_point);
	
	// after reaching, update current_cell_no
	current_cell_no = target_cell_no;

	/*buzzer_on();
	_delay_ms(100);
	buzzer_off();
	_delay_ms(100);
	buzzer_on();
	_delay_ms(100);
	buzzer_off();*/
}

/** It is used to get the pickup direction i.e. L or R i.e. left or right respectively
 *
 */
void get_pickup_direction () {
	// current_direction, current_cell_no, current_coordinate, current_grid
	//char direction;
	int i, left = 0, right = 0;	
	
	if (current_direction == 'N') { // if north, just compare columns
		for (i=0; i<4; i++) {
			//GLCD_Printf("@ %d~%d,", current_coordinate[1], d1_position_map[current_cell_no][i][1]);
			//GLCD_Printf("@%1d,%1d-%2d-%1d-%1d,", current_coordinate[0], current_coordinate[1], current_cell_no, i, 1);
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
	
	//GLCD_Printf(" #L: %d, R: %d", left, right);
	
	if (left > right) pickup_direction = 'L';
	else pickup_direction = 'R';
	
	//return direction;
}

/** It is used to pickup a number from D1
 *
 * @param num is an int. It the number to be picked.
 * @param skip_over is an int. It decides whether backward or skip over the cell after pickup a number
 */	
void pickup (int num, int skip_over) {
	Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
	Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
	Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
	//forward_mm(30);
	follow_black_line_mm(50, 'F');
	
	get_pickup_direction();
	//GLCD_Printf("#direction: %c", pickup_direction);
	
	// turn on the left/right led
	if (pickup_direction == 'L') left_led_on();
	else right_led_on();
	
	GLCD_Clear();
	//GLCD_Printf("%d", num);
	GLCD_DisplayBitmap(num); // show the num in big font
	
	_delay_ms(500);
	
	if (skip_over == 1) {
		move_one_cell();
	} else {
		//back_mm(30);
		follow_black_line_mm(25, 'B');
	}
}

/** It is used to deposit a number in D2
 *
 * @param completed is an int. It decides whether a number in D2 is completed or not. 1 = completed, 2 = not completed.
 * @param isEnd is an int. It decides whether the robot has finished the task. 1 = finished, 2 = not finished.
 */
void deposit (int completed, int isEnd) {
	Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
	Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
	Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
	
	// current_direction, current_cell_no, current_coordinate, current_grid, pickup_direction
	//GLCD_Clear();
	//GLCD_Printf("$%d~%d <> %d~%d$", current_coordinate[0], current_coordinate[1], d1_position_map[current_cell_no][1][0], d1_position_map[current_cell_no][1][1]);
	//GLCD_Printf("\ncrnt_cell:%d", current_cell_no);
	
	/*if ((current_coordinate[0] == d1_position_map[current_cell_no][0][0]) && // current_coordinate is on top left of the cell
		(current_coordinate[1] == d1_position_map[current_cell_no][0][1])) {
		debug(1, 0);
		if (pickup_direction == 'L') change_direction('S');
		else change_direction('E');
	} else if ((current_coordinate[0] == d1_position_map[current_cell_no][1][0]) && // current_coordinate is on top right of the cell
		(current_coordinate[1] == d1_position_map[current_cell_no][1][1])) {
		debug(2, 0);
		if (pickup_direction == 'L') change_direction('W');
		else change_direction('S');
	} else if ((current_coordinate[0] == d1_position_map[current_cell_no][2][0]) && // current_coordinate is on bottom right of the cell
		(current_coordinate[1] == d1_position_map[current_cell_no][2][1])) {
		debug(3, 0);
		if (pickup_direction == 'L') change_direction('N');
		else change_direction('W');
	} else if ((current_coordinate[0] == d1_position_map[current_cell_no][3][0]) && // current_coordinate is on bottom left of the cell
		(current_coordinate[1] == d1_position_map[current_cell_no][3][1])) {
		debug(4, 0);
		if (pickup_direction == 'L') change_direction('E');
		else change_direction('N');
	}*/
	
	if ((current_coordinate[0] == d2_position_map[current_cell_no][0][0]) && // current_coordinate is on top left of the cell
	(current_coordinate[1] == d2_position_map[current_cell_no][0][1])) {
		//debug(1);
		if (pickup_direction == 'L') change_direction('S');
		else change_direction('E');
	} else if ((current_coordinate[0] == d2_position_map[current_cell_no][1][0]) && // current_coordinate is on top right of the cell
	(current_coordinate[1] == d2_position_map[current_cell_no][1][1])) {
		//debug(2);
		if (pickup_direction == 'L') change_direction('W');
		else change_direction('S');
	} else if ((current_coordinate[0] == d2_position_map[current_cell_no][2][0]) && // current_coordinate is on bottom right of the cell
	(current_coordinate[1] == d2_position_map[current_cell_no][2][1])) {
		//debug(3);
		if (pickup_direction == 'L') change_direction('N');
		else change_direction('W');
	} else if ((current_coordinate[0] == d2_position_map[current_cell_no][3][0]) && // current_coordinate is on bottom left of the cell
	(current_coordinate[1] == d2_position_map[current_cell_no][3][1])) {
		//debug(4);
		if (pickup_direction == 'L') change_direction('E');
		else change_direction('N');
	}
	
	//forward_mm(30);
	follow_black_line_mm(50, 'F');	
	
	//turn off led
	if (pickup_direction == 'L') left_led_off();
	else right_led_off();
	
	GLCD_Clear();
	//GLCD_DisplayString("Deposit");
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
		//back_mm(30);
		follow_black_line_mm(25, 'B');
	}
}

// my functions and variables end ##########################################################################

//Main Function start ######################################################################################
int main() {
	init_devices();
	
	// make the robot busy until detecting boot switch i.e. interrupt is pressed
	while (1) {
		if((PINE | 0x7F) == 0x7F) { // interrupt switch is pressed
			GLCD_Clear();
			break;
		}
	}
	
	/*************************** converting input string to int array start ****************************/
	//	it must be done after pressing the interrupt key else data will be lost
	int i, j;
	int path_points[100]; // -1 refers to invalid/null point; odd index = the point is in D1, even index = the point is in D2
	for (i=0; i<100; i++) path_points[i] = -1;
	
	print_str_to_pc(input_str);
	
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
	// left wheel is physically 7.18% slower than the right wheel, so increase velocity
	left_velocity_float = current_velocity + current_velocity * 0/100.0; // 12 for the old robot
	right_velocity_float = current_velocity;
	left_velocity = (unsigned char) left_velocity_float;
	right_velocity = (unsigned char) right_velocity_float;
	
	// go to 9th cell from start
	velocity(left_velocity, right_velocity);
	
	//forward_mm(50);
	
	//turn_robot('R');
	
	//follow_black_line_mm(190*4, 'B'); // 20 cm = 185 mm
	
	follow_black_line_mm(50, 'F');
	//forward_mm(50);
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
						
						if (current_direction == 'N') current_coordinate[0]--;
						if (current_direction == 'E') current_coordinate[1]++;
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
					// update current_coordinate
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
				// update current_coordinate
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
					deposit(0, 0); // 0 = number in D2 is not completed
				}
				
				// it should be deleted before video submission/final
				if (path_points[i+2] != -1) {
					//GLCD_Printf("Gonna pick %d from cell#%d", path_points[i+3], path_points[i+2]);
				} else {
					// continuous buzzer on finished the task
					buzzer_on();
					while(1);
				}
				
			}
			
			j++;
		}
	}
}
//Main Function end ########################################################################################