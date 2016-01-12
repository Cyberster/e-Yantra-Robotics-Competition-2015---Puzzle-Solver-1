/************************************************************************************
 LCD Connections:
 			  LCD	  Microcontroller Pins
 			  RS  --> PC0
			  RW  --> PC1
			  EN  --> PC2
			  DB7 --> PC7
			  DB6 --> PC6
			  DB5 --> PC5
			  DB4 --> PC4

 ADC Connection:
 			  ACD CH.	PORT	Sensor
			  0			PF0		Battery Voltage
			  1			PF1		White line sensor 3
			  2			PF2		White line sensor 2
			  3			PF3		White line sensor 1
			  4			PF4		IR Proximity analog sensor 1*****
			  5			PF5		IR Proximity analog sensor 2*****
			  6			PF6		IR Proximity analog sensor 3*****
			  7			PF7		IR Proximity analog sensor 4*****
			  8			PK0		IR Proximity analog sensor 5
			  9			PK1		Sharp IR range sensor 1
			  10		PK2		Sharp IR range sensor 2
			  11		PK3		Sharp IR range sensor 3
			  12		PK4		Sharp IR range sensor 4
			  13		PK5		Sharp IR range sensor 5
			  14		PK6		Servo Pod 1
			  15		PK7		Servo Pod 2

 ***** For using Analog IR proximity (1, 2, 3 and 4) sensors short the jumper J2. 
 	   To use JTAG via expansion slot of the microcontroller socket remove these jumpers.  
 
 Motion control Connection:
 			L-1---->PA0;		L-2---->PA1;
   			R-1---->PA2;		R-2---->PA3;
   			PL3 (OC5A) ----> PWM left; 	PL4 (OC5B) ----> PWM right; 
 
 ****************************************************************************
*********************************************************************************/

#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <math.h> //included to support power function
#include "lcd.h"

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

unsigned char data; //to store received data from UDR1
unsigned char input_str[100] = ""; // stores the raw input string
unsigned char D1[12] = ""; // to store D1 array
unsigned char D2[8] = ""; // to store D2 array

//MOSFET switch port configuration
void MOSFET_switch_config (void)
{
	DDRH = DDRH | 0x0C; //make PORTH 3 and PORTH 1 pins as output
	PORTH = PORTH & 0xF3; //set PORTH 3 and PORTH 1 pins to 0

	DDRG = DDRG | 0x04; //make PORTG 2 pin as output
	PORTG = PORTG & 0xFB; //set PORTG 2 pin to 0
}

void turn_on_sharp234_wl (void) //turn on Sharp IR range sensors 2, 3, 4 and white line sensor's red LED
{
	PORTG = PORTG & 0xFB;
}

void turn_off_sharp234_wl (void) //turn off Sharp IR range sensors 2, 3, 4 and white line sensor's red LED
{
	PORTG = PORTG | 0x04;
}

void turn_on_sharp15 (void) //turn on Sharp IR range sensors 1,5
{
	PORTH = PORTH & 0xFB;
}

void turn_off_sharp15 (void) //turn off Sharp IR range sensors 1,5
{
	PORTH = PORTH | 0x04;
}

void turn_on_ir_proxi_sensors (void) //turn on IR Proximity sensors
{
	PORTH = PORTH & 0xF7;
}

void turn_off_ir_proxi_sensors (void) //turn off IR Proximity sensors
{
	PORTH = PORTH | 0x08;
}

void turn_on_all_proxy_sensors (void) // turn on Sharp 2, 3, 4, red LED of the white line sensors
// Sharp 1, 5 and IR proximity sensor
{
	PORTH = PORTH & 0xF3; //set PORTH 3 and PORTH 1 pins to 0
	PORTG = PORTG & 0xFB; //set PORTG 2 pin to 0
}

void turn_off_all_proxy_sensors (void) // turn off Sharp 2, 3, 4, red LED of the white line sensors
// Sharp 1, 5 and IR proximity sensor
{
	PORTH = PORTH | 0x0C; //set PORTH 3 and PORTH 1 pins to 1
	PORTG = PORTG | 0x04; //set PORTG 2 pin to 1
}

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

//Function to Initialize PORTS
void port_init()
{
	MOSFET_switch_config();
	lcd_port_config();
	adc_pin_config();
	motion_pin_config();	
	left_encoder_pin_config(); //left encoder pin config
	right_encoder_pin_config(); //right encoder pin config
	buzzer_pin_config();
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

int counter=0;
SIGNAL(SIG_USART2_RECV) {		// ISR for receive complete interrupt
	data = UDR2; 				//making copy of data from UDR2 in 'data' variable
	UDR2 = data; 				//echo data back to PC

	//if (counter < 12) strcat(D1, &data);
	//else strcat(D2, &data);
	strcat(input_str, &data);

	//lcd_wr_command(0x01);
	//lcd_cursor(1, 1);
	//lcd_string(data);
	//lcd_wr_char(data);
	counter++;
}

void init_devices (void)
{
 	cli(); //Clears the global interrupts
	port_init();
	lcd_set_4bit();
	lcd_init();
	uart2_init(); //Initialize UART2 for serial communication
	adc_init();
	timer5_init();
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	sei();   //Enables the global interrupts
}

// my functions and variables start ########################################################################
// mapping cell no as array index with row-column of grids
int d1_position_map[12][2] = {
	{1, 1},	{1, 2},	{1, 3},	{1, 4},
	{2, 1},	{2, 2},	{2, 3},	{2, 4},
	{3, 1},	{3, 2},	{3, 3},	{3, 4}
};
int d2_position_map[24][2] = {
	{1, 1},	{1, 2},	{1, 3},	{1, 4},	{1, 5},	{1, 6},
	{2, 1},	{2, 2},	{2, 3},	{2, 4},	{2, 5},	{2, 6},
	{3, 1},	{3, 2},	{3, 3},	{3, 4},	{3, 5},	{3, 6},
	{4, 1},	{4, 2},	{4, 3},	{4, 4},	{4, 5},	{4, 6}
};

unsigned char current_velocity = 100;	// default velocity 100
unsigned char current_direction = 'N';	// E/W/N/S
unsigned char pickup_direction = 'M';	// initially M
int current_grid = 1;					// 1, 2 i.e. D1, D2
int current_cell_no = -1;				// initially a invalid one
int target_cell_no = 9;					// initially 9
int job_operation = 1;					// 1=pickup, 2=deposit


void follow_black_line (unsigned char Left_white_line, unsigned char Center_white_line, unsigned char Right_white_line) {
	flag=0;
	
	// left wheel is physically 7.18% slower than the right wheel, so increase velocity
	float left_velocity_float = current_velocity + current_velocity * 12/100.0;
	float right_velocity_float = current_velocity;
	unsigned char left_velocity = (unsigned char) left_velocity_float;
	unsigned char right_velocity = (unsigned char) right_velocity_float;
	
		
	if (((Left_white_line <= 16) && (Center_white_line <= 16) && (Right_white_line <= 16) && flag == 0) || (Center_white_line > 16)) {
		flag=1;
		forward();
		velocity(left_velocity, right_velocity);
	}

	if((Left_white_line <= 16) && (Center_white_line <= 16) && (Right_white_line > 16) && (flag == 0)) {
		flag=1;
		forward();
		velocity(left_velocity+30, right_velocity-50);
	}

	if((Left_white_line > 16) && (Center_white_line <= 16) && (Right_white_line <= 16) && (flag == 0)) {
		flag=1;
		forward();
		velocity(left_velocity-50, right_velocity+30);
	}
}

void turn_left () {
	left_degrees(30); // rotate 30 degree to skip current line
	Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor

	while (Center_white_line < 16) {
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		left_degrees(5);
	}
}

void turn_right () {
	right_degrees(30); // rotate 30 degree to skip current line
	Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
	
	while (Center_white_line < 16) {
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		right_degrees(5);
	}
}

void change_direction (unsigned char desired_direction) {
	if (current_direction == desired_direction) return;
	
	if (current_direction == 'N' && desired_direction == 'W') { // north
		//left_degrees(90);
		turn_left();
		current_direction = 'W';
	} else if (current_direction == 'N' && desired_direction == 'E') { // north
		//right_degrees(90);
		turn_right();
		current_direction = 'E';
	} else if (current_direction == 'N' && desired_direction == 'S') { // north
		//right_degrees(180);
		turn_right();
		turn_right();
		current_direction = 'S';
	} else if (current_direction == 'S' && desired_direction == 'N') { //south
		//left_degrees(180);
		turn_left();
		turn_left();
		current_direction = 'N';
	} else if (current_direction == 'S' && desired_direction == 'E') { //south
		//left_degrees(90);
		turn_left();
		current_direction = 'E';
	} else if (current_direction == 'S' && desired_direction == 'W') { //south
		//right_degrees(90);
		turn_right();
		current_direction = 'W';
	} else if (current_direction == 'E' && desired_direction == 'N') { //east
		//left_degrees(90);
		turn_left();
		current_direction = 'N';
	} else if (current_direction == 'E' && desired_direction == 'W') { //east
		//left_degrees(180);
		turn_left();
		turn_left();
		current_direction = 'W';
	} else if (current_direction == 'E' && desired_direction == 'S') { //east
		//right_degrees(90);
		turn_right();
		current_direction = 'S';
	} else if (current_direction == 'W' && desired_direction == 'N') { //west
		//right_degrees(90);
		turn_right();
		current_direction = 'N';
	} else if (current_direction == 'W' && desired_direction == 'E') { //west
		//left_degrees(180);
		turn_left();
		turn_left();
		current_direction = 'E';
	} else if (current_direction == 'W' && desired_direction == 'S') { //west
		//left_degrees(90);
		turn_left();
		current_direction = 'S';
	}
}

void move_one_cell (unsigned int is_rotated) {
	Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
	Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
	Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
	
	if (is_rotated == 0) {
		// forward until detecting 1cm black line if there is no previous rotation
		while (!((Left_white_line < 16) && (Center_white_line > 16) && (Right_white_line < 16))) { // center on black		
			Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
			Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
			Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
		
			print_sensor(1,1,3);	//Prints value of White Line Sensor1
			print_sensor(1,5,2);	//Prints Value of White Line Sensor2
			print_sensor(1,9,1);	//Prints Value of White Line Sensor3		
		
			follow_black_line(Left_white_line, Center_white_line, Right_white_line);
		}		
	}
			
	buzzer_on();
	_delay_ms(50);		//delay
	buzzer_off();
	_delay_ms(50);		//delay
		
	// forward until detecting next 3x3 black box
	//while (!((Left_white_line > 20) && (Center_white_line > 20) && (Right_white_line > 20))) { // all on black
	while (!(((Left_white_line > 16) && (Center_white_line > 16)) || ((Center_white_line > 16) && (Right_white_line > 16)) // 1-2 or 3-2 on white
			|| ((Left_white_line > 16) && (Center_white_line > 16) && (Right_white_line < 16)))) { // center on black
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
		
		print_sensor(1,1,3);	//Prints value of White Line Sensor1
		print_sensor(1,5,2);	//Prints Value of White Line Sensor2
		print_sensor(1,9,1);	//Prints Value of White Line Sensor3
		
		follow_black_line(Left_white_line, Center_white_line, Right_white_line);
	}
		
	buzzer_on();
	_delay_ms(200);		//delay
	buzzer_off();
	_delay_ms(200);		//delay
	
	stop();
	_delay_ms(500);
	forward_mm(50); // adjust 11 cm forward
}

void go_to_cell_no (int target_division, int target_cell_no) {
	if (target_division == 1) { // go to cell no in D1
		while (d1_position_map[current_cell_no][0] > d1_position_map[target_cell_no][0]) {// go north/south until both position on same row
			change_direction('N');
			_delay_ms(500);
			move_one_cell(1);
			_delay_ms(500);
			current_cell_no -= 4; // 8, 4, 0; 9, 5, 1; ...
			// move one cell and update robot's status
		}
	
		while (d1_position_map[current_cell_no][0] < d1_position_map[target_cell_no][0]) {// go north/south until both position on same row
			change_direction('S');
			_delay_ms(500);
			move_one_cell(1);
			_delay_ms(500);
			current_cell_no += 4; // 8, 4, 0; 9, 5, 1; ...
			// move one cell and update robot's status
		}
	
		while (d1_position_map[current_cell_no][1] > d1_position_map[target_cell_no][1]) {// go east/west until both position on same column
			change_direction('W');
			_delay_ms(500);
			move_one_cell(1);
			_delay_ms(500);
			current_cell_no--; // 1, 2, 3; 4, 5, 6; ...
			// move one cell and update robot's status
		}
	
		while (d1_position_map[current_cell_no][1] < d1_position_map[target_cell_no][1]) {// go east/west until both position on same column
			change_direction('E');
			_delay_ms(500);
			move_one_cell(1);
			_delay_ms(500);
			current_cell_no++; // 1, 2, 3; 4, 5, 6; ...
			// move one cell and update robot's status
		}
	} else { // go to cell no in D2
		while (d2_position_map[current_cell_no][0] > d2_position_map[target_cell_no][0]) {// go north/south until both position on same row
			change_direction('N');
			_delay_ms(500);
			move_one_cell(1);
			_delay_ms(500);
			current_cell_no -= 4; // 8, 4, 0; 9, 5, 1; ...
			// move one cell and update robot's status
		}
	
		while (d2_position_map[current_cell_no][0] < d2_position_map[target_cell_no][0]) {// go north/south until both position on same row
			change_direction('S');
			_delay_ms(500);
			move_one_cell(1);
			_delay_ms(500);
			current_cell_no += 4; // 8, 4, 0; 9, 5, 1; ...
			// move one cell and update robot's status
		}
	
		while (d2_position_map[current_cell_no][1] > d2_position_map[target_cell_no][1]) {// go east/west until both position on same column
			change_direction('W');
			_delay_ms(500);
			move_one_cell(1);
			_delay_ms(500);
			current_cell_no--; // 1, 2, 3; 4, 5, 6; ...
			// move one cell and update robot's status
		}
	
		while (d2_position_map[current_cell_no][1] < d2_position_map[target_cell_no][1]) {// go east/west until both position on same column
			change_direction('E');
			_delay_ms(500);
			move_one_cell(1);
			_delay_ms(500);
			current_cell_no++; // 1, 2, 3; 4, 5, 6; ...
			// move one cell and update robot's status
		}
	}

	
	buzzer_on();
	_delay_ms(100);
	buzzer_off();
	_delay_ms(100);
	buzzer_on();
	_delay_ms(100);
	buzzer_off();
	_delay_ms(100);
}

void pickup () {
	change_direction('N');
	_delay_ms(500);
	forward_mm(50);
	pickup_direction = current_direction;
	_delay_ms(1000);
	
	// turn on the left/right led
	// do stuff
	
	_delay_ms(1000);	
	back_mm(50);
}

// my functions and variables end ##########################################################################

// helper/debugger functions start #########################################################################
void print_str_to_pc (char str[]) {
	int i;
	UDR2 = 0x0D;
	for (i=0; i<strlen(str); i++) {
		UDR2 = (unsigned char) str[i];
		_delay_ms(10);
	}
}

void print_int_to_pc (int num) {
	int i;	
	unsigned char str[10];
	snprintf(str, 10, "%d", num);
	
	UDR2 = 0x0D;
	for (i=0; i<strlen(str); i++) {
		UDR2 = (unsigned char) str[i];
		_delay_ms(10);
	}
}
// helper/debugger functions end ###########################################################################

//Main Function
int main() {
	init_devices();
	
	// save battery by turning off all unnecessary sensors
	turn_off_ir_proxi_sensors();
	turn_off_sharp15();
	
	// make the robot busy until detecting boot switch i.e. interrupt is pressed
	while (1) {
		if((PINE | 0x7F) == 0x7F) { // interrupt switch is pressed
			break;
		}
	}
	
	/*************************** converting input string to int array start ****************************/
    unsigned int i, j, k, flag;
    unsigned char solutions_str[4][50]; // stores operands for each number of D2 temporarily as string
    unsigned int no_of_solutions = 0;
    unsigned char * token;
    int solutions[4][15] = {-1}; // solutions will be stored in this format: 2,10,2,7,5,3;6,12,0,9,6,3;22,14,1,8,3,6

    for(i=0; i<4; i++) {
		for (j=0; j<15; j++) {
			solutions[i][j] = -1;
		}
    }

	// converting solutions (string) to integer and store to 2D array solutions[4][15]
	i = 0;
	token = strtok(input_str, ";");
	while (token != '\0') {	
		strcpy(solutions_str[i++], token);
		token = strtok('\0', ";");
		no_of_solutions++;
	}

	for (i=0; i<no_of_solutions; i++) {
		j = 0;
		token = strtok(solutions_str[i], ",");
        while (token != '\0') {
	        //print_str_to_pc(token);
	        //print_int_to_pc(atoi(token));
			solutions[i][j++] = atoi(token);
			token = strtok('\0', ",");
        }
	}
	/*************************** converting input string to int array end ******************************/
	
	//go_to_cell_no(1, 4);
	//pickup();
	
	
	//change_direction('S');
	//lcd_cursor(2, 1);
	//lcd_wr_char(current_direction);
	//turn_left();
	//turn_right();
	
	//UDR2 = solutions_str[0][1];
	/*for (i=0; i<4; i++) {
		//UDR2 = solutions_str[i][1];
		UDR2 = i + '0';
		_delay_ms(1);
	}*/
	
	/*for (i=0; i<no_of_solutions; i++) {
		for (j=2; j<15; j+=2) {
			if (solutions[i][j] != -1)
				print_int_to_pc(solutions[i][j]);
		}		
	}*/
	
	// generating paths from input data
	// following are the data format
	// data stored as position, value sequence
	// received data: 2,10,2,7,5,3;6,12,0,9,6,3;22,14,1,8,3,6,7,5 
	// will become => 2,7,2,10,5,3,2,10,0,9,6,12,6,3,6,12,1,8,22,14,3,6,22,14 as traversal path of the robot
	int path_points[100]; // -1 refers to invalid/null point; odd index = the point is in D1, even index = the point is in D2
	for (i=0; i<100; i++) path_points[i] = -1;
	k = 0;
	
	for (i=0; i<no_of_solutions; i++) {
		for (j=2; j<15; j+=2) {
			if (solutions[i][j] != -1) {
				path_points[k++] = solutions[i][j];
				path_points[k++] = solutions[i][j+1];
				path_points[k++] = solutions[i][0];
				path_points[k++] = solutions[i][1];
			}
		}
	}
	
	/*for (i=0; i<100; i++) {
		if (path_points[i] != -1)
			print_int_to_pc(path_points[i]);
	}*/
	
	// go to 9th cell from start
	//move_one_cell(0);
	//_delay_ms(500);
	//current_cell_no = 9;

	//go_to_cell_no(1, 2);
	//pickup();
	
	// start traversal	
	//	iterate through all positions
	//	to get value, add 1 to i, e.g.: path_points[i+1]
	//	use j to count iteration number, even value of j = position is in D1, odd value of j = position is in D2
	j = 0; // iteration counter
	
	for (i=0; i<100; i+=2) {
		if (path_points[i] != -1) {
			print_int_to_pc(path_points[i]);
			
			/*if (j%2 == 0) { // j is even i.e. position is in D1
				// target cell is in D1 i.e. pickup operation
				if (current_grid == 1) { // if robot is already in D1
					go_to_cell_no(1, path_points[i]);
					pickup();				
				} else { // robot is in D2, need to cross the bridge to go to D1
					// 1. move to bridge point 6 in D2
					go_to_cell_no(2, 6);
					
					// 2. go west two cells
					change_direction('W');
					move_one_cell(0);
					move_one_cell(0);
					
					// 3. update current_grid=1 and current_cell_no=7 (D1 bridge point)
					current_grid = 1;
					current_cell_no = 7;
					
					// 4. go_to_cell_no()
					go_to_cell_no(1, path_points[i]);
					
					// 5. pickup
					pickup();
					// turn on left LED, show number path_points[i+1] on GLCD
				}			
				
				print_int_to_pc(path_points[i]);
			} else { // j is odd i.e. position is in D2
				// target cell is in D2 i.e. deposit operation
				
				// 1. move to bridge point 7 in D1
				go_to_cell_no(1, 7);
				
				// 2. go east two cells
				change_direction('E');
				move_one_cell(0);
				move_one_cell(0);
				
				// 3. update current_grid=2 and current_cell_no=6 (D2 bridge point)
				current_grid = 2;
				current_cell_no = 6;
				
				// 4. go_to_cell_no()
				go_to_cell_no(2, path_points[i]);
				
				// 5. deposit
				// deposit();
				// turn off left LED, show Deposited on GLCD
				// 1000ms buzzer on completion of each solution
			
				print_int_to_pc(path_points[i]);
			}*/
			
			j++;
		}
	}
	
	while(1) {
		// continuous buzzer on finished the task
		
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor

		print_sensor(1,1,3);	//Prints value of White Line Sensor1
		print_sensor(1,5,2);	//Prints Value of White Line Sensor2
		print_sensor(1,9,1);	//Prints Value of White Line Sensor3
		
		/*lcd_cursor(1, 1);
		lcd_string(input_str);
				
		lcd_print(2, 1, solutions[0][1], 2);
		lcd_print(2, 4, solutions[1][1], 2);
		lcd_print(2, 7, solutions[2][1], 2);
		
		lcd_print(2, 10, no_of_solutions, 2);*/
		
		
	}
}
