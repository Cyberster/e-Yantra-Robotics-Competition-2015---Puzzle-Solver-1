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
 
 LCD Display interpretation:
 ****************************************************************************
 *LEFT WL SENSOR	CENTER WL SENSOR	RIGHT WL SENSOR		BLANK			*
 *BLANK				BLANK				BLANK				BLANK			*
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

unsigned char current_direction = 'N';	// E/W/N/S
unsigned char pickup_direction = 'M';	// initially M
unsigned char current_velocity = 0;		// initially 0
int current_grid = 1;					// 1, 2 i.e. D1, D2
int current_cell_no = -1;				// initially a invalid one
int target_cell_no = 9;					// initially 9
int job_operation = 1;					// 1=pickup, 2=deposit


void follow_black_line (unsigned char Left_white_line, unsigned char Center_white_line, unsigned char Right_white_line) {
	flag=0;
	
	// left wheel is physically 7.18% slower than the right wheel, so increase velocity
	float left_velocity_float = current_velocity + current_velocity * 10.18/100;
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
	
	current_velocity = 100;
	
	if (is_rotated == 0) {
		// forward until detecting 1cm black line
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
	while (!(((Left_white_line > 20) && (Center_white_line > 20)) || ((Center_white_line > 20) && (Right_white_line > 20))
			|| ((Left_white_line > 20) && (Center_white_line > 20) && (Right_white_line < 20)))) { // center on black
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

void go_to_cell_no (int target_cell_no) {
	while (d1_position_map[current_cell_no][0] > d1_position_map[target_cell_no][0]) {// go north/south until both position on same row
		change_direction('N');
		move_one_cell(1);
		current_cell_no -= 4; // 8, 4, 0; 9, 5, 1; ...
		// move one cell and update robot's status
	}
	
	while (d1_position_map[current_cell_no][0] < d1_position_map[target_cell_no][0]) {// go north/south until both position on same row
		change_direction('S');
		move_one_cell(1);
		current_cell_no += 4; // 8, 4, 0; 9, 5, 1; ...
		// move one cell and update robot's status
	}
	
	while (d1_position_map[current_cell_no][1] > d1_position_map[target_cell_no][1]) {// go east/west until both position on same column
		change_direction('W');
		move_one_cell(1);
		current_cell_no--; // 1, 2, 3; 4, 5, 6; ...
		// move one cell and update robot's status
	}
	
	while (d1_position_map[current_cell_no][1] < d1_position_map[target_cell_no][1]) {// go east/west until both position on same column
		change_direction('E');
		move_one_cell(1);
		current_cell_no++; // 1, 2, 3; 4, 5, 6; ...
		// move one cell and update robot's status
	}
	
	buzzer_on();
	_delay_ms(1000);
	buzzer_off();
	_delay_ms(1000);
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

// my functions and variables end ##########################################################################

//Main Function
int main() {
	init_devices();
	
	/*************************** converting input string to int array start ****************************/
    int i, j, k, flag;
    unsigned char * D1_str;
    unsigned char * D2_str;
    unsigned char * solutions_str;
    int D1[12], D2[8], solutions[4][13] = {-1};
	unsigned int is_rotated;

    for(i=0; i<4; i++) {
		for (j=0; j<13; j++) {
			solutions[i][j] = -1;
		}
    }

    printf("%s\n", input_str);

	// splitting input_str to three different strings
	const char delimiter[2] = "#";
	char *token;

	token = strtok(input_str, delimiter);
	D1_str = (char *) malloc(sizeof(char) * strlen(token));
	strcpy(D1_str, token);
	token = strtok('\0', delimiter);
	D2_str = (char *) malloc(sizeof(char) * strlen(token));
	strcpy(D2_str, token);
	token = strtok('\0', delimiter);
	solutions_str = (char *) malloc(sizeof(char) * strlen(token));
	strcpy(solutions_str, token);

	printf("\nD1_str: %s", D1_str);
	printf("\nD2_str: %s", D2_str);
	printf("\nsolutions: %s", solutions_str);

	// converting D1_str (string) to D1 (integer)
    for (i=0; i<12; i++) {
		char temp[2];
		strcpy(temp, &D1_str[i]);
		temp[1] = '\0';
        D1[i] = atoi(temp);
    }

    // converting D2_str (string) to D2 (integer)
    token = strtok(D2_str, ",");
	int D2_size = 0;
    while (token != '\0') {
		D2[D2_size++] = atoi(token);
		token = strtok('\0', ",");
    }
	printf("\n");

	// converting solutions (string) to integer array
	// 10,7,3;12,9,3;14,8,6
	char solutions_temp[4][50]; // stores operands for each number of D2 temporarily as string

	token = strtok(solutions_str, ";");
    i = 0;
    int no_of_solutions = 0;
	while (token != '\0') {
		strcpy(solutions_temp[i++], token);
		token = strtok('\0', ";");
		no_of_solutions++;
	}

	for (i=0; i<no_of_solutions; i++) {
		j = 0;
		token = strtok(solutions_temp[i], ",");
        while (token != '\0') {
			solutions[i][j++] = atoi(token);
			token = strtok('\0', ",");
        }
	}
	/*************************** converting input string to int array end ******************************/
	
	// make the robot busy until detecting boot switch i.e. interrupt is pressed
	while (1) {
		if((PINE | 0x7F) == 0x7F) { // interrupt switch is pressed
			break;
		}
	}	
	
	move_one_cell(0); // i.e. go to 9th cell from start
	_delay_ms(500);
	current_cell_no = 9;
	
	
	
	
	go_to_cell_no(0);
	go_to_cell_no(3);
	go_to_cell_no(11);
	go_to_cell_no(8);
	
	//change_direction('S');
	//lcd_cursor(2, 1);
	//lcd_wr_char(current_direction);
	//turn_left();
	//turn_right();
	
	/*for (int i=0; i<4; i++) {
		move_one_cell(); // i.e. go to 9th cell from start
		_delay_ms(500);
		//change_direction('E');
		turn_left();
		//turn_right();
	}*/
	

	
	while(1) {
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor

		print_sensor(1,1,3);	//Prints value of White Line Sensor1
		print_sensor(1,5,2);	//Prints Value of White Line Sensor2
		print_sensor(1,9,1);	//Prints Value of White Line Sensor3
		
		/*buzzer_on();
		_delay_ms(250);		//delay
		buzzer_off();
		_delay_ms(250);		//delay*/
		
		lcd_cursor(2, 1);
		//lcd_string(input_str);	
		lcd_print(2, 1, D1[0], 2);
		lcd_print(2, 4, D1[1], 2);
		lcd_print(2, 7, D1[2], 2);
		
		/*move_one_cell(is_rotated); // i.e. go to 9th cell from start
		_delay_ms(500);
		//change_direction('E');
		turn_left();
		is_rotated = 1;
		//turn_right();*/
		
		/*if((PINE | 0x7F) == 0x7F) { //switch is pressed
			buzzer_on(); //Turn on buzzer
			PORTJ = 0xFF; //Turn on bargraph LEDs
		}  else {
			buzzer_off(); //Turn off buzzer
			PORTJ = 0x00; //Turn off bargraph LEDs
		}*/
	}
}
