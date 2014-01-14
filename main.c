#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdlib.h>
//#include "uart.h"
#include <avr/interrupt.h>
//#include "lcd.h"
#define F_CPU 16000000
//#define UART_BAUD_RATE      9600

#define sbi(x,y) x |= (1<<y) 			//set bit
#define cbi(x,y) x &= ~(1<<y) 		//clear bit

void InitPorts (void);
void InitPWM(void);						//initializes 8 bit PWM for the motors
void CalcError(void);
void LeftMF(void);						//left motor forward
void LeftMB(void);						//left motor backward
void LeftMS(void);						//left motor stop
void RightMB(void);						//right motor backward
void RightMS(void);						//right motor stop
void RightMF(void);						//left motor forward
void Motorsstop(void);
void Motorsback(void);
void ReachTable(void);
//void delay(float);

bool pause = false;
char lastreading = 'r';
unsigned int leftpulse, rightpulse,s1, s2, s3, s4, s5, s6, s7, s8, maxspeed = 205;
float error = 0, perror = 0;
//unsigned short int menu = 0;
unsigned short int basespeed = 205;
float P, I, D, correction,  Kp = 22 , Ki = 0.6,Kd = 3;
int c=0;
char a='f';
int table=0;
char buffer[10];
int x=0;
int menu;


int main(void)
{
	unsigned int p;
	InitPorts();
	InitPWM();	
	while(1)
	{
		PORTC=0b11111111;
		cbi(PORTC,0);
		cbi(PORTC,1);
		cbi(PORTC,2);
		cbi(PORTC,3);
		do
		{
			menu=PINC&0b11110000;
		}while(menu==0b11110000);
		
		PORTC=0b11111111;
		
		if(menu==16)
		{
			table=menu/16;
			x=table+1;
			ReachTable();
			sbi(PORTC,0);
			cbi(PORTC,1);
			cbi(PORTC,2);
			cbi(PORTC,3);
			_delay_ms(1000);
		}
		
		if(menu==32)
		{
			table=menu/16;
			x=table+1;
			ReachTable();
			sbi(PORTC,0);
			cbi(PORTC,1);
			cbi(PORTC,2);
			cbi(PORTC,3);
			_delay_ms(1000);
		}
		
		
		
		
	}
	
}

void ReachTable(void)
{
	c=0;
	while(c!=x)
	{		
		
		
			
		if(pause == false)					
		{					
			CalcError();			
			
			if((s1+s2+s3+s4+s5+s6+s7+s8) == 0)		//robot has overshot
			{
				if(lastreading == 'r')				//checks if the last sensor to the activated was right
				{
					RightMS();						//turn right at full speed
					LeftMF();
					OCR1A = maxspeed;
					OCR1B = maxspeed;	
				}
				
				else if(lastreading == 'l')			//checks if the last sensor to the activated was left
				{
					RightMF();						//turn left at full speed

					LeftMS();	
					OCR1A = maxspeed;
					OCR1B = maxspeed;					
				}				
			}
			else if((s1+s2+s3+s4+s5+s6+s7+s8) == 8)
			{
					c++;
					if(c>table)
					{
						RightMS();
						LeftMS();
						break;
					}
					else if(c==table)
					{
						LeftMF();
						RightMS();
						_delay_ms(2600);
					}
					else
					{
						RightMF();
						LeftMF();
						_delay_ms(1000);
					}
			}
			else  									//robot on line
			{	
				
				P = error * Kp;	
			
				I += error;
				I = I * Ki;
				
				D = error - perror;
				D = D*Kd;
				
				correction = P + I + D;
				
				rightpulse =  basespeed - correction;
				leftpulse =   basespeed + correction;		
				
				LeftMF();
				RightMF();
			
					
				if(leftpulse > 205)
				leftpulse = 205;
				
				if(leftpulse < 0)
				leftpulse = 0;
				
				if(rightpulse > 205)
				rightpulse = 205;

				if(rightpulse < 0)
				rightpulse = 0;

				OCR1A = rightpulse;
				OCR1B = leftpulse;		
			}
			
		}	
			
	}
		RightMS();
		LeftMS();
		_delay_ms(10);
		
}

void InitPorts(void) 						//initialize the pins 
{
	//initializes the pins connected to motor driver as output
	sbi(DDRD,5);						
	sbi(DDRB,2);
	sbi(DDRB,3);	
	sbi(DDRD,4);
	sbi(DDRB,0);
	sbi(DDRB,1);
	cbi(PORTD,5);
	cbi(PORTB,2);
	cbi(PORTB,3);	
	cbi(PORTD,4);
	cbi(PORTB,0);
	cbi(PORTB,1);
	
	//initiallizes the motor enable pins as output
	sbi(PORTD,4);	
	sbi(PORTD,5);
	sbi(DDRD,2);
	sbi(DDRD,3);
	sbi(PORTD,2);
	sbi(PORTD,3);
	
	DDRA = 0;    								//sets the port connected to sensors as input	
	PORTA = 255;
	sbi(DDRC,0);
	sbi(DDRC,1);
	sbi(DDRC,2);
	sbi(DDRC,3);
	cbi(DDRC,4);
	cbi(DDRC,5);
	cbi(DDRC,6);
	cbi(DDRC,7);
	PORTC=0b11111111;							//enable pull up	
}


void InitPWM(void)								//initialize PWM				
{	
	TCCR1A = 0b10100001;		
	TCCR1B = 0b00001010; 
		
}

void CalcError()								//calculates the current error
{				
	s1 = 0;
	s2 = 0;
	s3 = 0;
	s4 = 0;
	s5 = 0;
	s6 = 0;	
	s7 = 0; 
	s8 = 0;
	
	if((PINA&0b00000001)==(0b00000001))
	{
		lastreading = 'r';
		s8 = 1;	
	}

	if((PINA&0b00000010)==(0b00000010))
	{
		//lastreading = 'r';
		s7 = 1;
	}
	
	if((PINA&0b00000100)==(0b00000100))		
		s6 = 1;
		
	if((PINA&0b00001000)==(0b00001000))
		s5 = 1;
		
	if((PINA&0b00010000)==(0b00010000))		
		s4 = 1;
		
	if((PINA&0b00100000)==(0b00100000))	
		s3 = 1;
		
	if((PINA&0b01000000)==(0b01000000))
	{
		//lastreading = 'l';
		s2 = 1;
	}
	
	
	if((PINA&0b10000000)==(0b10000000))		
	{
		lastreading = 'l';
		s1 = 1;	
	}
	perror = error;
	
	//the following statements calculate the error
	
	error = (s1 * 1) + (s2 * 2) + (s3 * 3) + (s4 * 4) + (s5 * 5) + (s6 * 6) +(s7 * 7) + (s8*8);
	error = (error)/(s1+s2+s3+s4+s5+s6+s7+s8);	
	error = error - 4.5;
}

void LeftMB(void)	
{
	//sbi(PORTD,5);
	cbi(PORTB,2);
	sbi(PORTB,3);
}

void RightMF(void)
{
	//sbi(PORTD,4);
	cbi(PORTB,0);
	sbi(PORTB,1);
}
void LeftMF(void)
{
	//sbi(PORTD,5);
	sbi(PORTB,2);
	cbi(PORTB,3);
}

void RightMB(void)
{
	//sbi(PORTD,4);
	sbi(PORTB,0);
	cbi(PORTB,1);
}
void LeftMS(void)
{
	//sbi(PORTD,5);
	cbi(PORTB,2);
	cbi(PORTB,3);
}

void RightMS(void)
{
	//sbi(PORTD,4);
	cbi(PORTB,0);
	cbi(PORTB,1);
}