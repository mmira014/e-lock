/*
 * mmira014_customproject.c
 *
 * Created: 3/5/2018 10:07:23 AM
 * Author : Marcos M
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <scheduler.h>
#include <timer.h>
#include <usart.h>
#include <nokia5110_chars.h>
//#include <nokia5110.h>
#include "nokia5110.c"

char status_str[] = "UNARMED";
unsigned char lock_str = 0;
unsigned char action = 0;
enum Status_state{unarmed, armed} status;
enum Servo_state{servo_idle, servo_lock, servo_unlock} servo;

enum Bluetooth_States{Bluetooth_Init, Bluetooth_Wait, Bluetooth_Arm, Bluetooth_Disarm};
int Bluetooth_tick(int Bluetooth_State)
{
	switch(Bluetooth_State)
	{
		case Bluetooth_Init:
			Bluetooth_State = Bluetooth_Wait;
			break;
			
		case Bluetooth_Wait:
			initUSART(0);
			unsigned char signal = 0;
			signal = USART_Receive(0) & 0x03;
			if(signal == 2) // button is green (armed)
			{
				Bluetooth_State = Bluetooth_Arm;
			}
			else if(signal == 1) // button is red (not armed)
			{
				Bluetooth_State = Bluetooth_Disarm;
			}
			else 
			{
				Bluetooth_State = Bluetooth_Wait;
			}
			USART_Flush(0);
			break;
			
		case Bluetooth_Arm:
			Bluetooth_State = Bluetooth_Wait;
			break;
			
		case Bluetooth_Disarm:
			Bluetooth_State = Bluetooth_Wait;
			break;
			
		default:
			Bluetooth_State = Bluetooth_Init;
			break;
	}
	switch (Bluetooth_State)
	{
		case Bluetooth_Init:
			break;
		
		case Bluetooth_Wait:
			action = 0;
			break;
			
		case Bluetooth_Arm:
			action = 1;
			status = armed;
			break;
			
		case Bluetooth_Disarm:
			action = 1;
			status = unarmed;
			break;
			
		default:
			break;
	}
	return Bluetooth_State;
}


enum Servo_States{Servo_Init, Servo_Wait, Servo_Lock, Servo_Unlock};
int Servo_tick(int Servo_State)
{
	switch (Servo_State)
	{
		case Servo_Init:
			Servo_State = Servo_Wait;
			break;
			
		case Servo_Wait: // check if new action identified, then execute appropriate state change
			if(action && status == armed) 
			{
				Servo_State = Servo_Lock;
			}
			else if(action && status == unarmed)
			{
				Servo_State = Servo_Unlock;
			}
			else // no new action
			{
				Servo_State = Servo_Wait;
			}
			break;
			
		case Servo_Lock: 
			Servo_State = Servo_Wait;	
			break;
			
		case Servo_Unlock:
			Servo_State = Servo_Wait;
			break;
			
		default:
			Servo_State = Servo_Init;
			break;
	}
	switch (Servo_State)
	{
		case Servo_Init:
			break;
			
		case Servo_Wait:
			servo = servo_idle;
			break;
			
		case Servo_Lock:
			servo = servo_lock;
			break;
			
		case Servo_Unlock:
			servo = servo_unlock;
			break;
			
		default:
			break;
	}
	return Servo_State;
}

enum LCD_States{LCD_Init, LCD_SetString};
int LCD_tick(int LCD_State)
{
	switch (LCD_State)
	{
		case LCD_Init:
			LCD_State = LCD_SetString;
			break;
		
		case LCD_SetString: 
			LCD_State = LCD_SetString;
			break;
			
		default:
			LCD_State = LCD_Init;
			break;
	}
	switch (LCD_State)
	{
		case LCD_Init:
			break;
			
		case LCD_SetString:
			if(status == armed)
			{
				lock_str = 1;
				strcpy(status_str, " ARMED");
			}
			else
			{
				lock_str = 0;
				strcpy(status_str, "UNARMED");
			}
			break;
			
		default:
			break;
	}
	return LCD_State;
}

enum Combine_States{Combine_Init, Combine_Execute};
int Combine_tick(int Combine_State)
{
	switch (Combine_State)
	{
		case Combine_Init:
			Combine_State = Combine_Execute;
			break;
			
		case Combine_Execute:
			Combine_State = Combine_Execute;
			break;
			
		default:
			Combine_State = Combine_Init;
			break;
	}
	switch (Combine_State)
	{
		case Combine_Init:	
			break;
			
		case Combine_Execute:
			// display string
			nokia_lcd_init();
			nokia_lcd_clear();
			
			nokia_lcd_write_lock(lock_str);
			nokia_lcd_set_cursor(17,40);
			nokia_lcd_write_string(status_str, 1);
			nokia_lcd_render();
			
			// do servo stuff
			switch (servo)
			{
				case servo_idle:
					break;
					
				/*
				* To use Waveform Generation Mode 14 (fast PWM), the following must be set:
					WGM13 = 1
					WGM12 = 1
					WGM11 = 1
					WGM10 = 0
				
				
				* WGM11, 10 are in TCCR1A bits 1, 0 respectively
				* we also need to set output mode to inverted by setting COM1A1, COM1A0 = 1 (located in TCCR1A)
					* TCCR1A |= 1 << WGM11 | 1 << COM1A1 | 1 << COM1A0;
	
				* WGM13, 12 are in TCCR1B bits 4, 3 respectively
				* We also need to set CS10 to 1 for prescaler of 1 (CS10 is bit 0 of TCCR1B)
					* TCCR1B |= 1 << WGM12 | 1 << WGM13 | 1 << CS10;
				
				* servo is connected to OC1A => pin 19 on Atmega1284
				*/	
				case servo_lock:
					//rotate counter clockwise to lock position
					TimerOff();
					TCCR1A |= 1<<WGM11 | 1<<COM1A1 | 1<<COM1A0;
					TCCR1B |= 1<<WGM13 | 1<<WGM12 | 1<<CS10;
					ICR1 = 19999;
					OCR1A = 16500;
					_delay_ms(3000);
					TimerOn();
					break;
					
				case servo_unlock:
					//rotate clockwise to unlocked position
					TimerOff();
					TCCR1A |= 1<<WGM11 | 1<<COM1A1 | 1<<COM1A0;
					TCCR1B |= 1<<WGM13 | 1<<WGM12 | 1<<CS10;
					ICR1 = 19999;
					OCR1A = 6000;
					_delay_ms(3000);
					TimerOn();
					break;
					
				default:
					break;
			}
			break;
		
		default:
			break;
	}
	return Combine_State;
}

unsigned long Bluetooth_period = 200;
unsigned long Servo_period = 100;
unsigned long LCD_period = 100;
unsigned long Combine_period  = 100;
unsigned long tasksGCD_period = 100; 
	
int main(void)
{
	// Servo on PD5, bluetooth module on PD0, PD1
	// nokia lcd on PORTB
	DDRD = 0xF0; PORTD = 0x00;
	DDRB = 0xFF; PORTB = 0x00;
	nokia_lcd_init();
	initUSART(0);
	status = unarmed;
	
	TimerSet(tasksGCD_period);
	TimerOn();
	
    unsigned char tasksNum = 4;
    unsigned char i = 0;
    task tasks[tasksNum];
    // tasks are { bluetooth, lcd, servo, combine }
    tasks[i].state = Bluetooth_Init;
	tasks[i].period = Bluetooth_period;
	tasks[i].elapsedTime = tasks[i].period;
	tasks[i].TickFct = &Bluetooth_tick;
	++i;
	tasks[i].state = LCD_Init;
	tasks[i].period = LCD_period;
	tasks[i].elapsedTime = tasks[i].period;
	tasks[i].TickFct = &LCD_tick;
	++i;
	tasks[i].state = Servo_Init;
	tasks[i].period = Servo_period;
	tasks[i].elapsedTime = tasks[i].period;
	tasks[i].TickFct = &Servo_tick;
	++i;
	tasks[i].state = Combine_Init;
	tasks[i].period = Combine_period;
	tasks[i].elapsedTime = tasks[i].period;
	tasks[i].TickFct = &Combine_tick;
	
    while (1) 
    {
		for(i = 0; i < tasksNum; ++i)
		{
			if(tasks[i].elapsedTime >= tasks[i].period) 
			{
				tasks[i].state = tasks[i].TickFct(tasks[i].state);
				tasks[i].elapsedTime = 0; 
			}
			tasks[i].elapsedTime += tasksGCD_period;
		}
		while(!TimerFlag);
		TimerFlag = 0;
    }
}
