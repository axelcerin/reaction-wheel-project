/*
 * ATmega16 Interface with MPU-6050
 * http://www.electronicwings.com
 *
 */ 


#define BAUD 4800                 						/* Baud rate for serial communication */
#define F_CPU 1000000UL
#include <avr/io.h>										/* Include AVR std. library file */
#include <util/delay.h>									/* Include delay header file */
#include <inttypes.h>									/* Include integer type header file */
#include <stdlib.h>										/* Include standard library file */
#include <stdio.h>										/* Include standard library file */
#include "MPU6050_res_define.h"							/* Include MPU6050 register define file */
#include "I2C_Master_H_file.h"							/* Include I2C Master header file */
#include "USART_RS232_H_file.h"							/* Include USART header file */
#include <avr/interrupt.h>

volatile float /* Acc_x,Acc_y,Acc_z,Temperature, */ Gyro_x; // ,Gyro_y,Gyro_z;
volatile uint8_t data_ready = 0;  // Flag for data reading

ISR(TIMER0_COMPA_vect)
{
    // Read_RawValue();  // Read data from MPU6050 when interrupt occurs
	data_ready = 1;
    // // Optional: Process the gyro data (e.g., convert to real units)
    // Xg = Gyro_x / 16.4;

    // // // Convert Xg to string and send over USART
    // dtostrf(Xg, 3, 2, float_);
    // sprintf(buffer, " X = %s\n", float_);
    // USART_SendString(buffer);
	// char init_message[] = "init complete\n";  // Null-terminated string
	// USART_SendString(init_message);  
}

void timer_init(void)
{
    // Set Timer0 to CTC mode (Clear Timer on Compare Match)
    TCCR0A = 0;            // Normal mode (WGM01 and WGM00 both 0)
    TCCR0B = (1 << WGM02); // CTC mode (WGM02 = 1)

    // Set the compare match value (OCR0A) for the interrupt frequency
    OCR0A = 124;           // Example: Compare match value, 125 gives a 1ms interrupt with 1 MHz clock and prescaler 64

    // Set prescaler to 64 (CS01 = 1, CS00 = 1)
    TCCR0B |= (1 << CS02) |(0 << CS01) | (1 << CS00);

    // Enable the Timer0 compare match interrupt (Timer0 compare match A)
    TIMSK0 |= (1 << OCIE0A);

    // Enable global interrupts
	sei();
}

void MPU6050_Init()										/* Gyro initialization function */
{
	_delay_ms(150);										/* Power up time >100ms */
	I2C_Start_Wait(0xD0);								/* Start with device write address */
	I2C_Write(SMPLRT_DIV);								/* Write to sample rate register */
	I2C_Write(0x07);									/* 1KHz sample rate */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(PWR_MGMT_1);								/* Write to power management register */
	I2C_Write(0x01);									/* X axis gyroscope reference frequency */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(CONFIG);									/* Write to Configuration register */
	I2C_Write(0x00);									/* Fs = 8KHz */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(GYRO_CONFIG);								/* Write to Gyro configuration register */
	I2C_Write(0x18);									/* Full scale range +/- 2000 degree/C */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(INT_ENABLE);								/* Write to interrupt enable register */
	I2C_Write(0x01);
	I2C_Stop();
}

void MPU_Start_Loc()
{
	I2C_Start_Wait(0xD0);								/* I2C start with device write address */
	I2C_Write(ACCEL_XOUT_H);							/* Write start location address from where to read */ 
	I2C_Repeated_Start(0xD1);							/* I2C start with device read address */
}

void Read_RawValue()
{
	MPU_Start_Loc();									/* Read Gyro values */
	// Acc_x = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	// Acc_y = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	// Acc_z = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	// Temperature = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Gyro_x = (((int)I2C_Read_Ack_Safe()<<8) | (int)I2C_Read_Ack_Safe());
	// Gyro_y = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	// Gyro_z = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Nack());
	I2C_Stop();
}

int main()
{
	
	char buffer[20], float_[10];
	// float Xa,Ya,Za,t;
	float Xg=0; //,Yg=0,Zg=0;
	I2C_Init();											/* Initialize I2C */
	MPU6050_Init();										/* Initialize MPU6050 */
	USART_Init(BAUD);									/* Initialize USART with 9600 baud rate */
	char init_message[] = "init complete\n";  // Null-terminated string
	USART_SendString(init_message);         // Send the whole string at once
	
	timer_init();
	// uint32_t timer = 0;

	while(1)
	{

		if (data_ready)
		{
			data_ready = 0;
			Read_RawValue();  // Handle the I2C communication in the main loop
		}
			// Read_RawValue();

			// Xa = Acc_x/16384.0;								/* Divide raw value by sensitivity scale factor to get real values */
			// Ya = Acc_y/16384.0;
			// Za = Acc_z/16384.0;
			
			Xg = Gyro_x/16.4;
			// Yg = Gyro_y/16.4;
			// Zg = Gyro_z/16.4;

			// t = (Temperature/340.00)+36.53;					/* Convert temperature in �/c using formula */

			// dtostrf( Xa, 3, 2, float_ );					/* Take values in buffer to send all parameters over USART */
			// sprintf(buffer," Ax = %s g\t",float_);
			// USART_SendString(buffer);

			// dtostrf( Ya, 3, 2, float_ );
			// sprintf(buffer," Ay = %s g\t",float_);
			// USART_SendString(buffer);
			
			// dtostrf( Za, 3, 2, float_ );
			// sprintf(buffer," Az = %s g\t",float_);
			// USART_SendString(buffer);

			// dtostrf( t, 3, 2, float_ );
			// sprintf(buffer," T = %s%cC\t",float_,0xF8);           /* 0xF8 Ascii value of degree '�' on serial */
			// USART_SendString(buffer);

			dtostrf( Xg, 3, 2, float_ );
			sprintf(buffer," X = %s\n",float_);
			USART_SendString(buffer);

			// dtostrf( Yg, 3, 2, float_ );
			// sprintf(buffer," Gy = %s",float_);
			// USART_SendString(buffer);
			
			// dtostrf( Zg, 3, 2, float_ );
			// sprintf(buffer," Gz = %s\n",float_);
			// USART_SendString(buffer);

			_delay_ms(100);

	}
}
