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
#include <stdio.h>
#include <time.h>
#include <avr/io.h> // external library which describes our chip
#include <util/delay.h> // external library for time delay functions

#define BUFFER_SIZE 5
float data[BUFFER_SIZE];
int head = 0; // Index for the next write
float sum = 0.0;

// Interupt variables
// volatile double  Acc_x = 0;
volatile double Gyro_x = 0; 
// volatile uint8_t data_ready = 0;  // Flag for data reading
// volatile uint8_t timer = 0;  // Flag for data writing


void timer_init(void)
{
    // Set Timer0 to CTC mode (Clear Timer on Compare Match)
    TCCR0A = 0;            // Normal mode (WGM01 and WGM00 both 0)
    TCCR0B = (1 << WGM02); // CTC mode (WGM02 = 1)

    // Set the compare match value (OCR0A) for the interrupt frequency
    OCR0A = 125;           // Example: Compare match value, 125 gives a 1ms interrupt with 1 MHz clock and prescaler 64

    // Set prescaler
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
	MPU_Start_Loc();									/* Read values */
	
	// Read Accelerometer
	// Acc_x = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());

	// Read Gyro
	Gyro_x = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());

	I2C_Stop();
}

// PID variables
double pid_output, integral, previous = 0;
double dt = 0.13;
double kp = 300.0;
double ki = 0.00;
double kd = 100.0;
double proportional, derivative;

double pid(double error)
{
    /* double */ proportional = error;
    integral += error*dt;
    /* double */ derivative = (error - previous) / dt;
    // derivative = Acc_x;
    previous = error;
    pid_output =  (kp * proportional) + (ki * integral) + (kd * derivative);

    // Clamp the PID output to prevent extreme values
    if (pid_output > 19999) {pid_output = 19999;}
    else if (pid_output < -19999) {pid_output = -19999;}

}

void motor_init(void)
{       
DDRB |= (1 << PB1) | (1 << PB2);  // Set PB1 (OC1A) and PB2 (OC1B) as outputs
    TCCR1A = (1 << COM1A1) | (0 << COM1A0)  // Non-inverting mode for OC1A
            | (1 << COM1B1) | (0 << COM1B0)  // Non-inverting mode for OC1B
            | (1 << WGM11) | (0 << WGM10);   // Fast PWM (mode 14)
    TCCR1B = (1 << WGM13) | (1 << WGM12)     // Fast PWM with ICR1 as top
            | (0 << CS12) | (1 << CS11) | (0 << CS10);  // Prescaler of 8

    OCR1A = 0;   // Duty cycle initially 0
    OCR1B = 0;
    ICR1 = 19999;  // 1 kHz PWM frequency
}

void set_motor_speed(int16_t speed) { // Takes values between -19999 and 19999 (dependent on ICR1)
    if (speed > 0) {
        OCR1A = speed;  // Forward PWM
        OCR1B = 0;
    } else if (speed < 0) {
        OCR1A = 0;
        OCR1B = -speed;  // Reverse PWM
    } else {
        OCR1A = 0;
        OCR1B = 0;  // Stop
    }
}

ISR(TIMER0_COMPA_vect)
{
	cli();
    Read_RawValue();  // Read data from MPU6050 when interrupt occurs

    Gyro_x = Gyro_x/16.4;
    data[head] = Gyro_x;
    // Acc_x = Acc_x/16384.0;
    head = (head + 1) % BUFFER_SIZE; // Move head to the next position
    for (int i = 0; i < BUFFER_SIZE; i++) {
        sum += data[i];
    }
    sum = sum/BUFFER_SIZE;

    pid(sum);
    set_motor_speed(pid_output);
	sei();
}

int main()
{
	char buffer[20], double_[10];
	// double Xa=0;
	double Xg=0;	

    // Initialize the buffer
    for (int i = 0; i < BUFFER_SIZE; i++) {
        data[i] = 0.0; // Optional initialization
    }

    double compare_register = 125;
    double prescaler = 256;
    double f_timer = (double)F_CPU / prescaler;
    double dt = (compare_register + 1)/f_timer;

    char new_row[] = "\n";  // Null-terminated string
    char space[] = "        ";  // Null-terminated string
    char init_message[] = "\n\n -------- init complete -------- \n\n";  // Null-terminated string

	I2C_Init();											/* Initialize I2C */
	MPU6050_Init();										/* Initialize MPU6050 */
	USART_Init(BAUD);									/* Initialize USART with 9600 baud rate */
    motor_init();

    USART_SendString(new_row);
	USART_SendString(init_message);         // Send the whole string at once

    dtostrf(dt, 3, 2, double_ );
    sprintf(buffer," dt = %s",double_);
    USART_SendString(buffer);

    USART_SendString(space); 

    dtostrf(f_timer, 3, 2, double_ );
    sprintf(buffer," f_timer = %s",double_);
    USART_SendString(buffer);

    USART_SendString(space); 

    dtostrf(F_CPU, 3, 2, double_ );
    sprintf(buffer," F_CPU = %s",double_);
    USART_SendString(buffer);

    USART_SendString(space); 
    USART_SendString(new_row);

	timer_init();

	while(1)
	{
		
        // // set_motor_speed(pid_output);

        // dtostrf(pid_output, 3, 2, double_ );
        // sprintf(buffer," PID output = %s",double_);
        // USART_SendString(buffer);

        // // Display Accelerometer Data
        // // Xa = Acc_x/16384.0;
        // // Xa = Acc_x;
        // // dtostrf( Xa, 3, 2, double_ );
        // // sprintf(buffer," X = %s\n",double_);
        // // USART_SendString(buffer);

        // // // Display Gyro Data
        // // Xg = Gyro_x/16.4;
        // Xg = sum;
        // dtostrf( Xg, 3, 2, double_ );
        // sprintf(buffer," X = %s\n",double_);
        // USART_SendString(buffer);

        // USART_SendString(new_row);       // Send the whole string at once

        // // dtostrf(proportional, 3, 2, double_ );
        // // sprintf(buffer," P = %s",double_);
        // // USART_SendString(buffer);

        // // USART_SendString(space); 

        // // dtostrf(integral, 3, 2, double_ );
        // // sprintf(buffer," I = %s",double_);
        // // USART_SendString(buffer);

        // // USART_SendString(space); 

        // // dtostrf(derivative, 3, 2, double_ );
        // // sprintf(buffer," D = %s",double_);
        // // USART_SendString(buffer);


        // // USART_SendString(new_row);

        // _delay_ms(100);
	}
}