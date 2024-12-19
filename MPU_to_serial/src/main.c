/*
 * ATmega16 Interface with MPU-6050
 * Motor Control using SN754410 Motor Driver
 * PWM on PB2 and PB1 for direction and speed control
 * PID control based on Gyro data
 */

#define BAUD 4800                     /* Baud rate for serial communication */
#define F_CPU 1000000UL               /* Clock speed (1 MHz) */
#include <avr/io.h>
#include <util/delay.h>
#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include "MPU6050_res_define.h"        /* Include MPU6050 register define file */
#include "I2C_Master_H_file.h"         /* Include I2C Master header file */
#include "USART_RS232_H_file.h"        /* Include USART header file */
#include <avr/interrupt.h>

// PID constants (tune these for your system)
#define Kp 1.0    // Proportional gain
#define Ki 0.1    // Integral gain
#define Kd 0.05   // Derivative gain

// Target gyro value (desired angular velocity)
#define TARGET_GYRO 300  // Example target value, set to 0 for no tilt

volatile float Gyro_x;
volatile uint8_t data_ready = 0;  // Flag for data reading
volatile uint8_t timer = 0;  // Flag for data writing

float prev_error = 0.0;
float integral = 0.0;
float pid_output = 0.0;

// Function to initialize PWM (for motor control)
void setupPWM() {
    DDRB |= (1 << PB2) | (1 << PB1); // Set PB2 and PB1 as output (PWM for motor control)

    // Timer1 setup for PWM on PB2 (OC1A) and PB1 (OC1B)
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);  // Fast PWM mode with non-inverted outputs
    TCCR1B = (1 << WGM12) | (1 << CS11); // Prescaler = 8
}

// Function to set motor speed and direction
void setMotorSpeedAndDirection(uint8_t speed, int8_t direction) {

    // Stop motor by clearing both PWM signals first
    PORTB &= ~((1 << PB2) | (1 << PB1));

    // Check direction and set PWM signals accordingly
    if (direction > 0) {
        // Forward direction
        PORTB |= (1 << PB2);   // PB2 HIGH: Input 1 for forward
        PORTB &= ~(1 << PB1);  // PB1 LOW: Input 2 for forward
        OCR1A = speed;         // Set speed for PWM signal on PB2
        OCR1B = 0;             // No PWM signal on PB1 for forward
    } else if (direction < 0) {
        // Reverse direction
        PORTB &= ~(1 << PB2);  // PB2 LOW: Input 1 for reverse
        PORTB |= (1 << PB1);   // PB1 HIGH: Input 2 for reverse
        OCR1A = 0;             // No PWM signal on PB2 for reverse
        OCR1B = speed;         // Set speed for PWM signal on PB1
    } else {
        // Stop the motor
        OCR1A = 0;
        OCR1B = 0;
    }
}

// Timer0 interrupt handler
ISR(TIMER0_COMPA_vect) {
    cli();
    data_ready = 1;  // Flag for data reading from MPU6050
    sei();
}

// Timer1 interrupt handler
ISR(TIMER1_COMPA_vect) {
    timer = 1;  // Timer flag for PID control
}

// Timer initialization function
void timer_init(void) {
    // Set Timer0 for data reading at regular intervals
    OCR0A = 140;  // Example: 1ms interrupt with 1 MHz clock and prescaler 64
    TCCR0B |= (0 << CS02) | (1 << CS01) | (1 << CS00);  // Prescaler 64
    TIMSK0 |= (1 << OCIE0A);  // Enable Timer0 compare match interrupt

    // Set Timer1 for PID control at regular intervals
    OCR1A = 255;  // Example: 1ms interrupt
    TCCR1B |= (1 << CS12) | (0 << CS11) | (1 << CS10);  // Prescaler 64
    TIMSK1 |= (1 << OCIE1A);  // Enable Timer1 compare match interrupt

    sei();  // Enable global interrupts
}

// MPU6050 initialization function
void MPU6050_Init() {
    _delay_ms(150);  // Power-up delay
    I2C_Start_Wait(0xD0);  // Write to device address
    I2C_Write(SMPLRT_DIV);  // Sample rate register
    I2C_Write(0x07);        // 1 KHz sample rate
    I2C_Stop();

    I2C_Start_Wait(0xD0);
    I2C_Write(PWR_MGMT_1);  // Power management register
    I2C_Write(0x01);        // Gyroscope reference frequency
    I2C_Stop();

    I2C_Start_Wait(0xD0);
    I2C_Write(GYRO_CONFIG);  // Gyro config register
    I2C_Write(0x18);         // Full scale range +/- 2000 degree/C
    I2C_Stop();

    I2C_Start_Wait(0xD0);
    I2C_Write(INT_ENABLE);  // Interrupt enable register
    I2C_Write(0x01);        // Enable interrupt
    I2C_Stop();
}

// MPU6050 read function
void Read_RawValue() {
    I2C_Start_Wait(0xD0);  // I2C start with device write address
    I2C_Write(ACCEL_XOUT_H);  // Start location to read
    I2C_Repeated_Start(0xD1);  // I2C start with device read address
    Gyro_x = (((int)I2C_Read_Ack() << 8) | (int)I2C_Read_Ack());  // Read Gyro data
    I2C_Stop();
}

// PID Controller Calculation
float calculatePID(float target, float current_value) {
    // Calculate error
    float error = target - current_value;

    // Proportional term
    float proportional = Kp * error;

    // Integral term
    integral += error;
    float integral_term = Ki * integral;

    // Derivative term
    float derivative = Kd * (error - prev_error);

    // Calculate PID output
    pid_output = proportional + integral_term + derivative;

    // Store current error for next calculation
    prev_error = error;

    return pid_output;
}

// Main function
int main() {
    char buffer[20], float_[10];
    I2C_Init();             // Initialize I2C
    MPU6050_Init();         // Initialize MPU6050
    USART_Init(BAUD);       // Initialize USART
    char init_message[] = "init complete\n";
    USART_SendString(init_message);  // Send init message
    timer_init();           // Initialize timers
    setupPWM();             // Initialize PWM

    while (1) {
        if (data_ready) {
            data_ready = 0;
            Read_RawValue();  // Handle data reading from MPU6050
        }

        if (timer) {
            timer = 0;

            // Use PID controller to calculate motor speed based on Gyro_x
            pid_output = calculatePID(TARGET_GYRO, Gyro_x);
            pid_output = pid_output*0.0085; // Rescaled so 30000 = 255

            int8_t motor_speed = 0;

            if (pid_output > 255)
            {
                motor_speed = 255;
            }

            if (pid_output > 0)
            {
                motor_speed = 0;
            }

            else
            {
            motor_speed = pid_output;
            }
            // // Scale the pid_output to a value between 0 and 250
            // uint8_t motor_speed = (uint8_t)(pid_output > 250 ? 250 : (pid_output < 0 ? 0 : pid_output));

            // // Optionally, you can map the PID output to the range 0-250 using linear scaling
            // motor_speed = (uint8_t)(pid_output > 250 ? 250 : (pid_output < 0 ? 0 : pid_output));


            // Set motor direction based on PID output
            int8_t motor_direction = (pid_output > 0) ? 1 : (pid_output < 0) ? -1 : 0;

            // Set motor speed and direction
			// setMotorSpeedAndDirection(255,1);
			// _delay_ms(1000);
			// setMotorSpeedAndDirection(255,-1);
			// _delay_ms(1000);
            setMotorSpeedAndDirection(motor_speed, motor_direction);

            // Display Gyro Data via USART
            float Xg = Gyro_x;  // Gyro data
            dtostrf(motor_direction, 3, 2, float_);
            sprintf(buffer, "motor direction: %s", float_);
            USART_SendString(buffer);

            char space_between_data[] = "        ";  // Null-terminated string
            USART_SendString(space_between_data);    // Send space between data

            dtostrf(motor_speed, 3, 2, float_);
            sprintf(buffer, "motor speed: %s", float_);
            USART_SendString(buffer);

            USART_SendString(space_between_data);    // Send space between data

            dtostrf(pid_output, 3, 2, float_);
            sprintf(buffer, "pid output: %s", float_);
            USART_SendString(buffer);

            USART_SendString(space_between_data);    // Send space between data

            dtostrf(Xg, 3, 2, float_);
            sprintf(buffer, " X = %s\n", float_);
            USART_SendString(buffer);
        }

        _delay_ms(100);
    }
}
