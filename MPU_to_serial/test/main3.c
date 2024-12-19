#define BAUD 4800
#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "MPU6050_res_define.h"
#include "I2C_Master_H_file.h"
#include "USART_RS232_H_file.h"

volatile uint8_t data_ready = 0;  // Flag for interrupt
volatile float Gyro_x;
char buffer[20], float_[10];
float Xg;

// Minimal ISR
ISR(TIMER0_COMPA_vect)
{
    data_ready = 1;  // Signal main loop to read MPU6050 data
}

void timer_init(void)
{
    TCCR0A = (1 << WGM01);  // CTC mode
    TCCR0B = (0 << CS02) | (1 << CS01) | (1 << CS00);  // Prescaler = 64
    OCR0A = 124;  // 1 ms interrupt
    TIMSK0 |= (1 << OCIE0A);  // Enable Timer0 Compare Match A interrupt
    sei();  // Enable global interrupts
}

void MPU6050_Init()
{
    _delay_ms(150);
    I2C_Start_Wait(0xD0);
    I2C_Write(SMPLRT_DIV);
    I2C_Write(0x07);
    I2C_Stop();

    I2C_Start_Wait(0xD0);
    I2C_Write(PWR_MGMT_1);
    I2C_Write(0x01);
    I2C_Stop();

    I2C_Start_Wait(0xD0);
    I2C_Write(CONFIG);
    I2C_Write(0x00);
    I2C_Stop();

    I2C_Start_Wait(0xD0);
    I2C_Write(GYRO_CONFIG);
    I2C_Write(0x18);
    I2C_Stop();

    I2C_Start_Wait(0xD0);
    I2C_Write(INT_ENABLE);
    I2C_Write(0x01);
    I2C_Stop();
}

int main(void)
{
    timer_init();
    I2C_Init();
    MPU6050_Init();
    USART_Init(BAUD);

    USART_SendString("init complete\n");

    while (1)
    {
        if (data_ready)
        {
            data_ready = 0;
            Read_RawValue();  // Read data from MPU6050

            Xg = Gyro_x / 16.4;  // Process gyro data
            dtostrf(Xg, 3, 2, float_);
            sprintf(buffer, "X = %s\n", float_);
            USART_SendString(buffer);
        }

        // Other tasks (non-blocking)
    }
    return 0;
}
