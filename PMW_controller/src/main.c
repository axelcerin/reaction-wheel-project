#include <avr/io.h> // external library which describes our chip
#include <util/delay.h> // external library for time delay functions

void motor_init(void)
{       
DDRB |= (1 << PB1) | (1 << PB2);  // Set PB1 (OC1A) and PB2 (OC1B) as outputs
    TCCR1A = (1 << COM1A1) | (0 << COM1A0)  // Non-inverting mode for OC1A
            | (1 << COM1B1) | (0 << COM1B0)  // Non-inverting mode for OC1B
            | (1 << WGM11) | (0 << WGM10);   // Fast PWM (mode 14)
    TCCR1B = (1 << WGM13) | (1 << WGM12)     // Fast PWM with ICR1 as top
            | (0 << CS12) | (1 << CS11) | (0 << CS10);  // Prescaler of 8
    TIMSK1 = 0;  // No interrupts

    OCR1A = 0;   // Duty cycle initially 0
    OCR1B = 0;
    ICR1 = 19999;  // 1 kHz PWM frequency
}

void set_motor_speed(int16_t speed) { // Takes values between -19999 and 19999
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

int main(void)
{
    motor_init();

    while (1)
    {
        set_motor_speed(0);
        _delay_ms(1000);
        set_motor_speed(12000);
        _delay_ms(1000);
        set_motor_speed(-12000);
        _delay_ms(1000);
    }
}




