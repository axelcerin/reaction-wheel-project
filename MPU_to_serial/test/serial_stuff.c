#include <avr/io.h>
#include <util/delay.h>
#include <USART_RS232_H_file.h>

#define BAUD 4800         // Baud rate for serial communication
#define MY_UBRR F_CPU/16/BAUD-1  // UBRR value for baud rate calculation


int main(void) {
    USART_Init(4800);
    unsigned int ubrr = MY_UBRR;
    USART_Init(ubrr);  // Initialize USART

    char my_string = "hello";

    while (1) {
        // Send 'A' character through Serial
        USART_SendString(my_string);
        _delay_ms(1000);  // Wait for a second

    }

    return 0;
}
