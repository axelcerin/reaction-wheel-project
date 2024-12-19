#include <avr/io.h>
#include <util/delay.h>
#include <USART_RS232_H_file.h>  // Ensure this file exists and is correct

// Remove F_CPU redefinition if it's already defined by the build system.

#define BAUD 4800                  // Baud rate for serial communication
#define MY_UBRR F_CPU/16/BAUD-1    // Calculate UBRR value based on F_CPU and BAUD

int main(void) {
    unsigned int ubrr = MY_UBRR;  // Calculate UBRR value
    USART_Init(ubrr);            // Initialize USART with calculated UBRR

    char *my_string = "hello";   // Correctly define the string

    while (1) {
        USART_SendString(my_string);  // Send the string through Serial
        _delay_ms(1000);             // Wait for a second
    }

    return 0;
}
