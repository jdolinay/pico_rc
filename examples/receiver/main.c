/* Example for pico_rc library.
 Read pulses from RC receiver and print them to UART.
*/
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/rc.h"

#define CHANNEL1_PIN     2
#define CHANNEL2_PIN     3
#define CHANNEL3_PIN     4


int main() {
    setup_default_uart();
    printf("Example for rc library\n");
    

    // set pin as input from RC receiver and start measuring pulses 
    // on this pin
    rc_init_input(CHANNEL1_PIN, true);
    rc_init_input(CHANNEL2_PIN, true);
    rc_init_input(CHANNEL3_PIN, true);

    while (1)
    {

        // Read input from RC receiver - that is pulse width on input pin.
        // Valid range is about 1000 to 2000 us
        // 0 means that the pulses are out of range in most cases.
        // It can also mean there are no pulses if there were no pulses at all since the program started.
        // If the signal is lost on the input pin, the last valid value will be
        // returned by rc_get_input_pulse_width because the ISR is not called
        // so the value is never updated.
        // todo: It would be good idea to average several pulses
        uint32_t pulse = rc_get_input_pulse_width(CHANNEL1_PIN);
        printf("Pulse ch1= %lu\n", pulse);

        pulse = rc_get_input_pulse_width(CHANNEL2_PIN);
        printf("Pulse ch2= %lu\n", pulse);
        
        pulse = rc_get_input_pulse_width(CHANNEL3_PIN);
        printf("Pulse ch3= %lu\n", pulse);
        
        // Example how to test if a channel is active:
        // The pulse will be 0 if the channel is inactive from the beginning but
        // it will be the last valid value if the pulses stop later.
        // To determine if there are pulses...
        rc_reset_input_pulse_width(CHANNEL1_PIN);
        sleep_ms(30);   // RC period is 20 ms so in 30 there should be some pulse
        pulse = rc_get_input_pulse_width(CHANNEL1_PIN);
        if ( pulse == 0 )
            printf("Channel 1 disconnected\n");

        sleep_ms(400);
    }

    return 0;
}