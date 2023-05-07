/* Example for pico_rc library 
 Read input from RC receiver and control servos accordingly.
 Channel 3 controls on-board LED
 Connect RC Receiver to pins 2, 3 and 4
 Connect two RC servos to pins 6 and 7
 */
#include <stdio.h>
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "pico/rc.h"

#define CHANNEL1_PIN    2
#define CHANNEL2_PIN    3
#define CHANNEL3_PIN    4
#define SERVO1_PIN      6
#define SERVO2_PIN      7
#define LED_PIN         25


int main(void) {
    
    setup_default_uart();
    printf("Control example for rc library\n");
    
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    // set pins as input from RC receiver and start measuring pulses 
    // on these pins
    rc_init_input(CHANNEL1_PIN, true);
    rc_init_input(CHANNEL2_PIN, true);
    rc_init_input(CHANNEL3_PIN, true);

     // create servo "objects"
    rc_servo myServo1 = rc_servo_init(SERVO1_PIN);  
    rc_servo myServo2 = rc_servo_init(SERVO2_PIN);  

    // start controlling the servos (generate PWM signal)
    rc_servo_start(&myServo1, 90);   
    rc_servo_start(&myServo2, 90);  
    
    while (1)
    {
        // Read input from RC receiver - that is pulse width on input pin.
        // todo: It would be good idea to average several values
        
        uint32_t pulse1 = rc_get_input_pulse_width(CHANNEL1_PIN);
        printf("Pulse ch1= %lu\n", pulse1);

        uint32_t pulse2 = rc_get_input_pulse_width(CHANNEL2_PIN);
        printf("Pulse ch2= %lu\n", pulse2);
        
        uint32_t pulse3 = rc_get_input_pulse_width(CHANNEL3_PIN);
        printf("Pulse ch3= %lu\n", pulse3);
        
        // Write the values to servos
        // The value returned by rc_get_input_pulse_width is
        // always in the correct range or it is 0.
        if ( pulse1 > 0 )
            rc_servo_set_micros(&myServo1, pulse1);  
        if ( pulse2 > 0 )
            rc_servo_set_micros(&myServo2, pulse2);  
        
        // channel 3 controls LED
        if ( pulse3 > 1600 ) 
            gpio_put(LED_PIN, 1);
        else 
            gpio_put(LED_PIN, 0);
        
        sleep_ms(100);
    }

    return 0;
}