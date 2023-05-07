/* Example for pico_rc library.
Sweep two servos connected to pins 6 and 7.
*/
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/rc.h"

// Note that neighbour pins like 0,1 or 2,3 or 4,5 share the same 
// pwm hardware called SLICE. 
#define SERVO1_PIN      6
#define SERVO2_PIN      7


int main(void) {
    set_sys_clock_48mhz();

    setup_default_uart();
    printf("Servo example for rc library\n");

    // create servo "objects"
    rc_servo myServo1 = rc_servo_init(SERVO1_PIN);  
    rc_servo myServo2 = rc_servo_init(SERVO2_PIN);  

    // start controlling the servos (generate PWM signal)
    rc_servo_start(&myServo1, 90);   // set servo1 na 90 degrees
    rc_servo_start(&myServo2, 180);   // set servo to 180 deg

    uint angle = 0;
    bool up = true;
    while ( 1 ) {
        
        if ( up ) {
            angle++;
            if ( angle == 180 )
                up = false;
        }
        else {
            angle--;
            if ( angle == 0 )
                up = true;    
        }
    
        rc_servo_set_angle(&myServo1, angle);  
        rc_servo_set_angle(&myServo2, 180 - angle);  
    
        sleep_ms(25);          
    }     

    return 0;
}