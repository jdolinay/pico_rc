/* rc.c 
Library to control RC servos and read input from RC receivers for Raspberry Pi Pico.

Author: Jan Dolinay

Servo is controlled using PWM module.
RC receiver input is read using GPIO interrupts. 

For more info see rc.h
 
This is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 3 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU Lesser General Public License for more details.
*/
#include "pico/rc.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

/** Pico SDK style param checking:
 PICO_CONFIG: PARAM_ASSERTIONS_ENABLED_RC, Enable/disable assertions 
 in the RC library, type=bool, default=0, group=rc
 */
#ifndef PARAM_ASSERTIONS_ENABLED_RC
#define PARAM_ASSERTIONS_ENABLED_RC 0
#endif

/*! @brief max number of RC receiver input channels (pins) supported. 
*/ 
#ifndef RC_MAX_CHANNELS
#define RC_MAX_CHANNELS     5
#endif

/*! @brief Minimal input pulse width accepted as valid for RC receiver. In micro seconds
*/
#ifndef RC_MIN_PULSE_WIDTH
#define RC_MIN_PULSE_WIDTH    750    
#endif

/*! @brief Maximum input pulse width accepted as valid for RC receiver. In micro seconds
*/
#ifndef RC_MAX_PULSE_WIDTH
#define RC_MAX_PULSE_WIDTH    2250   
#endif

/* Servo definitions  */
/* Define the boundaries for valid pulse width for servo.
PULSES in micro seconds; angles in degrees.
These are checked in functions which set servo position.
Note that angle to us is converted using these values, see servo_angle_to_micros().
For example, if you limit the MIN_PULSE=1100 and MAX_PULSE=1900, the output pulse 
width for angle 0 will be 1100 us and for 180 degrees it will be 1900 us. 
 */
#ifndef RC_SERVO_MIN_PULSE
#define RC_SERVO_MIN_PULSE (1000)
#endif

#ifndef RC_SERVO_MAX_PULSE
#define RC_SERVO_MAX_PULSE (2000)
#endif

#ifndef RC_SERVO_MIN_ANGLE
#define RC_SERVO_MIN_ANGLE (0)
#endif

#ifndef RC_SERVO_MAX_ANGLE 
#define RC_SERVO_MAX_ANGLE (180)
#endif

// Internal definitions
// Events we monitor on gpio pins
#define     EVENT_EDGE_RISE     (1 << 3)
#define     EVENT_EDGE_FALL     (1 << 2)
#define     RC_GPIO_EVENTS      (EVENT_EDGE_FALL | EVENT_EDGE_RISE)

// Internal struct to keep info about pin and associated pulse width
struct rc_channel_info {
    uint gpio_pin;
    uint32_t pulse_us;
    uint64_t pulse_start;   
};

// Allocate the structs for supported number of pins;
// need to store pulse with for each channel in ISR
struct rc_channel_info gRcInputChannels[RC_MAX_CHANNELS];

// Index to gRcInputChannels array - also the number of initialized inputs.
static uint gRcLastPulsesIndex = 0;     

// prototypes for internal functions
static int get_pin_index(uint pin);
static void rc_isr_internal( uint pin_index, uint32_t events);
static void rc_gpio_irq_raw_handler(void);
static uint servo_angle_to_micros(uint angle);

//
// Public functions
//

bool rc_init_input(uint gpio_pin, bool start_monitoring) {
    
    assert(gRcLastPulsesIndex < RC_MAX_CHANNELS );   // cannot enable more channels, increase RC_MAX_CHANNELS
    if ( !(gRcLastPulsesIndex  < RC_MAX_CHANNELS) )
        return false;

    // Save info about the pin
    gRcInputChannels[gRcLastPulsesIndex].gpio_pin = gpio_pin;
    gRcInputChannels[gRcLastPulsesIndex].pulse_us = 0;
    gRcInputChannels[gRcLastPulsesIndex].pulse_start = 0;
    gRcLastPulsesIndex++;

    gpio_init(gpio_pin);
    gpio_set_dir(gpio_pin, GPIO_IN);
        
    gpio_set_irq_enabled(gpio_pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, start_monitoring); 
    gpio_add_raw_irq_handler(gpio_pin, rc_gpio_irq_raw_handler);
    
    if ( !irq_is_enabled(IO_IRQ_BANK0)  && start_monitoring )
        irq_set_enabled(IO_IRQ_BANK0, true);        
    // no else to disable - cannot disable all gpio interrupts; user may need them for other things.

    return true;
}

void rc_set_input_enabled(uint gpio_pin, bool enable) {
    // enable/disable irq for given pin
    gpio_set_irq_enabled(gpio_pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, enable);
    
    // if irq is not enabled globally, enable it now.
    if ( !irq_is_enabled(IO_IRQ_BANK0)  && enable )
        irq_set_enabled(IO_IRQ_BANK0, true);
    // no else to disable - cannot disable all gpio interrupts;  user may need them for other things.
}

uint32_t rc_get_input_pulse_width(uint gpio_pin) {
    int index = get_pin_index(gpio_pin);
    if ( index < 0 )
        return 0;       // todo: return error?
    
	// There could be race condition with the ISR, but since the 
	// pulse_us is 32 bit integer I assume the CPU updates it in
	// one instruction, so there is no need to deal with it.
    return gRcInputChannels[index].pulse_us;
}

void rc_reset_input_pulse_width(uint gpio_pin) {
    int index = get_pin_index(gpio_pin);
    if ( index < 0 )
        return;    
    gRcInputChannels[index].pulse_us = 0;    
}


// channel within slice is A for even pins (0,2,4,..) and B for odd pins (1,3,...)
#define SERVO_PIN2CHANNEL(pin)  pwm_gpio_to_channel(pin)
//((pin) & 1) ? PWM_CHAN_B : PWM_CHAN_A)      

/* create servo "object" */
rc_servo rc_servo_init(uint pin) {
    rc_servo s =  { 0, 0};
    s.pin = pin;

    // Tell the GPIO pin it is allocated to the PWM
    gpio_set_function(pin, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO 0 (it's slice 0)
    s.slice_num = pwm_gpio_to_slice_num(pin);
    
    // Support for various system clock settings, e.g. if user
    // sets 48 MHz using set_sys_clock_48mhz()
    uint32_t clk = clock_get_hz(clk_sys); // clk_sys
    // aim at 50 Hz with counter running to 20 000 
    uint32_t div = clk / (20000 * 50); 
    // div must be between 1 and 255
    // which is true for clock from 1 MHz to 255 MHz, so it
    // should be safe to assume the div is within range
    if ( div < 1 )
        div = 1;
    if ( div > 255 )
        div = 255;    

    pwm_config config = pwm_get_default_config();
    // Set divider to get 50 Hz
    pwm_config_set_clkdiv(&config, (float)div);
    // Set wrap to count to 20000, so the period is 20 ms
    pwm_config_set_wrap(&config, 20000); 
    // Load the configuration into our PWM slice, and set it running.
    pwm_init(s.slice_num, &config, false);
    // pwm will be started by calling pwm_set_enabled later
    
    return s;
}

/* Start generating pwm*/
void rc_servo_start(const rc_servo* servo, uint angle) {
    valid_params_if(RC, (angle >= RC_SERVO_MIN_ANGLE && angle <= RC_SERVO_MAX_ANGLE));
    // if angle is not within range, do nothing
    if ( angle < RC_SERVO_MIN_ANGLE || angle > RC_SERVO_MAX_ANGLE )
        return;

    const uint channel = SERVO_PIN2CHANNEL(servo->pin);
    uint pulse = servo_angle_to_micros(angle);
    pwm_set_chan_level(servo->slice_num, channel, pulse);
    pwm_set_enabled(servo->slice_num, true);
}

void rc_servo_stop(const rc_servo* servo, bool stop_slice) {
    if ( stop_slice ) {
        // stop the slice
        pwm_set_enabled(servo->slice_num, false);
    } else {
        // otherwise just set the pulse width to 0
        const uint channel = SERVO_PIN2CHANNEL(servo->pin);
        pwm_set_chan_level(servo->slice_num, channel, 0);
    }
}

/* set angle for servo. */
void rc_servo_set_angle(const rc_servo* servo, uint angle) {
    valid_params_if(RC, (angle >= RC_SERVO_MIN_ANGLE && angle <= RC_SERVO_MAX_ANGLE));
    // if angle is not within range, do nothing
    if ( angle < RC_SERVO_MIN_ANGLE || angle > RC_SERVO_MAX_ANGLE )
        return;

    const uint channel = SERVO_PIN2CHANNEL(servo->pin);
    uint pulse = servo_angle_to_micros(angle);
    pwm_set_chan_level(servo->slice_num, channel, pulse);
}

/* Set pulse width in microseconds (1000 to 2000)*/
void rc_servo_set_micros(const rc_servo* servo, uint micros) {
    valid_params_if(RC, (micros >= RC_SERVO_MIN_PULSE && micros <= RC_SERVO_MAX_PULSE));
    // normal runtime check of params
    if ( micros < RC_SERVO_MIN_PULSE || micros > RC_SERVO_MAX_PULSE )
        return;     // do nothing

    const uint channel = SERVO_PIN2CHANNEL(servo->pin);
    pwm_set_chan_level(servo->slice_num, channel, micros);
}

/* internal helper, Arduino-style map() function. */
static inline uint map(uint x, uint in_min, uint in_max, uint out_min, uint out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* Convert angle to microseconds = pulse length
 With default ranges it maps 0-180 deg to 1000-2000 us
 0 deg = 1000 us
 180 deg = 2000 us
 */
static uint servo_angle_to_micros(uint angle) {
    // we don't check if angle is valid; the caller must do so.
    uint us = map(angle, RC_SERVO_MIN_ANGLE, RC_SERVO_MAX_ANGLE, RC_SERVO_MIN_PULSE, RC_SERVO_MAX_PULSE);
    return us;
}

//
// Internal functions
//
// This handler is called for all pins called from the actual handler
static void rc_isr_internal( uint pin_index, uint32_t events) { 
    uint64_t now = to_us_since_boot(get_absolute_time());
    
    if ( events & EVENT_EDGE_RISE ) {
        gRcInputChannels[pin_index].pulse_start = now;
    }

    if ( events & EVENT_EDGE_FALL ) {
        if ( gRcInputChannels[pin_index].pulse_start > 0 ) {
            uint32_t diff = (uint32_t)(now - gRcInputChannels[pin_index].pulse_start);
            gRcInputChannels[pin_index].pulse_start = 0;
            if (diff >= RC_MIN_PULSE_WIDTH && diff <= RC_MAX_PULSE_WIDTH)
                gRcInputChannels[pin_index].pulse_us = (uint32_t)diff;
            else
                gRcInputChannels[pin_index].pulse_us = 0;    
            // todo: else gPulse1 = 0; to indicate invalid pulse?
        }
    }
}

// The raw irq handler set for each pin.
// It calls common internal handler for each pin that has pending rising or falling edge.
static void rc_gpio_irq_raw_handler(void) {
    // must check all active pins    
    uint32_t events;
    for ( uint i = 0; i < gRcLastPulsesIndex; i++ ) {
        events = gpio_get_irq_event_mask(gRcInputChannels[i].gpio_pin);
        // if the events are pending for this pin, call the handler
        if ( events & RC_GPIO_EVENTS) {
            gpio_acknowledge_irq(gRcInputChannels[i].gpio_pin, events);
            rc_isr_internal(i, events);            
        }
    }
}

// Helper to find struct with pin info 
// Returns index to gRcInputChannels array.
static int get_pin_index(uint pin) {
    for ( int i = 0; i< gRcLastPulsesIndex; i++ ) {
        if ( gRcInputChannels[i].gpio_pin == pin )
            return i;
    }
    return -1;  // invalid pin, should not happen
}

