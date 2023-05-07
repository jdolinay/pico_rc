/* rc.c 

Library to control RC servos and read input from RC receivers for Raspberry Pi Pico.

Author: Jan Dolinay

Servo is controlled using PWM module.
RC receiver input is read using GPIO interrupts.  
 
 Notes:
 Using PWM to read input pulse width (for RC receiver) seems not feasible as the
 PWM module can only count edges (see SDK example to measure frequency) or 
 run counter while input is high (see SDK example to measure duty cycle). 
 Both these methods can be used for fast signals to "statistically" obtain the
 frequency by counting number of edgesd on input for given time or counting 
 how long the input is high in ratio to how long we measure. 
 But the RC signal is only 50 Hz, so we would have to sample it for long time to obtain
 reliable results. But then we would not be able to respond quikly to changing pulse
 width. That's why GPIO interrupts on rising and falling edge are used to measure pulse width.
 

This is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 3 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU Lesser General Public License for more details.
*/

#ifndef _PICO_RC_LIB_H
#define _PICO_RC_LIB_H

#include "pico.h"


#ifdef __cplusplus
extern "C" {
#endif


/*! \brief init mesuring of input pulse width
    \param gpio_pin 
    \return false if no more channels can be added
*/
bool rc_init_input(uint gpio_pin, bool start_monitoring);

/*! \brief enable or disable monitoring of given pin 
*   \param gpio_pin 
*   \param enabled ture to enable channel; pulse with can be read by pulseio_get_rc_input_pulse
*/
void rc_set_input_enabled(uint gpio_pin, bool enable);

/*! \brief Obtain the measured pulse with on given pin. Pin must be initialized by 
* rc_init_input_channel() first.
* Valid range is about 1000 to 2000 us
* 0 means that the pulses are out of range in most cases. 
* It can also mean there are no pulses if there were no pulses at all since the program started.
* If the signal is lost on the input pin, the last valid value will be 
* returned by rc_get_input_pulse_width because the ISR is not called,
* so the value is never updated.
*   \param gpio_pin 
*   \return the pulse width in micro seconds. 0 can be returned if the pin is invalid or there is no signal.
*/
uint32_t rc_get_input_pulse_width(uint gpio_pin);

/*! \brief Reset the internal variable for pulse width. If after some 20 ms
* the pulse width for this pin is still 0, it means there is no signal on this pin.
*/
void rc_reset_input_pulse_width(uint gpio_pin);

/*
**** Servo motor control
*/

/*! \brief Servo data returned when servo is initialized 
*/ 
typedef struct {
    uint pin;
    uint slice_num;

} rc_servo;

/**
 * \brief Creates servo object attached to given pin. This is then passed to servo functions.
 * \ingroup servo
 * 
 * \param pin GPIO pin to be used for this servo.
*/
rc_servo rc_servo_init(uint pin);

/**
 * \brief Start generating pwm.
 * \ingroup servo
 * 
 * \param servo Servo struct that must be initialized before by servo_init
 * \param angle The angle to set on the servo (0 - 180 degrees). It can be changed by servo_set_angle.
*/
void rc_servo_start(const rc_servo* servo, uint angle);

/**
 * \brief Stop generating PWM on the servo pin.
 * \ingroup servo
 * 
 * \param servo
 * \param stop_slice   If True, the slice is stopped which can stop another servo
 * sharing this slice. Set to true is this is the only servo or if you want to stop
 * both servos which share this hardware slice.
*/
void rc_servo_stop(const rc_servo* servo, bool stop_slice);

/**
 * \brief Set the angle of servo lever.
 * \param servo
 * \param angle The angle to set, 0 to 180.
*/
void rc_servo_set_angle(const rc_servo* servo, uint angle);

/**
 * \brief Set the pulse width for the servo directly in microseconds.
 * \param servo
 * \param angle Pulse length to set, 1000 to 2000.
*/
void rc_servo_set_micros(const rc_servo* servo, uint micros);




#ifdef __cplusplus
}
#endif

#endif  // _PICO_RC_LIB_H