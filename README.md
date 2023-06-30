# picodrone
A 14 oscillator drone made from a raspberry pi pico
/*************************************************************************************************************
 * Stylo.c this is code for raspberry pi pico RP2040. It is the  14 oscillator Picodrone.
 *
 *
 * 14 Bit D to A pins 0-13
 * Pins 14-17 are MUX drivers
 * Pins 18-21 switch pins with a diode and MUX pins this gives 16 switches
 * Pin  22 is the LED pin. With the MUX drivers this controls 4 LEDS (they can even share a single current
 * limiting resistor)
 * Pins 23-24-25-26 are internal to the Pico                                                                    
 * Pin  26-27-28 are set to ADC for three pots                                                          
 *
 * Martin Parker        -       The Inner Loop
 * Version              -       0.1 First complete version with 
 * 
 *
 *      Unreliable Sequencer
 *      Triangle and Saw waves
 *      Wave shaper to square  with variable level
 *      Octave twiddler
 *      Delay and reverb
 *      Crude ilow pass filter
 *      dual LFO for vibrato and tremelo
 *      45455 Samples per second output rate
 *      16 Keys and 4 LEDS multiplexed
 *
 * YOUTBE https://youtu.be/8awpn7hZmbM
 *
 **********************************************************************************************************/
