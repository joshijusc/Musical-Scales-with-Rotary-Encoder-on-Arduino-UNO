/********************************************
 *
 *  Name:
 *  Email:
 *  Section:
 *  Assignment: Lab 8 - Rotary Encoder
 *
 ********************************************/

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <stdlib.h>     /* abs */
#include "lcd.h"

void play_note(uint16_t);
void variable_delay_us(int16_t);
void timer2_init();

// Frequencies for natural notes from middle C (C4)
// up one octave to C5.
uint16_t frequency[8] =
    { 262, 294, 330, 349, 392, 440, 494, 523 };

volatile uint8_t changed = 0;  // Flag for state change
volatile uint8_t new_state, old_state;
volatile uint8_t a, b;
volatile int16_t count = 128;		// Count to display
volatile int16_t isrCount = 0;		// Count for ISR
volatile int16_t isrInvoked = 0;		// Count for ISR


int main(void) {
	timer2_init();
	// Setting up the interrupt handlers
    PCICR |= (1<<PCIE1);
    PCMSK1 |= ((1<<PCINT4)|(1<<PCINT5));

    // Initialize the global interrupt
    sei();


    // Initialize DDR and PORT registers and LCD
	PORTC |= (1 << PC4) | (1 << PC5);
	DDRB |= (1 << PB3);
	lcd_init();	

    // Write a spash screen to the LCD
	char buf[12];
    char* title = "EE109 Lab 8";
    snprintf(buf, 12, "%s", title);
    lcd_moveto(0, 0);
    lcd_stringout(buf);
    char* name = "Jason Joshi";
    snprintf(buf, 12, "%s", name);
    lcd_moveto(1, 0);
    lcd_stringout(buf);
    _delay_ms(1000);
    lcd_writecommand(1);



    // Read the A and B inputs to determine the intial state.
    // In the state number, B is the MSB and A is the LSB.
    // Warning: Do NOT read A and B separately.  You should read BOTH inputs
    // at the same time, then determine the A and B values from that value.

	a = (PINC & (1<<4));
    b = (PINC & (1<<5));
    if (!b && !a)
	old_state = 0;
    else if (!b && a)
	old_state = 1;
    else if (b && !a)
	old_state = 2;
    else
	old_state = 3;

    new_state = old_state;

    while (1) {                 // Loop forever
	// Read the input bits and determine A and B.
	a = (PINC & (1<<4));
    b = (PINC & (1<<5));
    if (!b && !a)
	old_state = 0;
    else if (!b && a)
	old_state = 1;
    else if (b && !a)
	old_state = 2;
    else
	old_state = 3;

    new_state = old_state;

		// if (a == 0) {
		// 	OCR2A = 51;
		// } else {
		// 	OCR2A = 204;
		// }


		
        if (changed) { // Did state change?
			changed = 0;        // Reset changed flag
			char buf3[15];
			snprintf(buf3, 15, "%4d", count);
			lcd_moveto(0, 0);
			lcd_stringout(buf3);
			OCR2A = count;



			// Output count to LCD
			// char buf3[15];
			// snprintf(buf3, 15, "%4d", count);
			// lcd_moveto(0, 0);
			// lcd_stringout(buf3);
			// // Do we play a note?
			// if ((count % 8) == 0) {
			// 	// Determine which note (0-7) to play
			// 	int note = ((abs(count)) % 64) / 8;
			// 	// Find the frequency of the note

			// 	// Call play_note and pass it the frequency
			// 	play_note(frequency[note]);
			// }
        }
	}

	// For the Checkpoint, print the values of A and B on the LCD.
		// lcd_moveto(0,0);
		// char buf2[15];
		// snprintf(buf2, 15, "%d %d", a, b);
		// lcd_stringout(buf2);

	// The following code is for Tasks 4 and later.
	// For each state, examine the two input bits to see if state
	// has changed, and if so set "new_state" to the new state,
	// and adjust the count value.
	
}

/*
  Play a tone at the frequency specified for one second
*/
void play_note(uint16_t freq)
{
    uint32_t period;

    period = 1000000 / freq;    // Period of note in microseconds
	isrCount = 2 * freq;
	isrInvoked = 0;
	OCR1A = 16000000 / (2 * freq);
    TCCR1B |= (1 << CS10);

    // while (freq--) {
	// PORTB |= (1 << PB4);    // Buzzer output high

	// variable_delay_us(period / 2);  // Delay for half the period
	// PORTB &= ~(1 << PB4);   // Buzzer output log
	// variable_delay_us(period / 2);  // Delay for half the period
    // }
}

/*
    variable_delay_us - Delay a variable number of microseconds
*/
void variable_delay_us(int delay)
{
    int i = (delay + 5) / 10;

    while (i--)
        _delay_us(10);
}

ISR(PCINT1_vect)
{
    // In Task 6, add code to read the encoder inputs and determine the new
    // count value

	uint8_t input = PINC;
	a = input & (1 << PC4);
	b = input & (1 << PC5);

	if (a != 0){
		a = 1;
	}
	if (b != 0){
		b = 1;
	}

	if (old_state == 0) {

	    // Handle A and B inputs for state 0
		if (a==1) {new_state = 1; count++;}
		if (b==1) {new_state = 2; count--;}
		if (a==0 && b==0) new_state = 0;
	}
	else if (old_state == 1) {

	    // Handle A and B inputs for state 1
		if (a==0) {new_state = 0; count--;}
		if (b==1) {new_state = 3; count++;}
		if (a==1 && b==0) new_state = 1;
	}
	else if (old_state == 2) {

	    // Handle A and B inputs for state 2
		if (a==1) {new_state = 3; count--;}
		if (b==0) {new_state = 0; count++;}
		if (a==0 && b==1) new_state = 2;
	}
	else {   // old_state = 3

	    // Handle A and B inputs for state 3
		if (a==0) {new_state = 2; count++;}
		if (b==0) {new_state = 1; count--;}
		if (a==1 && b==1) new_state = 3;
	}

	// If state changed, update the value of old_state,
	// and set a flag that the state has changed.
	if (new_state != old_state) {
	    changed = 1;
	    old_state = new_state;
	}
	if (count > 255){
		count = 255;
	} else if (count < 0){
		count = 0;
	}
}


void timer1_init()
{
    // In Task 7, add code to inititialize TIMER1, but don't start it counting
	// Add code to configure TIMER1 by setting or clearing bits in
    // the TIMER1 registers.

    // Set to CTC mode
    TCCR1B |= (1 << WGM12);

    // Enable Timer Interrupt
    TIMSK1 |= (1 << OCIE1A);

    // OCR1A = 25000;
    // TCCR1B |= (1 << CS11) | (1 << CS10);

}
ISR(TIMER1_COMPA_vect)
{
    // In Task 7, add code to change the output bit to the buzzer, and to turn
    // off the timer after enough periods of the signal
		isrInvoked++;
		PORTB ^= (1 << PB4);
		if (isrInvoked == isrCount){		
			TCCR1B &= ~(1 << CS10);
			isrInvoked = 0;
		}
}

void timer2_init(void)
{
    TCCR2A |= (0b11 << WGM20);  // Fast PWM mode, modulus = 256
    TCCR2A |= (0b10 << COM2A0); // Turn D11 on at 0x00 and off at OCR2A
    OCR2A = 128;                // Initial pulse duty cycle of 50%
    TCCR2B |= (0b111 << CS20);  // Prescaler = 1024 for 16ms period
}