///*
 //* BoostConverter.cpp
 //*
 //* Created: 17.10.2016 19:16:08
 //* Author : U0161529
 //*/ 
//
//#define F_CPU 1000000
//
//#include <avr/io.h>
//#include <util/delay.h>
//#include <avr/interrupt.h>
//
//#define POT_CHANNEL 3
//#define POT 3
//#define BLINK 2
//#define PWM 0
////
////void startAdc();
////
////ISR(ADC_vect) {
	//////OCR0A = ADCH;
	//////startAdc();
////}
////
////
////inline void setAdcChannel(uint8_t channel) {
	//////ADMUX |= (0b11110000 & channel);	
////}
////
////inline void startAdc() {
	//////ADCSRA |= (1 << ADSC);
////}
////
////inline uint8_t readAdc() {
	//////return ADCH;
	////return 10;
////}
////
////void initAdc() {
	//////ADMUX |= (1 << ADLAR);			// left align the ADC value- so we can read highest 8 bits
	//////
	//////ADCSRA |= (1 << ADEN) |			// enable ADC
				//////(1 << ADATE) |		// set auto-triggering
				//////(1 << ADPS1) | (1 << ADPS0) | (1 << ADEN) |		// divide CPU frequency by by 128
				//////(1 << ADIE);		// raise interrupt on ADC conversion complete
				//////
	//////ADCSRB = 0; // free running mode
////}
//
//void initTimer() {
    //TCCR0A = 0;
	//TCCR0B = 0;
	//
	//TCCR0A |= (1 << WGM00) | (1 << WGM01);	// fast PWM mode with compare with OCR0 (to set WGM02)
	//TCCR0B |= (1 << WGM02);					// finish fast PWM mode initialization
	//TCCR0A |= (1 << COM0A1);				// non-inverted mode
    //
	//OCR0A = 1;
	//
	//TCCR0B |= (1 << CS00);					// no prescaling
//}	
//
//
//void initPins() {
	//DDRB |= (1 << BLINK) | (1 << PWM);
	////DDRB &= ~(1 << POT);
//}
//
//void init() {
	//cli();
	//initPins();
	////initAdc();
	//initTimer();
	//sei();
	////setAdcChannel(POT_CHANNEL);
	////startAdc();
//}
//
//
//int main(void) {
	//init();
    //while (1) {
		//PORTB |= (1 << BLINK);
		////_delay_ms(500);
		//PORTB &= ~(1 << BLINK);
		////_delay_ms(500);
    //}
//}
//


/* ---------------------------------------------------------------------
 * PWM LED Brightness control for ATtiny13.
 * Datasheet for ATtiny13: http://www.atmel.com/images/doc2535.pdf
 * 
 * Pin configuration -
 * PB1/OC0B: LED output (Pin 6)
 * PB2/ADC1: Potentiometer input (Pin 7)
 *
 * ~100 bytes.
 * 
 * Find out more: http://bit.ly/1eBhqHc
 * -------------------------------------------------------------------*/
 
// 9.6 MHz, built in resonator
#define F_CPU 9600000
#define LED PB1 
 
 
#include <avr/io.h>
 
void adc_setup (void)
{
    // Set the ADC input to PB2/ADC1
    ADMUX |= (1 << MUX0);
    ADMUX |= (1 << ADLAR);
 
    // Set the prescaler to clock/128 & enable ADC
    // At 9.6 MHz this is 75 kHz.
    // See ATtiny13 datasheet, Table 14.4.
    ADCSRA |= (1 << ADPS1) | (1 << ADPS0) | (1 << ADEN);
}
 
int adc_read (void)
{
    // Start the conversion
    ADCSRA |= (1 << ADSC);
 
    // Wait for it to finish
    while (ADCSRA & (1 << ADSC));
 
    return ADCH;
}
 
void pwm_setup (void)
{
    // Set Timer 0 prescaler to clock/8.
    // At 9.6 MHz this is 1.2 MHz.
    // See ATtiny13 datasheet, Table 11.9.
    TCCR0B |= (1 << CS01);
 
    // Set to 'Fast PWM' mode
    TCCR0A |= (1 << WGM01) | (1 << WGM00);
 
    // Clear OC0B output on compare match, upwards counting.
    TCCR0A |= (1 << COM0B1);
}
 
void pwm_write (int val)
{
    OCR0B = val;
}
 
int main (void)
{
    int adc_in;
 
    // LED is an output.
    DDRB |= (1 << LED);  
 
    adc_setup();
    pwm_setup();
  
    while (1) {
        // Get the ADC value
        adc_in = adc_read();
        // Now write it to the PWM counter
        pwm_write(adc_in);
    }
}