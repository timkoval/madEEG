/*
 * 6-Channel ModularEEG firmware for one-way transmission, v0.1-p2 for ATmega1280 (6-Ch EEG) at 11059,2 kHz
 * Modified by: Bastian Holtermann
 * Date: March 2008
 * License: GNU General Public License (GPL) v3
 * Compiles with AVR-GCC v4.2.2
 * This is a modified Version of:
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * ModularEEG firmware for one-way transmission, v0.5.4-p2                             
 * Copyright (c) 2002-2003, Joerg Hansmann, Jim Peters, Andreas Robinson        
 * License: GNU General Public License (GPL) v2                                                       
 * Compiles with AVR-GCC v3.3.                                                    
 *                                                                                                                                        
 * Note: -p2 in the version number means this firmware is for packet version 2.     
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 */

//////////////////////////////////////////////////////////////

/*

////////// Packet Format Version 2 ////////////

// 17-byte packets are transmitted from the ModularEEG at 256Hz,
// using 1 start bit, 8 data bits, 1 stop bit, no parity, 57600 bits per second.

// Minimial transmission speed is 256Hz * sizeof(modeeg_packet) * 10 = 43520 bps.

struct modeeg_packet
{
	uint8_t		sync0;	// = 0xa5
	uint8_t		sync1;	// = 0x5a
	uint8_t		version;	// = 2
	uint8_t		count;	// packet counter. Increases by 1 each packet.
	uint16_t	          data[6];	// 10-bit sample (= 0 - 1023) in big endian (Motorola) format.
	uint8_t		switches;	// State of PA6 to PA3, in bits 3 to 0.
};

// Note that data is transmitted in big-endian format.
// By this measure together with the unique pattern in sync0 and sync1 it is guaranteed, 
// that re-sync (i.e after disconnecting the data line) is always safe.

// At the moment communication direction is only from Atmel-processor to PC.
// The hardware however supports full duplex communication. This feature
// will be used in later firmware releases to support the PWM-output and
// LED-Goggles.

*/

//////////////////////////////////////////////////////////////

/*
 * Program flow:
 *
 * When 256Hz timer expires: goto ISR(TIMER0_OVF_vect)
 * ISR(TIMER0_OVF_vect) enables the ADC
 *
 * Repeat for each channel in the ADC: 
 * Sampling starts. When it completes: goto ISR(ADC_vect)
 * ISR(ADC_vect) reads the sample and restarts the ADC.
 *
 * ISR(ADC_vect) writes first byte to UART  0 data register
 * (UDR0) which starts the transmission over the serial port.
 *
 * Repeat for each byte in packet:
 * When transmission begins and UDR empties: goto ISR(USART0_UDRE_vect)
 *
 * Start over from beginning.
 */

#include <avr/io.h>
#include <inttypes.h>
#include <avr/interrupt.h>
//#include <avr/signal.h>

#define NUMCHANNELS 6
#define HEADERLEN 4
#define PACKETLEN (NUMCHANNELS * 2 + HEADERLEN + 1)

#define SAMPFREQ 256
#define TIMER0VAL 256 - ((11059200 / 256) / SAMPFREQ)

char const channel_order[]= { 10, 13, 11, 14, 12, 15 };

/** The transmission packet */

volatile uint8_t TXBuf[PACKETLEN];

/** Next byte to read or write in the transmission packet. */

volatile uint8_t TXIndex;  

/** Current channel being sampled. */

volatile uint8_t CurrentCh;

/** Sampling timer (timer 0) interrupt handler */

//SIGNAL(SIG_OVERFLOW0)
ISR(TIMER0_OVF_vect)
{
    //outb(TCNT0, TIMER0VAL); //Reset timer to get correct sampling frequency.
	TCNT0 = TIMER0VAL; //Reset timer to get correct sampling frequency.
	
    CurrentCh = 0;

    // Write header and footer:

    // Increase packet counter (fourth byte in header)

    TXBuf[3]++;

    //Get state of switches on PA3..6, if any (last byte in packet).

    //TXBuf[2 * NUMCHANNELS + HEADERLEN] = (inp(PINA) >> 3) &0x0F;
	TXBuf[2 * NUMCHANNELS + HEADERLEN] = (PINA >> 3) &0x0F;

    //cbi(UCSR0B, UDRIE0);      //Ensure UART IRQ's are disabled.
	UCSR0B &= ~(1 << UDRIE0); //Ensure UART IRQ's are disabled.
    //sbi(ADCSRA, ADIF);       //Reset any pending ADC interrupts
	ADCSRA |= (1 << ADIF); //Reset any pending ADC interrupts
    //sbi(ADCSRA, ADIE);       //Enable ADC interrupts.
	ADCSRA |= (1 << ADIE);  //Enable ADC interrupts.

    //The ADC will start sampling automatically as soon
    //as sleep is executed in the main-loop.
}

/** AD-conversion-complete interrupt handler. */

//SIGNAL(SIG_ADC)
ISR(ADC_vect)
{
    volatile uint8_t i;

    i = 2 * CurrentCh + HEADERLEN;

    //TXBuf[i+1] = inp(ADCL);
	TXBuf[i+1] = ADCL;
    //TXBuf[i] = inp(ADCH);
	TXBuf[i] = ADCH;

    CurrentCh++;

    if (CurrentCh < NUMCHANNELS)
    {
		//Select the next channel.
		//The right value for ADMUX  e.g. for Channel 10 is 2 and not 10, so 8 need to be sutracted
		ADMUX = (channel_order[CurrentCh] - 8);
		
		//The next sampling is started automatically.
    }
    else
    {
        //outb(ADMUX, channel_order[0]);      //Prepare next conversion, on channel 10.
		//Prepare next conversion, on channel 10.
        ADMUX |= (1 << MUX1);
	    ADMUX &= ~((1 << MUX0) | (1 << MUX2) | (1 << MUX3) | (1 << MUX4));
		ADCSRB |= (1 << MUX5);
		
        // Disable ADC interrupts to prevent further calls to SIG_ADC.
        //cbi(ADCSRA, ADIE);
		ADCSRA &= ~(1 << ADIE);
		
        // Hand over to SIG_UART_DATA, by starting
        // the UART transfer and enabling UDR IRQ's.

        //outb(UDR0, TXBuf[0]); 
		UDR0 = TXBuf[0];
        //sbi(UCSR0B, UDRIE0);
		UCSR0B |= (1 << UDRIE0);
        TXIndex = 1;
    }
}

/*** UART data transmission register-empty interrupt handler ***/

//SIGNAL(SIG_UART_DATA)
ISR(USART0_UDRE_vect)
{
    //outb(UDR0, TXBuf[TXIndex]);  //Send next byte
	UDR0 = TXBuf[TXIndex]; //Send next byte
    TXIndex++;
    if (TXIndex == PACKETLEN)   //See if we're done with this packet
    {
        //cbi(UCSR0B, UDRIE0);      //Disable SIG_UART_DATA interrupts.
		UCSR0B &= ~(1 << UDRIE0); //Disable SIG_UART_DATA interrupts.
                                //Next interrupt will be a SIG_OVERFLOW0.
    }
}

/** Initialize PWM output (PB1 = 14Hz square wave signal) */

void pwm_init(void)
{
	// Set timer/counter 1 to use 10-bit PWM mode.
	// The counter counts from zero to 1023 and then back down
	// again. Each time the counter value equals the value
	// of OCR1(A), the output pin is toggled.
	// The counter speed is set in TCCR1B, to clk / 256 = 28800Hz.
	// Effective frequency is then clk / 256 / 2046 = 21,11 Hz

    //outb(OCR1AH,2);	// Set OCR1A = 512
	OCR1AH = 0x02; 	// Set OCR1A = 512
    //outb(OCR1AL,0);
	OCR1AL = 0x00;
    //outb(TCCR1A, BV(COM1A1) + BV(WGM11) + BV(WGM10)); 					// Set 10-bit PWM mode
	TCCR1A |= (1 << COM1A1) | (1 << WGM11) | (1 << WGM10); 	// Set 10-bit PWM mode
    //outb(TCCR1B, (1 << CS12));		// Start and let run at clk / 256 Hz.
	TCCR1B |= (1 << CS12); 	// Start and let run at clk / 256 Hz.

}

int main( void )
{
    //Write packet header and footer

    TXBuf[0] = 0xa5;        //Sync 0
    TXBuf[1] = 0x5a;        //Sync 1
    TXBuf[2] = 2;           //Protocol version
    TXBuf[3] = 0;           //Packet counter

    //Set up the ports.

    //outb(DDRA,  0x00);
	DDRA = 0x00;
    //outb(DDRB,  0x60);
	DDRB = 0x60;
	//outb(DDRC,  0x00);
	DDRC = 0x00;
	//outb(DDRD,  0x70);
	DDRD = 0x70;
	//outb(DDRE,  0x02);
	DDRE = 0x02;
	//outb(DDRG,  0x00);
	DDRG = 0x00;
	//outb(DDRH,  0x00);
	DDRH = 0x00;
	//outb(DDRJ,  0x00);
	DDRJ = 0x00;
	//outb(DDRL,  0x00);
	DDRL = 0x00;
    //outb(PORTA, 0xff);
	PORTA = 0xff;
    //outb(PORTB, 0xff);
	PORTB = 0xff;
	//outb(PORTC, 0xff);
	PORTC = 0xff;
	//outb(PORTD, 0xff);
	PORTD = 0xff;
	//outb(PORTE, 0xff);
	PORTE = 0xff;
	//outb(PORTG, 0xff);
	PORTG = 0xff;
	//outb(PORTH, 0xff);
	PORTH = 0xff;
	//outb(PORTJ, 0xff);
	PORTJ = 0xff;
	//outb(PORTL, 0xff);
	PORTL = 0xff;
	

    //Select sleep mode = idle.

	//outb(SMCR,(inp(SMCR) | BV(SE)) & (~BV(SM0) | ~BV(SM1) | ~BV(SM2)));
	SMCR |= (1 << SE);
	SMCR &= ~((1 << SM0) | (1 << SM1) | (1 << SM2));

    //Initialize the ADC

    // Timings for sampling of one 10-bit AD-value:
    //
    // prescaler > ((XTAL / 200kHz) = 55.296 => 
    // prescaler = 64 (ADPS2 = 1, ADPS1 = 1, ADPS0 = 0)
    // ADCYCLE = XTAL / prescaler = 172800Hz or 5.787 us/cycle
    // 14 (single conversion) cycles = 81,01 us (12344 samples/sec)
    // 26 (1st conversion) cycles = 150.46 us

    //Select channel 10
	//outb(ADMUX, (inp(ADMUX) | BV(MUX1)) & (~BV(MUX0) | ~BV(MUX2) | ~BV(MUX3) | ~BV(MUX4)); 
	ADMUX |= (1 << MUX1);
	ADMUX &= ~((1 << MUX0) | (1 << MUX2) | (1 << MUX3) | (1 << MUX4));
	//outb(ADCSRB,(inp(ADCSRB) | BV(MUX5)));
	ADCSRB |= (1 << MUX5);

    //Prescaler = 64, free running mode = off, interrupts off.

    //outb(ADCSRA, BV(ADPS2) | BV(ADPS1));
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1);
    //sbi(ADCSRA, ADIF);      		 //Reset any pending ADC interrupts
	ADCSRA |= (1 << ADIF); 	//Reset any pending ADC interrupts
    //sbi(ADCSRA, ADEN);      		 //Enable the ADC
	ADCSRA |= (1 << ADEN);  //Enable the ADC

    //Initialize the UART

	//outb(UBRR0H,0x00);              //Set speed to 57600 bps
	UBRR0H = 0x00; 		//Set speed to 57600 bps
    //outb(UBRR0L,0x0b);
	UBRR0L = 0x0b;
	//outb(UCSR0A, 0);
	UCSR0A = 0x00;
	//outb(UCSR0C, BV(UCSZ01) | BV(UCSZ00));
	UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);
    //outb(UCSR0B, BV(TXEN0));
	UCSR0B |= (1 << TXEN0);


    //Initialize timer 0    

    //outb(TCNT0, 0);            //Clear it.
	TCNT0 = 0x00;	//Clear it.
	//outb(TCCR0B, 4);             		//Start it. Frequency = clk / 256
	TCCR0B = (1 << CS02);  	//Start it. Frequency = clk / 256
    //outb(TIMSK0, BV(TOIE0));        	//Enable the interrupts.
	TIMSK0 |= (1 << TOIE0); //Enable the interrupts.

    //Initialize PWM (optional)

    pwm_init();

    sei();

    //Now, we wait. This is an event-driven program, so nothing much
    //happens here.

    while (1)
    {
        __asm__ __volatile__ ("sleep");
    }
}
