/*
 * 16-Channel ModularEEG firmware for one-way transmission, v0.1-p2 for ATmega1280 (16-Ch EEG) at 11059,2 kHz
 * Modified by: Bastian Holtermann
 * Date: March 2008
 * License: GNU General Public License (GPL) v3
 * Compiles with AVR-GCC v4.2.2
 * This is a modified Version of:
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * ModularEEG firmware for one-way transmission, v0.5.4-p2                                  *
 * Copyright (c) 2002-2003, Joerg Hansmann, Jim Peters, Andreas Robinson         *
 * License: GNU General Public License (GPL) v2                                                        *
 * Compiles with AVR-GCC v3.3.                                                                                     *
 *                                                                                                                                        *
 * Note: -p2 in the version number means this firmware is for packet version 2.     *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 */

//////////////////////////////////////////////////////////////

/*

////////// Packet Format  ////////////

// 28-byte packets are transmitted from the ModularEEG at 256Hz,
// using 1 start bit, 8 data bits, 1 stop bit, no parity, 115200 bits per second.

// Minimial transmission speed is 256Hz * sizeof(modeeg_packet) * 10 = 71680 bps.

struct modeeg_packet
{
	uint8_t		sync0;	// = 0xa5
	uint8_t		sync1;	// = 0x5a
	uint8_t		count;	// packet counter. Increases by 1 each packet.
	uint16_t		data[16];	// 10-bit sample (= 0 - 1023) 
	uint8_t		switches;	// State of PA6 to PA3, in bits 3 to 0.
};


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

#define NUMCHANNELS 16
#define HEADERLEN 3
//2 Channels share 3 Bytes in Packet + 3 Bytes HEADERLEN + 1 Byte for the footer
#define PACKETLEN (((NUMCHANNELS/2)*3) + HEADERLEN + 1)

#define SAMPFREQ 256
#define TIMER0VAL 256 - ((11059200 / 256) / SAMPFREQ)

char const channel_order[]= { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };

/** The transmission packet */
volatile uint8_t TXBuf[PACKETLEN];

/** Next byte to read or write in the transmission packet. */
volatile uint8_t TXIndex;  

/** Current channel being sampled. */
volatile uint8_t CurrentCh;

volatile uint8_t i;


/** Sampling timer (timer 0) interrupt handler */
ISR(TIMER0_OVF_vect)
{
    TCNT0 = TIMER0VAL; //Reset timer to get correct sampling frequency.
	
    //Reset channelcounter
	CurrentCh = 0;
	i = 4;

    // Increase packet counter (third byte in header)
    TXBuf[2]++;

    //Get state of switches on PA3..6, if any (last byte in packet).
    TXBuf[((NUMCHANNELS/2)*3) + HEADERLEN] = (PINA >> 3) &0x0F;

   	UCSR0B &= ~(1 << UDRIE0);  //Ensure UART IRQ's are disabled.
    ADCSRA |= (1 << ADIF);     //Reset any pending ADC interrupts
    ADCSRA |= (1 << ADIE);     //Enable ADC interrupts.

    //The ADC will start sampling automatically as soon
    //as sleep is executed in the main-loop.
}


/** AD-conversion-complete interrupt handler. */
ISR(ADC_vect)
{
    
	if ((CurrentCh % 2) == 0)
    {
		TXBuf[i-1] = ADCL;
		TXBuf[i] = ADCH;
    }
    else
    {
        TXBuf[i+1] = ADCL;
		TXBuf[i] |= (ADCH << 6);
		i += 3;
    }

    //Increase channel counter
	CurrentCh++;

    if (CurrentCh < NUMCHANNELS)
    {
        //Select the next channel.
		//If the channel number (not the counter) is lower than 8, the MUX5 bit has to be cleared
        if(channel_order[CurrentCh] < 8)
        {
            //Clear the MUX5 bit for channels 0-7
			ADCSRB &= ~(1 << MUX5);
			//Set the MUX0-4 bits to the right value for channel 0-7
            ADMUX = channel_order[CurrentCh]; 
        }
        //channel number is higher than 7
		else
        {
            //Set the MUX5 bit for channels 8-15
			ADCSRB |= (1 << MUX5);
			//Set the MUX0-4 bits to the right value for channel 0-7
            ADMUX = (channel_order[CurrentCh] - 8);
        }
        //The next sampling is started automatically.
    }
    else
    {
        //Prepare next conversion, on channel 0.
        ADMUX &= ~((1 << MUX0) | (1 << MUX1) | (1 << MUX2) | (1 << MUX3) | (1 << MUX4));
        ADCSRB &= ~(1 << MUX5);

        // Disable ADC interrupts to prevent further calls to SIG_ADC.
        ADCSRA &= ~(1 << ADIE);
		
        // Hand over to SIG_UART_DATA, by starting
        // the UART transfer and enabling UDR IRQ's.

        UDR0 = TXBuf[0];
        UCSR0B |= (1 << UDRIE0);
        TXIndex = 1;
    }
}

/*** UART data transmission register-empty interrupt handler ***/
ISR(USART0_UDRE_vect)
{
    UDR0 = TXBuf[TXIndex]; //Send next byte
    TXIndex++;
    if (TXIndex == PACKETLEN)   //See if we're done with this packet
    {
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

    OCR1AH = 0x02; 	// Set OCR1A = 512
    OCR1AL = 0x00;
    TCCR1A |= (1 << COM1A1) | (1 << WGM11) | (1 << WGM10); 	// Set 10-bit PWM mode
    TCCR1B |= (1 << CS12); 	// Start and let run at clk / 256 Hz.

}

int main( void )
{
    //Write packet header
    TXBuf[0] = 0xa5;        //Sync 0
    TXBuf[1] = 0x5a;        //Sync 1
    TXBuf[2] = 0x00;        //Packet counter

    //Set up the ports.
	//Write a 1 for pin is output, 0 for pin is input
    DDRA = 0x00;
    DDRB = 0x60;
	DDRC = 0x00;
	DDRD = 0x70;
	DDRE = 0x02;
	DDRG = 0x00;
	DDRH = 0x00;
	DDRJ = 0x00;
	DDRL = 0x00;
	//Activate pull-up resistor for all pins that are inputs
    PORTA = 0xff;
    PORTB = 0xff;
	PORTC = 0xff;
	PORTD = 0xff;
	PORTE = 0xff;
	PORTG = 0xff;
	PORTH = 0xff;
	PORTJ = 0xff;
	PORTL = 0xff;
	

    //Select sleep mode = idle.
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

    //Select channel 0
    ADMUX &= ~((1 << MUX0) | (1 << MUX1) | (1 << MUX2) | (1 << MUX3) | (1 << MUX4));
    ADCSRB &= ~(1 << MUX5);

    //Prescaler = 64, free running mode = off, interrupts off.
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1);
    ADCSRA |= (1 << ADIF); 	//Reset any pending ADC interrupts
    ADCSRA |= (1 << ADEN);  //Enable the ADC

    //Initialize the USART
	UBRR0H = 0x00; 		//Set speed to 115200 bps
    UBRR0L = 0x05;
	UCSR0A = 0x00;
	UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);
    UCSR0B |= (1 << TXEN0);


    //Initialize timer 0    

    TCNT0 = 0x00;	//Clear it.
	TCCR0B = (1 << CS02);  	//Start it. Frequency = clk / 256
    TIMSK0 |= (1 << TOIE0); //Enable the interrupts.

    //Initialize PWM (optional)
    pwm_init();
   
    //Enables interrupts by setting the global interrupt mask.
    sei();

    //Now, we wait. This is an event-driven program, so nothing much
    //happens here.

    while (1)
    {
        __asm__ __volatile__ ("sleep");
    }
}
