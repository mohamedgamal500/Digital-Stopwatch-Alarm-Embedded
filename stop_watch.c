/*
 * stop_watch.c
 *
 *  Author: Mohamed Gamal
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#define SIZE 6
#define ALARM_seconds 0
#define ALARM_minutes 1 // set alarm after 5 minutes
#define ALARM_hours 0
/* global variables for alarm */
volatile unsigned char g_alarm_flag = 0;
volatile unsigned char g_alarm_seconds = 0;
volatile unsigned char g_alarm_minutes = 0;
volatile unsigned char g_alarm_hours = 0;
/* global variables for clk */
volatile unsigned char g_clk_seconds = 0;
volatile unsigned char g_clk_minutes = 0;
volatile unsigned char g_clk_hours = 0;

/* External INT0 Interrupt Service Routine */
ISR(INT0_vect) {
	// reset
	g_clk_seconds = 0;
	g_clk_minutes = 0;
	g_clk_hours = 0;
	// disable alarm sound
	g_alarm_flag = 0;
}
/* External INT0 enable and configuration function */
void INT0_Init(void) {
	DDRD &= (~(1 << PD2));				  // Configure INT0/PD2 as input pin
	PORTD |= (1 << PD2);     // Enable the internal pull up resistor at PD2 pin
	GICR |= (1 << INT0);				  // Enable external interrupt pin INT0
	// Trigger INT0 with the falling edge
	MCUCR |= (1 << ISC01);
	MCUCR &= ~(1 << ISC00);
}

/* External INT1 Interrupt Service Routine */
ISR(INT1_vect) {
	// stop clk
	TCCR1B &= ~(1 << CS12) & ~(1 << CS11) & ~(1 << CS10);
	// disable alarm sound
	g_alarm_flag = 0;
}
/* External INT1 enable and configuration function */
void INT1_Init(void) {

	DDRD &= (~(1 << PD3));  // Configure INT1/PD3 as input pin
	GICR |= (1 << INT1);    // Enable external interrupt pin INT1
	MCUCR |= (1 << ISC11) | (1 << ISC10); // Trigger INT1 with the raising edge

}

/* External INT2 Interrupt Service Routine */
ISR(INT2_vect) {

	// start clk
	TCCR1B |= (1 << CS12) | (1 << CS10);
	// disable alarm sound
	g_alarm_flag = 0;
}
/* External INT2 enable and configuration function */
void INT2_Init(void) {

	DDRB &= (~(1 << PB2));   // Configure INT2/PB2 as input pin
	PORTB |= (1 << PB2);      // Enable the internal pull up resistor at PD2 pin
	GICR |= (1 << INT2);	 // Enable external interrupt pin INT2
	MCUCSR &= ~(1 << ISC2);     // Trigger INT2 with the raising edge

}

/* Interrupt Service Routine for timer1 compare mode */
ISR(TIMER1_COMPA_vect) {

	g_clk_seconds++;
	if (g_clk_seconds == 60) {
		g_clk_minutes++;
		g_clk_seconds = 0;
	}

	if (g_clk_minutes == 60) {
		g_clk_hours++;
		g_clk_minutes = 0;
	}

	if (g_clk_hours == 24) {     // clear display after 24 hours and stop timer
		g_clk_seconds = 0;
		g_clk_minutes = 0;
		g_clk_hours = 0;
		// stop clk
		TCCR1B &= ~(1 << CS12) & ~(1 << CS11) & ~(1 << CS10);

	}

	g_alarm_seconds++;
	if (g_alarm_seconds == 60) {
		g_alarm_minutes++;
		g_alarm_seconds = 0;
	}

	if (g_alarm_minutes == 60) {
		g_alarm_hours++;
		g_alarm_minutes = 0;
	}

	if (g_alarm_seconds == ALARM_seconds && g_alarm_minutes == ALARM_minutes
			&& g_alarm_hours == ALARM_hours) {
		// set alarm
		g_alarm_flag = 1;
		// stop clk
		TCCR1B &= ~(1 << CS12) & ~(1 << CS11) & ~(1 << CS10);
	}

}

void timer1_init_CTC_mode(unsigned short tick) {
	TCNT1 = 0;    // Set Timer initial value to 0
	OCR1A = tick; // Set Compare Value
	TIMSK |= (1 << OCIE1A); // Enable Timer1 Compare Interrupt
	/* Configure timer1 control register
	 * 2. CTC Mode WGM13=0 & WGM12=1 & WGM11=0 & WGM10=0
	 * 4. clock = F_CPU/1024 CS12=1 CS11=0 CS10=1
	 */
	TCCR1A = (1 << FOC1A) | (1 << FOC1B);
	TCCR1B = (1 << WGM12) | (1 << CS12) | (1 << CS10);
}

void display(unsigned char digit[], int n) {

	digit[0] = g_clk_seconds % 10; //1st digit for seconds
	digit[1] = g_clk_seconds / 10; //2nd digit for seconds
	digit[2] = g_clk_minutes % 10; //1st digit for minutes
	digit[3] = g_clk_minutes / 10; //2nd digit for minutes
	digit[4] = g_clk_hours % 10; //1st digit for hours
	digit[5] = g_clk_hours / 10; //2nd digit for hours
	unsigned char i = 0;
	for (i = 0; i < n; i++) { // 7-segment multiplexing
		PORTC = (PORTC & 0xF0) | (digit[i] & 0x0F);
		PORTA |= (1 << i); // enable for 1 us
		_delay_us(1);
		PORTA &= ~(1 << i); // disable for 1 us
		_delay_us(1);
	}
}
int main(void) {
	DDRA |= 0x0F | (1 << 4) | (1 << 5) | (1 << 6);
	DDRC |= 0x0F; // Configure the first four pins in PORTC as output pins.
	PORTC &= 0xF0; // Initialize the 7-seg display zero at the beginning..
	SREG |= (1 << 7); // Enable global interrupts in MC.
	INT0_Init(); // Enable external INT0
	INT1_Init(); // Enable external INT1
	INT2_Init(); // Enable external INT2
	timer1_init_CTC_mode(977); // Start the timer.
	volatile unsigned char g_seven_sgement_digit[SIZE];

	while (1) {
		display(g_seven_sgement_digit, SIZE);
		if (g_alarm_flag == 1) {

			PORTA |= (1 << 6); // set alarm

		} else {
			PORTA &= ~(1 << 6);
		}
	}
}
