//	Nabíječka na 2S/3S li-pol články s 5V USB zdroje.
//	(c) Matěj Novotný 2014		
//	Určeno pro ATTiny 25/45/85


//#define F_CPU 1000000UL
//----- Include Files ---------------------------------------------------------
#include <avr/io.h> // include I/O definitions (port names, pin names, etc)
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/power.h>

//----- Begin Code ------------------------------------------------------------

// (8,23 V + 0,095V) /57*10/2,56*1024
#define TARGET_VOLTAGE 584

#define PWM_TOP 255

#define LED PB4
#define PWM PB1

uint16_t voltage = 0;

//initialize watchdog
void WDT_Init(void) {
	//disable interrupts
	cli();
	//reset watchdog
	wdt_reset();
	//set up WDT interrupt
	WDTCR = (1<<WDCE)|(1<<WDE);
	//Start watchdog timer with 0,25s prescaller
	WDTCR = (1<<WDIE)|(1<<WDE)|(1<<WDP2);
	//Enable global interrupts
	sei();
}

void measure_voltage(void) {
	uint16_t voltage_sum = 0;
	uint8_t i;
	power_adc_enable();
	// setup A/D convertor
	ADMUX = (1<< REFS2) | (1<< REFS1) | (1<< REFS0) | (1<<MUX1) | (1<<MUX0); // PB3, right aligned, int ref 2,56 V with cap
	
	for (i = 0; i < 8; i++) {
		ADCSRA |= (1<<ADEN) | (1<<ADSC) | (1<<ADPS2); // enable and start conversion, 1:16 clock
		while (! (ADCSRA & (1<<ADIF)));	// wait for conversion complete
		voltage_sum += ADCL + (ADCH << 8); // read value
	}
	voltage = (voltage_sum >> 3);
	ADCSRA &= ~(1<<ADEN); // disable converter to save power
	power_adc_disable();
}

void timer_enable(void) {
	power_timer0_enable();
    TCCR0A =  (1<<COM0B1) | (1<<WGM00) | (1<<WGM01);
    TCCR0B = (1<<CS00);
	OCR0B = 0;
}


ISR(WDT_vect) {
	WDTCR |= (1<<WDIE);
}

int main(void) {
	power_all_disable();
	WDT_Init();
	timer_enable();
	set_sleep_mode(SLEEP_MODE_IDLE);
	DDRB = (1<<LED) | (1<<PWM);
	PORTB &= ~((1<<LED) | (1<<PWM));
	PORTB |= (1<<PB2);
	while(1) {
		measure_voltage();
		if (voltage >= TARGET_VOLTAGE) {
			PORTB |= (1<<LED);
		} else {
			PORTB &= ~(1<<LED);
		}
		if (voltage < TARGET_VOLTAGE) {
			if (OCR0B > 0) {
				OCR0B--;
			}
		}
		if (voltage > TARGET_VOLTAGE) {
			if (OCR0B < PWM_TOP) {
				OCR0B++;
			}
		}
		
		sleep_enable();
		sleep_cpu();
		sleep_disable();
	}
	return 0;
}

