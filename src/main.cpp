/********************************************************************************
	Includes
********************************************************************************/

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <util/delay.h>

#include "../nrf24l01/RF24.h"
#include "../atmega328/mtimer.h"
#include "../common/util.h"

extern "C" {
#include "../atmega328/usart.h"
}

/********************************************************************************
	Macros and Defines
********************************************************************************/
#define PI_TWO_CHANNEL   110
#define SENSOR_DATA_KEY  100 // PI TWO knows this is sensor data and not a command
#define SENSOR_ID        6   // current sensor identifier for PI Database
#define EVENT_BATTERY    4   // sensor battery event stored in database
#define EVENT_START      5   // Start MCU event

#define BT_PAUSE         PD4 // PD4
#define BT_PREV          PC1 // PC1
#define BT_PLAY          PC2 // PC2
#define BT_PREV_PLAYLIST PC3 // PC3
#define BT_NEXT_PLAYLIST PB0 // PB0
#define BT_STOP          PD7 // PD7
#define BT_NEXT          PD6 // PD6

#define VOLUME_MAX   79

#define VOLATAGE_COEFFICIENT 4.48 // 1.9v -> 425, 2.0v -> 448, 2.5 -> 560, 3.1 -> 668

/********************************************************************************
	Function Prototypes
********************************************************************************/
void initGPIO();
void powerOnRF();
void powerDownRF();
void handleButton();
void sendToChannel(uint8_t channel, uint8_t data);
void sendToChannel(uint8_t channel, uint8_t data1, uint8_t data2);
void sendToChannel(uint8_t event, uint8_t channel, uint8_t data_high, uint8_t data_low);
void goToSleep();
void measureAndSendBatteryLevel();
uint16_t adc_read(uint8_t adcx);

/********************************************************************************
	Global Variables
********************************************************************************/
volatile uint8_t last_channel = PI_TWO_CHANNEL;
volatile uint8_t volume = 0;

RF24 radio;
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

volatile bool ir_preamble_detected = false;
volatile bool awake = true;
volatile bool rfAwake = false;
volatile bool volChanged = false;
volatile bool volUp = false;
volatile bool btPress = false;

volatile bool pausePress = false;
volatile bool prevPress = false;
volatile bool playPress = false;
volatile bool prevPlaylistPress = false;
volatile bool nextPlaylistPress = false;
volatile bool stopPress = false;
volatile bool nextPress = false;

/********************************************************************************
	Interrupt Service
********************************************************************************/
ISR(USART_RX_vect)
{
	handle_usart_interrupt();
}

ISR(TIMER1_OVF_vect)
{
	incrementOvf();
}

ISR(INT0_vect)
{
    // Keep the processor awake
    awake = true;

    uint8_t port = PINC & 0b00110000; // mask for PC5 & PC4
    if (port == 0b00010000) {
    	volUp = true;
    	volChanged = true;
    } else if (port == 0b00100000) {
    	volUp = false;
    	volChanged = true;
    }
}

ISR(INT1_vect)
{
	// Keep the processor awake
    awake = true;

    uint8_t portB = ~PINB;
    uint8_t portC = ~PINC;
    uint8_t portD = ~PIND;

    pausePress =        GET_REG1_FLAG(portD, BT_PAUSE);
    prevPress =         GET_REG1_FLAG(portC, BT_PREV);
    playPress =         GET_REG1_FLAG(portC, BT_PLAY);
    prevPlaylistPress = GET_REG1_FLAG(portC, BT_PREV_PLAYLIST);
    nextPlaylistPress = GET_REG1_FLAG(portB, BT_NEXT_PLAYLIST);
    stopPress =         GET_REG1_FLAG(portD, BT_STOP);
    nextPress =         GET_REG1_FLAG(portD, BT_NEXT);

    btPress = true;
}

ISR(TIMER2_OVF_vect)
{
	_NOP();
}

/********************************************************************************
	Main
********************************************************************************/
int main(void) {

    // initialize usart module
	usart_init();

    // Init Timer 1
    //initTimer();

    // Init GPIO
    initGPIO();

    // enable interrupts
    sei();

    // Configure Sleep Mode - Power-down
    SMCR = (0<<SM2)|(1<<SM1)|(0<<SM0)|(0<<SE);

	// Console friendly output
    printf("Start...");
    printf(CONSOLE_PREFIX);

    // Init NRF24L01+
    radio.begin();
    radio.setRetries(15,15);
    radio.setPayloadSize(8);
    radio.setPALevel(RF24_PA_MAX);
    radio.setChannel(last_channel);

    radio.openWritingPipe(pipes[0]);
    radio.openReadingPipe(1,pipes[1]);

    // Some RF module diagnostics logs
    radio.printDetails();

    sendToChannel(PI_TWO_CHANNEL, EVENT_START, 0, 0);

	// main loop
    while (1) {
    	// main usart loop for console
    	usart_check_loop();

    	if (awake) {
    		awake = false;

    	    // Wake up the RF module
    	    powerOnRF();

    		handleButton();

    		if (btPress) {
    			btPress = false;

    			_delay_ms(100);

    			measureAndSendBatteryLevel();
    		}

    	} else {
    		// Can we sleep? we are hungry when awake...
    		goToSleep();
    	}
    }
}

/********************************************************************************
	Functions
********************************************************************************/
void initGPIO() {

	_in(DDD4, DDRD); // BT_PAUSE
	_on(PD4, PIND); // Pull-up

	_in(DDC1, DDRC); // BT_PREV
	_on(PC1, PINC); // Pull-up

	_in(DDC2, DDRC); // BT_PLAY
	_on(PC2, PINC); // Pull-up

	_in(DDC3, DDRC); // BT_PREV_PLAYLIST
	_on(PC3, PINC); // Pull-up

	_in(DDB0, DDRB); // BT_NEXT_PLAYLIST
	_on(PB0, PINB); // Pull-up

	_in(DDD7, DDRD); // BT_STOP
	_on(PD7, PIND); // Pull-up

	_in(DDD6, DDRD); // BT_NEXT
	_on(PD6, PIND); // Pull-up

	_in(DDC0, DDRC); // Battery voltage - analog in
	// _on(PC0, PINC); // Pull-up

	_in(DDC4, DDRC); // Rotary Encoder A
	_on(PC4, PINC); // Pull-up

	_in(DDC5, DDRC); // Rotary Encoder B
    _on(PC5, PINC); // Pull-up

    _in(DDD2, DDRD); // INT0 input
    _in(DDD3, DDRD); // INT1 input

    _on(PD2, PIND); // Pull-up
    _on(PD3, PIND); // Pull-up

    // GPIO Interrupt INT0
    EICRA = (1<<ISC11)|(0<<ISC10)|(1<<ISC01)|(0<<ISC00); // The falling edge of INT0 and INT1 generates an interrupt request.
    EIMSK = (1<<INT1)|(1<<INT0); // Enable INT0
}

void powerOnRF() {
    if (!rfAwake) {
    	_out(SPI_CE, DDRB); // CE - power issue
        _out(DDB5, DDRB); // SCK - power issue

        radio.powerUp();
        rfAwake = true;
    }
}

void powerDownRF() {
    // need to flush tx buffer, fixed the issue with packet shift...
    radio.stopListening();
    radio.powerDown();
    rfAwake = false;

    _in(SPI_CE, DDRB); // CE - power issue
    _in(DDB5, DDRB); // SCK - power issue
}

void sendToChannel(uint8_t channel, uint8_t data) {
	// change channel if necessary
	if (channel != last_channel) {
		last_channel = channel;
		radio.setChannel(channel);
	}

    // Send data via RF module
    uint8_t buf[] = {data};
    radio.write(buf, 1);
}

void sendToChannel(uint8_t channel, uint8_t data1, uint8_t data2) {
	// change channel if necessary
	if (channel != last_channel) {
		last_channel = channel;
		radio.setChannel(channel);
	}

    // Send data via RF module
    uint8_t buf[] = {data1, data2};
    radio.write(buf, 2);
}

void sendToChannel(uint8_t channel, uint8_t event, uint8_t data_high, uint8_t data_low) {
	// change channel if necessary
	if (channel != last_channel) {
		last_channel = channel;
		radio.setChannel(channel);
	}

    uint8_t data[] = { SENSOR_DATA_KEY, SENSOR_ID, event, data_high, data_low };
    radio.write(data, 5);
}

void handleButton() {

	if (pausePress) {
		sendToChannel(PI_TWO_CHANNEL, 23);
	}

	if (prevPress) {
		sendToChannel(PI_TWO_CHANNEL, 28);
	}

	if (playPress) {
		sendToChannel(PI_TWO_CHANNEL, 21);
	}

	if (prevPlaylistPress) {
		sendToChannel(PI_TWO_CHANNEL, 101);
	}

	if (nextPlaylistPress) {
		sendToChannel(PI_TWO_CHANNEL, 102);
	}

	if (stopPress) {
		sendToChannel(PI_TWO_CHANNEL, 103);
	}

	if (nextPress) {
		sendToChannel(PI_TWO_CHANNEL, 27);
	}

	if (volChanged) {
		//sendToChannel(124, volUp ? 100 : 101);
		if (volUp) {
	    	if (volume > 1) {
	    		volume -= 2;
	    	}
		} else {
			if (volume < VOLUME_MAX) {
				volume += 2;
			}
		}
		sendToChannel(124, 102, volume);
	}

    pausePress =        false;
    prevPress =         false;
    playPress =         false;
    prevPlaylistPress = false;
    nextPlaylistPress = false;
    stopPress =         false;
    nextPress =         false;

	volChanged = false;

	_delay_ms(10);
}

void goToSleep() {
	//printf("\n sleep...");

	// Power down the RF module
	powerDownRF();

	_delay_ms(10);

	sleep_mode();

	//printf("\n wake...");
}

void measureAndSendBatteryLevel() {
	_on(PC0, PORTC); // enable pull-up resistor for ADC battery level
	_delay_ms(1); // wait voltage to raise

    ADCSRA |= _BV(ADEN); // Enable the ADC
    _delay_ms(5);

    uint16_t mux0Value = adc_read(MUX0);

    uint16_t voltage = (uint16_t) ( (( ((uint32_t)mux0Value) * 100) * (VOLATAGE_COEFFICIENT * 100)) / 10000 );

    uint8_t b_low = (uint8_t) voltage;
    uint8_t b_high = (uint8_t) (voltage >> 8);

    // Send battery level via NRF24L01 transceiver
    sendToChannel(PI_TWO_CHANNEL, EVENT_BATTERY, b_high, b_low);

    ADCSRA &= ~_BV(ADEN); // Disable ADC

	_off(PC0, PORTC); // disable pull-up resistor for ADC battery level
}

uint16_t adc_read(uint8_t adcx) {
    /* adcx is the analog pin we want to use.  ADMUX's first few bits are
     * the binary representations of the numbers of the pins so we can
     * just 'OR' the pin's number with ADMUX to select that pin.
     * We first zero the four bits by setting ADMUX equal to its higher
     * four bits. */
    ADMUX = adcx;
    ADMUX |= (1 << REFS1) | (1 << REFS0) | (0 << ADLAR);

    _delay_us(300);

    /* This starts the conversion. */
    ADCSRA |= _BV(ADSC);

    /* This is an idle loop that just wait around until the conversion
     * is finished.  It constantly checks ADCSRA's ADSC bit, which we just
     * set above, to see if it is still set.  This bit is automatically
     * reset (zeroed) when the conversion is ready so if we do this in
     * a loop the loop will just go until the conversion is ready. */
    while ((ADCSRA & _BV(ADSC)))
        ;

    /* Finally, we return the converted value to the calling function. */
    return ADC;
}

void handle_usart_cmd(char *cmd, char *args) {
	if (strcmp(cmd, "test") == 0) {
		printf("\n TEST [%s]", args);
	}
}
