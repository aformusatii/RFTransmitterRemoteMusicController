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
#define BT_PAUSE         0 // PC0
#define BT_PREV          1 // PC1
#define BT_PLAY          2 // PC2
#define BT_PREV_PLAYLIST 3 // PC3
#define BT_NEXT_PLAYLIST 4 // PB0
#define BT_STOP          5 // PD7
#define BT_NEXT          6 // PD6



/********************************************************************************
	Function Prototypes
********************************************************************************/
void initGPIO();
void powerOnRF();
void powerDownRF();
void handleButton();
void sendToChannel(uint8_t channel, uint8_t data);
void sendToChannel(uint8_t channel, uint8_t data1, uint8_t data2);
void goToSleep();

/********************************************************************************
	Global Variables
********************************************************************************/
volatile uint8_t vol = 0;
volatile uint8_t button = 0;
volatile uint8_t last_channel = 110;

RF24 radio;
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

bool ir_preamble_detected = false;
bool awake = true;
bool rfAwake = false;
bool volChanged = false;

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
    	if (vol != 255) {
    		vol+=5;
    	}
    	volChanged = true;
    } else if (port == 0b00100000) {
    	if (vol != 0) {
        	vol-=5;
    	}
    	volChanged = true;
    }
}

ISR(INT1_vect)
{
	// Keep the processor awake
    awake = true;

    uint8_t port1 = ~PINB;
    uint8_t port2 = ~PINC;
    uint8_t port3 = ~PIND;

    button = (port2 & 0b00001111); // mask for PC0 - PC3

    if (GET_REG1_FLAG(port1, PB0)) {
    	SET_REG1_FLAG(button, BT_NEXT_PLAYLIST);
    }

    if (GET_REG1_FLAG(port3, PD6)) {
    	SET_REG1_FLAG(button, BT_NEXT);
    }

    if (GET_REG1_FLAG(port3, PD7)) {
    	SET_REG1_FLAG(button, BT_STOP);
    }
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
    initTimer();

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

	// main loop
    while (1) {
    	// main usart loop for console
    	usart_check_loop();

    	if (awake) {
    		awake = false;
    		handleButton();
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

	_in(DDD7, DDRD);
	_in(DDD6, DDRD);
	_in(DDB0, DDRB);
	_in(DDC3, DDRC);
	_in(DDC2, DDRC);
	_in(DDC1, DDRC);
	_in(DDC0, DDRC);

	_on(PD7, PIND); // Pull-up
	_on(PD6, PIND); // Pull-up
	_on(PB0, PINB); // Pull-up
	_on(PC3, PINC); // Pull-up
	_on(PC2, PINC); // Pull-up
	_on(PC1, PINC); // Pull-up
	_on(PC0, PINC); // Pull-up

	_in(DDC4, DDRC); // Rotary Encoder A
	_in(DDC5, DDRC); // Rotary Encoder B

    _on(PC4, PINC); // Pull-up
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
        radio.powerUp();
        rfAwake = true;
    }
}

void powerDownRF() {
    // need to flush tx buffer, fixed the issue with packet shift...
    radio.stopListening();
    radio.powerDown();
    rfAwake = false;
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

void handleButton() {

    // Wake up the RF module
    powerOnRF();

	if (GET_REG1_FLAG(button, BT_PAUSE)) {
		sendToChannel(110, 23);
		//printf("\n BT_PAUSE");
	}

	if (GET_REG1_FLAG(button, BT_PREV)) {
		sendToChannel(110, 28);
		//printf("\n BT_PREV");
	}

	if (GET_REG1_FLAG(button, BT_PLAY)) {
		sendToChannel(110, 21);
		//printf("\n BT_PLAY");
	}

	if (GET_REG1_FLAG(button, BT_PREV_PLAYLIST)) {
		sendToChannel(110, 100);
		//printf("\n BT_PREV_PLAYLIST");
	}

	if (GET_REG1_FLAG(button, BT_NEXT_PLAYLIST)) {
		sendToChannel(110, 101);
		//printf("\n BT_NEXT_PLAYLIST");
	}

	if (GET_REG1_FLAG(button, BT_STOP)) {
		sendToChannel(110, 20);
		//printf("\n BT_STOP");
	}

	if (GET_REG1_FLAG(button, BT_NEXT)) {
		sendToChannel(110, 27);
		//printf("\n BT_NEXT");
	}

	if (volChanged) {
		sendToChannel(116, 102, vol);
		//printf("\n vol = %d", vol);
	}

	volChanged = false;
	button = 0;
}

void goToSleep() {
	//printf("\n sleep...");

	// Power down the RF module
	powerDownRF();

	_delay_ms(10);

	sleep_mode();

	//printf("\n wake...");
}

void handle_usart_cmd(char *cmd, char *args) {
	if (strcmp(cmd, "test") == 0) {
		printf("\n TEST [%s]", args);
	}
}
