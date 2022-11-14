/* 
 * File:   main.c
 * Author: JJ1MDY
 *
 * Created on 2022/11/04, 21:54
 */

// PIC16F1769 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = HS        // Oscillator Selection Bits (HS Oscillator, High-speed crystal/resonator connected between OSC1 and OSC2 pins)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover Mode (Internal/External Switchover Mode is disabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PPS1WAY = ON     // Peripheral Pin Select one-way control (The PPSLOCK bit cannot be cleared once it is set by software)
#pragma config ZCD = OFF        // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR)
#pragma config PLLEN = ON       // Phase Lock Loop enable (4x PLL is always enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)

#define _XTAL_FREQ 32000000

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "myLCD_sc1602.h"

#define SAMPLING_FREQ 32000 // up to 100 kHz
#define FREQ_STEP 40
#define SINE_TABLE_N 200    // SAMPLING_FREQ / FREQ_STEP / 4

#define RE_A 
#define RE_B
#define LED1
#define SW3
#define SW4
#define SW5

typedef enum {
    Sinusoidal,
    Pulse,
    Triangle,
    Saw,
} Wavetype;

typedef struct {
    uint16_t freq;
    uint8_t amp;
    uint8_t index;
    uint8_t index_step;
    uint8_t stage;
} wave;

void Timer_init(void);
void DAC_init(void);
void DAC_valueset(uint8_t value);
//uint8_t RE_read(void);

void sine_table_gen(uint8_t amp);
uint8_t wave_out(wave* w);

uint8_t sine_table[SINE_TABLE_N];
Wavetype wavetype = Sinusoidal;
wave wave1, wave2;

int main(void) {
    ANSELA = 0x00;    // Disable analog inputs
    ANSELC = 0x00;    // Disable analog inputs
    TRISA  = 0b00000000;    // Input: none, Output: all pins
    TRISB  = 0b00000000;    // Input: none, Output: all pins
    TRISC  = 0b00011111;    // Input: RC0-5(SW), Output: other pins
    PORTA  = 0x00;
    PORTB  = 0x00;
    PORTC  = 0x00;
    
    uint8_t out = 0;
    
    wave1.freq = 1800;
    wave1.index_step = (uint8_t) (wave1.freq / FREQ_STEP);
    wave1.stage = 0;
    wave2.freq = 2000;
    wave2.index_step = (uint8_t) (wave2.freq / FREQ_STEP);
    wave2.stage = 0;
    
    lcd_init();
    Timer_init();
    DAC_init();
    sine_table_gen(127);
    
    lcd_puts("Hello,world!");
    
    while(1) {
        __delay_ms(1000);
    }
    
    return (EXIT_SUCCESS);
}

void Timer_init(void) {
	// Timer2 settings
    // Interrupt interval = 1 / SAMPLING_FREQ
    T2CLKCON = 0b00000001;  // Timer2 clock source = Fosc/4
    TMR2IE = 1; // Enables the Timer2 to PR2 match interrupt
    PEIE = 1;   // Enables all active peripheral interrupts
    GIE = 1;    // Enables all active interrupts
    PR2 = (uint8_t)((uint32_t)_XTAL_FREQ/((uint32_t)SAMPLING_FREQ * 8) - 1);   // PR2=(Fosc/(Fout×4×TMR2 Prescaler Value))-1
    T2CON = 0b10010000;     // Timer2 ON, prescaller 1:1, postscaler1:1
    
}

void DAC_init(void) {
    DAC1CON0 = 0b11101000;    // DAC config: EN=1, FM=1(8bit+2bit), OE1=1, PSS=10=FVR
    FVRCON = 0b10000100;    // FVR config: FVREN=1, CDAFVR=01(1.024V), 10(2.048V), 11(4.096V)
}

void DAC_valueset(uint8_t value) {
    // output voltage = value(upper 8bit) * 4 * FVR voltage
	DAC1REFH = value; // set upper 8bit
    DACLD = 0b00000001; // write to buffer
}

//uint8_t RE_read(void){
//  static const uint8_t RE_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
//  static uint8_t RE_old = 0;               // 共通変数（値は保存）
//  char RE_now;
//  RE_now = B_PIN * 2 + A_PIN;           // 今回情報の読取
//  RE_old <<= 2;                         // 前回の読取値と
//  RE_old |= ( RE_now & 0x03 );          // 今回の読取値を組合わせる
//  return (RE_states[(RE_old & 0x0F)]);  // 変化分を戻り値をする
//}

void sine_table_gen(uint8_t amp) {
    // Generate a sine wave table for a quarter cycle
    for (uint8_t i = 0; i < SINE_TABLE_N; i++) {
        *(sine_table + i) = (uint8_t)((float)31 * sinf(1.5708f * (float)i / SINE_TABLE_N) + 63);
    }
}

uint8_t wave_out(wave* w) {
    uint8_t out;
    
    if (wavetype == Sinusoidal) {
        if (w->stage == 0) {
            out = sine_table[w->index];
        } else if (w->stage == 1) {
            out = sine_table[SINE_TABLE_N - w->index - 1];
        } else if (w->stage == 2) {
            out = 126 - sine_table[w->index];
        } else if (w->stage == 3) {
            out = 126 - sine_table[SINE_TABLE_N - w->index - 1];
        }
        
        w->index += w->index_step;
        if (w->index >= SINE_TABLE_N) {
            w->index -= SINE_TABLE_N;
            if (++(w->stage) >= 4) w->stage = 0;
        }
    }
    return out;
}

// Timer2 Interrupt
void __interrupt() isr(void) {
    TMR2IF = 0;
    DAC_valueset((uint8_t)(wave_out(&wave1) + wave_out(&wave2)));
//    DAC_valueset(wave_out(&wave1));
}