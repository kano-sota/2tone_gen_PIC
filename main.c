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

#define SAMPLING_FREQ 80000 // up to 100 kHz
#define FREQ_STEP 100
#define SINE_TABLE_N 200    // SAMPLING_FREQ / FREQ_STEP / 4

#define RE_A 
#define RE_B
#define LED1
#define SW3
#define SW4
#define SW5

void Timer_init(void);
void DAC_init(void);
void DAC_valueset(uint8_t value);
uint8_t RE_read(void);

void sine_table_gen(uint8_t amp);
void wave_out(void);

typedef enum {
    Sinusoidal,
    Pulse,
    Triangle,
    Saw,
} Wavetype;

uint8_t sine_table[SINE_TABLE_N];
uint16_t freq = 1000;
uint8_t step;
Wavetype wavetype = Sinusoidal;

int main(void) {
    ANSELA = 0x00;    // Disable analog inputs
    ANSELC = 0x00;    // Disable analog inputs
    TRISA  = 0b00000000;    // Input: none, Output: all pins
    TRISB  = 0b00000000;    // Input: none, Output: all pins
    TRISC  = 0b00011111;    // Input: RC0-5(SW), Output: other pins
    PORTA  = 0x00;
    PORTB  = 0x00;
    PORTC  = 0x00;
    
    lcd_init();
    Timer_init();
    DAC_init();
    sine_table_gen(127);
    
    lcd_puts("Hello,world!");
    
    uint8_t out = 0;
    
    freq = 700;
    step = (uint8_t) (freq / FREQ_STEP);
    
    while(1) {
//        out++;
//        if (out > 255) out = 0;
//        DAC_valueset(out);
        freq = 700; step = (uint8_t) (freq / FREQ_STEP); RA2 = 1;
        __delay_ms(1000);
        RA2 = 0;
        freq = 2000; step = (uint8_t) (freq / FREQ_STEP);
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
    PR2 = (uint8_t)((uint32_t)_XTAL_FREQ/((uint32_t)SAMPLING_FREQ * 4) - 1);   // PR2=(Fosc/(Fout×4×TMR2 Prescaler Value))-1
    T2CON = 0b10000000;     // Timer2 ON, prescaller 1:1, postscaler1:1
    
}

void DAC_init(void) {
    DAC1CON0 = 0b11101000;    // DAC config: EN=1, FM=1(8bit+2bit), OE1=1, PSS=10=FVR
    FVRCON = 0b10001100;    // FVR config: FVREN=1, CDAFVR=01(1.024V), 10(2.048V), 11(4.096V)
}

void DAC_valueset(uint8_t value) {
    // output voltage = value(upper 8bit) * 4 * FVR voltage
	DAC1REFH = value; // set upper 8bit
    DACLD = 0b00000001; // write to buffer
}

uint8_t RE_read(void){
  static const uint8_t RE_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  static uint8_t RE_old = 0;               // 共通変数（値は保存）
  char RE_now;
  RE_now = B_PIN * 2 + A_PIN;           // 今回情報の読取
  RE_old <<= 2;                         // 前回の読取値と
  RE_old |= ( RE_now & 0x03 );          // 今回の読取値を組合わせる
  return (RE_states[(RE_old & 0x0F)]);  // 変化分を戻り値をする
}

void sine_table_gen(uint8_t amp) {
    // Generate a sine wave table for a quarter cycle
    for (uint8_t i = 0; i < SINE_TABLE_N; i++) {
        *(sine_table + i) = (uint8_t)((float)127 * sinf(1.5708f * (float)i / SINE_TABLE_N)) + 127;
    }
}

void wave_out(void) {
    static uint8_t i = 0;
    static uint8_t stage = 0;
    
    if (wavetype == Sinusoidal) {
        if (stage == 0) {
            DAC_valueset(sine_table[i]);
        } else if (stage == 1) {
            DAC_valueset(sine_table[SINE_TABLE_N - i - 1]);
        } else if (stage == 2) {
            DAC_valueset(254 - sine_table[i]);
        } else if (stage == 3) {
            DAC_valueset(254 - sine_table[SINE_TABLE_N - i - 1]);
        }
        
        i += step;
        if (i >= SINE_TABLE_N) {
            i -= SINE_TABLE_N;
            if (++stage >= 4) stage = 0;
        }
    }
}

// Timer2 Interrupt
void __interrupt() isr(void) {
    TMR2IF = 0;
    wave_out();
}