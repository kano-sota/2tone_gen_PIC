/* 
 * File:   myLCD_SC1602.c
 * SC1602 4-bit Mode Control
 * Reference: http://ananbobobo.blog2.fc2.com/blog-entry-379.html
 * Modified by JJ1MDY
 *
 * Created on 2022/11/04, 21:59
 */

#ifndef _XTAL_FREQ
#define _XTAL_FREQ 32000000
#endif

#define LCD_RS_PIN RC5
#define LCD_RW_PIN RC6
#define LCD_EN_PIN RC7
#define LCD_DATA_PORT PORTB // PB7 - PB4 に配線

#include <xc.h>

//ENの設定
void LCD_EN(uint8_t enable){
    LCD_EN_PIN = (enable ? 1 : 0);
}

//RSの設定
void LCD_RS(uint8_t enable){
    LCD_RS_PIN = (enable ? 1 : 0);
}

//LCDへの書き込み動作（E:X→H→L)
void LCD_STROBE(void){
    LCD_EN(1);
    __delay_us(230);
    LCD_EN(0);
}

//LCDから見て、上位4bit分だけデータ送信
void lcd_data_byteset(unsigned char c){
    LCD_DATA_PORT = (LCD_DATA_PORT & 0b00001111) | ((c << 4) & 0b11110000);
    LCD_STROBE();
}

//LCDから見て、上位4bit分と下位4bitデータ送信
void lcd_data_set(unsigned char c){
    LCD_DATA_PORT = (LCD_DATA_PORT & 0b00001111) | (c & 0b11110000);  // 上位4bit
    LCD_STROBE();
    LCD_DATA_PORT = (LCD_DATA_PORT & 0b00001111) | ((c << 4) & 0b11110000);   // 下位4bit
    LCD_STROBE();
}

//LCDにデータを送信
void lcd_write(unsigned char c){
    LCD_RS(0);//コントロールレジスタに切り替え
    lcd_data_set(c);
    __delay_us(40);
}

//LCDに文字を送信
void lcd_putch(unsigned char c){
    LCD_RS(1); //文字データレジスタに切り替え
    lcd_data_set(c);
    __delay_us(40);
}

//LCD初期化命令
void lcd_init(void){
    LCD_RS(0); //コントロールレジスタに切り替え
    __delay_ms(15); //LCD起動に少し待つ

    __delay_ms(20);
    lcd_data_byteset(0x03);
    __delay_ms(5);
    lcd_data_byteset(0x03);
    __delay_ms(1);
    lcd_data_byteset(0x03);
    __delay_ms(1);
    lcd_data_byteset(0x02); // 4bit mode set
    __delay_ms(1);
    __delay_us(10);
    lcd_data_set(0x28); // DL=0 4bit mode
    __delay_ms(2);
    lcd_data_set(0x08); // display off C=D=B=0
    __delay_ms(2);
    lcd_data_set(0x0C); // display on  D=1 B=C=0
    __delay_ms(2);
    lcd_data_set(0x06); // entry I/D=1 S=0
    __delay_ms(2);
    lcd_data_set(0x02); // cursor home
}

//LCD全消去
void lcd_clear(void){
    LCD_RS(0);//コントロールレジスタに切り替え
    lcd_write(0x1);
    __delay_ms(2);
}

//LCDに文字列を送信
void lcd_puts(const char * s){
    LCD_RS(1); //文字データレジスタに切り替え
    while(*s){
        lcd_putch(*s++);
    }
}

//指定した所にポジションを移動
void lcd_goto(unsigned char pos){
    LCD_RS(0);//コントロールレジスタに切り替え
    lcd_write(0x80+pos);
}