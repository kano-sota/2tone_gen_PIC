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
#define LCD_DATA_PORT PORTB // PB7 - PB4 �ɔz��

#include <xc.h>

//EN�̐ݒ�
void LCD_EN(uint8_t enable){
    LCD_EN_PIN = (enable ? 1 : 0);
}

//RS�̐ݒ�
void LCD_RS(uint8_t enable){
    LCD_RS_PIN = (enable ? 1 : 0);
}

//LCD�ւ̏������ݓ���iE:X��H��L)
void LCD_STROBE(void){
    LCD_EN(1);
    __delay_us(230);
    LCD_EN(0);
}

//LCD���猩�āA���4bit�������f�[�^���M
void lcd_data_byteset(unsigned char c){
    LCD_DATA_PORT = (LCD_DATA_PORT & 0b00001111) | ((c << 4) & 0b11110000);
    LCD_STROBE();
}

//LCD���猩�āA���4bit���Ɖ���4bit�f�[�^���M
void lcd_data_set(unsigned char c){
    LCD_DATA_PORT = (LCD_DATA_PORT & 0b00001111) | (c & 0b11110000);  // ���4bit
    LCD_STROBE();
    LCD_DATA_PORT = (LCD_DATA_PORT & 0b00001111) | ((c << 4) & 0b11110000);   // ����4bit
    LCD_STROBE();
}

//LCD�Ƀf�[�^�𑗐M
void lcd_write(unsigned char c){
    LCD_RS(0);//�R���g���[�����W�X�^�ɐ؂�ւ�
    lcd_data_set(c);
    __delay_us(40);
}

//LCD�ɕ����𑗐M
void lcd_putch(unsigned char c){
    LCD_RS(1); //�����f�[�^���W�X�^�ɐ؂�ւ�
    lcd_data_set(c);
    __delay_us(40);
}

//LCD����������
void lcd_init(void){
    LCD_RS(0); //�R���g���[�����W�X�^�ɐ؂�ւ�
    __delay_ms(15); //LCD�N���ɏ����҂�

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

//LCD�S����
void lcd_clear(void){
    LCD_RS(0);//�R���g���[�����W�X�^�ɐ؂�ւ�
    lcd_write(0x1);
    __delay_ms(2);
}

//LCD�ɕ�����𑗐M
void lcd_puts(const char * s){
    LCD_RS(1); //�����f�[�^���W�X�^�ɐ؂�ւ�
    while(*s){
        lcd_putch(*s++);
    }
}

//�w�肵�����Ƀ|�W�V�������ړ�
void lcd_goto(unsigned char pos){
    LCD_RS(0);//�R���g���[�����W�X�^�ɐ؂�ւ�
    lcd_write(0x80+pos);
}