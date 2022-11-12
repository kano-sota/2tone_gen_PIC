/* 
 * File:   myLCD_SC1602.h
 * Author: JJ1MDY
 *
 * Created on 2022/11/04, 23:43
 */

#ifndef MYLCD_SC1602_H
#define	MYLCD_SC1602_H

#ifdef	__cplusplus
extern "C" {
#endif

void LCD_EN(uint8_t enable);
void LCD_RS(uint8_t enable);
void LCD_STROBE(void);
void lcd_data_byteset(unsigned char c);
void lcd_data_set(unsigned char c);
void lcd_write(unsigned char c);
void lcd_putch(unsigned char c);
void lcd_init(void);
void lcd_clear(void);
void lcd_puts(const char * s);
void lcd_goto(unsigned char pos);


#ifdef	__cplusplus
}
#endif

#endif	/* MYLCD_SC1602_H */

