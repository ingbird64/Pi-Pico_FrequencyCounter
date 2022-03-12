#ifndef LCD_I2C_H
#define LCD_I2C_H

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include <pico/stdlib.h>
#include <hardware/i2c.h>

// INSTRUCTION DEFINITIONS
#define LCD_ADDRESS				0x27
#define LCD_CLEARDISPLAY		0x01
#define LCD_RETURNHOME 			0x02
#define LCD_ENTRYMODESET 		0x04
#define LCD_DISPLAYCONTROL 		0x08
#define LCD_CURSORSHIFT 		0x10
#define LCD_FUNCTIONSET 		0x20
#define LCD_SETCGRAMADDR		0x40
#define LCD_SETDDRAMADDR        0x80
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYLEFT          	0x02
#define LCD_BLINKON				0x01
#define LCD_CURSORON         	0x02
#define LCD_DISPLAYON        	0x04
#define LCD_MOVERIGHT        	0x04
#define LCD_DISPLAYMOVE      	0x08
#define LCD_5x10DOTS          	0x04
#define LCD_8BITMODE          	0x10
#define LCD_BACKLIGHT          	0x08
#define LCD_ENABLE_BIT          0x04

#define DEFAULT_I2C_CLOCK		100000UL

enum LCD_NUM_LINE {
	LCD_2LINE = 0x08,
	LCD_4LINE = 0x08,	/* Addition for 20x4 display, have to look up in official library */
};


// Modes for lcd_send_byte
#define LCD_CHARACTER  1
#define LCD_COMMAND    0

typedef struct lcd_instance{

	i2c_inst_t *i2c_bus;
	uint8_t 	address;
	uint8_t		currentRegisterValue;

} LCD_i2c_t;


static inline void init_i2c_if_necessary(i2c_inst_t *i2c_instance);

void lcd_init				(i2c_inst_t *i2c_instance, uint8_t address);
void lcd_set_cursor			(int line, int position);
static inline void lcd_char	(char val);
void lcd_print				(const char *s);
void lcd_clear				(void);

void i2c_write_byte			(uint8_t val);
void lcd_toggle_enable		(void);
void lcd_send_byte			(uint8_t val, int mode);

#endif

