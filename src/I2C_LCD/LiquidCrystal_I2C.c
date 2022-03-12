#include "LiquidCrystal_I2C.h"

#include <stdint.h>
#include <stdbool.h>
#include <pico/stdlib.h>
#include <hardware/i2c.h>
#include <stdio.h>
#include <string.h>

/* Example library to drive a 16x2 LCD panel via a I2C bridge chip (e.g. PCF8574)

   You will need to use a level shifter on the I2C line!. 
   It's possible though to run only the I2C lines using level shifter,
   we can connect LCD VCC & Backlight pin by using VBUS pin from the RPi-Pico. From
   the datasheet, VBUS pin is connected directly to 5V supply from the micro USB port.
   You can also connect VCC to VSYS, which also supplies 5V (output from regulated
   VBUS).

   Connections on Raspberry Pi Pico board, other boards may vary.

   GPIO 4 (pin 6)-> SDA on LCD bridge board
   GPIO 5 (pin 7)-> SCL on LCD bridge board
   VBUS          -> VCC on LCD bridge board
   GND (pin 38)  -> GND on LCD bridge board
*/



// Private declaration - user doesn't need to know the details
LCD_i2c_t __lcd_inst_t;
LCD_i2c_t *__self = &__lcd_inst_t;
const static uint8_t row_address_offset[4] = {0x80, 0xC0, 0x80+20, 0xC0+20};

/**
 * @brief private function in case user forgot to initialize I2C before
 * the call to lcd_init() function.
*/
static inline void init_i2c_if_necessary(i2c_inst_t *i2c_instance)
{
	gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
	gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
	gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
	gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);

	i2c_init(i2c_instance, DEFAULT_I2C_CLOCK);
}
// END OF PRIVATE DECLARATION-----------------------------------


/** @fn lcd_init
 * @brief Initialize LCD I2C using specified i2c bus & address.
 * Note that you don't have to worry which type of LCD you use (16x2 / 20x4),
 * It's obvious that you won't do something like "lcd_set_cursor(3,0)" on 16x2 displays.
 * The address mapping of 16x2 & 20x4 are the same
 * @param i2c_bus ptr to i2c_hw_struct (pass in i2c_default if you only use 1 bus)
 * @param address address of the LCD PCF module. Typically PCF address should be 0x27.
*/
void lcd_init(i2c_inst_t *i2c_bus, uint8_t address) {

	__self->i2c_bus = i2c_bus;
	__self->address = address;

    // safety call in case user forgot to initialize I2C bus
    if(gpio_get_function(PICO_DEFAULT_I2C_SCL_PIN) != GPIO_FUNC_I2C || 
       gpio_get_function(PICO_DEFAULT_I2C_SDA_PIN) != GPIO_FUNC_I2C)
    {
        init_i2c_if_necessary(__self->i2c_bus);
    }

    lcd_send_byte(0x03, LCD_COMMAND);
    lcd_send_byte(0x03, LCD_COMMAND);
    lcd_send_byte(0x03, LCD_COMMAND);
    lcd_send_byte(0x02, LCD_COMMAND);

    lcd_send_byte(LCD_ENTRYMODESET | LCD_ENTRYLEFT, LCD_COMMAND);
    lcd_send_byte(LCD_FUNCTIONSET | LCD_2LINE, LCD_COMMAND);
    lcd_send_byte(LCD_DISPLAYCONTROL | LCD_DISPLAYON, LCD_COMMAND);
    lcd_clear();
}


/** @fn lcd_set_cursor
 * @brief Set LCD cursor to desired position
 * @param line Row LCD (0-1 for 16x2 display  &  0-3 for 20x4 display)
 * @param position Column LCD (0 indexing, ex: 16x2 is 0-15)
*/
void lcd_set_cursor(int line, int position) 
{
    // int val = (line == 0) ? 0x80 + position : 0xC0 + position;   // default
    int val = row_address_offset[line] + position;

    lcd_send_byte(val, LCD_COMMAND);
}


/** @fn lcd_char
 * @brief send a single character.
 * @param val Character to be sent.
 * 
*/
static inline void lcd_char(char val) 
{
    lcd_send_byte(val, LCD_CHARACTER);
}


/** @fn lcd_print
 * @brief Prints a sentence.
 * Beware that if the length of string is > column LCD, then you will see
 * something funny happening
 * @param s	String to be sent.
*/
void lcd_print(const char *s) {
    while (*s) {
        lcd_char(*s++);
    }
}

/** @fn
 * @brief Clears the LCD. Beware that this operation takes approximately 4ms.
*/
void lcd_clear(void) 
{
    lcd_send_byte(LCD_CLEARDISPLAY, LCD_COMMAND);
}


/** @fn
 * @brief Quick helper function for single byte transfers 
 * @param val value to be sent
*/
void i2c_write_byte(uint8_t val) 
{
	// transmit single byte
    i2c_write_blocking(__self->i2c_bus, __self->address, &val, 1, false);

	// update register value so we can track it on 
	// lcd_toggle_enable()
	__self->currentRegisterValue = val;
}

/** @fn lcd_toggle_enable
 * @brief Toggle the enable pin, this instructs the LCD to accept the data present
*/
void lcd_toggle_enable() 
{
    // Toggle enable pin on LCD display
    // We cannot do this too quickly or things don't work
    sleep_us(600);
    i2c_write_byte(__self->currentRegisterValue | LCD_ENABLE_BIT);
    sleep_us(600);
    i2c_write_byte(__self->currentRegisterValue & ~LCD_ENABLE_BIT);
    sleep_us(600);
}

/** @fn lcd_send_byte
 * @brief Sends a single byte. Wrapper for i2c_write_byte().
 * @param value data value to be sent
 * @param mode what type of data are we sending (COMMAND / CHARACTER)
*/
// The display is sent a byte as two separate nibble transfers
void lcd_send_byte(uint8_t val, int mode) 
{
    uint8_t high = mode | (val & 0xF0) | LCD_BACKLIGHT;
    uint8_t low = mode | ((val << 4) & 0xF0) | LCD_BACKLIGHT;

    i2c_write_byte(high);
    lcd_toggle_enable();
    i2c_write_byte(low);
    lcd_toggle_enable();
}

/* ---END OF HELPER FUNCTION --------------------------*/

