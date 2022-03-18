/**
 *
 * reciprocal frequency counter
 * with Pi Pico
 * using PWM-channel 4 - 7
 *
 */

#include "I2C_LCD/LiquidCrystal_I2C.h"
#include <stdio.h>
#include <pico/stdlib.h>
#include <hardware/pwm.h>
#include <hardware/irq.h>
#include <hardware/i2c.h>
#include <hardware/flash.h>
#include <hardware/sync.h>

// GLOBAL DECLARATIONS -------------------
#define GATECONTROL_PIN 8
#define GATESTATUS_PIN 10
#define FCLOCK_A_PIN 9
#define FCLOCK_B_PIN 13
#define FIN_A_PIN 11
#define FIN_B_PIN 15
#define LED_PIN 25
#define RANGE_BUTTON_PIN 18
#define KAL_BUTTON_PIN 19

#define debounceDelay 50
#define F_REFERENCE 10000000.0

// DECLARATIONS FOR I2C
const unsigned long I2C_CLOCK = 100000;
const uint8_t LCD_I2C_ADDRESS = 0x27;

// DECLARATIONS FOR EEPROM-FUNCTION USING FLASH
#define FLASH_TARGET_OFFSET (1024 * 1024)
const uint8_t * flash_target_contents = (const uint8_t * )(XIP_BASE + FLASH_TARGET_OFFSET);

// COUNTINGREGISTER FOR BLOCK A
uint16_t f_counter_a_high = 0;
uint16_t f_counter_a_low = 0;
uint32_t f_counter_a_oldvalue = 0;
uint16_t t_counter_a_high = 0;
uint16_t t_counter_a_low = 0;
uint32_t t_counter_a_oldvalue = 0;

// COUNTINGREGISTER FOR BLOCK B
uint16_t f_counter_b_high = 0;
uint16_t f_counter_b_low = 0;
uint32_t f_counter_b_oldvalue = 0;
uint16_t t_counter_b_high = 0;
uint16_t t_counter_b_low = 0;
uint32_t t_counter_b_oldvalue = 0;

uint32_t f_counter = 0;
uint32_t f_value = 0;
uint32_t t_counter = 0;
uint32_t t_value = 0;
uint calibration_count = 2;
double frequency_f = 0.0;

uint range = 1;
uint old_range = 1;
uint32_t gatetime = 980;
uint resolution = 9;
int16_t frequence_calibration_offset = 0;

uint32_t pwm_interupt_status = 0;

bool gate_control = false;
bool gate_status = false;
bool gate_status_old = false;
bool no_signal = false;
bool calibration_flag = false;

char output_string[40];
char lcd_output_string[40];
char unit_string[10];
char gatetime_string[20] = "1s    ";

bool rangebutton_pressed() {
    static bool buttonActState = false;
    static bool buttonLastState = false;
    static bool buttonSteadyState = false;
    static absolute_time_t lastDebounceTime = 0;
    bool result = false;

    buttonActState = gpio_get(RANGE_BUTTON_PIN);
    // If the switch changed:
    if (buttonActState != buttonSteadyState) {
        if (buttonActState != buttonLastState) {
            // reset the debouncing timer
            buttonLastState = buttonActState;
            lastDebounceTime = make_timeout_time_ms(debounceDelay);
        }
        if (get_absolute_time() > lastDebounceTime) {
            // whatever the reading is at, it's been there for longer than the debounce
            // delay, so take it as the actual current state:
            buttonSteadyState = buttonActState;
            result = !buttonSteadyState;
        }
    }
    return result;
}

void on_pwm_wrap() {
    pwm_interupt_status = pwm_get_irq_status_mask();
    if (pwm_interupt_status & 0x10) {
        pwm_clear_irq(pwm_gpio_to_slice_num(FCLOCK_A_PIN));
        t_counter_a_high++;
    }

    if (pwm_interupt_status & 0x20) {
        pwm_clear_irq(pwm_gpio_to_slice_num(FIN_A_PIN));
        f_counter_a_high++;
    }

    if (pwm_interupt_status & 0x40) {
        pwm_clear_irq(pwm_gpio_to_slice_num(FCLOCK_B_PIN));
        t_counter_b_high++;
    }

    if (pwm_interupt_status & 0x80) {
        pwm_clear_irq(pwm_gpio_to_slice_num(FIN_B_PIN));
        f_counter_b_high++;
    }
}

int main() {

    stdio_init_all();

    /* Initialize LCD I2C module */
    lcd_init(i2c_default, LCD_I2C_ADDRESS);
    lcd_set_cursor(0, 0);
    lcd_print("Frequencycounter");
    lcd_set_cursor(1, 0);
    lcd_print("v0.9.5");
    if (flash_target_contents[0] == 0x55 && flash_target_contents[1] == 0xaa) {
        frequence_calibration_offset = (int16_t)flash_target_contents[2] << 8 | (int16_t)flash_target_contents[3];
    }
    printf("Frequency Offset: %d\n", frequence_calibration_offset);
    sleep_ms(1000);

    // Tell the used GPIOs  they are allocated to the PWM
    gpio_set_function(FCLOCK_A_PIN, GPIO_FUNC_PWM);
    gpio_set_function(FCLOCK_B_PIN, GPIO_FUNC_PWM);
    gpio_set_function(FIN_A_PIN, GPIO_FUNC_PWM);
    gpio_set_function(FIN_B_PIN, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to used pins
    uint slice_fclock_a = pwm_gpio_to_slice_num(FCLOCK_A_PIN);
    uint slice_fclock_b = pwm_gpio_to_slice_num(FCLOCK_B_PIN);
    uint slice_fin_a = pwm_gpio_to_slice_num(FIN_A_PIN);
    uint slice_fin_b = pwm_gpio_to_slice_num(FIN_B_PIN);

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(slice_fclock_a);
    pwm_set_irq_enabled(slice_fclock_a, true);
    pwm_clear_irq(slice_fclock_b);
    pwm_set_irq_enabled(slice_fclock_b, true);
    pwm_clear_irq(slice_fin_a);
    pwm_set_irq_enabled(slice_fin_a, true);
    pwm_clear_irq(slice_fin_b);
    pwm_set_irq_enabled(slice_fin_b, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // Get some sensible defaults for the slice configuration. By default, the
    // counter is allowed to wrap over its maximum range (0 to 2**16-1)
    pwm_config config = pwm_get_default_config();
    // Set Count on falling edge of the B pin input
    pwm_config_set_clkdiv_mode( &config, PWM_DIV_B_FALLING);
    // Load the configuration into our PWM slice, and set it running.
    pwm_init(slice_fclock_a, &config, true);
    pwm_init(slice_fclock_b, &config, true);
    pwm_init(slice_fin_a, &config, true);
    pwm_init(slice_fin_b, &config, true);

    gpio_init(GATECONTROL_PIN);
    gpio_set_dir(GATECONTROL_PIN, GPIO_OUT);
    gpio_init(GATESTATUS_PIN);
    gpio_set_dir(GATESTATUS_PIN, GPIO_IN);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    gpio_init(RANGE_BUTTON_PIN);
    gpio_set_dir(RANGE_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(RANGE_BUTTON_PIN);

    gpio_init(KAL_BUTTON_PIN);
    gpio_set_dir(KAL_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(KAL_BUTTON_PIN);

    gate_control = true;
    gpio_put(GATECONTROL_PIN, gate_control);
    absolute_time_t GateChangeTime = make_timeout_time_ms(gatetime);

    lcd_clear();
    while (true) {
        if (rangebutton_pressed()) {
            switch (range) {
            case 0:
                gatetime = 980;
                resolution = 9;
                range = 1;
                strcpy(gatetime_string, "1s    ");
                break;
            case 1:
                gatetime = 9980;
                resolution = 10;
                range = 2;
                strcpy(gatetime_string, "10s   ");
                break;
            default:
                gatetime = 250;
                resolution = 8;
                range = 0;
                strcpy(gatetime_string, "0.25s ");
            }
            GateChangeTime = make_timeout_time_ms(100);
        }

        if (get_absolute_time() > GateChangeTime) {
            gate_control = !gate_control;
            gpio_put(GATECONTROL_PIN, gate_control);
            gpio_put(LED_PIN, gate_control);
            GateChangeTime = make_timeout_time_ms(4 * gatetime);
            if (no_signal) {
                printf("--\n");
                lcd_set_cursor(0, 0);
                lcd_print("--              ");
            }
            no_signal = true;
        }

        gate_status = gpio_get(GATESTATUS_PIN);
        if (gate_status != gate_status_old) {
            if (calibration_flag) {
                calibration_count--;
            }
            no_signal = false;
            gate_status_old = gate_status;
            GateChangeTime = make_timeout_time_ms(gatetime);
            if (gate_status) {
                t_counter_b_low = pwm_get_counter(slice_fclock_b);
                f_counter_b_low = pwm_get_counter(slice_fin_b);
                f_counter = f_counter_b_high;
                f_counter = f_counter << 16;
                f_counter += f_counter_b_low;
                f_value = f_counter - f_counter_b_oldvalue;
                f_counter_b_oldvalue = f_counter;
                t_counter = t_counter_b_high;
                t_counter = t_counter << 16;
                t_counter += t_counter_b_low;
                t_value = t_counter - t_counter_b_oldvalue;
                t_counter_b_oldvalue = t_counter;
            } else {
                t_counter_a_low = pwm_get_counter(slice_fclock_a);
                f_counter_a_low = pwm_get_counter(slice_fin_a);
                f_counter = f_counter_a_high;
                f_counter = f_counter << 16;
                f_counter += f_counter_a_low;
                f_value = f_counter - f_counter_a_oldvalue;
                f_counter_a_oldvalue = f_counter;
                t_counter = t_counter_a_high;
                t_counter = t_counter << 16;
                t_counter += t_counter_a_low;
                t_value = t_counter - t_counter_a_oldvalue;
                t_counter_a_oldvalue = t_counter;
            }
            frequency_f = (F_REFERENCE + (double)frequence_calibration_offset / 10.0) * (double)f_value / (double)t_value;
            strcpy(unit_string, "Hz ");
            if (frequency_f > 990.0) {
                frequency_f = frequency_f / 1000.0;
                strcpy(unit_string, "kHz");
            }
            if (frequency_f > 990.0) {
                frequency_f = frequency_f / 1000.0;
                strcpy(unit_string, "MHz");
            }
            sprintf(output_string, "%.8f", frequency_f);

            if (!gpio_get(KAL_BUTTON_PIN)) {
                calibration_count = 2;
                calibration_flag = true;
                old_range = range;
                gatetime = 9980;
                resolution = 10;
                range = 2;
                strcpy(gatetime_string, "cal  ");
            }

            if (calibration_count <= 0) {
                frequence_calibration_offset = t_value - 10 * F_REFERENCE;
                printf("Frequence offset: %d\n", frequence_calibration_offset);
                sprintf(lcd_output_string, "Calibration");
                lcd_set_cursor(0, 0);
                lcd_print(lcd_output_string);
                sprintf(lcd_output_string, "saved");
                lcd_set_cursor(1, 0);
                lcd_print(lcd_output_string);
                uint8_t eeprom_data[FLASH_PAGE_SIZE];
                for (int i = 0; i < FLASH_PAGE_SIZE; ++i) {
                    eeprom_data[i] = 0;
                }
                eeprom_data[0] = 0x55;
                eeprom_data[1] = 0xaa;
                eeprom_data[2] = (frequence_calibration_offset >> 8) & 0xff;
                eeprom_data[3] = frequence_calibration_offset & 0xff;
                uint32_t ints = save_and_disable_interrupts();
                flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
                flash_range_program(FLASH_TARGET_OFFSET, eeprom_data, FLASH_PAGE_SIZE);
                restore_interrupts(ints);
                calibration_flag = false;
                calibration_count = 2;
                range = old_range;
                switch (range) {
                case 1:
                    gatetime = 980;
                    resolution = 9;
                    range = 1;
                    strcpy(gatetime_string, "1s    ");
                    break;
                case 2:
                    gatetime = 9980;
                    resolution = 10;
                    range = 2;
                    strcpy(gatetime_string, "10s   ");
                    break;
                default:
                    gatetime = 250;
                    resolution = 8;
                    range = 0;
                    strcpy(gatetime_string, "0.25s ");
                }
                GateChangeTime = make_timeout_time_ms(100);
            }

            printf("%s %s\n", output_string, unit_string);
            output_string[resolution] = 0x00;
            sprintf(lcd_output_string, "%s %s   ", output_string, unit_string);
            lcd_set_cursor(0, 0);
            lcd_print(lcd_output_string);

            sprintf(lcd_output_string, "gatetime: %s ", gatetime_string);
            lcd_set_cursor(1, 0);
            lcd_print(lcd_output_string);
        }
    }
}
