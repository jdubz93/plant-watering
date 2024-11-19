// STM32F103C8T6

// misra standard link: https://gist.github.com/hemmerling/6b4b1bf6e635033e1313

#include <FreeRTOS.h>
#include <task.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/timer.h>

#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>

#include <string.h>
#include <stdbool.h>
#include <setjmp.h>

#include "miniprintf.h"
#include "i2c.h"

enum {
  SEESAW_STATUS_BASE = 0x00,
  SEESAW_GPIO_BASE = 0x01,
  SEESAW_SERCOM0_BASE = 0x02,
  SEESAW_TIMER_BASE = 0x08,
  SEESAW_ADC_BASE = 0x09,
  SEESAW_DAC_BASE = 0x0A,
  SEESAW_INTERRUPT_BASE = 0x0B,
  SEESAW_DAP_BASE = 0x0C,
  SEESAW_EEPROM_BASE = 0x0D,
  SEESAW_NEOPIXEL_BASE = 0x0E,
  SEESAW_TOUCH_BASE = 0x0F,
  SEESAW_KEYPAD_BASE = 0x10,
  SEESAW_ENCODER_BASE = 0x11,
  SEESAW_SPECTRUM_BASE = 0x12,
};

/** touch module function address registers
 */
enum {
  SEESAW_TOUCH_CHANNEL_OFFSET = 0x10,
};

// 0x36 Soil Sensor
#define SOIL_SEN_ADDR 0x36
#define WATER_PUMP_RELAY 0x18

static I2C_Control i2c;

// rtos
static void task1(void *args);
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,signed portCHAR *pcTaskName);
// printing
static inline void uart_putc(char ch);
int usart_printf(const char *format,...) __attribute((format(printf,1,2)));
void usart_print_uint(uint16_t value);

static void gpioSetup(void);
static void uartSetup(void);
static void i2cSetup(void);

void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed portCHAR *pcTaskName)
{
	(void)pxTask;
	(void)pcTaskName;
	for(;;);
}

static void task1(void *args) 
{
	(void)args;
	usart_printf("helloworld\n");
	uint8_t buf[2];
	uint8_t addr = SOIL_SEN_ADDR;
	I2C_Fails fc; // I2C fail code

	for (;;) 
	{
		if ( (fc = setjmp(i2c_exception)) != I2C_Ok )
		{
			// I2C exception occurred:
			usart_printf("I2C failure code %d\n\n", fc, i2c_error(fc));
			break;
		}

		usart_printf("looping...\n");

		// blink led
		gpio_toggle(GPIOC, GPIO13);
		

		i2c_start_addr(&i2c, addr, Write);
		// send base and channel
		i2c_write(&i2c, SEESAW_TOUCH_BASE);
    	i2c_write(&i2c, SEESAW_TOUCH_CHANNEL_OFFSET + 0);
		i2c_stop(&i2c);

		vTaskDelay(pdMS_TO_TICKS(500));

		// read touch
		i2c_start_addr(&i2c, addr, Read);
		buf[0] = i2c_read(&i2c, false);
		buf[1] = i2c_read(&i2c, true);
		i2c_stop(&i2c);

		uint16_t moisture = ((uint16_t)buf[0] << 8) | buf[1];
    	usart_printf("Soil Moisture: ");
		usart_print_uint(moisture);
		usart_printf("\n");



// Read Temperature of Soil Sensor
/*
  uint8_t buf[4];
  this->read(SEESAW_STATUS_BASE, SEESAW_STATUS_TEMP, buf, 4, 1000);
  int32_t ret = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
                ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];
  return (1.0 / (1UL << 16)) * ret;
*/


// Read Touch of Soil Sensor
/*
  uint8_t buf[2];
  uint8_t p = 0; // pin
  uint16_t ret = 65535;

  for (uint8_t retry = 0; retry < 5; retry++) {
    if (this->read(SEESAW_TOUCH_BASE, SEESAW_TOUCH_CHANNEL_OFFSET + p, buf, 2,
                   3000 + retry * 1000)) {
      ret = ((uint16_t)buf[0] << 8) | buf[1];
      break;
    }
  }
*/
		// note could use i2c_write_restart() for repeated start but not writing so not worth
	}
}


static void gpioSetup(void) 
{
	// todo - gpio on PC13 (onboard led make sure rtos task is working)
	rcc_periph_clock_enable(RCC_GPIOC);
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
}


static void i2cSetup(void) 
{
	// todo - i2c on PB6, PB7
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_I2C1);
	gpio_set_mode(GPIOB,GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO6); // PB6 SCL
	gpio_set_mode(GPIOB,GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO7); // PB7_I2C6SDA
	gpio_set(GPIOB, GPIO6 | GPIO7); // remember the bus (both lines) will be held high when idle
	i2c_configure(&i2c, I2C1, 1000);
}

static void uartSetup(void)
{
	// todo - usart on PA9, PA10, baudrate = 38400
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_USART1);

	// tx & rx are flipped on serial usb side
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX); // PA9
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_USART1_RX); // PA10

	usart_set_baudrate(USART1, 38400);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_enable(USART1);
}

void usart_print_uint(uint16_t value) 
{
    char buffer[6];
    int index = 0;

    do {
        buffer[index++] = (value % 10) + '0';
        value /= 10;
    } while (value > 0);

    while (index > 0) {
        uart_putc(buffer[--index]);
    }
}

static inline void uart_putc(char ch) 
{
	usart_send_blocking(USART1, ch); // reason for inline it sends one char at a time, called frequently
}

int usart_printf(const char *format, ...)
{
	va_list args;
	va_start(args,format);
	int rc = mini_vprintf_cooked(uart_putc, format, args);
	va_end(args);
	return rc;
}

int main(void) {
	rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

	gpioSetup();
	uartSetup();
	i2cSetup();

	// rtos setup
	xTaskCreate(task1, "task1", 100, NULL, configMAX_PRIORITIES-1, NULL);
	vTaskStartScheduler();

	for (;;);

	return 0;
}
