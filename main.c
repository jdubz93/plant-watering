/* Simple LED task demo:
 *
 * The LED on PC13 is toggled in watering_task.
 */
#include "FreeRTOS.h"
#include "task.h"
#include "mcuio.h"
#include "miniprintf.h"
#include "ili9341_driver.h"
#include "fonts.h"
#include "uartlib.h"

// #include <stdlib.h> // for atof
// #include <string.h> // for memset
#include <stdio.h>
#include <stdbool.h>
#include <ctype.h>
#include <setjmp.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/cm3/nvic.h>
// #include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/spi.h>

#define SLEEP 100

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,signed portCHAR *pcTaskName);
static void relayOFF(void);
static void relayON(void);
static void watering_task(void *args);
static uint16_t read_adc(uint8_t channel);
void usart_print_uint(uint16_t value);
static inline void uart_putc(char ch);
int usart_printf(const char *format,...) __attribute((format(printf,1,2)));

void
vApplicationStackOverflowHook(xTaskHandle *pxTask,signed portCHAR *pcTaskName) {
	(void)pxTask;
	(void)pcTaskName;
	for(;;);
}

static void 
relayOFF(void) {
	gpio_set(GPIOA, GPIO5);
}

static void
relayON(void) {
	gpio_clear(GPIOA, GPIO5);
}

static void
watering_task(void *args) {
	(void)args;
	int moisture = 0;
	static char soil_print[10];
	ili9341_fill_screen(0x0000); // black
	relayOFF();
	for (;;) {
		gpio_toggle(GPIOC,GPIO13);

		char ph_rx_buf[6];
		getline_uart(1, ph_rx_buf, sizeof(ph_rx_buf));

		char ph_str[10];
		snprintf(ph_str, sizeof(ph_str), " PH: %s", ph_rx_buf);
		ili9341_draw_string_scaled(
			50,         // x
			90,         // y
			ph_str,     // buffer
			0xFFFF,     // foreground color
			0x0000,     // background color
			3           // scale factor
		);

		moisture = read_adc(1);

		snprintf(soil_print, 11, "SOIL: %d", moisture);
        ili9341_draw_string_scaled(
			10,
			50,
			soil_print,
			0xFFFF,
			0x0000,
			3
		);
		
		// prevent watering if obvious invalid value
		if (moisture < 1000) {
            relayOFF();
            vTaskDelay(pdMS_TO_TICKS(SLEEP));
            continue;
        }

		// -----------------------------------------
        // PULSE WATERING LOGIC
        // -----------------------------------------
        if (moisture > 1500) {
			relayON();
        } else {
            relayOFF();
        }

        vTaskDelay(pdMS_TO_TICKS(SLEEP));
		// // for (int i = 0; i < 1000000; i++)
		// // 	__asm__("nop");
	}
}

static uint16_t
read_adc(uint8_t channel) {
	adc_set_sample_time(ADC1, channel, ADC_SMPR_SMP_239DOT5CYC);
	adc_set_regular_sequence(ADC1, 1, &channel);
	adc_start_conversion_direct(ADC1);
	while ( !adc_eoc(ADC1) )
		taskYIELD();
	return adc_read_regular(ADC1);
}

void 
usart_print_uint(uint16_t value) {
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

static inline void 
uart_putc(char ch) {
	usart_send_blocking(USART1, ch); // call frequently inline gives performance boost
}

int 
usart_printf(const char *format, ...) {
	va_list args;
	va_start(args,format);
	int rc = mini_vprintf_cooked(uart_putc, format, args);
	va_end(args);
	return rc;
}

int
main(void) {
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	// CLK
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_USART1);
	rcc_periph_clock_enable(RCC_SPI1);
	
	// GPIO 
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO1|GPIO2);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO5); // relay

	relayOFF();

	// ADC 
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC1EN);
	adc_power_off(ADC1);
	rcc_peripheral_reset(&RCC_APB2RSTR, RCC_APB2RSTR_ADC1RST);
	rcc_peripheral_clear_reset(&RCC_APB2RSTR, RCC_APB2RSTR_ADC1RST);
	rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV6);
	adc_set_dual_mode(ADC_CR1_DUALMOD_IND);
	adc_disable_scan_mode(ADC1);
	adc_set_right_aligned(ADC1);
	adc_set_single_conversion_mode(ADC1);
	adc_set_sample_time(ADC1, ADC_CHANNEL_TEMP, ADC_SMPR_SMP_239DOT5CYC);
	adc_set_sample_time(ADC1, ADC_CHANNEL_VREF, ADC_SMPR_SMP_239DOT5CYC);
	adc_power_on(ADC1);
	adc_reset_calibration(ADC1);
	adc_calibrate_async(ADC1);
	while ( adc_is_calibrating(ADC1) );

	// USART
	gpio_set_mode(GPIOA,GPIO_MODE_OUTPUT_50_MHZ,GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,GPIO_USART1_TX); // PA9
	gpio_set_mode(GPIOA,GPIO_MODE_OUTPUT_50_MHZ,GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,GPIO11);
	gpio_set_mode(GPIOA,GPIO_MODE_INPUT,GPIO_CNF_INPUT_FLOAT,GPIO_USART1_RX); // PA10
	gpio_set_mode(GPIOA,GPIO_MODE_INPUT,GPIO_CNF_INPUT_FLOAT,GPIO12);
	// open_uart(1, 115200, "8N1", "rw", 1, 1);	// with RTS/CTS flow control
	open_uart(1, 9600, "8N1", "rw", 0, 0);

	// SPI - I can't use PA5 for SCK on SPI1 because I am using that for my relay, so instead I remap SPI1 to PB3 for SCK
    gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_OFF, AFIO_MAPR_SPI1_REMAP);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO3 | GPIO5); // PB3=SCK, PB5=MOSI
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO10 | GPIO11 | GPIO7); // PB10=D/C, PB11=RES, PB7=CS/NSS
	// (MISO) unused 
    gpio_set(GPIOB, GPIO7); // CS=inactive
    gpio_set(GPIOB, GPIO11); // RES=inactive
    spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_2, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE, SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
    spi_enable(SPI1);
	
	// TFT
    ili9341_reset();
	ili9341_init();
    ili9341_select();

	// RTOS
	xTaskCreate(watering_task,"LED",256,NULL,configMAX_PRIORITIES-1,NULL);
	vTaskStartScheduler();
	for (;;);

	ili9341_deselect();
	return 0;
}

// End
