/* Simple LED task demo:
 *
 * The LED on PC13 is toggled in watering_task.
 */
#include "FreeRTOS.h"
#include "task.h"
#include "mcuio.h"
#include "miniprintf.h"

#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <ctype.h>
#include <setjmp.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/cm3/nvic.h>

static uint16_t read_adc(uint8_t channel);
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,signed portCHAR *pcTaskName);
// printing
static inline void uart_putc(char ch);
int usart_printf(const char *format,...) __attribute((format(printf,1,2)));
void usart_print_uint(uint16_t value);

void
vApplicationStackOverflowHook(xTaskHandle *pxTask,signed portCHAR *pcTaskName) {
	(void)pxTask;
	(void)pcTaskName;
	for(;;);
}

static void
watering_task(void *args) {
	(void)args;
	int adc1 = 0;
	int counter = 0;
	int total = 0;

	for (;;) {
		gpio_toggle(GPIOC, GPIO13);
		// for (int i = 0; i < 1000000; i++)
		// 	__asm__("nop");
		gpio_set(GPIOA, GPIO5);

		adc1 = read_adc(1);
		usart_printf("Soil Moisture: ");
		usart_print_uint(adc1);
		usart_printf("\n");

		if (adc1 == 0)
		{
			counter = 0;
			continue;
		}
		else if (adc1 < 500)
		{
			counter = 0;
			continue;
		}
		else
		{
			counter++;
			total += adc1;
			usart_printf("counter: ");
			usart_print_uint(counter);
			usart_printf("\n");
			usart_printf("total: ");
			usart_print_uint(total);
			usart_printf("\n");
			if (counter >= 5)
			{
				total /= 5;
				usart_printf("average: ");
				usart_print_uint(total);
				usart_printf("\n");
				if (total > 1310)
				{
					gpio_clear(GPIOA, GPIO5);
					usart_printf("Relay ON\n");

					vTaskDelay(pdMS_TO_TICKS(1000)); // run for one second

					gpio_set(GPIOA, GPIO5);
					usart_printf("Relay OFF\n");

					int timeout = 1000 * 30;
					vTaskDelay(pdMS_TO_TICKS(timeout)); // 30 seconds
				}

				total = 0;
				counter = 0;
			}
		}
		
		vTaskDelay(pdMS_TO_TICKS(3000));
	}
}

/*********************************************************************
 * Read ADC Channel
 *********************************************************************/
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

	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_USART1);
	
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO1|GPIO2); // PA1 & PA2
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO5); // PA5 relay

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

	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX); // PA9
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_USART1_RX); // PA10
	usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_enable(USART1);

	// std_set_device(mcu_usb);
	// usb_start(1,1);

	xTaskCreate(watering_task,"LED",100,NULL,configMAX_PRIORITIES-1,NULL);
	vTaskStartScheduler();
	for (;;);

	return 0;
}

// End
