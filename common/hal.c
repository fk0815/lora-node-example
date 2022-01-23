/*
Copyright © 2022 Frank Kunz

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the “Software”), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <stdint.h>
#include <stdio.h>
#include <oslmic.h>
#include <lmic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencmsis/core_cm3.h>
#include <libopencm3/stm32/exti.h>
#ifdef STM32F103C8
#include <libopencm3/stm32/rtc.h>
#define TICKTIMER_OVR (1ull << 32)
#endif
#ifdef STM32L431CB
#include <libopencm3/stm32/lptimer.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/dbgmcu.h>
#define TICKTIMER_OVR (1ull << 16)
#endif

#ifndef HAL_RESET_PIN
#define HAL_RESET_PIN GPIO3
#endif
#ifndef HAL_RESET_PORT
#define HAL_RESET_PORT GPIOA
#endif
#ifndef HAL_NSS_PIN
#define HAL_NSS_PIN GPIO4
#endif
#ifndef HAL_NSS_PORT
#define HAL_NSS_PORT GPIOA
#endif

/* assume no sleep time longer than 2 weeks */
#define HAL_TIME_OVERRUN_DETECT sec2osticks(3600 * 24 * 7 * 2)

#ifdef HAL_TIME_DEBUG
struct al_debug {
	u4_t tm;
	u4_t al;
};
#endif

// HAL state
static struct {
	int irqlevel;
	int alarm_programmed;
	ostime_t alarm_time;
	int alarm_time_valid;
	ostime_t tick_ovr;
	ostime_t tick_alm;
#ifdef STM32F103C8
	uint32_t tick_cmp;
#endif
#ifdef STM32L431CB
	uint16_t tick_cmp;
	uint16_t lptim1_evt_ena;
#endif
#ifdef HAL_TIME_DEBUG
	struct al_debug dbg[32];
	int dbg_idx;
#endif
} HAL;

const char lmic_pins;
ostime_t hal_time_offset_secs = 0;

#ifdef STM32F103C8
static void hal_io_init(void)
{
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_AFIO);

#ifdef SWD_ENABLE
	/* disable JTAG, only SWDP */
	gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, 0);
#else
	gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_OFF, 0);
#endif

	/* switch unneeded pins to input to reduce power */
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO0 | GPIO1 | GPIO2 | GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO15);
#ifndef SWD_ENABLE
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO13 | GPIO14);
#endif
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO6 | GPIO7 | GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO13 | GPIO14 | GPIO15);
	gpio_set_mode(GPIOC, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO13);

	gpio_set_mode(HAL_RESET_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, HAL_RESET_PIN);

	/*
	* dio0 dio1
	* PA0, PA1
	*/
	nvic_enable_irq(NVIC_EXTI0_IRQ);
	nvic_set_priority(NVIC_EXTI0_IRQ, 1);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO0);
	exti_select_source(EXTI0, GPIOA);
	exti_set_trigger(EXTI0, EXTI_TRIGGER_BOTH);
	exti_disable_request(EXTI0);
	exti_reset_request(EXTI0);
	exti_enable_request(EXTI0);

	nvic_enable_irq(NVIC_EXTI1_IRQ);
	nvic_set_priority(NVIC_EXTI1_IRQ, 1);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO1);
	exti_select_source(EXTI1, GPIOA);
	exti_set_trigger(EXTI1, EXTI_TRIGGER_BOTH);
	exti_disable_request(EXTI1);
	exti_reset_request(EXTI1);
	exti_enable_request(EXTI1);

	gpio_set_mode(HAL_NSS_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, HAL_NSS_PIN);
	gpio_set(HAL_NSS_PORT, HAL_NSS_PIN);
#if defined(HAL_RXTX_PORT) && defined(HAL_RXTX_PIN)
	gpio_set_mode(HAL_RXTX_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, HAL_RXTX_PIN);
#endif
}

static void hal_spi_init(void)
{
	uint32_t baudrate = (rcc_apb2_frequency == 72000000)?(SPI_CR1_BAUDRATE_FPCLK_DIV_4):(SPI_CR1_BAUDRATE_FPCLK_DIV_2);

	rcc_periph_clock_enable(RCC_SPI1);

	/* Configure GPIOs: SCK=PA5, MISO=PA6 and MOSI=PA7 */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO5 | GPIO7 );
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO6);

	/* Reset SPI, SPI_CR1 register cleared, SPI is disabled */
	spi_reset(SPI1);

	/* Set up SPI in Master mode with:
	 * Clock baud rate: 1/64 of peripheral clock frequency
	 * Clock polarity: Idle High
	 * Clock phase: Data valid on 2nd clock pulse
	 * Data frame format: 8-bit
	 * Frame format: MSB First
	 */
	spi_init_master(SPI1, baudrate, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE, SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);

	/*
	 * Set NSS management to software.
	 *
	 * Note:
	 * Setting nss high is very important, even if we are controlling the GPIO
	 * ourselves this bit needs to be at least set to 1, otherwise the spi
	 * peripheral will not send any data out.
	 */
	spi_enable_software_slave_management(SPI1);
	spi_set_nss_high(SPI1);

	/* Enable SPI1 periph. */
	spi_enable(SPI1);
}

static void hal_time_init(void)
{
	nvic_enable_irq(NVIC_RTC_ALARM_IRQ);
	nvic_set_priority(NVIC_RTC_ALARM_IRQ, 1);
	exti_set_trigger(EXTI17, EXTI_TRIGGER_RISING);
	exti_disable_request(EXTI17);
	exti_reset_request(EXTI17);
	exti_enable_request(EXTI17);
}

static void set_alarm_time(ostime_t almtm)
{
	hal_disableIRQs();
	HAL.tick_alm = almtm / TICKTIMER_OVR;
	HAL.tick_cmp = almtm % TICKTIMER_OVR;
	if(HAL.tick_ovr == HAL.tick_alm) {
		rtc_set_alarm_time(HAL.tick_cmp);
	} else {
		/* trigger alarm on overflow */
		rtc_set_alarm_time(0);
	}
	rtc_clear_flag(RTC_ALR);
	rtc_interrupt_enable(RTC_ALR);
	hal_enableIRQs();
}
#endif
#ifdef STM32L431CB
static void hal_io_init(void)
{
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_SYSCFG);
#ifdef SWD_ENABLE
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_PULLUP, 0xffff & ~(GPIO13|GPIO14));
	gpio_mode_setup(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_PULLUP, 0xffff & ~(GPIO3));
#else
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_PULLDOWN, 0xffff);
	gpio_mode_setup(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_PULLDOWN, 0xffff);
#endif
	gpio_mode_setup(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_PULLDOWN, 0xffff);
	gpio_mode_setup(HAL_RESET_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_PULLUP, HAL_RESET_PIN);

	gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO1);
	nvic_enable_irq(NVIC_EXTI1_IRQ);
	exti_select_source(EXTI1, GPIOB);
	exti_set_trigger(EXTI1, EXTI_TRIGGER_BOTH);
	exti_enable_request(EXTI1);

	gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO0);
	nvic_enable_irq(NVIC_EXTI0_IRQ);
	exti_select_source(EXTI0, GPIOB);
	exti_set_trigger(EXTI0, EXTI_TRIGGER_BOTH);
	exti_enable_request(EXTI0);

	gpio_mode_setup(HAL_NSS_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, HAL_NSS_PIN);
	gpio_set_output_options(HAL_NSS_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, HAL_NSS_PIN);
	gpio_set(HAL_NSS_PORT, HAL_NSS_PIN);
#if defined(HAL_RXTX_PORT) && defined(HAL_RXTX_PIN)
	gpio_mode_setup(HAL_RXTX_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, HAL_RXTX_PIN);
#endif
}

static void hal_spi_init(void)
{
	uint32_t baudrate = (rcc_apb2_frequency == 72000000)?(SPI_CR1_BAUDRATE_FPCLK_DIV_2):(SPI_CR1_BAUDRATE_FPCLK_DIV_2);

	rcc_periph_clock_enable(RCC_SPI1);
	rcc_periph_clock_enable(RCC_GPIOA);

	/* Configure GPIOs: SCK=PA5, MISO=PA6 and MOSI=PA7 */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLDOWN, GPIO5 | GPIO6 | GPIO7);
	gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO5 | GPIO7);
	gpio_set_af(GPIOA, GPIO_AF5, GPIO5 | GPIO6 | GPIO7);

	/* Reset SPI, SPI_CR1 register cleared, SPI is disabled */
	spi_reset(SPI1);

	/* Set up SPI in Master mode with:
	 * Clock baud rate: 1/64 of peripheral clock frequency
	 * Clock polarity: Idle High
	 * Clock phase: Data valid on 2nd clock pulse
	 * Data frame format: 8-bit
	 * Frame format: MSB First
	 */
	spi_init_master(SPI1, baudrate, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE, SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_MSBFIRST);
	spi_set_data_size(SPI1, SPI_CR2_DS_8BIT);
	spi_fifo_reception_threshold_8bit(SPI1);

	/*
	 * Set NSS management to software.
	 *
	 * Note:
	 * Setting nss high is very important, even if we are controlling the GPIO
	 * ourselves this bit needs to be at least set to 1, otherwise the spi
	 * peripheral will not send any data out.
	 */
	spi_enable_software_slave_management(SPI1);
	spi_set_nss_high(SPI1);

	/* Enable SPI1 periph. */
	spi_enable(SPI1);
}

void lptim1_isr(void)
{
	if(lptimer_get_flag(LPTIM1, LPTIM_ISR_CMPM)) {
		/* match wakeup time */
		lptimer_disable_irq(LPTIM1, LPTIM_ISR_CMPM);
		HAL.lptim1_evt_ena &= ~LPTIM_ISR_CMPM;
	}
	if(lptimer_get_flag(LPTIM1, LPTIM_ISR_ARRM)) {
		HAL.tick_ovr++;
		/* check for wakeup event pending */
		if(HAL.tick_ovr == HAL.tick_alm) {
			lptimer_enable_irq(LPTIM1, LPTIM_ISR_CMPOK);
			lptimer_set_compare(LPTIM1, HAL.tick_cmp);
		}
	}
	if(lptimer_get_flag(LPTIM1, LPTIM_ISR_CMPOK)) {
		HAL.lptim1_evt_ena |= LPTIM_ISR_CMPM;
		lptimer_enable_irq(LPTIM1, LPTIM_ISR_CMPM);
	}
	lptimer_clear_flag(LPTIM1, LPTIM_ISR_ARRM|LPTIM_ISR_CMPM|LPTIM_ISR_EXTTRIG|LPTIM_ISR_CMPOK|LPTIM_ISR_ARROK|LPTIM_ISR_UP|LPTIM_ISR_DOWN);
}

static void hal_time_init(void)
{
	rcc_periph_clock_enable(RCC_PWR);
	pwr_disable_backup_domain_write_protect();
	RCC_BDCR &= ~(RCC_BDCR_LSEDRV_MASK << RCC_BDCR_LSEDRV_SHIFT);
	RCC_BDCR |= (RCC_BDCR_LSEDRV_HIGH << RCC_BDCR_LSEDRV_SHIFT);
	rcc_osc_on(RCC_LSE);
	rcc_wait_for_osc_ready(RCC_LSE);
	rcc_periph_clock_enable(RCC_LPTIM1);

	/* use LSE for LPTIM1 */
	RCC_CCIPR &= ~(RCC_CCIPR_LPTIMxSEL_MASK << RCC_CCIPR_LPTIM1SEL_SHIFT);
	RCC_CCIPR |= (RCC_CCIPR_LPTIMxSEL_LSE << RCC_CCIPR_LPTIM1SEL_SHIFT);

	lptimer_enable(LPTIM1);
	lptimer_set_prescaler(LPTIM1, LPTIM_CFGR_PRESC_1);
	lptimer_set_internal_clock_source(LPTIM1);
	lptimer_select_trigger_source(LPTIM1, LPTIM_CFGR_TRIGEN_SW);
	lptimer_set_period(LPTIM1, 0xffff);
	lptimer_clear_flag(LPTIM1, LPTIM_ISR_ARRM);
	lptimer_enable_irq(LPTIM1, LPTIM_ISR_ARRM);
	nvic_enable_irq(NVIC_LPTIM1_IRQ);
	lptimer_start_counter(LPTIM1, LPTIM_CR_CNTSTRT);

#ifdef CONFIG_HAL_MCO_ENABLE
	/* enable MCO */
	{
		uint32_t reg32 = RCC_CFGR;
		reg32 &= ~(RCC_CFGR_MCOPRE_MASK << RCC_CFGR_MCOPRE_SHIFT);
		RCC_CFGR = (reg32 | (RCC_CFGR_MCOPRE_DIV1 << RCC_CFGR_MCOPRE_SHIFT));
		reg32 &= ~(RCC_CFGR_MCO_MASK << RCC_CFGR_MCO_SHIFT);
		RCC_CFGR = (reg32 | (RCC_CFGR_MCO_LSE << RCC_CFGR_MCO_SHIFT));
		gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8);
		gpio_set_af(GPIOA, GPIO_AF0, GPIO8);
	}
#endif
}

static void set_alarm_time(ostime_t almtm)
{
	hal_disableIRQs();
	HAL.tick_alm = almtm / TICKTIMER_OVR;
	HAL.tick_cmp = almtm % TICKTIMER_OVR;
	if(HAL.tick_ovr == HAL.tick_alm) {
		lptimer_set_compare(LPTIM1, HAL.tick_cmp);
		lptimer_clear_flag(LPTIM1, LPTIM_ISR_CMPOK);
		lptimer_enable_irq(LPTIM1, LPTIM_ISR_CMPOK);
	}
	hal_enableIRQs();
}

static void hal_lowpower_init(void)
{
	RCC_APB1ENR1 |= RCC_APB1ENR1_PWREN;

	PWR_CR1 &= ~(PWR_CR1_LPMS_MASK << PWR_CR1_LPMS_SHIFT);
	PWR_CR1 |= (PWR_CR1_LPMS_STOP_2 << PWR_CR1_LPMS_SHIFT);
	RCC_APB1ENR1 &= ~RCC_APB1ENR1_PWREN;
	RCC_AHB1SMENR = RCC_AHB1SMENR_SRAM1SMEN;
	RCC_AHB2SMENR = RCC_AHB2SMENR_SRAM2SMEN;
	RCC_AHB3SMENR = 0;
	RCC_APB1SMENR1 = RCC_APB1SMENR1_LPTIM1SMEN;
	RCC_APB1SMENR2 = 0;
	RCC_APB2SMENR = 0;

#ifdef SWD_ENABLE
	DBGMCU_CR |= (DBGMCU_CR_SLEEP|DBGMCU_CR_STOP|DBGMCU_CR_STANDBY);
#else
	DBGMCU_CR &= ~(DBGMCU_CR_SLEEP|DBGMCU_CR_STOP|DBGMCU_CR_STANDBY);
#endif
}
#endif

void hal_init_ex (const void *pContext)
{
	memset(&HAL, 0x00, sizeof(HAL));

	hal_disableIRQs();

	// configure radio I/O and interrupt handler
	hal_io_init();
	// configure radio SPI
	hal_spi_init();
	// configure timer and interrupt handler
	hal_time_init();
#ifdef STM32L431CB
	hal_lowpower_init();
#endif
	hal_enableIRQs();
	(void)pContext;
}

void hal_pin_rxtx (u1_t val)
{
#if defined(HAL_RXTX_PORT) && defined(HAL_RXTX_PIN)
	if(val)
		gpio_set(HAL_RXTX_PORT, HAL_RXTX_PIN);
	else
		gpio_clear(HAL_RXTX_PORT, HAL_RXTX_PIN);
#else
	(void)val;
#endif
}

#ifdef STM32F103C8
void hal_pin_rst (u1_t val)
{
	if(val == 0 || val == 1) { // drive pin
		gpio_set_mode(HAL_RESET_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, HAL_RESET_PIN);
		if(val)
			gpio_set(HAL_RESET_PORT, HAL_RESET_PIN);
		else
			gpio_clear(HAL_RESET_PORT, HAL_RESET_PIN);
	} else { // keep pin floating
		gpio_set_mode(HAL_RESET_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, HAL_RESET_PIN);
	}
}
#endif
#ifdef STM32L431CB
void hal_pin_rst (u1_t val)
{
	if(val == 0 || val == 1) { // drive pin
		gpio_mode_setup(HAL_RESET_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, HAL_RESET_PIN);
		gpio_set_output_options(HAL_RESET_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, HAL_RESET_PIN);
		if(val)
			gpio_set(HAL_RESET_PORT, HAL_RESET_PIN);
		else
			gpio_clear(HAL_RESET_PORT, HAL_RESET_PIN);
	} else { // keep pin floating
		gpio_mode_setup(HAL_RESET_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_PULLUP, HAL_RESET_PIN);
	}
}
#endif

void hal_disableIRQs (void)
{
	cm_disable_interrupts();
	HAL.irqlevel++;
}

void hal_enableIRQs (void)
{
	if(--HAL.irqlevel == 0) {
		cm_enable_interrupts();
	}
}

#ifdef STM32F103C8
ostime_t hal_ticks (void)
{
	return rtc_get_counter_val() + HAL.tick_ovr * TICKTIMER_OVR;
}
#endif
#ifdef STM32L431CB
ostime_t hal_ticks (void)
{
	return lptimer_get_counter(LPTIM1) + HAL.tick_ovr * TICKTIMER_OVR;
}
#endif

u1_t hal_checkTimer (ostime_t targettime)
{
	ostime_t current = hal_ticks();
	u1_t ret;

	/* correct reload timing */
	targettime -= 4;

	if(current < targettime) {
		/* targettime is in future */
		ret = 0;
	} else if(current == targettime) {
		/* exact match */
		ret = 1;
	} else if((current - targettime) > HAL_TIME_OVERRUN_DETECT) {
		/*
		 * the difference between current and target time is
		 * more than two weeks in the past, assume overflow,
		 * targettime is in future overflowed
		 */
		ret = 0;
	} else {
		/* target time is in the past */
		ret = 1;
	}
	if(!ret && HAL.alarm_time != targettime) {
		/* prepare for sleep */
		HAL.alarm_time_valid = 1;
		HAL.alarm_programmed = 0;
		HAL.alarm_time = targettime;
	}
	return ret;
}

static u1_t alarm_repare(void)
{
	u1_t ret = 0;
	if(!HAL.alarm_programmed) {
		ostime_t a = HAL.alarm_time;
		ostime_t c = hal_ticks();
		if(HAL.alarm_time_valid) {
			if(a > c) {
				/*
				 * Avoid small sleep time to not risk that the RTC
				 * time increases to the same value as the alarm that
				 * is programmed.
				 */
				if((a-c) > 5)
					ret = 1;
				else {
					/*
					 * Do busy wait for short time
					 */
					volatile int i;
					do {
						for(i=0;i<100;i++)
							__asm__("nop");
					} while (a > hal_ticks());
				}
			}
			/*
			 * assume no sleep time longer than 2 weeks
			 */
			if(a < c && (c-a) > HAL_TIME_OVERRUN_DETECT)
				ret = 1;
		}
		if(ret) {
			HAL.alarm_programmed = 1;
			HAL.alarm_time_valid = 0;
			set_alarm_time(a);
#ifdef HAL_TIME_DEBUG
			HAL.dbg[HAL.dbg_idx].tm = c;
			HAL.dbg[HAL.dbg_idx].al = a;
			HAL.dbg_idx++;
			HAL.dbg_idx %= 32;
#endif
		}
	}
	/*
	 * also return already programmed time, this is
	 * needed when we wake up from a non timer/rtc
	 * interrupt and should sleep again when there
	 * is no time update needed.
	 */
	return ret || HAL.alarm_programmed;
}

void hal_poll(void) __attribute__((weak));
void hal_poll(void) {}

void hal_wakeup(void) __attribute__((weak));
void hal_wakeup(void) {}

void hal_sleep (void)
{
	if(HAL.irqlevel) cm_enable_interrupts();
	if(alarm_repare()) {
#ifdef STM32F103C8
		rtc_clear_flag(RTC_ALR);
#endif
#ifdef DEEP_SLEEP_ENABLE
#ifdef STM32F103C8
		pwr_set_stop_mode();
		pwr_voltage_regulator_low_power_in_stop();
		pwr_clear_wakeup_flag();
		pwr_clear_standby_flag();
#endif
		SCB_SCR |= SCB_SCR_SLEEPDEEP;
		__WFI();
		SCB_SCR &= ~SCB_SCR_SLEEPDEEP;
#endif
	}
#ifndef DEEP_SLEEP_ENABLE
	else hal_poll();
#endif
	if(HAL.irqlevel) cm_disable_interrupts();
}

ostime_t hal_waitUntil (ostime_t time)
{
	if(HAL.irqlevel) {
		/* busy wait when interrupts are locked */
		while(hal_ticks() < time) {}
	} else {
		/* sleep loop with wakeup timer checking */
		while(!hal_checkTimer(time))
			hal_sleep();
	}
	return 0;
}

void hal_failed (const char *file, u2_t line)
{
	printf("%s  %s:%u\n", __func__, file, line);
    hal_disableIRQs();
    while(1);
}

#ifdef STM32F103C8
void rtc_alarm_isr(void)
{
	if(rtc_check_flag(RTC_ALR)) {
		uint32_t al = rtc_get_alarm_val();
		rtc_clear_flag(RTC_ALR);

		/* match wakeup time */
		if(HAL.tick_cmp == al)
			HAL.alarm_programmed = 0;

		if(al == 0) {
			/* overflow detected */
			HAL.tick_ovr++;
			if(HAL.tick_ovr == HAL.tick_alm) {
				rtc_set_alarm_time(HAL.tick_cmp);
			} else {
				/* trigger alarm on overflow */
				rtc_set_alarm_time(0);
			}
		}
	}
	exti_reset_request(EXTI17);
}
#endif

#ifdef STM32F103C8
void exti0_isr(void)
{
	exti_reset_request(EXTI0);
	radio_irq_handler(0);
}

void exti1_isr(void)
{
	exti_reset_request(EXTI1);
	radio_irq_handler(1);
}
#endif
#ifdef STM32L431CB
void exti0_isr(void)
{
	exti_reset_request(EXTI0);
	radio_irq_handler(1);
}

void exti1_isr(void)
{
	exti_reset_request(EXTI1);
	radio_irq_handler(0);
}
#endif

void hal_processPendingIRQs(void)
{
}

ostime_t hal_setModuleActive (bit_t val)
{
	(void)val;
	return 0;
}

uint8_t hal_getTxPowerPolicy(
	u1_t inputPolicy,
	s1_t requestedPower,
	u4_t frequency
) {
	(void)inputPolicy;
	(void)requestedPower;
	(void)frequency;
	return LMICHAL_radio_tx_power_policy_paboost;
}

bit_t hal_queryUsingTcxo(void) {
	return 0;
}

s1_t hal_getRssiCal (void) {
	return 0;
}

static void hal_spi_trx(u1_t cmd, u1_t* buf, size_t len, bit_t is_read) {
	gpio_clear(HAL_NSS_PORT, HAL_NSS_PIN);
#ifdef STM32F103C8
	spi_xfer(SPI1, cmd);
#endif
#ifdef STM32L431CB
	spi_send8(SPI1, cmd);
	(void)spi_read8(SPI1);
#endif
	for (; len > 0; --len, ++buf) {
		u1_t data = is_read ? 0x00 : *buf;
#ifdef STM32F103C8
		data = spi_xfer(SPI1, data);
#endif
#ifdef STM32L431CB
		spi_send8(SPI1, data);
		data = spi_read8(SPI1);
#endif
		if (is_read)
			*buf = data;
	}
	gpio_set(HAL_NSS_PORT, HAL_NSS_PIN);
}

void hal_spi_write(u1_t cmd, const u1_t* buf, size_t len) {
	hal_spi_trx(cmd, (u1_t*)buf, len, 0);
}

void hal_spi_read(u1_t cmd, u1_t* buf, size_t len) {
	hal_spi_trx(cmd, buf, len, 1);
}
