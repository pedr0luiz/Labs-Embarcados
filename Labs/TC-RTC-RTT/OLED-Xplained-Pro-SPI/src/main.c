#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

// LED1 PA0
#define LED1_PIO      PIOA 
#define LED1_PIO_ID   ID_PIOA
#define LED1_IDX      0
#define LED1_IDX_MASK (1 << LED1_IDX)

// LED2 PC30
#define LED2_PIO      PIOC
#define LED2_PIO_ID   ID_PIOC
#define LED2_IDX      30
#define LED2_IDX_MASK (1 << LED2_IDX)

// LED3 PB2
#define LED3_PIO      PIOB
#define LED3_PIO_ID   ID_PIOB
#define LED3_IDX      2
#define LED3_IDX_MASK (1 << LED3_IDX)

//LED_PLACA PC8
#define LED_PLACA_PIO      PIOC
#define LED_PLACA_PIO_ID   ID_PIOC
#define LED_PLACA_IDX      8
#define LED_PLACA_IDX_MASK (1 << LED_PLACA_IDX)

volatile char flag_tc = 0;
volatile char flag_tc0 = 0;
volatile char flag_rtt = 1;
volatile char flag_rtc = 0;
volatile char flag_rtc_sec = 0;

typedef struct  {
	uint32_t year;
	uint32_t month;
	uint32_t day;
	uint32_t week;
	uint32_t hour;
	uint32_t minute;
	uint32_t seccond;
} calendar;

void io_init(void);
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);
void pin_toggle(Pio *pio, uint32_t mask);
void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type);
void pisca_led(int n, int t);

void io_init(void){
	pmc_enable_periph_clk(LED1_PIO_ID);
	pmc_enable_periph_clk(LED2_PIO_ID);
	pmc_enable_periph_clk(LED3_PIO_ID);
	pmc_enable_periph_clk(LED_PLACA_PIO);
	
	pio_configure(LED1_PIO, PIO_OUTPUT_1, LED1_IDX_MASK, PIO_DEFAULT);
	pio_configure(LED2_PIO, PIO_OUTPUT_1, LED2_IDX_MASK, PIO_DEFAULT);
	pio_configure(LED3_PIO, PIO_OUTPUT_1, LED3_IDX_MASK, PIO_DEFAULT);
	pio_configure(LED_PLACA_PIO, PIO_OUTPUT_1, LED_PLACA_IDX_MASK, PIO_DEFAULT);
}

void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	pmc_enable_periph_clk(ID_TC);
	
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);

	tc_start(TC, TC_CHANNEL);
}

void TC1_Handler(void){
	volatile uint32_t ul_dummy;

	ul_dummy = tc_get_status(TC0, 1);

	UNUSED(ul_dummy);

	flag_tc = 1;
}

void TC0_Handler(void){
	volatile uint32_t ul_dummy;

	ul_dummy = tc_get_status(TC0, 0);

	UNUSED(ul_dummy);

	flag_tc0 = 1;
}
static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses){
	uint32_t ul_previous_time;

	/* Configure RTT for a 1 second tick interrupt */
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	ul_previous_time = rtt_read_timer_value(RTT);
	while (ul_previous_time == rtt_read_timer_value(RTT));
	
	rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);

	/* Enable RTT interrupt */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 0);
	NVIC_EnableIRQ(RTT_IRQn);
	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN);
}

void RTT_Handler(void){
	uint32_t ul_status;

	ul_status = rtt_get_status(RTT);

// 	/* IRQ due to Time has changed */
// 	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {  }

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		flag_rtt = 1;  
	}
}

void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type){
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(rtc, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(rtc, t.year, t.month, t.day, t.week);
	rtc_set_time(rtc, t.hour, t.minute, t.seccond);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(id_rtc);
	NVIC_ClearPendingIRQ(id_rtc);
	NVIC_SetPriority(id_rtc, 0);
	NVIC_EnableIRQ(id_rtc);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(rtc,  irq_type);
}

void RTC_Handler(void){
	uint32_t ul_status = rtc_get_status(RTC);

	/*
	*  Verifica por qual motivo entrou
	*  na interrupcao, se foi por segundo
	*  ou Alarm
	*/
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		flag_rtc_sec = 1;
		rtc_clear_status(RTC, RTC_SCCR_SECCLR);
	}
	
	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
			rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
      flag_rtc = 1;
	}
	
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
}

void pin_toggle(Pio *pio, uint32_t mask){
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}

void pisca_led(int n, int t){
	for (int i=0;i<n;i++){
		pio_clear(LED_PLACA_PIO, LED_PLACA_IDX_MASK);
		delay_ms(t);
		pio_set(LED_PLACA_PIO, LED_PLACA_IDX_MASK);
		delay_ms(t);
	}
}


int main (void){
	board_init();
	sysclk_init();
	WDT->WDT_MR = WDT_MR_WDDIS;
	io_init();
	delay_init();
	
	TC_init(TC0, ID_TC1, 1, 4);
	TC_init(TC0, ID_TC0, 0, 5);
	
	calendar rtc_initial = {2018, 3, 19, 12, 15, 45 ,1};
	RTC_init(RTC, ID_RTC, rtc_initial, RTC_IER_ALREN | RTC_SR_SEC);
	rtc_set_date_alarm(RTC, 1, rtc_initial.month, 1, rtc_initial.day);
	rtc_set_time_alarm(RTC, 1, rtc_initial.hour, 1, rtc_initial.minute, 1, rtc_initial.seccond + 20);
	
	uint32_t hour;
	uint32_t minute;
	uint32_t second;
	char timeString[512];
	
	gfx_mono_ssd1306_init();

	while(1) {
		if(flag_tc){
			pin_toggle(LED1_PIO, LED1_IDX_MASK);
			flag_tc = 0;
		}
		if(flag_rtc_sec){
			rtc_get_time(RTC, &hour, &minute, &second);
			sprintf(timeString, "%2d:%2d:%2d", hour, minute, second);
			gfx_mono_draw_string(timeString, 50, 16, &sysfont);
			flag_rtc_sec = 0;
		}
		if(flag_tc0){
			pin_toggle(LED3_PIO, LED3_IDX_MASK);
			flag_tc0 = 0;
		}
		if (flag_rtt){
		  pin_toggle(LED2_PIO, LED2_IDX_MASK);
		  
		  uint16_t pllPreScale = (int) (((float) 32768) / 4.0);
		  uint32_t irqRTTvalue = 16;

		  RTT_init(pllPreScale, irqRTTvalue);         
      
		  flag_rtt = 0;
		}
		if(flag_rtc){
			pisca_led(5, 50);
			flag_rtc = 0;
		}
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
	}
}
