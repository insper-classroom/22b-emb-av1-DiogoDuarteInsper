#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/* Botao da placa */
#define BUT_PIO     PIOA
#define BUT_PIO_ID  ID_PIOA
#define BUT_PIO_PIN 11
#define BUT_PIO_PIN_MASK (1 << BUT_PIO_PIN)

/*Botao Led1 - 45 graus*/
#define BUT_LED1		PIOD
#define BUT_LED1_ID		ID_PIOD
#define BUT_LED1_IDX	28
#define BUT_LED1_IDX_MASK (1 << BUT_LED1_IDX)

/*Botao Led2 - 90 graus*/
#define BUT_LED2		PIOC
#define BUT_LED2_ID		ID_PIOC
#define BUT_LED2_IDX	31
#define BUT_LED2_IDX_MASK (1 << BUT_LED2_IDX)

/*Botao Led3 - 180 graus*/

#define BUT_LED3		PIOA
#define BUT_LED3_ID		ID_PIOA
#define BUT_LED3_IDX	19
#define BUT_LED3_IDX_MASK (1 << BUT_LED3_IDX)


/* Pino 1 - Fase 0*/
#define Fase0		PIOA
#define Fase0_ID		ID_PIOA
#define Fase0_IDX	13
#define Fase0_IDX_MASK (1 << Fase0_IDX)

/* Pino 2 - Fase 1*/
#define Fase1		PIOC
#define Fase1_ID		ID_PIOC
#define Fase1_IDX	19
#define Fase1_IDX_MASK (1 << Fase1_IDX)

/* Pino 3 - Fase 2*/
#define Fase2		PIOA
#define Fase2_ID		ID_PIOA
#define Fase2_IDX	4
#define Fase2_IDX_MASK (1 << Fase2_IDX)

/* Pino 4 - Fase 3*/
#define Fase3		PIOA
#define Fase3_ID		ID_PIOA
#define Fase3_IDX	3
#define Fase3_IDX_MASK (1 << Fase3_IDX)

QueueHandle_t xQueueModo;
QueueHandle_t xQueueSteps;
SemaphoreHandle_t xSemaphoreRTT;



/** RTOS  */
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/** prototypes */
void but_callback(void);

void but1_callback(void);
void but2_callback(void);
void but3_callback(void);

void fase0_callback(void);
void fase1_callback(void);
void fase2_callback(void);
void fase3_callback(void);

static void BUT_init(void);

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void but_callback(void) {
}

void but1_callback(void){
	char id = '1';
	xQueueSendFromISR(xQueueModo, &id, 0);
}

void but2_callback(void){
	char id = '2';
	xQueueSendFromISR(xQueueModo, &id, 0);
}

void but3_callback(void){
	char id = '3';
	xQueueSendFromISR(xQueueModo, &id, 0);
}

void fase0_callback(void){
	pio_set(Fase0, Fase0_IDX_MASK);
	pio_clear(Fase1, Fase1_IDX_MASK);
	pio_clear(Fase2, Fase2_IDX_MASK);
	pio_clear(Fase3, Fase3_IDX_MASK);

}

void fase1_callback(void){
	pio_clear(Fase0, Fase0_IDX_MASK);
	pio_set(Fase1, Fase1_IDX_MASK);
	pio_clear(Fase2, Fase2_IDX_MASK);
	pio_clear(Fase3, Fase3_IDX_MASK);
}

void fase2_callback(void){
	pio_clear(Fase0, Fase0_IDX_MASK);
	pio_clear(Fase1, Fase1_IDX_MASK);
	pio_set(Fase2, Fase2_IDX_MASK);
	pio_clear(Fase3, Fase3_IDX_MASK);

}

void fase3_callback(void){
	pio_clear(Fase0, Fase0_IDX_MASK);
	pio_clear(Fase1, Fase1_IDX_MASK);
	pio_clear(Fase2, Fase2_IDX_MASK);
	pio_set(Fase3, Fase3_IDX_MASK);
}
/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_oled(void *pvParameters) {
	gfx_mono_ssd1306_init();
  gfx_mono_draw_string("oii", 0, 20, &sysfont);

	for (;;)  {
    

	}
}

static void task_modo(void *pvParameters){
	char id;
	int steps;
	
	for (;;) {
		if( xQueueReceive( xQueueModo, &id, ( TickType_t ) 500 )){
			if(id == '1'){
				steps = 45/0.17578125;
				xQueueSend(xQueueSteps, &steps, 10);
				gfx_mono_draw_filled_rect(0,30,20,10,0);
				gfx_mono_draw_string("+45", 0, 20, &sysfont);
			}
			
			else if(id == '2'){
				steps = 90/0.17578125;
				xQueueSend(xQueueSteps, &steps, 10);
				gfx_mono_draw_filled_rect(0,30,20,10,0);
				gfx_mono_draw_string("+90", 0, 20, &sysfont);
			}
			
			else if(id == '3'){
				steps = 180/0.17578125;
				xQueueSend(xQueueSteps, &steps, 10);
				gfx_mono_draw_filled_rect(0,30,20,10,0);
				gfx_mono_draw_string("+180", 0, 20, &sysfont);
			}
		
		}
	}
}

static void task_motor(void *pvParameters){
	
	int flag_rtt = 1;
	int steps;
	int contador = 0;
	for(;;){
		if( xQueueReceive( xQueueSteps, &steps, ( TickType_t ) 500 )){
			
			while(contador<steps){
				if (contador == 0){
					fase0_callback();
					contador+=1;
				}
				if (contador == 1){
					fase1_callback();
					contador+=1;
				}
				if (contador == 2){
					fase2_callback();
					contador+=1;
				}
				if (contador == 3){
					fase3_callback();
					contador = 0;
				}
				/*RTT_init(2000,10,0);
				if(flag_rtt){
					xSemaphoreGiveFromISR(xSemaphoreRTT, 0);
					flag_rtt = 0;
				}
				if( xSemaphoreTake(xSemaphoreRTT, 5000) == pdTRUE ){
					if (contador == 0){
						fase0_callback();
						contador+=1;
					}
					if (contador == 1){
						fase1_callback();
						contador+=1;
					}
					if (contador == 2){
						fase2_callback();
						contador+=1;
					}
					if (contador == 3){
						fase3_callback();
						contador = 0;
					}
					
				}*/
			}
			
		}
	}
}


/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

	uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
	
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	if (rttIRQSource & RTT_MR_ALMIEN) {
		uint32_t ul_previous_time;
		ul_previous_time = rtt_read_timer_value(RTT);
		while (ul_previous_time == rtt_read_timer_value(RTT));
		rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
	}

	/* config NVIC */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);

	/* Enable RTT interrupt */
	if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
	else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
	
}




static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

static void BUT_init(void) {

	
	// Inicializa clock do periférico PIO responsavel pelo botao
	pmc_enable_periph_clk(BUT_LED1_ID);
	pmc_enable_periph_clk(BUT_LED2_ID);
	pmc_enable_periph_clk(BUT_LED3_ID);

	// Configura PIO para lidar com o pino do botão como entrada
	// com pull-up
	pio_configure(BUT_LED1, PIO_INPUT, BUT_LED1_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_LED1, BUT_LED1_IDX_MASK, 60);
	
	pio_configure(BUT_LED2, PIO_INPUT, BUT_LED2_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_LED2, BUT_LED2_IDX_MASK, 60);
	
	pio_configure(BUT_LED3, PIO_INPUT, BUT_LED3_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_LED3, BUT_LED3_IDX_MASK, 60);

	// Configura interrupção no pino referente ao botao e associa
	// função de callback caso uma interrupção for gerada
	// a função de callback é a: but_callback()
	pio_handler_set(BUT_LED1,
	BUT_LED1_ID,
	BUT_LED1_IDX_MASK,
	PIO_IT_EDGE,
	but1_callback);
	
	pio_handler_set(BUT_LED2,
	BUT_LED2_ID,
	BUT_LED2_IDX_MASK,
	PIO_IT_EDGE,
	but2_callback);
	
	pio_handler_set(BUT_LED3,
	BUT_LED3_ID,
	BUT_LED3_IDX_MASK,
	PIO_IT_EDGE,
	but3_callback);

	// Ativa interrupção e limpa primeira IRQ gerada na ativacao
	pio_enable_interrupt(BUT_LED1, BUT_LED1_IDX_MASK);
	pio_get_interrupt_status(BUT_LED1);
	
	pio_enable_interrupt(BUT_LED2, BUT_LED2_IDX_MASK);
	pio_get_interrupt_status(BUT_LED2);
	
	pio_enable_interrupt(BUT_LED3, BUT_LED3_IDX_MASK);
	pio_get_interrupt_status(BUT_LED3);
	
	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BUT_LED1_ID);
	NVIC_SetPriority(BUT_LED1_ID, 4); // Prioridade 4
	
	NVIC_EnableIRQ(BUT_LED2_ID);
	NVIC_SetPriority(BUT_LED2_ID, 4);
	
	NVIC_EnableIRQ(BUT_LED3_ID);
	NVIC_SetPriority(BUT_LED3_ID, 4);
	
	/* conf botão como Fase0 */
	pio_configure(Fase0, PIO_INPUT, Fase0_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(Fase0,  Fase0_IDX_MASK, 60);
	pio_enable_interrupt(Fase0,  Fase0_IDX_MASK);
	pio_handler_set(Fase0, Fase0_ID,  Fase0_IDX_MASK, PIO_IT_FALL_EDGE , fase0_callback);
	
	/* conf botão como Fase1 */
	pio_configure(Fase1, PIO_INPUT, Fase1_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(Fase1,  Fase1_IDX_MASK, 60);
	pio_enable_interrupt(Fase1,  Fase1_IDX_MASK);
	pio_handler_set(Fase1, Fase1_ID,  Fase1_IDX_MASK, PIO_IT_FALL_EDGE , fase1_callback);
	
	/* conf botão como Fase2 */
	pio_configure(Fase2, PIO_INPUT, Fase2_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(Fase2,  Fase2_IDX_MASK, 60);
	pio_enable_interrupt(Fase2,  Fase2_IDX_MASK);
	pio_handler_set(Fase2, Fase2_ID,  Fase2_IDX_MASK, PIO_IT_FALL_EDGE , fase2_callback);
	
	/* conf botão como Fase3 */
	pio_configure(Fase3, PIO_INPUT, Fase3_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(Fase3,  Fase3_IDX_MASK, 60);
	pio_enable_interrupt(Fase3,  Fase3_IDX_MASK);
	pio_handler_set(Fase3, Fase3_ID,  Fase3_IDX_MASK, PIO_IT_FALL_EDGE , fase3_callback);
	
	
	
	/* configura prioridae */
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4);

	/* conf botão como entrada */
	pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_PIO, BUT_PIO_PIN_MASK, 60);
	pio_enable_interrupt(BUT_PIO, BUT_PIO_PIN_MASK);
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIO_PIN_MASK, PIO_IT_FALL_EDGE , but_callback);
}


/************************************************************************/
/* main                                                                 */
/************************************************************************/


int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	BUT_init();

	/* Initialize the console uart */
	configure_console();

	/* Create task to control oled */
	if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create oled task\r\n");
	}
	
	if (xTaskCreate(task_modo, "Modo", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create modo task\r\n");
	}
	
	if (xTaskCreate(task_motor, "Motor", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create motor task\r\n");
	}
	
	xQueueModo = xQueueCreate(32, sizeof(char) );
	
	if (xQueueModo == NULL){
	printf("falha em criar a fila \n");
	}

	xQueueSteps = xQueueCreate(32, sizeof(int) );

	if (xQueueSteps == NULL){
	printf("falha em criar a fila \n");
	}
	
	// cria semáforo binário
	xSemaphoreRTT = xSemaphoreCreateBinary();

	// verifica se semáforo foi criado corretamente
	if (xSemaphoreRTT == NULL){
	printf("falha em criar o semaforo \n");
	}
	/* Start the scheduler. */
	vTaskStartScheduler();

  /* RTOS não deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
