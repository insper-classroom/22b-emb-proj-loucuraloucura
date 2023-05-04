/************************************************************************
* 5 semestre - Eng. da Computao - Insper
*
* 2021 - Exemplo com HC05 com RTOS
*
*/

#include <asf.h>
#include "conf_board.h"
#include <string.h>

/************************************************************************/
/* defines                                                              */
/************************************************************************/

typedef struct {
	volatile char id;
	volatile char button;
	volatile int value;
} Data;

#define BUT_PIO_RED            	PIOA
#define BUT_PIO_ID_RED         	ID_PIOA
#define BUT_IDX_RED            	3
#define BUT_IDX_MASK_RED       	(1 << BUT_IDX_RED)

#define BUT_PIO_YELLOW         	PIOA
#define BUT_PIO_ID_YELLOW      	ID_PIOA
#define BUT_IDX_YELLOW         	21
#define BUT_IDX_MASK_YELLOW     (1 << BUT_IDX_YELLOW)

#define BUT_PIO_GREEN          	PIOC
#define BUT_PIO_ID_GREEN       	ID_PIOC
#define BUT_IDX_GREEN          	13
#define BUT_IDX_MASK_GREEN     	(1 << BUT_IDX_GREEN)

#define BUT_PIO_BLUE          	PIOD
#define BUT_PIO_ID_BLUE       	ID_PIOD
#define BUT_IDX_BLUE          	11
#define BUT_IDX_MASK_BLUE     	(1 << BUT_IDX_BLUE)

#define BUT_PIO_UP            	PIOD
#define BUT_PIO_ID_UP         	ID_PIOD
#define BUT_IDX_UP            	30
#define BUT_IDX_MASK_UP       	(1 << BUT_IDX_UP)

#define BUT_PIO_DOWN          	PIOA
#define BUT_PIO_ID_DOWN       	ID_PIOA
#define BUT_IDX_DOWN          	6
#define BUT_IDX_MASK_DOWN     	(1 << BUT_IDX_DOWN)

#define BUT_PIO_LEFT          	PIOC
#define BUT_PIO_ID_LEFT       	ID_PIOC
#define BUT_IDX_LEFT          	19
#define BUT_IDX_MASK_LEFT     	(1 << BUT_IDX_LEFT)

#define BUT_PIO_RIGHT	        PIOC
#define BUT_PIO_ID_RIGHT        ID_PIOC
#define BUT_IDX_RIGHT           17
#define BUT_IDX_MASK_RIGHT      (1 << BUT_IDX_RIGHT)

#define AFEC_POT AFEC0
#define AFEC_POT_ID ID_AFEC0
#define AFEC_POT_CHANNEL 5 // Canal do pino PB2

// usart (bluetooth ou serial)
// Descomente para enviar dados
// pela serial debug

//#define DEBUG_SERIAL

#ifdef DEBUG_SERIAL
#define USART_COM USART1
#define USART_COM_ID ID_USART1
#else
#define USART_COM USART0
#define USART_COM_ID ID_USART0
#endif

/************************************************************************/
/* RTOS                                                                 */
/************************************************************************/

#define TASK_BLUETOOTH_STACK_SIZE            (4096/sizeof(portSTACK_TYPE))
#define TASK_BLUETOOTH_STACK_PRIORITY        (tskIDLE_PRIORITY)

void task_bluetooth(void);
void task_handshake(void);

/************************************************************************/
/* prototypes                                                           */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);
static void config_AFEC_pot(Afec *afec, uint32_t afec_id, uint32_t afec_channel,
afec_callback_t callback);
/************************************************************************/
/* constants                                                            */
/************************************************************************/
QueueHandle_t xQueueComando;
TimerHandle_t xTimer;

TaskHandle_t xHandshake = NULL;
TaskHandle_t xBluetooth = NULL;
/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/

/************************************************************************/
/* RTOS application HOOK                                                */
/************************************************************************/

/* Called if stack overflow during execution */
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	* identify which task has overflowed its stack.
	*/
	for (;;) {
	}
}

/* This function is called by FreeRTOS idle task */
extern void vApplicationIdleHook(void) {
	pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
}

/* This function is called by FreeRTOS each tick */
extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/
void vTimerCallback(TimerHandle_t xTimer) {
	/* Selecina canal e inicializa conversão */
	afec_channel_enable(AFEC_POT, AFEC_POT_CHANNEL);
	afec_start_software_conversion(AFEC_POT);
}
void callback_azul(void){
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	volatile Data data;
	data.id = '4';
	data.value = 0;
	if(pio_get(BUT_PIO_UP, PIO_INPUT, BUT_IDX_MASK_UP) == 0) {
		data.button = '4';
	}
	else {
		data.button = 'D';
	}
	xQueueSendFromISR(xQueueComando, &data, &xHigherPriorityTaskWoken);
}
void callback_verde(void){
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	volatile Data data;
	data.id = '3';
	data.value = 0;
	if(pio_get(BUT_PIO_UP, PIO_INPUT, BUT_IDX_MASK_UP) == 0) {
		data.button = '3';
	}
	else {
		data.button = 'C';
	}
	xQueueSendFromISR(xQueueComando, &data, &xHigherPriorityTaskWoken);
}
void callback_amarelo(void){
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	volatile Data data;
	data.id = '2';
	data.value = 0;
	if(pio_get(BUT_PIO_UP, PIO_INPUT, BUT_IDX_MASK_UP) == 0) {
		data.button = '2';
	}
	else {
		data.button = 'B';
	}
	xQueueSendFromISR(xQueueComando, &data, &xHigherPriorityTaskWoken);
}
void callback_vermelho(void){
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	volatile Data data;
	data.id = '1';
	data.value = 0;
	if(pio_get(BUT_PIO_UP, PIO_INPUT, BUT_IDX_MASK_UP) == 0) {
		data.button = '1';
	}
	else {
		data.button = 'A';
	}
	xQueueSendFromISR(xQueueComando, &data, &xHigherPriorityTaskWoken);
}
void callback_up(void){
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	volatile Data data;
	data.id = '5';
	data.value = 0;
	if(pio_get(BUT_PIO_UP, PIO_INPUT, BUT_IDX_MASK_UP) == 0) {
		data.button = '5';
	}
	else {
		data.button = 'E';
	}
	xQueueSendFromISR(xQueueComando, &data, &xHigherPriorityTaskWoken);
}
void callback_down(void){
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	volatile Data data;
	data.id = '6';
	data.value = 0;
	if(pio_get(BUT_PIO_UP, PIO_INPUT, BUT_IDX_MASK_UP) == 0) {
		data.button = '6';
	}
	else {
		data.button = 'F';
	}
	xQueueSendFromISR(xQueueComando, &data, &xHigherPriorityTaskWoken);
}

void callback_left(void){
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	volatile Data data;
	data.id = '7';
	data.value = 0;
	if(pio_get(BUT_PIO_UP, PIO_INPUT, BUT_IDX_MASK_UP) == 0) {
		data.button = '7';
	}
	else {
		data.button = 'G';
	}
	xQueueSendFromISR(xQueueComando, &data, &xHigherPriorityTaskWoken);
}

void callback_right(void){
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	volatile Data data;
	data.id = '8';
	data.value = 0;
	if(pio_get(BUT_PIO_UP, PIO_INPUT, BUT_IDX_MASK_UP) == 0) {
		data.button = '8';
	}
	else {
		data.button = 'H';
	}
	xQueueSendFromISR(xQueueComando, &data, &xHigherPriorityTaskWoken);
}
static void AFEC_pot_callback(void) {
	int adc;
	adc = afec_channel_get_value(AFEC_POT, AFEC_POT_CHANNEL);
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	//xQueueSendFromISR(, &adc, &xHigherPriorityTaskWoken);
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/
void send_package(Data data){
	char eof = 'X';
	while(!usart_is_tx_ready(USART_COM)) {
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
	usart_write(USART_COM, data.id);

	while(!usart_is_tx_ready(USART_COM)) {
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
	usart_write(USART_COM, data.button);

	while(!usart_is_tx_ready(USART_COM)) {
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
	usart_write(USART_COM, data.value);

	while(!usart_is_tx_ready(USART_COM)) {
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
	
	usart_write(USART_COM, data.value >> 8);

	while(!usart_is_tx_ready(USART_COM)) {
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
	usart_write(USART_COM, eof);
}

void io_init(void) {

	pmc_enable_periph_clk(BUT_PIO_ID_RED);
	pmc_enable_periph_clk(BUT_PIO_ID_YELLOW);
	pmc_enable_periph_clk(BUT_PIO_ID_GREEN);
	pmc_enable_periph_clk(BUT_PIO_ID_BLUE);
	
	pmc_enable_periph_clk(BUT_PIO_ID_UP);
	pmc_enable_periph_clk(BUT_PIO_ID_DOWN);
	pmc_enable_periph_clk(BUT_PIO_ID_LEFT);
	pmc_enable_periph_clk(BUT_PIO_ID_RIGHT);

	pio_configure(BUT_PIO_RED, PIO_INPUT, BUT_IDX_MASK_RED, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_PIO_RED, BUT_IDX_MASK_RED, 60);	
	pio_configure(BUT_PIO_YELLOW, PIO_INPUT, BUT_IDX_MASK_YELLOW, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_PIO_YELLOW, BUT_IDX_MASK_YELLOW, 60);	
	pio_configure(BUT_PIO_GREEN, PIO_INPUT, BUT_IDX_MASK_GREEN, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_PIO_GREEN, BUT_IDX_MASK_GREEN, 60);
	pio_configure(BUT_PIO_BLUE, PIO_INPUT, BUT_IDX_MASK_BLUE, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_PIO_BLUE, BUT_IDX_MASK_BLUE, 60);
	
	pio_configure(BUT_PIO_UP, PIO_INPUT, BUT_IDX_MASK_UP, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_PIO_UP, BUT_IDX_MASK_UP, 60);
	pio_configure(BUT_PIO_DOWN, PIO_INPUT, BUT_IDX_MASK_DOWN, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_PIO_DOWN, BUT_IDX_MASK_DOWN, 60);
	pio_configure(BUT_PIO_LEFT, PIO_INPUT, BUT_IDX_MASK_LEFT, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_PIO_LEFT, BUT_IDX_MASK_LEFT, 60);
	pio_configure(BUT_PIO_RIGHT, PIO_INPUT, BUT_IDX_MASK_RIGHT, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_PIO_RIGHT, BUT_IDX_MASK_RIGHT, 60);
			
	pio_handler_set(BUT_PIO_RED,
					BUT_PIO_ID_RED,
					BUT_IDX_MASK_RED,
					PIO_IT_EDGE,
					callback_vermelho);

	pio_handler_set(BUT_PIO_YELLOW,
					BUT_PIO_ID_YELLOW,
					BUT_IDX_MASK_YELLOW,
					PIO_IT_EDGE,
					callback_amarelo);

	pio_handler_set(BUT_PIO_GREEN,
					BUT_PIO_ID_GREEN,
					BUT_IDX_MASK_GREEN,
					PIO_IT_EDGE,
					callback_verde);
	
	pio_handler_set(BUT_PIO_BLUE,
					BUT_PIO_ID_BLUE,
					BUT_IDX_MASK_BLUE,
					PIO_IT_EDGE,
					callback_azul);
					
	pio_handler_set(BUT_PIO_UP,
	BUT_PIO_ID_UP,
	BUT_IDX_MASK_UP,
	PIO_IT_EDGE,
	callback_up);
	
	pio_handler_set(BUT_PIO_DOWN,
	BUT_PIO_ID_DOWN,
	BUT_IDX_MASK_DOWN,
	PIO_IT_EDGE,
	callback_down);
	
	pio_handler_set(BUT_PIO_LEFT,
	BUT_PIO_ID_LEFT,
	BUT_IDX_MASK_LEFT,
	PIO_IT_EDGE,
	callback_left);
	
	pio_handler_set(BUT_PIO_RIGHT,
	BUT_PIO_ID_RIGHT,
	BUT_IDX_MASK_RIGHT,
	PIO_IT_EDGE,
	callback_right);
	
	pio_enable_interrupt(BUT_PIO_UP, BUT_IDX_MASK_UP);
	pio_enable_interrupt(BUT_PIO_DOWN, BUT_IDX_MASK_DOWN);
	pio_enable_interrupt(BUT_PIO_LEFT, BUT_IDX_MASK_LEFT);
	pio_enable_interrupt(BUT_PIO_RIGHT, BUT_IDX_MASK_RIGHT);
	
	pio_enable_interrupt(BUT_PIO_GREEN, BUT_IDX_MASK_GREEN);
	pio_enable_interrupt(BUT_PIO_RED, BUT_IDX_MASK_RED);
	pio_enable_interrupt(BUT_PIO_YELLOW, BUT_IDX_MASK_YELLOW);
	pio_enable_interrupt(BUT_PIO_BLUE, BUT_IDX_MASK_BLUE);
	
	pio_get_interrupt_status(BUT_PIO_UP);
	pio_get_interrupt_status(BUT_PIO_DOWN);
	pio_get_interrupt_status(BUT_PIO_LEFT);
	pio_get_interrupt_status(BUT_PIO_RIGHT);
	
	pio_get_interrupt_status(BUT_PIO_RED);
	pio_get_interrupt_status(BUT_PIO_YELLOW);
	pio_get_interrupt_status(BUT_PIO_GREEN);
	pio_get_interrupt_status(BUT_PIO_BLUE);

	NVIC_EnableIRQ(BUT_PIO_ID_UP);
	NVIC_EnableIRQ(BUT_PIO_ID_DOWN);
	NVIC_EnableIRQ(BUT_PIO_ID_LEFT);
	NVIC_EnableIRQ(BUT_PIO_ID_RIGHT);

	NVIC_EnableIRQ(BUT_PIO_ID_GREEN);
	NVIC_EnableIRQ(BUT_PIO_ID_RED);
	NVIC_EnableIRQ(BUT_PIO_ID_YELLOW);
	NVIC_EnableIRQ(BUT_PIO_ID_BLUE);
	
	NVIC_SetPriority(BUT_PIO_ID_UP, 4);
	NVIC_SetPriority(BUT_PIO_ID_DOWN, 4);
	NVIC_SetPriority(BUT_PIO_ID_LEFT, 4);
	NVIC_SetPriority(BUT_PIO_ID_RIGHT, 4);
	
	NVIC_SetPriority(BUT_PIO_ID_GREEN, 4);
	NVIC_SetPriority(BUT_PIO_ID_RED, 4);
	NVIC_SetPriority(BUT_PIO_ID_YELLOW, 4);
	NVIC_SetPriority(BUT_PIO_ID_BLUE, 4);
}

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		#if (defined CONF_UART_CHAR_LENGTH)
		.charlength = CONF_UART_CHAR_LENGTH,
		#endif
		.paritytype = CONF_UART_PARITY,
		#if (defined CONF_UART_STOP_BITS)
		.stopbits = CONF_UART_STOP_BITS,
		#endif
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	#if defined(__GNUC__)
	setbuf(stdout, NULL);
	#else
	/* Already the case in IAR's Normal DLIB default configuration: printf()
	* emits one character at a time.
	*/
	#endif
}

uint32_t usart_puts(uint8_t *pstring) {
	uint32_t i ;

	while(*(pstring + i))
	if(uart_is_tx_empty(USART_COM))
	usart_serial_putchar(USART_COM, *(pstring+i++));
}

void usart_put_string(Usart *usart, char str[]) {
	usart_serial_write_packet(usart, str, strlen(str));
}

int usart_get_string(Usart *usart, char buffer[], int bufferlen, uint timeout_ms) {
	uint timecounter = timeout_ms;
	uint32_t rx;
	uint32_t counter = 0;

	while( (timecounter > 0) && (counter < bufferlen - 1)) {
		if(usart_read(usart, &rx) == 0) {
			buffer[counter++] = rx;
		}
		else{
			timecounter--;
			vTaskDelay(1);
		}
	}
	buffer[counter] = 0x00;
	return counter;
}

void usart_send_command(Usart *usart, char buffer_rx[], int bufferlen,
char buffer_tx[], int timeout) {
	usart_put_string(usart, buffer_tx);
	usart_get_string(usart, buffer_rx, bufferlen, timeout);
}

void config_usart0(void) {
	sysclk_enable_peripheral_clock(ID_USART0);
	usart_serial_options_t config;
	config.baudrate = 9600;
	config.charlength = US_MR_CHRL_8_BIT;
	config.paritytype = US_MR_PAR_NO;
	config.stopbits = false;
	usart_serial_init(USART0, &config);
	usart_enable_tx(USART0);
	usart_enable_rx(USART0);

	// RX - PB0  TX - PB1
	pio_configure(PIOB, PIO_PERIPH_C, (1 << 0), PIO_DEFAULT);
	pio_configure(PIOB, PIO_PERIPH_C, (1 << 1), PIO_DEFAULT);
}

int hc05_init(void) {
	char buffer_rx[128];
	usart_send_command(USART_COM, buffer_rx, 1000, "AT", 100);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
	usart_send_command(USART_COM, buffer_rx, 1000, "AT", 100);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
	usart_send_command(USART_COM, buffer_rx, 1000, "AT+NAMEDomDom", 100);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
	usart_send_command(USART_COM, buffer_rx, 1000, "AT", 100);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
	usart_send_command(USART_COM, buffer_rx, 1000, "AT+PIN0000", 100);
	
}
static void config_AFEC_pot(Afec *afec, uint32_t afec_id, uint32_t afec_channel,
                            afec_callback_t callback) {
  /*************************************
   * Ativa e configura AFEC
   *************************************/
  /* Ativa AFEC - 0 */
  afec_enable(afec);

  /* struct de configuracao do AFEC */
  struct afec_config afec_cfg;

  /* Carrega parametros padrao */
  afec_get_config_defaults(&afec_cfg);

  /* Configura AFEC */
  afec_init(afec, &afec_cfg);

  /* Configura trigger por software */
  afec_set_trigger(afec, AFEC_TRIG_SW);

  /*** Configuracao específica do canal AFEC ***/
  struct afec_ch_config afec_ch_cfg;
  afec_ch_get_config_defaults(&afec_ch_cfg);
  afec_ch_cfg.gain = AFEC_GAINVALUE_0;
  afec_ch_set_config(afec, afec_channel, &afec_ch_cfg);

  /*
  * Calibracao:
  * Because the internal ADC offset is 0x200, it should cancel it and shift
  down to 0.
  */
  afec_channel_set_analog_offset(afec, afec_channel, 0x200);

  /***  Configura sensor de temperatura ***/
  struct afec_temp_sensor_config afec_temp_sensor_cfg;

  afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
  afec_temp_sensor_set_config(afec, &afec_temp_sensor_cfg);

  /* configura IRQ */
  afec_set_callback(afec, afec_channel, callback, 1);
  NVIC_SetPriority(afec_id, 4);
  NVIC_EnableIRQ(afec_id);
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/
void task_handshake(void) {
	printf("Task HandShake started \n");
	
	printf("Inicializando HC05 \n");
	
	config_usart0();
	hc05_init();
	
	
	// configura LEDs e Botões
	io_init();
	
	char eof = 'X';

	Data data;
	char eof = 'X';

	data.id = 'W';
	data.button = 'W';
	data.value = 0;

	send_package(data);
	char resposta = 0;
	
	while(1) {
		if (usart_read(USART_COM, &resposta) == 0) {
			if (resposta == 'w') {
				xTaskCreate(task_bluetooth, "BLT", TASK_BLUETOOTH_STACK_SIZE, NULL,	TASK_BLUETOOTH_STACK_PRIORITY, &xBluetooth);
			}
		}
		else {
			send_package(data);
		}
		
		vTaskDelay(500 / portTICK_PERIOD_MS);
	}
}

void task_bluetooth(void) {
	vTaskDelete(xHandshake);
	printf("Task Bluetooth started \n");
	
	printf("Inicializando HC05 \n");
	//config_usart0();
	//hc05_init();

	// configura LEDs e Botões
	io_init();
	
	char eof = 'X';

	char comando;
	// Task não deve retornar.
	while(1) {
		if (xQueueReceive(xQueueComando, &(comando), 0)) {
			// envia status botão
			while(!usart_is_tx_ready(USART_COM)) {
				vTaskDelay(10 / portTICK_PERIOD_MS);
			}
			usart_write(USART_COM, comando);
		
			// envia fim de pacote
			while(!usart_is_tx_ready(USART_COM)) {
				vTaskDelay(10 / portTICK_PERIOD_MS);
			}
			usart_write(USART_COM, eof);
		}
	}
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/

int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	configure_console();
	
	xQueueComando = xQueueCreate(32, sizeof(Data));
	if (xQueueComando == NULL)
	printf("falha em criar a queue xQueueADC \n");

	/* Create task to make led blink */
	if (xTaskCreate(task_handshake, "HSK", TASK_BLUETOOTH_STACK_SIZE, NULL,	TASK_BLUETOOTH_STACK_PRIORITY, &xHandshake) != pdPASS) {
		printf("Erro ao criar task handshake \n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
