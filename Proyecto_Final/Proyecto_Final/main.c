//***************************************************************
//Universidad del Valle de Guatemala
//IE2023: Programación de Microcontroladores
//Autor: Héctor Alejandro Martínez Guerra
//Hardware: ATMEGA328P
//PROYECTO FINAL BRAZO ROBÓTICO
//***************************************************************


/****************************************/
//Encabezado (Libraries)
#define F_CPU 16000000
#include <avr/io.h>
#include <avr/interrupt.h>				//libreria de interrupciones
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <avr/eeprom.h>					//Librería EEPROM
#include "PWM/pwm0.h"
#include "PWM/pwm2.h"

/****************************************/
//Pines para botón y LEDs
#define BTN_PIN         PD2				//botón en PD2 (INT0)
#define LED1_PORT       PORTB
#define LED1_DDR        DDRB
#define LED1_PIN        PB0
#define LED2_PORT       PORTB
#define LED2_DDR        DDRB
#define LED2_PIN        PB1

/****************************************/
//Límites de PWM para cada servo (valores de OCR)
#define SERVO_A_MIN_PWM   8				//0°
#define SERVO_A_MAX_PWM  39				//180°
#define SERVO_B_MIN_PWM  17
#define SERVO_B_MAX_PWM  37
#define SERVO_C_MIN_PWM  17
#define SERVO_C_MAX_PWM  33
#define SERVO_D_MIN_PWM  30
#define SERVO_D_MAX_PWM  35

//Número de presets disponibles
#define MAX_PRESETS  5
//Cada preset almacena 4 bytes: A, B, C, D
uint8_t EEMEM eeprom_presets[MAX_PRESETS * 4];

//Modos de operación
#define MODE_MANUAL   0
#define MODE_REMOTE   1  //#POS via UART
#define MODE_PRESET   2  //#PLAY/#STORE via UART
volatile uint8_t mode = MODE_MANUAL;

//Valor inicial para el servo: usamos el valor mínimo (pulso ~1ms)
volatile uint8_t adc_value_A = SERVO_A_MIN_PWM;			//Para servo en OC0A (potenciómetro en ADC4)
volatile uint8_t adc_value_B = SERVO_B_MIN_PWM;			//Para servo en OC0B (potenciómetro en ADC5)
volatile uint8_t adc_value_C = SERVO_C_MIN_PWM;			//Para servo en OC2A (potenciómetro en ADC6)
volatile uint8_t adc_value_D = SERVO_D_MIN_PWM;			//Para servo en OC2B (potenciómetro en ADC7)
volatile uint8_t currentADCchannel = 0;		//0: leer ADC4, 1: leer ADC5, 2: leer ADC6, 3: leer ADC
//Variables para almacenar los valores mapeados de PWM.
uint8_t servoPWM_A = 0;						//OC0A
uint8_t servoPWM_B = 0;						//OC0B
uint8_t servoPWM_C = 0;						//OC2A
uint8_t servoPWM_D = 0;						//OC2B
//UART reception (rxBuf[] guarda hasta 31 caracteres más el '\0')
volatile char rxBuf[32];					//Buffer donde se acumula la línea entrante
volatile uint8_t bufIdx = 0;				//Índice de escritura en rxBuf
volatile uint8_t lineReady = 0;				//Flag: hay línea completa lista para procesar
//variable para definir el modo de operación
//volatile uint8_t modeUART = 1;  // 0 = manual, 1 = remoto

/****************************************/
//Function prototypes
void setup();
void initADC();
void initUART();
void UART_sendChar(char c);			//Enviar un carácter
void UART_sendString(char *texto);	//Enviar una cadena de valores
void procesarComando(char *cmd);
uint8_t mapADCToServo(uint8_t adc_val, uint8_t out_min, uint8_t out_max);
uint8_t readButtonDebounced();
/****************************************/
//Main Function
int main(void) {
	setup();
	uint8_t lastButtonState = 1, counter = 0;

	while (1) {
		//Leer botón con anti reborte
		uint8_t btn = readButtonDebounced();
		if (btn == 0 && lastButtonState == 1) {
			// flanco de bajada: ahora pulsado
			mode = (mode + 1) % 3;
		}
		lastButtonState = btn;

		// Modo manual: releer potenciómetros
		if (mode == MODE_MANUAL) {
			servoPWM_A = mapADCToServo(adc_value_A, SERVO_A_MIN_PWM, SERVO_A_MAX_PWM);
			servoPWM_B = mapADCToServo(adc_value_B, SERVO_B_MIN_PWM, SERVO_B_MAX_PWM);
			servoPWM_C = mapADCToServo(adc_value_C, SERVO_C_MIN_PWM, SERVO_C_MAX_PWM);
			servoPWM_D = mapADCToServo(adc_value_D, SERVO_D_MIN_PWM, SERVO_D_MAX_PWM);
		}
		// Actualizar PWM siempre
		updateDutyCycle0A(servoPWM_A);
		updateDutyCycle0B(servoPWM_B);
		updateDutyCycleA2(servoPWM_C);
		updateDutyCycleB2(servoPWM_D);

		//Procesar UART si hay línea completa
		if (lineReady) {
			procesarComando(rxBuf);
			bufIdx = 0; lineReady = 0;
		}

		//Actualizar LEDs según modo
		LED1_PORT = (LED1_PORT & ~((1<<LED1_PIN)|(1<<LED2_PIN)))
		| ((mode == MODE_REMOTE) << LED1_PIN)
		| ((mode == MODE_PRESET) << LED2_PIN);

		_delay_ms(8);
	}
	return 0;
}
/****************************************/
//NON-Interrupt subroutines
void setup()
{
	cli();
	//Configurar el reloj: reducir F_CPU a 1MHz (división por 16)
	CLKPR = (1 << CLKPCE);
	CLKPR = (1 << CLKPS2);				//Divide por 16: 16MHz/16 = 1MHz
	//Inicializar PWM en OC0A y OC0B
	initPWM0_FAST(PWM0_NO_INVERTIDO, 64);
	//Inicializar PWM en OC2A
	initPWM2_FAST(PWM2_NO_INVERTIDO, 64);
	//Inicializar ADC
	initADC();
	//Inicializar UART
	initUART();
	// Botón con pull-up en PD2
	DDRD &= ~(1<<BTN_PIN);
	PORTD |= (1<<BTN_PIN);
	// LEDs en PB0, PB1
	LED1_DDR |= (1<<LED1_PIN)|(1<<LED2_PIN);
	LED1_PORT &= ~((1<<LED1_PIN)|(1<<LED2_PIN));
	sei();
}

/****************************************/
//ADC
void initADC()
{
	ADMUX = 0;
	ADMUX |= (1<<REFS0);			//Voltaje de referencia AVcc
	ADMUX |= (1<<ADLAR);			//Justificación a la izquierda
	ADMUX |= (1<<MUX2);				//Configurar ADC4 del Arduino nano
	ADCSRA = 0;
	//Configurar Prescaler a 8 (1MHz/8 = 125KHz)
	ADCSRA |= (1 << ADPS1) | (1 << ADPS0);
	//Habilitar ADC y sus interrupciones
	ADCSRA |= (1<<ADEN) | (1<<ADIE);
	//Empezar ADC
	ADCSRA |= (1<<ADSC);
}

void initUART()
{
	//Configurar PD0 (RX) como entrada y PD1 (TX) como salida
	DDRD |= (1<<DDD1);
	DDRD &= ~(1<<DDD0);
	//Limpiar UCSR0A
	UCSR0A = 0;
	//Habilitar doble velocidad (U2X0=1) para trabajar a 1 MHz
	UCSR0A  = (1<<U2X0);
	//Configurar UCSR0B: habilitar transmisor, receptor y la interrupción de recepción.
	UCSR0B |= (1<<RXCIE0) | (1<<RXEN0)|(1<<TXEN0);
	//Configurar UCSR0C: 8 bits de datos, sin paridad, 1 bit de stop
	UCSR0C |= (1<<UCSZ00) | (1<<UCSZ01);
	//Configurar UBRR0 12 -> 9600 baudrate a 1Mhz
	UBRR0 = 12;
}

void UART_sendChar(char c)
{
	//Esperar a que UDR0 esté listo para recibir un nuevo dato
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = c;
}

void UART_sendString(char *texto)
{
	for (uint8_t i = 0; texto[i] != '\0'; i++)
	{
		UART_sendChar(texto[i]);
	}
}

/****************************************/
void procesarComando(char *cmd) {
	if (strncmp(cmd, "#POS,",5)==0) {
		uint8_t a,b,c,d;
		sscanf(cmd+5, "A:%hhu,B:%hhu,C:%hhu,D:%hhu", &a,&b,&c,&d);
		servoPWM_A = mapADCToServo((a*255)/180, SERVO_A_MIN_PWM, SERVO_A_MAX_PWM);
		servoPWM_B = mapADCToServo((b*255)/180, SERVO_B_MIN_PWM, SERVO_B_MAX_PWM);
		servoPWM_C = mapADCToServo((c*255)/180, SERVO_C_MIN_PWM, SERVO_C_MAX_PWM);
		servoPWM_D = mapADCToServo((d*255)/180, SERVO_D_MIN_PWM, SERVO_D_MAX_PWM);
		mode = MODE_REMOTE;
		UART_sendString("OK POS\n");
	}
	else if (strncmp(cmd, "#STORE,",7)==0) {
		uint8_t n = atoi(cmd+7);
		if (n < MAX_PRESETS) {
			uint16_t baddr = n*4;
			eeprom_write_byte(&eeprom_presets[baddr+0], servoPWM_A);
			eeprom_write_byte(&eeprom_presets[baddr+1], servoPWM_B);
			eeprom_write_byte(&eeprom_presets[baddr+2], servoPWM_C);
			eeprom_write_byte(&eeprom_presets[baddr+3], servoPWM_D);
			mode = MODE_PRESET;
			UART_sendString("OK STORE\n");
		}
	}
	else if (strncmp(cmd, "#PLAY,",6)==0) {
		uint8_t n = atoi(cmd+6);
		if (n < MAX_PRESETS) {
			uint16_t baddr = n*4;
			servoPWM_A = eeprom_read_byte(&eeprom_presets[baddr+0]);
			servoPWM_B = eeprom_read_byte(&eeprom_presets[baddr+1]);
			servoPWM_C = eeprom_read_byte(&eeprom_presets[baddr+2]);
			servoPWM_D = eeprom_read_byte(&eeprom_presets[baddr+3]);
			mode = MODE_PRESET;
			UART_sendString("OK PLAY\n");
		}
	}
}

//Función que mapea el valor ADC (0-255) al rango de PWM para el servo
uint8_t mapADCToServo(uint8_t adc_val, uint8_t out_min, uint8_t out_max)
{
	if (adc_val == 0)   return out_min;
	if (adc_val == 255) return out_max;
	return (uint8_t)(((uint16_t)adc_val * (out_max - out_min)) / 255 + out_min);
}

uint8_t readButtonDebounced(void) {
	static uint8_t state = 1, cnt = 0;
	uint8_t raw = (PIND & (1<<BTN_PIN)) ? 1 : 0;
	if (raw != state) {
		cnt++;
		if (cnt >= 2) { //20ms
			state = raw;
			cnt = 0;
		}
		} else {
		cnt = 0;
	}
	return state;
}
/****************************************/
//Interrupt routines
ISR(ADC_vect)
{
	if (currentADCchannel == 0)
	{
		//Leer ADC4 y se almacena en adc_value_A.
		adc_value_A = ADCH;
		ADMUX &= ~((1 << MUX2) | (1 << MUX1) | (1 << MUX0));	//Limpiar bits MUX
		ADMUX |= (1<<MUX2) | (1<<MUX0);							//Cambiar a ADC5
		currentADCchannel = 1;
	}
	else if (currentADCchannel == 1)
	{
		//Leer ADC5 y se almacena en adc_value_B.
		adc_value_B = ADCH;
		ADMUX &= ~((1 << MUX2) | (1 << MUX1) | (1 << MUX0));	//Limpiar bits MUX
		ADMUX |= (1 << MUX2) | (1<<MUX1);						//Cambiar a ADC6
		currentADCchannel = 2;
	}
	else if (currentADCchannel == 2)
	{
		//Leer ADC6 y se almacena en adc_value_C.
		adc_value_C = ADCH;
		ADMUX &= ~((1 << MUX2) | (1 << MUX1) | (1 << MUX0));	//Limpiar bits MUX
		ADMUX |= (1 << MUX2) | (1 << MUX1) | (1 << MUX0);		//Cambiar a ADC7
		currentADCchannel = 3;
	}
	else
	{
		//Leer ADC7 y se almacena en adc_value_D
		adc_value_D = ADCH;
		ADMUX &= ~((1 << MUX2) | (1 << MUX1) | (1 << MUX0));	//Limpiar bits MUX
		ADMUX |= (1<<MUX2);										//Regresar a ADC4
		currentADCchannel = 0;
	}
	
	//Reiniciar la conversión.
	ADCSRA |= (1 << ADSC);
}

ISR(USART_RX_vect) {
	char c = UDR0;
	if (c=='\r' || c=='\n') {
		rxBuf[bufIdx] = '\0';
		lineReady     = 1;
		} else if (bufIdx<31) {
		rxBuf[bufIdx++] = c;
	}
}
