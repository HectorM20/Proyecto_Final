#include "pwm0.h"

void initPWM0_FAST(uint8_t invertidoA, uint16_t prescaler)
{
	//Configurar PD6 como salida para OC0A
	DDRD |= (1 << PD6);
	
	//Configurar PD5 como salida para OC0B
	DDRD |= (1 << PD5);
	
	//Limpiar la configuración previa para OC0A en TCCR0A
	TCCR0A &= ~((1 << COM0A1) | (1 << COM0A0) | (1 << COM0B1) | (1 << COM0B0));
	
	//Configurar canal OC0A
	if (invertidoA == PWM0_INVERTIDO)
	{
		TCCR0A |= (1 << COM0A1) | (1 << COM0A0);
		TCCR0A |= (1 << COM0B1) | (1 << COM0B0);
	}
	else
	{
		TCCR0A |= (1 << COM0A1);		//Modo no invertido para OC0A
		TCCR0A |= (1 << COM0B1);		//Modo no invertido para OC0B.
	}
	
	//Configurar modo Fast PWM: Modo 3 (WGM01=1 y WGM00=1, con WGM02=0)
	TCCR0A |= (1 << WGM01) | (1 << WGM00);
	TCCR0B &= ~(1 << WGM02);
	
	//Configurar el prescaler en TCCR0B
	TCCR0B &= ~((1 << CS02) | (1 << CS01) | (1 << CS00));	//Limpiar bits del prescaler
	switch (prescaler)
	{
		case 1:
		TCCR0B |= (1 << CS00);
		break;
		case 8:
		TCCR0B |= (1 << CS01);
		break;
		case 64:
		TCCR0B |= (1 << CS01) | (1 << CS00);
		break;
		case 256:
		TCCR0B |= (1 << CS02);
		break;
		case 1024:
		TCCR0B |= (1 << CS02) | (1 << CS00);
		break;
		default:
		//Si se pasa un valor no soportado, no se configura nada
		break;
	}
	
	//Inicializar OCR0A con el valor inicial
	OCR0A = 8;
	OCR0B = 8;
}

void updateDutyCycleA(uint8_t dutyCycle)
{
	OCR0A = dutyCycle;
}
void updateDutyCycleB(uint8_t dutyCycle)
{
	OCR0B = dutyCycle;
}