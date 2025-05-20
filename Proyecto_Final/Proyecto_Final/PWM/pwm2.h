
#ifndef PWM2_H_
#define PWM2_H_

#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>

//Definiciones para el modo de PWM
#define PWM2_INVERTIDO    1
#define PWM2_NO_INVERTIDO 0

//Prototipos de funciones del módulo PWM
//Inicializa Timer0 en modo Fast PWM para OC0A y OC0B.
void initPWM2_FAST(uint8_t invertido, uint16_t prescaler);

//Actualiza el duty cycle del canal OC2A
void updateDutyCycleC(uint8_t dutyCycle);

//Actualiza el duty cycle del canal OC2B
void updateDutyCycleD(uint8_t dutyCycle);

#endif /* PWM2_H_ */