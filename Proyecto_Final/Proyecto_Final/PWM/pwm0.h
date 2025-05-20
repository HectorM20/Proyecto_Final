
#ifndef PWM_H
#define PWM_H

#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>

//Definiciones para el modo de PWM
#define PWM0_INVERTIDO    1
#define PWM0_NO_INVERTIDO 0

//Prototipos de funciones del módulo PWM
//Inicializa Timer0 en modo Fast PWM para OC0A y OC0B.
void initPWM0_FAST(uint8_t invertidoA, uint16_t prescaler);

//Actualiza el duty cycle del canal OC0A (para el primer servo).
void updateDutyCycleA(uint8_t dutyCycle);

//Actualiza el duty cycle del canal OC0B (para el segundo servo).
void updateDutyCycleB(uint8_t dutyCycle);

#endif /* PWM0_H_ */