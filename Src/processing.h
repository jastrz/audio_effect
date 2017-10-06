#ifndef _PROCESSING_H
#define _PROCESSING_H

#include "math.h"
#include "stm32l4xx_hal.h"
#include "hardware.h"

#define PI 3.14159
#define BUFFER_SIZE 32
#define OFFSET 1995
#define DELAY_SIZE 11000

extern uint16_t buffin[BUFFER_SIZE];
extern uint16_t buffout[BUFFER_SIZE];
extern uint16_t sinbuff[BUFFER_SIZE];
extern float buff[BUFFER_SIZE];
extern float lastbuff[DELAY_SIZE];

extern unsigned i;
extern unsigned j;
extern unsigned BuffIter;

extern int speed;
extern float depth;

enum TREMOLO_TYPE { SIN = 0, TRIANGLE = 1};
extern enum TREMOLO_TYPE tremolo_type;

void CircleBuffer(float*, float*);         //needed for delay effect (todo)
void Process();                            //applying simple tremolo
void ADC_GetValues();                      //getting values from adc
void DAC_SetValues();                      //writing values to dac
void HandleTim_2_IRQ();                    //on TIM2 interrupt (~48kHz) - main IRQ
void HandleTim_7_IRQ();                    //on TIM7 interrupt (~1Hz)   - GPIO IRQ
void UpdateIterators();


#endif