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

void CircleBuffer(float*, float*);
void Process();                            //applying simple tremolo
void ADC_GetValues();              
void DAC_SetValues();                
void OnInterrupt();                        //on TIM2 interrupt (~48kHz)
void UpdateIterators();



#endif