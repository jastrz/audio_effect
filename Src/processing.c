#include "processing.h"

#define TR_AMPL 2.0                  //amplitude of triangle wave
float modSignal;
float triangleWave;

uint16_t buffin[BUFFER_SIZE];
uint16_t buffout[BUFFER_SIZE];
float buff[BUFFER_SIZE];
float lastbuff[DELAY_SIZE];
unsigned i = 0;                      //controlling BUFFER_SIZE
unsigned j = 0;                      //controlling speed
unsigned BuffIter = 0;               //controlling CircleBuffer
int speed = 8192;
float depth = 0;
enum TREMOLO_TYPE tremolo_type = 0;

void CircleBuffer(float* input, float* buffer)
{
    int k = BuffIter*BUFFER_SIZE;
    for(; k < BuffIter*BUFFER_SIZE+BUFFER_SIZE; k++)
    {
        buffer[k] = input[k-BuffIter*BUFFER_SIZE];
    }
    
    BuffIter++;
    if(BuffIter > DELAY_SIZE/BUFFER_SIZE - 1)
    {
        BuffIter = 0; 
    }
}

/**** APPLYING SIMPLE TREMOLO ****/
void Process()			
{
    buff[i] = buffin[i] - OFFSET;
    
    switch(tremolo_type)
    {
    case SIN:
      modSignal = buff[i] * depth * sin( 2*PI*j/speed );
      break;
      
    case TRIANGLE:
      triangleWave = (2.0*TR_AMPL/speed) * ( fabs( fmod(j,speed) - speed/2 ) - speed/4 );    // https://en.wikipedia.org/wiki/Triangle_wave
      modSignal = buff[i] * depth * triangleWave;
      break;
      
    default:
      break;
    }
    
    buff[i] = 2 * ( buff[i] + modSignal );
    buffout[i] = (uint32_t)( buff[i] + OFFSET/2 );
	/*2*lastbuff[i+BuffIter*BUFFER_SIZE] +*/
}


/**** CONVERTING 3 VALUES
***** 1 - buffin - signal input
***** 2 - speed  - modulation speed
***** 3 - depth  - modulation depth  ****/
void ADC_GetValues()
{
    buffin[i] = HAL_ADC_GetValue(&hadc1);
    speed = 20*HAL_ADC_GetValue(&hadc1);                 
    depth = (float)HAL_ADC_GetValue(&hadc1)/4000.0;
    HAL_ADC_Start(&hadc1);             //starting new conversion
}

/**** WRITING VALUES TO DAC ****/
void DAC_SetValues()
{
    HAL_DAC_SetValue(&hdac1,DAC_CHANNEL_1,DAC_ALIGN_12B_R,(uint32_t)buffout[i]);
}

void UpdateIterators()
{
    i++;
    j++;
    
    if(i>BUFFER_SIZE-1)
    {
      i=0;
      CircleBuffer(buff, lastbuff);
    }
    if(j>speed-1)
      j=0;
}

void HandleTim_2_IRQ()
{
    ADC_GetValues();
    Process();
    DAC_SetValues();
    UpdateIterators();
}

void HandleTim_7_IRQ()
{
    if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == 1)
    {
      tremolo_type++;                           // switch tremolo
      tremolo_type %= 2;                        //      mode
    }
}





