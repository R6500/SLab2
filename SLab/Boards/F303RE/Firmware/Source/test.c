/**************************************************************************

 SLab v2
 File: test.c

 Test module source code

 Used only during the test stage

 **************************************************************************/

// Includes
#include "ch.h"
#include "hal.h"
#include "slab.h"
#include "analog.h"
#include "bits.h"

#ifdef USE_TEST

/**************** Constants *******************************************/

static void testGPTcallback(GPTDriver *gptp);
static const GPTConfig gpt3cfgTest = {
  100000,                /* 100 kHz clock for Timer 3 */
  testGPTcallback        /* Callback */
};

/**************** Private functions ***********************************/

static void testGPTcallback(GPTDriver *gptp)
 {
 (void)gptp;
 static int counter=0,led=0;

 counter++;
 if (counter==10000)
     {
	 counter = 0;
	 led = !led;
	 if (led)
		  DAC2_WRITE(65000);
	    else
	      DAC2_WRITE(0);
     }
 }

/**************** Public functions ***********************************/

// Program timer2
void programTimer3(void)
 {
 // Enable clock
 BIT_SET(RCC->APB1ENR,RCC_APB1ENR_TIM3EN);

 // Set TRGO on update event
 BIT_FIELD_WRITE(TIM3->CR2,4,7,2);

 // Set Reload counter from sample time
 TIM3->ARR = 1000;

 // Set NVIC
 nvicEnableVector(TIM3_IRQn,2);

 // Enable Interrupt on Update
 BIT_SET(TIM3->DIER,TIM_DIER_UIE);

 // Enable counter
 BIT_SET(TIM3->CR1,TIM_CR1_CEN);
 }


void testGPT(void)
 {
 // GPT 3 timer initialization
 gptStart(&GPTD3, &gpt3cfgTest);

 gptStartContinuous(&GPTD3,10);
 }

#endif //USE_TEST


/********* IRQ Handlers *******************************/

/******************************


// H_FAST_IRQ_HANDLER(TIM3_IRQHandler)

// Timer 3 RSI
CH_IRQ_HANDLER(TIM3_IRQHandler)
 {
 static int counter=0,led=0;

 CH_IRQ_PROLOGUE();

 PRO1_SET

 counter++;
 if (counter==10000)
     {
	 counter = 0;
	 led = !led;
	 if (led)
		  DAC2_WRITE(4095);
	    else
	      DAC2_WRITE(0);
     }

 // Clear pending register
 BIT_CLEAR(TIM3->SR,TIM_SR_UIF);

 PRO1_CLEAR

 CH_IRQ_EPILOGUE();
 }
*********************************/
