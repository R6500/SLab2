/**************************************************************************

 SLab v2
 File: analog.c

 Analog module source code

 Register names can be found in:
 ChibiOS_17.6.0\os\common\ext\CMSIS\ST\STM32F3xx\stm32f303xe.h

 **************************************************************************/

// Includes
#include "ch.h"
#include "hal.h"
#include "slab.h"
#include "bits.h"
#include "analog.h"

/******* EXTERN REFERENCES ******************************************/

extern int nread;

/******* PRIVATE FUNCTIONS ******************************************/

// Configure opamp
void adcOpamp(OPAMP_TypeDef *opamp,int nii)
 {
 // Enable clocks
 BIT_SET(RCC->APB2ENR,RCC_APB2ENR_SYSCFGEN);
 // Enable opamp
 BIT_SET(opamp->CSR,OPAMP_CSR_OPAMPxEN);
 // Set as follower
 BIT_FIELD_WRITE(opamp->CSR,5,3,3);
 // Set nii
 BIT_FIELD_WRITE(opamp->CSR,2,3,nii);
 }

// Set a channel sample time
// Parameters:
//       adc : Pointer to the ADC_TypeDef
//   channel : ADC channel 1..18
//     value : Value to set
//                0 :   1.5 cycles
//                1 :   2.5 cycles
//                2 :   4.5 cycles
//                3 :   7.5 cycles
//                4 :  19.5 cycles
//                5 :  62.5 cycles
//                6 : 181.5 cycles
//                7 : 601.5 cycles
void adcSetSampleTime(ADC_TypeDef *adc,int channel,int value)
 {
 if (channel<10)
    BIT_FIELD_WRITE(adc->SMPR1,3*channel,7,value);
 else
    BIT_FIELD_WRITE(adc->SMPR2,3*(channel-10),7,value);

 //pc.printf("SMPR1: %d\r\n",adc->SMPR1);
 }

// Activate the ADC
void adcActivate(ADC_TypeDef *adc)
 {
 // Set activation flag ADEN
 adc->CR |= 1;

 // Wait for ADRDY flag
 while (!((adc->ISR) & 1)) ;
 }

// Deactivate the ADC
void adcDeactivate(ADC_TypeDef *adc)
 {
 // Set deactivation flag ADDIS
 adc->CR |= (1<<1);

 // Wait for deactivation of ADDIS
 while (((adc->CR) & (1<<1)));
 }

// Set ADC channel
// Parameters:
//      adc  : Pointer to ADC to use
//   channel : Channel to read 1..
void adcSetChannel(ADC_TypeDef *adc,int channel)
 {
 // Configure for just one conversion
 BIT_FIELD_WRITE(adc->SQR1,0,15,0);
 // Set channel to use
 BIT_FIELD_WRITE(adc->SQR1,6,31,channel);
 }

// Read ADC
// Parameters:
//      adc  : Pointer to ADC to use
int adcRead(ADC_TypeDef *adc)
  {
  PRO1_SET
  adc->CR |= ADC_CR_ADSTART;
  while (!(adc->ISR & ADC_ISR_EOC));
  PRO1_CLEAR
  return adc->DR;
  }

/********* PUBLIC INIT FUNCTIONS ******************************/

// Initialize all 8 ADCs and 2 DACs
void analogInit(void)
 {
 // Configure ADCs

 // ADC clocks
 BIT_SET(RCC->AHBENR,RCC_AHBENR_ADC12EN);  // Turn ON ADC12 clock
 BIT_SET(RCC->AHBENR,RCC_AHBENR_ADC34EN);  // Turn ON ADC34 clock

 // ADC frequency
 BIT_FIELD_WRITE(ADC1_2_COMMON->CCR,16,3,ADC_CLOCK);  // Set freq for ADC1,2
 BIT_FIELD_WRITE(ADC3_4_COMMON->CCR,16,3,ADC_CLOCK);  // Set freq for ADC3,4

 // ADC Single operation
 BIT_FIELD_WRITE(ADC1_2_COMMON->CCR,0,31,0); // Single 1,2
 BIT_FIELD_WRITE(ADC3_4_COMMON->CCR,0,31,0); // Single 1,2

 // Enable the reference on ADC1,2
 BIT_SET(ADC1_2_COMMON->CCR,ADC12_CCR_VREFEN);

 // Activate all four ADCs
 adcActivate(ADC1);
 adcActivate(ADC2);
 adcActivate(ADC3);
 adcActivate(ADC4);

 // Configure all four ADCs

 // ADC1 will be AD1 at PA0(A0) on channel #1
 palSetPadMode(GPIOA,0,PAL_MODE_INPUT_ANALOG);
 adcSetChannel(ADC1,1);

 // ADC2 will be Opamp2 output PA6(D12) on channel #3
 palSetPadMode(GPIOA,6,PAL_MODE_INPUT_ANALOG);
  adcSetChannel(ADC2,3);
 // Associated to AD3 PB0(A3) on opamp2 input #2
 palSetPadMode(GPIOB,0,PAL_MODE_INPUT_ANALOG);
 adcOpamp(OPAMP2,2);
 // And AD5 PA7(D11) on opamp2 input #3
 palSetPadMode(GPIOA,7,PAL_MODE_INPUT_ANALOG);
 //adcOpamp(OPAMP2,3);
 // And AD6 PB14 on opamp2 (ch#3) input #1
 palSetPadMode(GPIOB,14,PAL_MODE_INPUT_ANALOG);
 //adcOpamp(OPAMP2,1);

 // ADC3 will be Opamp3 output PB1 on channel #1
 palSetPadMode(GPIOB,1,PAL_MODE_INPUT_ANALOG);
 adcSetChannel(ADC3,1);
 // Associated to AD2 PA1(A1) on opamp3 (ch#1) input #2
 palSetPadMode(GPIOA,1,PAL_MODE_INPUT_ANALOG);
 adcOpamp(OPAMP3,1);
 // And AD7 PB13 on opamp3 (ch#1) input #0
 palSetPadMode(GPIOB,13,PAL_MODE_INPUT_ANALOG);
 //adcOpamp(OPAMP3,0);

 // ADC4 will be Opamp4 output PB12 on channel #3
 palSetPadMode(GPIOB,12,PAL_MODE_INPUT_ANALOG);
 adcSetChannel(ADC4,3);
 // Associated to AD4 PB11 on opamp input #1
 palSetPadMode(GPIOB,11,PAL_MODE_INPUT_ANALOG);
 adcOpamp(OPAMP4,1);
 // Or AD8 at PB15 on channel #5
 palSetPadMode(GPIOB,15,PAL_MODE_INPUT_ANALOG);
 // adcSetChannel(ADC4,5);

 // Configure DACs
 // Set pins
 palSetPadMode(GPIOA,4,PAL_MODE_INPUT_ANALOG);
 palSetPadMode(GPIOA,5,PAL_MODE_INPUT_ANALOG);
  // Turn ON DAC clock
 BIT_SET(RCC->APB1ENR,RCC_APB1ENR_DAC1EN);
 // Enable the DAC
 BIT_SET(DAC->CR,DAC_CR_EN1|DAC_CR_EN2);
 // Set to zero
 DAC1_WRITE(0);
 DAC2_WRITE(0);
 }

/**** PUBLIC DC ADC READ ************************************/

// Read AD1 (DC)
// AD2 PA1 on opamp3 (ch#1) input #2
int readAD1(void)
 {
 // Configure Opamp3
 adcSetSampleTime(ADC3,1,DC_SAMPLE_TIME);
 adcOpamp(OPAMP3,2);
 adcSetChannel(ADC3,1);
 return adcRead(ADC3);
 }

// Read AD2 (DC)
// AD3 PB0 on opamp2 (ch#3) input #2
int readAD2(void)
 {
 // Configure Opamp2
 adcSetSampleTime(ADC2,3,DC_SAMPLE_TIME);
 adcOpamp(OPAMP2,2);
 adcSetChannel(ADC2,3);
 return adcRead(ADC2);
 }

// Read AD3 (DC)
// AD4 PB11 on opamp4 (ch#3) input #1
int readAD3(void)
 {
 // Configure Opamp2
 adcSetSampleTime(ADC4,3,DC_SAMPLE_TIME);
 adcOpamp(OPAMP4,1);
 adcSetChannel(ADC4,3);
 return adcRead(ADC4);
 }

// Read AD4 (DC)
// Direct ADC1 connection on channel #1
int readAD4(void)
 {
 adcSetSampleTime(ADC1,1,DC_SAMPLE_TIME);
 adcSetChannel(ADC1,1);
 return adcRead(ADC1);
 }

// Read AD5 (DC)
// AD5 PA7 on opamp2 (ch#3) input #0
int readAD5(void)
 {
 // Configure Opamp2
 adcSetSampleTime(ADC2,3,DC_SAMPLE_TIME);
 adcOpamp(OPAMP2,3);
 adcSetChannel(ADC2,3);
 return adcRead(ADC2);
 }

// Read AD6 (DC)
// AD6 PB14 on opamp2 (ch#3) input #1
int readAD6(void)
 {
 // Configure Opamp2
 adcSetSampleTime(ADC2,3,DC_SAMPLE_TIME);
 adcOpamp(OPAMP2,1);
 adcSetChannel(ADC2,3);
 return adcRead(ADC2);
 }

// Read AD7 (DC)
// AD7 PB13 on opamp3 (ch#1) input #1
int readAD7(void)
 {
 // Configure Opamp3
 adcSetSampleTime(ADC3,1,DC_SAMPLE_TIME);
 adcOpamp(OPAMP3,0);
 adcSetChannel(ADC3,1);
 return adcRead(ADC3);
 }

// Read AD8 (DC)
// Direct ADC4 connection on channel #5
int readAD8(void)
 {
 adcSetSampleTime(ADC4,5,DC_SAMPLE_TIME);
 adcSetChannel(ADC4,5);
 return adcRead(ADC4);
 }

/*********************** DC CODE ********************************/

// Reads one analog line 1...
// Discards the first reading
// Uses the indicated number of readings
int analogRead(int line)
 {
 int i,value=0;
 uint32_t sum;

 sum = 0;
 for(i=0;i<=nread;i++)
    {
    switch(line)
       {
       case 1: value = readAD1();
               break;
       case 2: value = readAD2();
               break;
       case 3: value = readAD3();
               break;
       case 4: value = readAD4();
               break;
       case 5: value = readAD5();
               break;
       case 6: value = readAD6();
               break;
       case 7: value = readAD7();
               break;
       case 8: value = readAD8();
               break;
       }

    if (i) sum+=(value<<4);
    }

 value = sum/nread;

 return value;
 }

/**** PUBLIC REFERENCE READ ************************************/

// Read the Vref voltage from the internal voltage reference
// It should be altmost equal to the Vdd voltage
float readVref(void)
 {
 int i,value;
 float vdd;

 // The internal register REFINTCAL gives the reference
 // reading during manufacture for 3V3 supply
 // We don't use it because it doesn't account for
 // temperature changes

 //uint16_t *vrefintcal;
 //vrefintcal = (uint16_t*)0x1FFFF7BA;

 // Set maximum sample time on reference channel
 adcSetSampleTime(ADC1,18,7);
 // Set conversion for reference channel
 adcSetChannel(ADC1,18);

 // Perform 10 readings
 value = 0;
 for(i=0;i<10;i++)
     value += adcRead(ADC1);

 // Vdd calculation if we used REFINTCAL
 //vdd = 33.0*(*vrefintcal)/value;

 // Vdd calculation from nominal Vref value
 vdd = 12.0*4096/value;

 return vdd;
 }

/**** PUBLIC AC FUNCTIONS **************************************/

// Enable ADCs for AC operation
void ACenable(void)
 {
 // ADC Dual operation
 BIT_FIELD_WRITE(ADC1_2_COMMON->CCR,0,31,1); // Dual 1+2
 BIT_FIELD_WRITE(ADC3_4_COMMON->CCR,0,31,1); // Dual 3+4

 // Configure all four ADCs

 // AD1 direct connection
 adcSetSampleTime(ADC1,1,AC_SAMPLE_TIME);
 adcSetChannel(ADC1,1);

 // AD2 PA1 on opamp3 (ch#1) input #2
 adcSetSampleTime(ADC3,1,AC_SAMPLE_TIME);
 adcOpamp(OPAMP3,2);
 adcSetChannel(ADC3,1);

 // AD3 PB0 on opamp2 (ch#3) input #2
 adcSetSampleTime(ADC2,3,AC_SAMPLE_TIME);
 adcOpamp(OPAMP2,2);
 adcSetChannel(ADC2,3);

 // AD4 PB11 on opamp4 (ch#3) input #1
 adcSetSampleTime(ADC4,3,AC_SAMPLE_TIME);
 adcOpamp(OPAMP4,1);
 adcSetChannel(ADC4,3);
 }

// Disable ADCs for AC operation
void ACdisable(void)
 {
 // ADC Single operation
 BIT_FIELD_WRITE(ADC1_2_COMMON->CCR,0,31,0); // Single 1,2
 BIT_FIELD_WRITE(ADC3_4_COMMON->CCR,0,31,0); // Single 3,4
 }

/********** AC READ FUNTIONS ******************************
 *
 *  Those functions are not used because we need to
 *  optimize the execution time, so we inline them
 *
 **********************************************************/

// Perform measurement on all four channels
// ADC meas time is 772ns
void ACreadAll(int *value1,int *value2,int *value3,int *value4)
 {
 PRO1_SET
 ADC1->CR |= ADC_CR_ADSTART;
 ADC3->CR |= ADC_CR_ADSTART;
 while (!(ADC3->ISR & ADC_ISR_EOC));
 PRO1_CLEAR
 (*value4) = ADC1->DR;
 (*value2) = ADC2->DR;
 (*value1) = ADC3->DR;
 (*value3) = ADC4->DR;
 }

// Perform measurement on channels 1,3
// ADC meas time is 550ns
void ACread13(int *value1,int *value3)
 {
 PRO1_SET
 ADC1->CR |= ADC_CR_ADSTART;
 while (!(ADC1->ISR & ADC_ISR_EOC));
 PRO1_CLEAR
 (*value1) = ADC1->DR;
 (*value3) = ADC2->DR;
 }

// Perform measurement on channels 2,4
// ADC meas time is 550ns
void ACread24(int *value2,int *value4)
 {
 PRO1_SET
 ADC3->CR |= ADC_CR_ADSTART;
 while (!(ADC3->ISR & ADC_ISR_EOC));
 PRO1_CLEAR
 (*value2) = ADC3->DR;
 (*value4) = ADC4->DR;
 }

/**** PUBLIC AC SET SINGLE READ ************************************/

// Use instead of ACenable()

void setADforAC(int n)
 {
 switch(n)
    {
    case 1:
      // Set AD1 (DC)
      // AD2 PA1 on opamp3 (ch#1) input #2
      adcSetSampleTime(ADC3,1,AC_SAMPLE_TIME);
      adcOpamp(OPAMP3,2);
      adcSetChannel(ADC3,1);
      break;

    case 2:
      // Set AD2 (DC)
      // AD3 PB0 on opamp2 (ch#3) input #2
      adcSetSampleTime(ADC2,3,AC_SAMPLE_TIME);
      adcOpamp(OPAMP2,2);
      adcSetChannel(ADC2,3);
      break;

    case 3:
      // Set AD3 (DC)
      // AD4 PB11 on opamp4 (ch#3) input #1
      adcSetSampleTime(ADC4,3,AC_SAMPLE_TIME);
      adcOpamp(OPAMP4,1);
      adcSetChannel(ADC4,3);
      break;

    case 4:
      // Read AD4 (DC)
      // Direct ADC1 connection on channel #1
      adcSetSampleTime(ADC1,1,AC_SAMPLE_TIME);
      adcSetChannel(ADC1,1);
      break;

    case 5:
      // Read AD5 (DC)
      // AD5 PA7 on opamp2 (ch#3) input #0
      adcSetSampleTime(ADC2,3,AC_SAMPLE_TIME);
      adcOpamp(OPAMP2,3);
      adcSetChannel(ADC2,3);
      break;

    case 6:
      // Read AD6 (DC)
      // AD6 PB14 on opamp2 (ch#3) input #1
      adcSetSampleTime(ADC2,3,AC_SAMPLE_TIME);
      adcOpamp(OPAMP2,1);
      adcSetChannel(ADC2,3);
      break;

    case 7:
      // Read AD7 (DC)
      // AD7 PB13 on opamp3 (ch#1) input #1
      adcSetSampleTime(ADC3,1,AC_SAMPLE_TIME);
      adcOpamp(OPAMP3,0);
      adcSetChannel(ADC3,1);
      break;

    case 8:
      // Read AD8 (DC)
      // Direct ADC1 connection on channel #6
      adcSetSampleTime(ADC4,5,AC_SAMPLE_TIME);
      adcSetChannel(ADC4,5);
      break;
    }
 }
