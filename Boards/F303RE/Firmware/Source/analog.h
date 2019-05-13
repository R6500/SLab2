/**************************************************************************

 SLab v2
 File: analog.h

 Analog module header code

 **************************************************************************/

#ifndef ANALOG
#define ANALOG

// DAC write pseudocode
#define DAC1_WRITE(value) DAC->DHR12R1 = ((value)>>4)
#define DAC2_WRITE(value) DAC->DHR12R2 = ((value)>>4)

// ADC Configuration

// Set ADC clock
//   0 : Independent
//   1 : HCLK    72 MHz
//   2 : HCLK/2  36 MHz
//   3 : HCLK/4  18 MHz
#define ADC_CLOCK 1

// Sample time for DC and AC
//      0 :   1.5 cycles
//      1 :   2.5 cycles
//      2 :   4.5 cycles
//      3 :   7.5 cycles
//      4 :  19.5 cycles
//      5 :  62.5 cycles
//      6 : 181.5 cycles
//      7 : 601.5 cycles
#define DC_SAMPLE_TIME 7
#define AC_SAMPLE_TIME 2

// Function prototypes

void analogInit(void);    // Initialize ADCs and DACs
float readVref(void);  // Read reference voltage
int readAD1(void);     // DC read of AD1
int readAD2(void);     // DC read of AD2
int readAD3(void);     // DC read of AD3
int readAD4(void);     // DC read of AD4
int readAD5(void);     // DC read of AD5
int readAD6(void);     // DC read of AD6
int readAD7(void);     // DC read of AD7
int readAD8(void);     // DC read of AD8
int analogRead(int line); // DC read of one AD
void ACenable(void);
void setADforAC(int n);
void ACdisable(void);
void ACreadAll(int *value1,int *value2,int *value3,int *value4);
void ACread13(int *value1,int *value3);
void ACread24(int *value2,int *value4);

#endif // ANALOG

