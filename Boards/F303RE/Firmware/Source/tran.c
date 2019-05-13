/**************************************************************************

 SLab v2
 File: transient.c

 Transient module source code

 Register names can be found in:
 ChibiOS_17.6.0\os\common\ext\CMSIS\ST\STM32F3xx\stm32f303xe.h

 We will use timer 2 for sample sync
   TRGO

 **************************************************************************/

// Includes
#include "ch.h"
#include "hal.h"
#include "slab.h"
#include "serial.h"
#include "bits.h"
#include "analog.h"
#include "tran.h"

/****************** External references ****************/

extern volatile int halt;

/************ Timer 3 configuration ********************/

static void asyncReadCallback(GPTDriver *gptp);
static const GPTConfig asyncCFG = {
  TIM_FREQ,             /* Timer 3 clock frequency */
  asyncReadCallback,    /* Callback */
  };
//static const GPTConfig asyncCFG_L = {
//  TIM_FREQL,            /* Timer 3 clock frequency */
//  asyncReadCallback,    /* Callback */
//  };

static void stepResponseCallback(GPTDriver *gptp);
static const GPTConfig stepCFG = {
  TIM_FREQ,                /* Timer 3 clock frequency */
  stepResponseCallback,    /* Callback */
  };
//static const GPTConfig stepCFG_L = {
//  TIM_FREQL,               /* Timer 3 clock frequency */
//  stepResponseCallback,    /* Callback */
//  };

static void triggeredReadCallback(GPTDriver *gptp);
static const GPTConfig trigCFG = {
  TIM_FREQ,                 /* Timer 3 clock frequency */
  triggeredReadCallback,    /* Callback */
  };
//static const GPTConfig trigCFG_L = {
//  TIM_FREQL,                /* Timer 3 clock frequency */
//  triggeredReadCallback,    /* Callback */
//  };

static void waveResponseCallback(GPTDriver *gptp);
static const GPTConfig waveCFG = {
  TIM_FREQ,                 /* Timer 3 clock frequency */
  waveResponseCallback,     /* Callback */
  };
//static const GPTConfig waveCFG_L = {
//  TIM_FREQL,                /* Timer 3 clock frequency */
//  waveResponseCallback,     /* Callback */
//  };

static void dualWaveResponseCallback(GPTDriver *gptp);
static const GPTConfig dualWaveCFG = {
  TIM_FREQ,                     /* Timer 3 clock frequency */
  dualWaveResponseCallback,     /* Callback */
  };
//static const GPTConfig dualWaveCFG_L = {
//  TIM_FREQL,                    /* Timer 3 clock frequency */
//  dualWaveResponseCallback,     /* Callback */
//  };

static void singleWaveResponseCallback(GPTDriver *gptp);
static const GPTConfig singleWaveCFG = {
  TIM_FREQ,                     /* Timer 3 clock frequency */
  singleWaveResponseCallback,   /* Callback */
  };
//static const GPTConfig dualWaveCFG_L = {
//  TIM_FREQL,                    /* Timer 3 clock frequency */
//  dualWaveResponseCallback,     /* Callback */
//  };

static void wavePlayCallback(GPTDriver *gptp);
static const GPTConfig wavePlayCFG = {
  TIM_FREQ,             /* Timer 3 clock frequency */
  wavePlayCallback,     /* Callback */
  };
//static const GPTConfig wavePlayCFG_L = {
//  TIM_FREQL,            /* Timer 3 clock frequency */
//  wavePlayCallback,     /* Callback */
//  };

static void dualWavePlayCallback(GPTDriver *gptp);
static const GPTConfig dualWavePlayCFG = {
  TIM_FREQ,                 /* Timer 3 clock frequency */
  dualWavePlayCallback,     /* Callback */
  };
//static const GPTConfig dualWavePlayCFG_L = {
//  TIM_FREQL,                /* Timer 3 clock frequency */
//  dualWavePlayCallback,     /* Callback */
//  };

/************** Module variables **********************/

// Sample time period defaults to 1ms
float stime = DEFAULT_STIME;

// Unified memory buffer
uint16_t buff[BSIZE];

// Start of all buffer sections
uint16_t *wave2buff = NULL;
uint16_t *waveDbuff = NULL;
uint16_t *tranBuff  = NULL;

// Input configuration
int n_ai = 1;     // Number of analog inputs
int n_di = 0;     // Number of digital inputs (always zero)
int n_s  = 1000;  // Number of samples

int w_s = 0;                      // Wavetable size (in samples)
volatile unsigned int w_n = 10;   // Number of cycles before measurement
volatile int w_pos = 0;           // Current wave position

int w_s2 = 0;            // Secondary wavetable size (in samples)
volatile int w_pos2 = 0; // Current secondary wave position

int w_d = 0;             // Digital wave size (in samples)
volatile int w_posd = 0; // Current digital wave position
int waved_base;          // Base of wave D
int waved_mask;          // Mask for waveD

int infiniteWave = 0;   // Flag for infinite wave play

// Sample information for ticker
int samples = 0;    // Number of processed samples
int inBuffPos = 0;  // Current buffer position

volatile int presamples;     // Number of samples before trigger
volatile int postsamples;    // Number of samples after trigger
volatile int triggerSample;  // Sample number at trigger point
volatile int samplePhase;    // Sample phase
int currentBsize;   // Current buffer size
int trigger;        // Trigger value
int triggerMode;    // Trigger mode (0 Rise 1 Fall)
int stepValue;      // Value for step analysis

int checkTimeOut;   // Indicates if we check timeout
uint32_t timeOut;   // Timeout for triggered capture

// Global to communicate with ISR ticker
volatile int endTicker = 0;

// Global to check overruns
volatile int overrun = 0;
volatile int overrun_error = 0;
volatile int timeout_error = 0;

// Channel for single wave response
int channelAC;

/************ Private functions ************************/

// Calculates available transize
static inline uint16_t tranBuffSize(void)
 {
 uint16_t size;

 size = BSIZE - w_s - w_s2;
 return size;
 }

// Calculates available wave 2 buff size
static inline uint16_t wave2buffSize(void)
 {
 uint16_t size;

 size = BSIZE - w_s;
 return size;
 }

// Calculates available digital buff size
static inline uint16_t waveDbuffSize(void)
 {
 uint16_t size;

 size = BSIZE - w_s - w_s2;
 return size;
 }

// Store analog inputs in circular buffer
// Checked with hardware profiling that it is inlined
static inline int storeAnalog(void)
 {
 int a1,a2,a3,a4;

 if (inBuffPos == currentBsize) inBuffPos = 0;

 ADC1->CR |= ADC_CR_ADSTART;
 ADC3->CR |= ADC_CR_ADSTART;
 while (!(ADC3->ISR & ADC_ISR_EOC));

 a4 = (ADC1->DR)<<4;
 a2 = (ADC2->DR)<<4;
 a1 = (ADC3->DR)<<4;
 a3 = (ADC4->DR)<<4;

 if (n_ai >= 1) tranBuff[inBuffPos++] = a1;
 if (n_ai >= 2) tranBuff[inBuffPos++] = a2;
 if (n_ai >= 3) tranBuff[inBuffPos++] = a3;
 if (n_ai >= 4) tranBuff[inBuffPos++] = a4;

 return a1;
 }

// Store digital inputs in circular buffer
static inline void storeDigital(void)
 {
 tranBuff[inBuffPos++] = DIO_PORT->IDR;
 }

// Store analog inputs in circular buffer
// Checked with hardware profiling that it is inlined
/*
static inline int storeAnalogAtPos(int pos)
 {
 int a1,a2,a3,a4;

 ADC1->CR |= ADC_CR_ADSTART;
 ADC3->CR |= ADC_CR_ADSTART;
 while (!(ADC3->ISR & ADC_ISR_EOC));

 a1 = (ADC3->DR)<<4;
 a2 = (ADC2->DR)<<4;
 a4 = (ADC1->DR)<<4;
 a3 = (ADC4->DR)<<4;

 pos = pos*n_ai;

 if (n_ai >= 1) tranBuff[pos  ] = a1;
 if (n_ai >= 2) tranBuff[pos+1] = a2;
 if (n_ai >= 3) tranBuff[pos+2] = a3;
 if (n_ai >= 4) tranBuff[pos+3] = a4;

 return a1;
 }
 */

// Start ADCs
// Used with storeADCs
static inline void runADCs(void)
 {
 ADC1->CR |= ADC_CR_ADSTART;
 ADC3->CR |= ADC_CR_ADSTART;
 }

// ADC Read
// Used with runADCs
static inline int storeADCs(void)
 {
 int a1,a2,a3,a4;

 a1 = (ADC3->DR)<<4;
 a2 = (ADC2->DR)<<4;
 a4 = (ADC1->DR)<<4;
 a3 = (ADC4->DR)<<4;

 if (n_ai >= 1) tranBuff[inBuffPos++] = a1;
 if (n_ai >= 2) tranBuff[inBuffPos++] = a2;
 if (n_ai >= 3) tranBuff[inBuffPos++] = a3;
 if (n_ai >= 4) tranBuff[inBuffPos++] = a4;

 if (inBuffPos == currentBsize) inBuffPos = 0;

 return a1;
 }

// Store a single ADC read
static inline int storeSingleADC(int line)
 {
 int a=0;

 switch(line)
	{
    case 4: // ADC1 channels
    	ADC1->CR |= ADC_CR_ADSTART;
    	while (!(ADC1->ISR & ADC_ISR_EOC));
    	a = (ADC1->DR)<<4;
    	break;
    case 2: // ADC2 channels
    case 5:
    case 6:
    	ADC2->CR |= ADC_CR_ADSTART;
    	while (!(ADC2->ISR & ADC_ISR_EOC));
    	a = (ADC2->DR)<<4;
    	break;
    case 1: // ADC3 channels
    case 7:
    	ADC3->CR |= ADC_CR_ADSTART;
    	while (!(ADC3->ISR & ADC_ISR_EOC));
    	a = (ADC3->DR)<<4;
    	break;
    case 3: // ADC4 channel
    case 8:
    	ADC4->CR |= ADC_CR_ADSTART;
    	while (!(ADC4->ISR & ADC_ISR_EOC));
    	a = (ADC4->DR)<<4;
    	break;
	}

 tranBuff[inBuffPos++] = a;
 return a;
 }

// Dumps the input buffer on serial
void dumpInBuffer(void)
  {
  int ia,is,ssize;

  // Response code
  if (halt)
        {
        sendByte(TRAN_HALT);
        return;
        }

  if (overrun_error)
        {
        sendByte(TRAN_OVERRUN);
        return;
        }
        else
        sendByte(TRAN_OK);

  sendByte(n_ai);  // Number of analog
  sendByte(n_di);  // Number of digital
  sendU16(n_s);  // Number of samples

  // Determine space used by each sample
  if (n_di)
      ssize = n_ai+1;
     else
      ssize = n_ai;

  // Send analog data
  if (n_ai)
    for(ia=0;ia<n_ai;ia++)                // For every analog input
       for(is=0;is<n_s;is++)              // For every sample
       sendU16(tranBuff[is*ssize+ia]);     // Send it

  // Send digital data
  if (n_di)
	  for(is=0;is<n_s;is++)              // For every sample
		  sendU16((tranBuff[is*ssize+n_ai])&(DIO_MASK));
  }

// Dumps the input buffer on serial
// Overrides the number of analog channels and sets it to 1
void dumpInSingleBuffer(void)
  {
  int is;

  PRO2_SET

  // Response code
  if (halt)
        {
        sendByte(TRAN_HALT);
        return;
        }

  if (overrun_error)
        {
        sendByte(TRAN_OVERRUN);
        return;
        }
        else
        sendByte(TRAN_OK);

  sendByte(1);  // Number of analog is 1
  sendByte(n_di);  // Number of digital (always zero)
  sendU16(n_s);  // Number of samples

  for(is=0;is<n_s;is++)         // For every sample
       sendU16(tranBuff[is]);     // Send it
  }

/****************** Public general functions *******************/

void tranInit(void)
 {
 // Sample time period defaults to 1ms
 stime = DEFAULT_STIME;

 // Input configuration
 n_ai = 1;     // Number of analog inputs
 n_di = 0;     // Number of digital inputs (always zero)
 n_s  = 1000;  // Number of samples

 // Eliminate wavetables
 w_s =  0;
 w_s2 = 0;

 // Initialize unified memory
 wave2buff = buff;
 tranBuff  = buff;
 waveDbuff = buff;

 // Wave play not infinite
 infiniteWave = 0;
 }


// Implements command 'R'
// Sets the sample period time
void setSampleTime(void)
 {
 //Get sample time
 stime = getFloat();

 // End of message, check CRC
 if (!crcResponse()) return;

 // Check limits
 if ((stime < MIN_STIME) || (stime > MAX_STIME))
      sendByte(NACK);
      else
      sendByte(ACK);

 // End of message
 sendCRC();
 }

// Implements command 'S'
// Configure storage
void setStorage(void)
 {
 int sample_size,size;
 int error = 0;

 // Get number of analog inputs
 n_ai = getByte();
 if (n_ai > 4) error = 1;

 // Get number of digital inputs
 n_di = getByte();
 if (n_di >= NDIO) error = 1;

 // Get the number of samples
 n_s = getU16();

 // End of message, check CRC
 if (!crcResponse()) return;

 // Check if it fits the buffer
 if (n_di)
    sample_size = n_ai+1;
    else
    sample_size = n_ai;

 size = n_s*sample_size;
 if (size > tranBuffSize()) error = 1;

 // Response depending on errors
 if (error)
      sendByte(NACK);
      else
      sendByte(ACK);

 // End of message
 sendCRC();
 }

/********************* ASYNC READ ***************************/

// Hardware profiling operation (if enabled)
//   PRO1 line high during ISR
//   PRO2 line mimics overrun variable
// Currently, minimum sample time is 25us (40kHz)


static void asyncReadCallback(GPTDriver *gptp)
 {
 (void)gptp; // So that the compiler don't complain about gptp usage

 // Store analog data (Non pipelined mode)
 if (n_ai) storeAnalog();
 // Store analog data (Pipelined mode)
 //if (samples) storeADCs();
 //runADCs();

 // Store digital data
 if (n_di) storeDigital();

 // Increase sample
 samples++;

 // Check if we should end
 if (samples >= n_s)
    {
    // Signal end
    endTicker = 1;
    return;
    }

 // Check for halt
 if (halt)
    {
    // Signal end
    endTicker = 1;
    //PRO1_CLEAR // Profiling
    return;
    }

 // Check for overrun
 if (overrun)
    {
	overrun_error = 1;
    }

 overrun = 1;

 }


// Implements command 'Y'
// Async read
 // This command don't get any parameter
void asyncRead(void)
 {
 int ticks; // Ticks for timer

 // Check of CRC
 if (!crcResponse()) return;

 // Send ACK to command
 sendByte(ACK);

 // Enable AC operation
 ACenable();

 // Configure ticker ISR
 samples = 0;    // Number of processed samples
 inBuffPos = 0;  // Current buffer position
 endTicker = 0;  // Ticker has not ended

 // Current size for buffer
 if (n_di)
	 currentBsize = (n_ai+1) * n_s;
   else
     currentBsize = n_ai * n_s;

 // Clear overrun variables
 overrun_error = 0;
 overrun = 0;

 // Programs the timer
 gptStart(&GPTD3, &asyncCFG);
 ticks = (int) TIM_FREQ_F*stime;
 gptStartContinuous(&GPTD3,ticks);

 // Wait till end
 while (!endTicker)
          {
	      overrun = 0;
	      }

 // Disable gpt
 gptStopTimer(&GPTD3);
 gptStop(&GPTD3);

 // Pipelined mode last sample
 //storeADCs();

 // Disable AC operation
 ACdisable();

 // Return data
 dumpInBuffer();  // Dump data

 sendCRC(); // End of Tx
 }

/********************* TRIGGERED READ ***************************/

// Dumps the input buffer on serial for triggered capture
void dumpTriggeredInBuffer(void)
  {
  int ia,is,pos,sample,first,pre,ssize;

  // Response code
  if (halt)
        {
        sendByte(TRAN_HALT);
        return;
        }

  if (overrun_error)
        {
        sendByte(TRAN_OVERRUN);
        return;
        }

  if (timeout_error)
        {
        sendByte(TRAN_TIMEOUT);
        return;
        }

  sendByte(TRAN_OK);

  sendByte(n_ai);  // Number of analog
  sendByte(n_di);  // Number of digital (always zero)
  sendU16(n_s);    // Number of samples

  pre = n_s/2;               // Number of samples before trigger

  // Determine space used by each sample
  if (n_di)
      ssize = n_ai+1;
     else
      ssize = n_ai;

  // First sample to send
  first = (n_s + triggerSample - pre + 1)%n_s;

  // Send analog data
  if (n_ai)
    for(ia=0;ia<n_ai;ia++)                  // For every analog input
       for(is=0;is<n_s;is++)                // For every sample
          {
          sample = (first+is)%n_s;          // Calculate sample
          pos = sample*ssize+ia;            // Calculate buff position
          sendU16(tranBuff[pos]);           // Send it
          }

  // Send digital data
  if (n_di)
	  for(is=0;is<n_s;is++)                // For every sample
	  {
	  sample = (first+is)%n_s;          // Calculate sample
	  pos = sample*ssize+n_ai;          // Calculate buff position
	  sendU16((tranBuff[pos])&(DIO_MASK));
	  }
  }

// ISR for the triggeredRead function
void triggeredReadCallback(GPTDriver *gptp)
 {
(void)gptp; // So that the compiler don't complain about gptp usage
 int a1;

 // Store analog data
 a1 = storeAnalog();

 // Store digital data
 if (n_di) storeDigital();

 // Increase sample
 samples++;
 if (samples == n_s) samples = 0;

 // Decrease timeout
 timeOut--;

 // Check halt
 if (halt)
    {
    // Signal end
    endTicker = 1;
    return;
    }

 // Check phase
 switch(samplePhase)
    {
    case 0: // Prefill of the buffer
      presamples--;
      if (!presamples) samplePhase = 1;
      if (!timeOut)
        {
        // Set error
        timeout_error = 1;
        // Signal end
        endTicker = 1;
        }
      break;
    case 1: // Wait for trigger precondition
      if (triggerMode == 0) // Rise
         if (a1 < trigger) {samplePhase = 2; }
      if (triggerMode == 1) // Fall
         if (a1 > trigger) {samplePhase = 2; }
      if (!timeOut)
        {
        // Set error
        timeout_error = 1;
        // Signal end
        endTicker = 1;
        }
      break;
    case 2: // Wait for trigger postcondition
      if (triggerMode == 0) // Rise
         if (a1 > trigger)
                     {
                     samplePhase = 3;
                     triggerSample = samples;
                     PRO1_SET
                     }
      if (triggerMode == 1) // Fall
         if (a1 < trigger)
                     {
                     samplePhase = 3;
                     triggerSample = samples;
                     PRO1_SET
                     }
      if (!timeOut)
        {
        // Set error
        timeout_error = 1;
        // Signal end
        endTicker = 1;
        }
      break;
    case 3: // Capture after trigger
      if (!postsamples)
            {
            // Signal end
            endTicker = 1;
            PRO1_CLEAR
            }
      postsamples--;
      break;
    }

 // Check for overrun
 if (overrun)
    overrun_error = 1;

 overrun = 1;
 }

// Implements command 'G'
// Triggered read
void triggeredRead(void)
 {
 int ticks;

 PRO1_CLEAR
 PRO1_CLEAR

 // Get trigger point
 trigger = getU16();
 // Get trigger mode
 triggerMode = getByte();
 // Get timeout in seconds
 timeOut = getByte();

 if (timeOut)
    {
    checkTimeOut=1;
    // Convert to samples
    timeOut=(int)(1.0*timeOut/stime);
    }
    else
    checkTimeOut = 0;

 // Erase timeout error
 timeout_error = 0;

 // Check of CRC
 if (!crcResponse()) return;

 // Check mode
 if ( (triggerMode != 0) && (triggerMode != 1) )
    {
    sendByte(NACK);
    sendCRC();
    return;
    }

 // All ok
 sendByte(ACK);

 // Enable AC operation
 ACenable();

 // Configure ticker ISR
 samples = 0;    // Number of processed samples
 inBuffPos = 0;  // Current buffer position
 endTicker = 0;  // Ticker has not ended

 presamples = n_s/2;               // Number of samples before trigger
 postsamples = n_s - presamples;   // Number of samples after trigger

 // Current size for buffer
 if (n_di)
	 currentBsize = (n_ai+1) * n_s;
   else
     currentBsize = n_ai * n_s;

 samplePhase = 0; // First phase: buffer prefill

 // Clear overrun variables
 overrun_error = 0;
 overrun = 0;

 // Programs the timer
 gptStart(&GPTD3, &trigCFG);
 ticks = (int) TIM_FREQ_F*stime;
 gptStartContinuous(&GPTD3,ticks);

 // Wait till end
 while (!endTicker) { overrun = 0; }

 // Disable gpt
 gptStopTimer(&GPTD3);
 gptStop(&GPTD3);

 // Disable AC operation
 ACdisable();

 // Return data
 dumpTriggeredInBuffer();  // Dump data

 // Send CRC to end Tx
 sendCRC();
 }

/********************* STEP RESPONSE ***************************/

// Hardware profiling operation (if enabled)
//   Not implemented yet

// Callback for the stepResponse function
void stepResponseCallback(GPTDriver *gptp)
 {
 (void)gptp; // So that the compiler don't complain about gptp usage

 // Store analog data (Non pipelined mode)
 if (n_ai) storeAnalog();

 // Store digital data
 if (n_di) storeDigital();

 // Increase sample
 samples++;

 // Check trigger position
 if (samples == triggerSample)
       DAC1_WRITE(stepValue);

 // Check halt
 if (halt)
    {
    // Signal end
    endTicker = 1;
    }

 // Check if we should end
 if (samples >= n_s)
    {
    // Signal end
    endTicker = 1;
    }

 // Check for overrun
 if (overrun)
    overrun_error = 1;

 overrun = 1;
 }

// Implements command 'P'
// Step response
void stepResponse(void)
 {
 int ticks; // Ticks for timer

 // Read step value
 stepValue = getU16();

 // Check of CRC
 if (!crcResponse()) return;

 sendByte(ACK); // All Ok

 // Enable AC operation
 ACenable();

 // Configure ticker ISR
 samples = 0;               // Number of processed samples
 inBuffPos = 0;             // Current buffer position
 endTicker = 0;            // Ticker has not ended

 triggerSample = n_s/5;

 // Current size for buffer
 if (n_di)
	 currentBsize = (n_ai+1) * n_s;
   else
     currentBsize = n_ai * n_s;

 // Clear overrun variables
 overrun_error = 0;
 overrun = 0;

 // Programs the ticker
 gptStart(&GPTD3, &stepCFG);
 ticks = (int) TIM_FREQ_F*stime;
 gptStartContinuous(&GPTD3,ticks);

 // Wait till end
 while (!endTicker) overrun = 0;

 // Disable gpt
 gptStopTimer(&GPTD3);
 gptStop(&GPTD3);

 // Disable AC operation
 ACdisable();

 // Return data
 dumpInBuffer();  // Dump data

 // Send CRC to end Tx
 sendCRC();
 }

/********************* WAVETABLE CODE ***************************/

// Load a wavetable
void loadWaveTable(void)
 {
 int i;

 // Eliminate secondary and digital wavetables
 w_s2 = 0;
 w_d  = 0;

 // Get size
 w_s = getU16();

 // Check size
 if (w_s > BSIZE)
    {
    w_s = 0; // Eliminate table

    // Calculate new memory configuration
    wave2buff=&buff[w_s];
    waveDbuff=&buff[w_s];
    tranBuff =&buff[w_s];

    sendByte(NACK);
    sendCRC();
    return;
    }

 // Calculate new memory configuration
 wave2buff=&buff[w_s];
 waveDbuff=&buff[w_s];
 tranBuff =&buff[w_s];

 if (w_s > 0)
   {
   // Load samples
   for(i=0;i<w_s;i++)
      buff[i] = getU16();
   }

 // Check of CRC
 if (!crcResponse()) return;

 sendByte(ACK);

 sendCRC();
 }

// Load a secondary wavetable
void loadSecondaryWaveTable(void)
 {
 int i;

 // Eliminate digital wavetable
 w_d  = 0;

 // Get size
 w_s2 = getU16();

 // Check size and primary wavetable
 if (w_s2 > wave2buffSize())
    {
    w_s2 = 0;
    // Calculate new memory configuration
    waveDbuff=&buff[w_s+w_s2];
    tranBuff =&buff[w_s+w_s2];

    sendByte(NACK);
    sendCRC();
    return;
    }

 // Calculate new memory configuration
 waveDbuff=&buff[w_s+w_s2];
 tranBuff =&buff[w_s+w_s2];

 if (w_s2>0)
    for(i=0;i<w_s2;i++)
        wave2buff[i] = getU16();

 // Check of CRC
 if (!crcResponse()) return;

 sendByte(ACK);

 sendCRC();
 }

// Load the digital wavetable
void loadDigitalTable(void)
 {
 int i;

 // Get size
 w_d = getU16();

 // Get mask
 waved_mask = getU16();

 // Set mask
 if (waved_mask)
	 waved_mask = waved_mask & DIO_MASK;
    else
     waved_mask = DIO_MASK;

 // Check size and primary wavetable
 if (w_d > waveDbuffSize())
    {
	w_d = 0;
    // Calculate new memory configuration
    tranBuff = &buff[w_s+w_s2+w_d];

    sendByte(NACK);
    sendCRC();
    return;
    }

 // Calculate new memory configuration
 tranBuff = &buff[w_s+w_s2+w_d];

 if (w_d > 0)
    for(i=0;i<w_d;i++)
       waveDbuff[i] = waved_mask & getU16();

 // Check of CRC
 if (!crcResponse()) return;

 sendByte(ACK);

 sendCRC();
 }

/****************** WAVE RESPONSE CODE *************************/

// ISR for the waveResponse function
void waveResponseCallback(GPTDriver *gptp)
 {
 (void)gptp; // So that the compiler don't complain about gptp usage

 // Write DAC if enabled
 if (w_s)
	 DAC1_WRITE(buff[w_pos++]);
 // Write digital if enabled
 if (w_d)
	 (DIO_PORT->ODR) = (waveDbuff[w_posd++])|waved_base;

 // Store analog data
 if (!w_n)
      {
	  // Store analog data (Non pipelined mode)
	  if (n_ai) storeAnalog();

	  // Store digital data
	  if (n_di) storeDigital();

      // Increase sample
      samples++;

      // Check if we should end
      if (samples >= n_s)
           {
           // Signal end
           endTicker = 1;
           }
      }
     else
      {
      // Decrease counter
      w_n--;
      }

 // Check wave rollover
 if (w_pos == w_s)  w_pos = 0;
 if (w_posd == w_d) w_posd = 0;

 // Check halt
 if (halt)
    {
    // Signal end
    endTicker = 1;
    return;
    }

 // Check for overrun
 if (overrun)
    overrun_error = 1;

 overrun = 1;
 }

// Wave response
void waveResponse()
 {
 int ticks;

 // Read number of waves before measurement
 w_n = getU16();

 if (w_s)
     {
	 // There is a primary wave
	 // Convert to cycles
	 w_n = w_n*w_s;
     }
    else
     {
     // There is no primary wave
     // Use digital wave if present
	 w_n = w_n*w_d;
     }



 // Check of CRC
 if (!crcResponse()) return;

 sendByte(ACK);

 // Enable AC operation
 ACenable();

 // Configure ticker ISR
 samples = 0;               // Number of processed samples
 inBuffPos = 0;             // Current buffer position
 endTicker = 0;             // Ticker has not ended
 w_pos = 0;                 // Current wave position
 w_posd = 0;				// Current digital wave position

 // Current size for buffer
 if (n_di)
	 currentBsize = (n_ai+1) * n_s;
   else
     currentBsize = n_ai * n_s;

 // Clear overrun variables
 overrun_error = 0;
 overrun = 0;

 // Store digital wave base
 waved_base = (DIO_PORT -> ODR)&(~waved_mask);

 // Programs the ticker
 gptStart(&GPTD3, &waveCFG);
 ticks = (int) TIM_FREQ_F*stime;
 gptStartContinuous(&GPTD3,ticks);

 // Wait till end
 while (!endTicker) overrun = 0;

 // Disable gpt
 gptStopTimer(&GPTD3);
 gptStop(&GPTD3);

 // Disable AC operation
 ACdisable();

 // Return data
 dumpInBuffer();  // Dump data

 // End sending CRC
 sendCRC();
 }

/*************** DUAL WAVE RESPONSE CODE *************************/

// ISR for the dualWaveResponse function
void dualWaveResponseCallback(GPTDriver *gptp)
 {
 (void)gptp; // So that the compiler don't complain about gptp usage

 // Write DACs
 DAC1_WRITE(buff[w_pos++]);
 DAC2_WRITE(wave2buff[w_pos2++]);

 // Write digital if enabled
 if (w_d)
 	 (DIO_PORT->ODR) = (waveDbuff[w_posd++])|waved_base;

 // Check if we are in store stage
 if (!w_n)
      {
	  // Store analog data (Non pipelined mode)
	  if (n_ai) storeAnalog();

	  // Store digital data
	  if (n_di) storeDigital();

      // Increase sample
      samples++;

      // Check if we should end
      if (samples >= n_s)
           {
           // Signal end
           endTicker = 1;
           }
      }
     else
      {
      // Decrease counter
      w_n--;
      }

 // Check wave rollover
 if (w_pos == w_s)   w_pos = 0;
 if (w_pos2 == w_s2) w_pos2 = 0;
 if (w_posd == w_d) w_posd = 0;

 // Check halt
 if (halt)
    {
    // Signal end
    endTicker = 1;
    return;
    }

 // Check for overrun
 if (overrun)
    overrun_error = 1;

 overrun = 1;
 }

// Dual wave response
void dualWaveResponse()
 {
 int ticks;

 // Read number of primary waves before measurement
 w_n = getU16();
 // Convert to cycles
 w_n = w_n*w_s;

 // Make zero if no primary wave
 if (!w_s) w_n = 0;

 // Check of CRC
 if (!crcResponse()) return;

 sendByte(ACK);

 // Enable AC operation
 ACenable();

 // Configure ticker ISR
 samples = 0;               // Number of processed samples
 inBuffPos = 0;             // Current buffer position
 endTicker = 0;             // Ticker has not ended
 w_pos = 0;                 // Current wave position
 w_pos2 = 0;                // Secondary wave position
 w_posd = 0;				// Current digital wave position

 // Current size for buffer
 if (n_di)
	 currentBsize = (n_ai+1) * n_s;
   else
     currentBsize = n_ai * n_s;

 // Clear overrun variables
 overrun_error = 0;
 overrun = 0;

 // Store digital wave base
 waved_base = (DIO_PORT -> ODR)&(~waved_mask);

 // Programs the ticker
 gptStart(&GPTD3, &dualWaveCFG);
 ticks = (int) TIM_FREQ_F*stime;
 gptStartContinuous(&GPTD3,ticks);

 // Wait till end
 while (!endTicker) overrun = 0;

 // Disable gpt
 gptStopTimer(&GPTD3);
 gptStop(&GPTD3);

 // Disable AC operation
 ACdisable();

 // Return data
 dumpInBuffer();  // Dump data

 // End sending CRC
 sendCRC();
 }


/*************** SINGLE WAVE RESPONSE CODE **********************/
// Hardware profiling operation (if enabled)
//   PRO1 line high during ISR
//   PRO2 line mimics overrun variable

// Callback for the singleWaveResponse function
void singleWaveResponseCallback(GPTDriver *gptp)
 {
 (void)gptp;

 // Write DAC1
 DAC1_WRITE(buff[w_pos++]);

 // Check if we are in store stage
 if (!w_n)
      {
      // Store data
      //a1 = ReadAnalogAC(channelAC);
      //tranBuff[inBuffPos++]=a1;
	  storeSingleADC(channelAC);

      // Increase sample
      samples++;

      // Check if we should end
      if (samples >= n_s)
           {
           // Signal end
           endTicker = 1;
           }

      // Check wave rollover
      if (w_pos == w_s) w_pos = 0;
      }
    else
      // We are in waves before storing
      {
      // Decrease counter
      w_n--;
      }

 // Check wave rollover
 if (w_pos == w_s) w_pos = 0;

 // Check for overrun
 if (overrun)
    overrun_error = 1;

 // Check halt
 if (halt)
    {
    // Signal end
    endTicker = 1;
    return;
    }

 overrun = 1;
 }

// Single Wave response
void singleWaveResponse(void)
 {
 int ticks;

 // Read channel to read
 channelAC = getByte();

 // Read number of waves before measurement
 w_n = getU16();
 // Make zero if no primary waves
 if (!w_s) w_n = 0;

 // Convert to cycles
 w_n = w_n*w_s;

 // Check of CRC
 if (!crcResponse()) return;

 // Send ACK
 sendByte(ACK);

 // Enable AC for the selected channel
 setADforAC(channelAC);

 // Configure ticker ISR
 samples = 0;               // Number of processed samples
 inBuffPos = 0;             // Current buffer position
 endTicker = 0;             // Ticker has not ended
 w_pos = 0;                 // Current wave position

 currentBsize = n_s;        // Current size for buffer

 // Clear overrun variables
 overrun_error = 0;
 overrun = 0;

 // Programs the ticker
 gptStart(&GPTD3, &singleWaveCFG);
 ticks = (int) TIM_FREQ_F*stime;
 gptStartContinuous(&GPTD3,ticks);

 // Wait till end
 while (!endTicker) overrun = 0;

 // Disable gpt
 gptStopTimer(&GPTD3);
 gptStop(&GPTD3);

 // Disable AC operation
 ACdisable();

 // Return data
 dumpInSingleBuffer();  // Dump data

 // End sending CRC
 sendCRC();
 }

/****************** WAVE PLAY CODE *************************/

// ISR for the wavePlay function
void wavePlayCallback(GPTDriver *gptp)
 {
 (void)gptp; // So that the compiler don't complain about gptp usage

 // Write DAC if enabled
 if (w_s)
	 DAC1_WRITE(buff[w_pos++]);
 // Write digital if enabled
 if (w_d)
	 (DIO_PORT->ODR) = (waveDbuff[w_posd++])|waved_base;

 // Check infinite wave
 if (!infiniteWave)
 	{
	if (!w_n)
	    {
	    // Signal end
		endTicker = 1;
		return;
		}
	   else
	    {
		w_n--;
		if (!w_n)
		    {
			// Signal end
			endTicker = 1;
		    return;
		    }
	    }
 	}

 // Check for wave rollover
 if (w_pos == w_s) w_pos = 0;
 if (w_posd == w_d) w_posd = 0;

 // Check for halt
 if (halt)
      {
      // Signal end
      endTicker = 1;
      return;
      }

 // Check for overrun
 if (overrun)
    overrun_error = 1;

 overrun = 1;
 }

// Wave Play
void wavePlay()
 {
 int ticks;

 // Read number of waves to send
 infiniteWave = 0;
 w_n = getU16();
 if (w_n==0)
       infiniteWave = 1;

 if (w_s)
     {
	 // There is a primary wave
	 // Convert to cycles
	 w_n = w_n*w_s;
     }
    else
     {
     // There is no primary wave
     // Use digital wave if present
	 w_n = w_n*w_d;
     }

 // Check of CRC
 if (!crcResponse()) return;

 sendByte(ACK);

 // Enable AC operation
 ACenable();

 // Configure ticker ISR
 endTicker = 0;             // Ticker has not ended
 w_pos = 0;                 // Current wave position
 w_posd = 0;				// Current digital wave position

 // Clear overrun variables
 overrun_error = 0;
 overrun = 0;

 // Programs the ticker
 gptStart(&GPTD3, &wavePlayCFG);
 ticks = (int) TIM_FREQ_F*stime;
 gptStartContinuous(&GPTD3,ticks);

 // Wait till end
 while (!endTicker) overrun = 0;

 // Disable gpt
 gptStopTimer(&GPTD3);
 gptStop(&GPTD3);

 // Disable AC operation
 ACdisable();

 // Response code
 if (halt)
    sendByte(TRAN_HALT);
    else
     if (overrun_error)
        sendByte(TRAN_OVERRUN);
        else
        sendByte(TRAN_OK);

 // End sending CRC
 sendCRC();
 }

/****************** DUAL WAVE PLAY CODE *************************/

// ISR for the dualWavePlay function
void dualWavePlayCallback(GPTDriver *gptp)
 {
 (void)gptp; // So that the compiler don't complain about gptp usage

 // Write DACs (New faster code)
 DAC1_WRITE(buff[w_pos++]);
 DAC2_WRITE(wave2buff[w_pos2++]);

 // Write digital if enabled
 if (w_d)
	 (DIO_PORT->ODR) = (waveDbuff[w_posd++])|waved_base;

 // Check infinite wave
 if (!infiniteWave)
 	{
	if (!w_n)
	    {
	    // Signal end
		endTicker = 1;
		return;
		}
	   else
	    {
		w_n--;
		if (!w_n)
		    {
			// Signal end
			endTicker = 1;
		    return;
		    }
	    }
 	}

 // Check primary wave rollover
 if (w_pos == w_s) w_pos = 0;

 // Check for secondary wave rollover
 if (w_pos2 == w_s2) w_pos2 = 0;

 // Check for digital wave rollover
 if (w_posd == w_d) w_posd = 0;

 // Check for halt
 if (halt)
      {
      // Signal end
      endTicker = 1;
      return;
      }

 // Check for overrun
 if (overrun)
    overrun_error = 1;

 overrun = 1;
 }

// Dual Wave Play
void dualWavePlay()
 {
 int ticks;

 // Read number of waves to send
 infiniteWave = 0;
 w_n = getU16();
 if (w_n==0)
       infiniteWave = 1;

 // Convert to cycles
 w_n = w_n*w_s;

 // Check of CRC
 if (!crcResponse()) return;

 sendByte(ACK);

 // Enable AC operation
 ACenable();

 // Configure ticker ISR
 endTicker = 0;             // Ticker has not ended
 w_pos =  0;                // Current primary wave position
 w_pos2 = 0;                // Current secondary wave position
 w_posd = 0;				// Current digital wave position

 // Clear overrun variables
 overrun_error = 0;
 overrun = 0;

 // Programs the ticker
 gptStart(&GPTD3, &dualWavePlayCFG);
 ticks = (int) TIM_FREQ_F*stime;
 gptStartContinuous(&GPTD3,ticks);

 // Wait till end
 while (!endTicker) overrun = 0;

 // Disable gpt
 gptStopTimer(&GPTD3);
 gptStop(&GPTD3);

 // Disable AC operation
 ACdisable();

 // Response code
 if (halt)
    sendByte(TRAN_HALT);
    else
      if (overrun_error)
        sendByte(TRAN_OVERRUN);
        else
        sendByte(TRAN_OK);

 // End sending CRC
 sendCRC();
 }

