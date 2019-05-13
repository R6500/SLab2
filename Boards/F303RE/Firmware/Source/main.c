/**************************************************************************

 SLab v2
 File: main.c

 Entry point for the program

 ChibiOS/RT/HAL Firmware for Nucleo Boards

 Program to operate a nucleo board from a PC
 in order to perform measurements.

 Designed for the Nucleo64 F303RE Board

 Commands implemented in version 1

 Global

  F     : Obtain a string that describes the firmware
  M     : Obtain 4 byte magic code
  I     : Board capabilities identification
  L     : Pin list
  E     : Soft Reset

 DC

  A + 1 : Read one ADC channel
  D + 3 : Write one DAC channel
  N + 2 : Set number of reads to average


 Transient

  R + 2 : Set sample time
  S + 4 : Set storage configuration
  Y     : Async read
  G + 3 : Triggered read
  P + 2 : Step response

 Wavetable

  W + n : Load a wavetable
  w + n : Load a secondary wavetable
  V + 2 : Wave response
  v + 2 : Dual wave response
  X + 3 : Single Wave Response (Eliminated)
  Q + 2 : Wave Play
  q + 2 : Dual Wave Play

 Digital I/O

  H + 2 : dio mode
  J + 2 : dio write
  K + 1 : dio read


 **************************************************************************/

#include "ch.h"
#include "hal.h"
#include "chprintf.h"       // ChibiOS/RT chprintf function

#include "slab.h"
#include "serial.h"
#include "analog.h"
#include "tran.h"
#include "dio.h"
#include "test.h"

/************* GLOBAL VARIABLES **********************/

// Board reference voltage
float vref = 3.3;

// DC analog read number of readings
int nread = DEFAULT_NREAD;

// Globals for CRC
int crcTx,crcRx;

// Indicate that board status is at reset condition
int resetState=1;

// Halt condition flag (Not implemented yet)
volatile int halt = 0;

/****************** EXT CONFIGURATION **********************/

static void buttonCall(EXTDriver *extp, expchannel_t channel);

static const EXTConfig extcfg = {
  {
	{EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    { EXT_CH_MODE_FALLING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOC, buttonCall},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL}
  }
};

/****************** INIT  FUNCTIONS ************************/

// Initializes used pins
static void initPins(void)
 {
 // Initialize global used pins
 #ifdef USE_PROFILING
 palSetPadMode(PRO1_PORT,PRO1_PIN,PAL_MODE_OUTPUT_PUSHPULL);
 palSetPadMode(PRO2_PORT,PRO2_PIN,PAL_MODE_OUTPUT_PUSHPULL);
 PRO1_CLEAR
 PRO2_CLEAR
 #endif

 // Button pin
 palSetPadMode(BUTTON_PORT,BUTTON_PIN,PAL_MODE_INPUT);
 }

// Initializes the system
void softReset(void)
 {
 // Initialize pins
 initPins();

 // Initialize DIO
 dioInit();

 // Initialize ADCs
 analogInit();

 // DC analog read number of readings
 nread = DEFAULT_NREAD;

 // Read reference voltage
 vref = readVref();

 // Initialize Transient
 tranInit();

 // Initialize EXT Driver for the halt button
 extStart(&EXTD1, &extcfg);
 }

/************* Button Callback *******************************/

static void buttonCall(EXTDriver *extp, expchannel_t channel)
  {
  (void)extp;
  (void)channel;

  halt = 1;
  }

/************* MAIN LOOP FUNCTIONS ***************************/

// Process one character received from the PC
void process(int car)
 {
 int i,value;
 int vref_m,vref_e;

 // Initialize Tx CRC
 startTx();

 switch(car)
    {
    case 'F': // Get firmware string
    	chprintf(SDBASE,"%s%s%s",BSTRING,VSTRING,BREAK);
        break;
    case 'M': // Get magic
        // Check CRC of command. Returns 1 if Ok
        // On error Sends ECRC + CRC and return 0
        if (!crcResponse()) return;
        sendByte(ACK);
        // Send magic
        for(i=0;i<MAGIC_SIZE;i++)
            sendByte(magic[i]);
        // Send CRC
        sendCRC();
        break;
    case 'I': // Get board capabilities
        // Calculate vref mantisa and exponent
        vref_m = (int) (vref*1000.0);
        vref_e = -3;

        // Check CRC of command. Returns 1 if Ok
        // On error Sends ECRC + CRC and return 0
        if (!crcResponse()) return;
        sendByte(ACK);

        sendByte(NDACS);                //  1
        sendByte(NADCS);                //  2
        sendU16(BSIZE);                 //  4 Buffer
        sendMantExp(MAX_S_M,MAX_S_E);   //  7
        sendMantExp(MIN_S_M,MIN_S_E);   // 10
        sendMantExp(vref_m,vref_e);     // 13
        sendMantExp(MAX_SF_M,MAX_SF_E); // 16
        sendMantExp(vref_m,vref_e);     // 29
        sendByte(DAC_BITS);             // 20
        sendByte(ADC_BITS);             // 21
        sendByte(NDIO);                 // 22
        sendByte(resetState);           // 23

        // Send CRC
        sendCRC();
        break;
    case 'L' : // Send pin list
        // Check CRC of command. Returns 1 if Ok
        // On error Sends ECRC + CRC and return 0
        if (!crcResponse()) return;
        sendByte(ACK);

        sendString(PIN_LIST);

        // Send CRC
        sendCRC();
        break;

    case 'A' : // ADC Read
        i = getByte();   // Channel to read
        // Check CRC of command. Returns 1 if Ok
        // On error Sends ECRC + CRC and return 0
        if (!crcResponse()) return;

        // Check that DAC number is correct
        if ((i<1)||(i>NADCS))
            {
            sendByte(NACK);
            sendCRC();
            return;
            }

        value = analogRead(i);

        sendByte(ACK);
        sendU16(value);
        sendCRC();
        break;

    case 'D' : // DAC Write
        i = getByte();          // Channel to write
        value = getU16();       // Read value to set
        // Check CRC of command. Returns 1 if Ok
        // On error Sends ECRC + CRC and return 0
        if (!crcResponse()) return;
        switch(i)
            {
            case 1:
               DAC1_WRITE(value);   // Scale and send
               break;
            case 2:
               DAC2_WRITE(value);   // Scale and send
               break;
            #ifdef EXIST_DAC3
            case 3:
               DAC3_WRITE(value);   // Scale and send
               break;
            #endif
            default:
               sendByte(NACK);
               sendCRC();
               return;
            }
        sendByte(ACK);
        sendCRC();
        resetState=0;  // State change
        break;

        case 'R': // Set sample period time
            setSampleTime();
            resetState=0;  // State change
            break;

        case 'S': // Set Storage
            setStorage();
            resetState=0;  // State change
            break;

        case 'Y': // Async Read
        	asyncRead();
            break;

        case 'G': // Triggered Read
            triggeredRead();
            break;

        case 'P': // Step response
            stepResponse();
            break;

        case 'W': // Load wavetable
            loadWaveTable();
            resetState=0;  // State change
            break;

        case 'w': // Load secondary wavetable
            loadSecondaryWaveTable();
            resetState=0;  // State change
            break;

        case 'V': // Wave response
            waveResponse();
            break;

        case 'v': // Dual wave response
            dualWaveResponse();
            break;

        case 'X': // Single Wave response
            singleWaveResponse();
            break;

        case 'Q': // Wave Play
            wavePlay();
            break;

        case 'q': // Wave Play
            dualWavePlay();
            break;

        case 'H': // DIO mode
            dioMode();
            resetState=0;  // State change
            break;

        case 'J': // DIO Write
            dioWrite();
            resetState=0;  // State change
            break;

        case 'K': // DIO Read
            dioRead();
            break;

        case 'j': // DIO Write All
            dioWriteAll();
            resetState=0;  // State change
            break;

        case 'k': // DIO Read All
            dioReadAll();
            break;

        case 'N': // Number of reads in DC
            value = getU16();             // Read value to set
            if (!crcResponse()) return;   // Check CRC
            if (value==0) value=1;        // At least it shall be one
            nread = value;
            sendByte(ACK);                // Send ACK and CRC
            sendCRC();
            break;

        case 'E': // Soft Reset
            if (!crcResponse()) return;

            softReset();
            resetState=1;  // Return to reset state

            sendByte(ACK);
            sendCRC();
            break;

        case 'O': // Load Digital wavetable
        	loadDigitalTable();
        	resetState=0;  // State change
        	break;

    default:
        // Unknown command
        sendByte(NACK);
        sendCRC();
        break;
    }
 }

// Program entry point
int main(void) {
  int car;

  // Initializations of HAL and OS
  halInit();
  chSysInit();

  // Start the serial module
  serialBegin();

  // Print string
  chprintf(SDBASE,"%s%s%s",BSTRING,VSTRING,BREAK);

  // Initializes the system
  softReset();

  // Test code goes here
  //testGPT();

  // Loop that processes each received char
  while(1)
     {
     startRx();        // Init Rx CRC
     car = getByte();  // Get command
     halt = 0;         // Remove halt condition if present
     process(car);     // Process command
     }
}
