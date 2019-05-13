/**************************************************************************

 SLab v2
 File: serial.c

 Serial module source code

 **************************************************************************/

// Includes
#include "ch.h"
#include "hal.h"
#include "chprintf.h"       // ChibiOS/RT chprintf function
#include "math.h"

#include "slab.h"
#include "serial.h"

/****************** CONSTANTS **************************/

// Magic code to identify a SLab Board
const uint8_t magic[MAGIC_SIZE]={56,41,18,1};

/****************** VARIABLES **************************/

SerialConfig SConfig;  // Serial configuration

/*************** PUBLIC FUNCTIONS **********************/

// Start the serial module
void serialBegin(void)
 {
 // Serial driver initialization
 SConfig.speed=SSPEED;         // Configure speed
 sdStart(&SDRIVER,&SConfig);   // Initializes serial driver
 palSetPadMode(USART2_TX_PORT,USART2_TX_PAD,PAL_MODE_ALTERNATE(7)); // Map TX
 palSetPadMode(USART2_RX_PORT,USART2_RX_PAD,PAL_MODE_ALTERNATE(7)); // Map RX
 }


// TX Code ----------------------------------------------

// Start Tx
// Clears the tx crc
void startTx(void)
 {
 crcTx = 0;
 }

// Send Tx CRC
// Usually that ends transmission
void sendCRC(void)
 {
 sdPut(&SDRIVER,crcTx);
 }

// Send one byte and computes crc
void sendByte(int value)
 {
 sdPut(&SDRIVER,value);
 crcTx = crcTx ^ value;
 }

// Send one uint16 and computes crc
void sendU16(int value)
 {
 int low,high;

 high = value / 256;
 low  = value % 256;

 sendByte(low);
 sendByte(high);
 }

// Send float as mantissa and exponent
void sendMantExp(int mantissa, int exponent)
 {
 sendByte(exponent+128);
 sendU16(mantissa+20000);
 }

// Send one string and computes crc
void sendString(char *str)
 {
 while (*str)
    {
    sendByte(*str);
    str++;
    }
 }

// RX Code ----------------------------------------------

// Start of a Rx reception
void startRx(void)
 {
 crcRx = 0;
 }

// Get CRC and check it
// It usually ends the Rx reception
// Returns 1 if CRC is ok, 0 if not
int getAndCheckCRC(void)
 {
 int crc;

 crc = sdGet(&SDRIVER);
 if (crc != crcRx) return 0;
 return 1;
 }

// Get and check CRC and sends ECRC in case of error
// If no error, don't respond anything
// Returns 1 if CRC is ok, 0 if not
int crcResponse(void)
 {
 // Check if CRC is ok
 if (getAndCheckCRC()) return 1;

 // If CRC is not ok
 sendByte(ECRC);
 // End transmission
 sendCRC();
 return 0;
 }

// Get one byte from the serial stream and computes crc
int getByte(void)
 {
 int byte;

 byte = sdGet(&SDRIVER);
 crcRx = crcRx ^ byte;
 return byte;
 }

// Get one uint16 and computes crc
int getU16(void)
 {
 int low, high, value;

 low  = getByte();
 high = getByte();
 value = (256 * high) + low;

 return value;
 }


// Get one float value and computes crc
float getFloat(void)
 {
 int exp,mant;
 float value;

 exp = getByte() - 128;
 mant = getU16() - 20000;

 value = ((float)mant) * pow((float)10.0,(float)exp);

 return value;
 }


