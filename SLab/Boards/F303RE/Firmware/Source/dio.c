/**************************************************************************

 SLab v2
 File: dio.c

 Digital I/O module source code

 **************************************************************************/

// Includes
#include "ch.h"
#include "hal.h"
#include "slab.h"
#include "serial.h"
#include "bits.h"
#include "dio.h"

/************* PUBLIC FUNCTIONS *********************************/

// Initialize DIO pins
void dioInit(void)
 {
 int i;

 // DIO pins
 for(i=0;i<NDIO;i++)
      {
	  // Set on input mode
      palSetPadMode(DIO_PORT,DIO_PIN0+i,PAL_MODE_INPUT_PULLDOWN);
      // Set to zero
      palWritePad(DIO_PORT,DIO_PIN0+i,PAL_LOW);
      }

 }

// Digital IO mode
void dioMode(void)
 {
 int line,mode,error;

 // Read line to configure
 line = getByte();

 // Read mode to set
 mode = getByte();

 // Check of CRC
 if (!crcResponse()) return;

 // No error for now
 error = 0;

 // Check line number
 if (line >= NDIO) error = 1;

 // Set dio mode
 if (!error)
    switch(mode)
        {
        case 10:
           palSetPadMode(DIO_PORT,DIO_PIN0+line,PAL_MODE_INPUT);
           break;
        case 11:
           palSetPadMode(DIO_PORT,DIO_PIN0+line,PAL_MODE_INPUT_PULLUP);
           break;
        case 12:
           palSetPadMode(DIO_PORT,DIO_PIN0+line,PAL_MODE_INPUT_PULLDOWN);
           break;
        case 20:
           palSetPadMode(DIO_PORT,DIO_PIN0+line,PAL_MODE_OUTPUT_PUSHPULL);
           break;
        case 21:
            palSetPadMode(DIO_PORT,DIO_PIN0+line,PAL_MODE_OUTPUT_OPENDRAIN);
            break;
        default:
           error = 1;
           break;
        }

 if (error)
    sendByte(NACK);
    else
    sendByte(ACK);

 // End sending CRC
 sendCRC();
 }

// Digital Write
void dioWrite(void)
 {
 int line,value;

 // Read line to write
 line = getByte();

 // Value to set
 value = getByte();

 // Check of CRC
 if (!crcResponse()) return;

 // Check line number
 if (line >= NDIO)
     {
     sendByte(NACK);
     sendCRC();
     return;
     }

 // Set dio value
 if (value)
	 palWritePad(DIO_PORT,DIO_PIN0+line,PAL_HIGH);
     else
     palWritePad(DIO_PORT,DIO_PIN0+line,PAL_LOW);

 // Send ACK and CRC
 sendByte(ACK);
 sendCRC();
 }

// Digital Read
void dioRead(void)
 {
 int line,value;

 // Read line to read
 line = getByte();

 // Check of CRC
 if (!crcResponse()) return;

 // Check line number
 if (line >= NDIO)
     {
     sendByte(NACK);
     sendCRC();
     return;
     }

 // Send ACK
 sendByte(ACK);

 // Read and send dio value
 value=palReadPad(DIO_PORT,DIO_PIN0+line);
 if (value)
    sendByte(1);
    else
    sendByte(0);

 // Send CRC
 sendCRC();
 }

// Digital Write all lines
void dioWriteAll(void)
 {
 int current,value,mask;

 // Value to set
 value = getU16();

 // Mask to set
 mask = getU16();

 if (!mask)
	 mask = DIO_MASK;
    else
     mask = mask & DIO_MASK;

 // Check of CRC
 if (!crcResponse()) return;

 // Read current port
 current = DIO_PORT->ODR;

 // Leave untouched bits out of dio mask
 current = current & (~mask);

 // Add masked received value
 current = current | (value & mask);

 // Set port
 (DIO_PORT->ODR) = current;

 // Send ACK and CRC
 sendByte(ACK);
 sendCRC();
 }

// Digital Read all lines
void dioReadAll(void)
 {
 int value;

 // Check of CRC
 if (!crcResponse()) return;

 // Send ACK
 sendByte(ACK);

 // Get value to send
 value = DIO_PORT->IDR;

 // Apply DIO mask
 value = value & DIO_MASK;

 // Send value
 sendU16(value);

 // Send CRC
 sendCRC();
 }
