/**************************************************************************

 SLab v2
 File: serial.h

 Serial module header code

 **************************************************************************/

#ifndef SERIAL
#define SERIAL

/**************** DEFINITIONS *************************/

#define SDRIVER SD2                           // Serial driver
#define SDBASE  (BaseSequentialStream*)&SD2   // Base sequential stream
#define BREAK   "\r\n"                        // Line break
#define SSPEED  38400                         // Serial speed

// Special serial codes
#define ACK  181
#define NACK 226
#define ECRC 37

/**************** PUBLIC VARIABLES *************************/

// Magic code to identify a SLab Board
extern const uint8_t magic[MAGIC_SIZE];

/**************** FUNCTION PROTOTYPES **********************/

void serialBegin(void);

void startTx(void);
void sendCRC(void);
void sendByte(int value);
void sendU16(int value);
void sendMantExp(int mantissa, int exponent);
void sendString(char *str);

void startRx(void);
int getAndCheckCRC(void);
int crcResponse(void);
int getByte(void);
int getU16(void);
float getFloat(void);

#endif // SERIAL





