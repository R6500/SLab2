/**************************************************************************

 SLab v2
 File: dio.h

 Digital I/O module header code

 **************************************************************************/

#ifndef DIO_MODULE
#define DIO_MODULE

/** Function prototypes *******************/

void dioInit(void);
void dioMode(void);
void dioWrite(void);
void dioRead(void);
void dioWriteAll(void);
void dioReadAll(void);

#endif //DIO_MODULE
