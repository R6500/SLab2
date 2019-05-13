/**************************************************************************

 SLab v2
 File: slab.h

 Main header file for the program
 Configured to use the Nucleo64 F303RE Board

 Analog configuration

 Line   ADC   Channel    Opamp    OpampIn  OpIN_Pin
 AD1     3       1         3         3
 AD2     2       3         2         2
 AD3     4       3         4         1
 AD4     1       1         -         -
 AD5     2       3         2         3
 AD6     2       3         2         1
 AD7     3       1         3         0
 AD8     1       6         -         -

 **************************************************************************/

// String identifications for the firmware
#define BSTRING  "Nucleo64-F303RE ChibiOS SLab2"
#define VSTRING  " v2.0"

// Major number version changes when new commands are added
#define VERSION 2

// Use profiling (Comment for final code)
//#define USE_PROFILING

// Use test code (Comment for final code)
#define USE_TEST

/************* DEFAULT VALUES **************************/

#define DEFAULT_NREAD 10
#define DEFAULT_STIME 0.001  // 1ms

/************* DEFINES *********************************/

// 2^16 in float format
#define MAX16F 65536.0f

// Codes for transient responses
#define TRAN_OK       0  // Ok
#define TRAN_OVERRUN  1  // Sample overrun
#define TRAN_TIMEOUT  2  // Triggered timeout
#define TRAN_HALT     3  // Halt interrupt generated

// Magic data size
#define MAGIC_SIZE 4

// Board capabilities implemented in firmware
#define NDACS       2           // Number of DACs
#define NADCS       8           // Number of ADCs
#define BSIZE       30000       // Unified buffer size (in samples)
#define MAX_STIME   60.0f       // Maximum sample period is 60s
#define MAX_S_M     60          //   Mantissa
#define MAX_S_E     0           //   Exponent
#define MIN_STIME   0.000015f   // Minimum sample period is 10us
#define MIN_S_M     15          //   Mantissa
#define MIN_S_E     -6          //   Exponent
#define VDD_M       33          //   Vdd Mantissa
#define VDD_E       -1          //       Exponent
#define VREF_M      33          //   Vref Mantissa
#define VREF_E      -1          //        Exponent
#define DAC_BITS    12          // Number of DAC bits
#define ADC_BITS    12          // Number of ADC bits
#define MAX_SF      60000       // Maximum sample freq. for f response
#define MAX_SF_M    60          //   Mantissa
#define MAX_SF_E     3          //   Exponent
#define NDIO        13          // Number of digital I/O

// List of DAC and ADC pins
#define PIN_LIST "PA4|PA5|"\
                 "PA1|PB0|PB11|PA0|PA7|PB14|PB13|PB15|"\
                 "PC0|PC1|PC2|PC3|PC4|PC5|PC6|PC7|PC8|PC9|PC10|PC11|PC12|$"

/************** GLOBAL VARIABLES AS EXTERN ****************/

// Globals for CRC
extern int crcTx,crcRx;

/*************** HARDWARE RESOURCES ***********************/

// USART
#define USART2_TX_PORT GPIOA
#define USART2_TX_PAD  2
#define USART2_RX_PORT GPIOA
#define USART2_RX_PAD  3

/************* USED PINS **********************************/

#define BUTTON_PORT GPIOC
#define BUTTON_PIN  13

#define DIO_PORT  GPIOC
#define DIO_PIN0  0
#define DIO_MASK  ((1<<13) - 1)

/************* HARDWARE PROFILING *************************/

// Hardware profiling defines
#ifdef USE_PROFILING

// Profiling pins to be declared as outputs at main()
#define PRO1_PORT  GPIOA
#define PRO1_PIN   8
#define PRO2_PORT  GPIOB
#define PRO2_PIN   10

// Profiling code
// Code to set or clear each profile line
// They use low level calls to minimize effect on profiled code
#define PRO1_SET   GPIOA->BSRR.H.set = 1u<<8;
#define PRO1_CLEAR GPIOA->BSRR.H.clear = 1u<<+8;
#define PRO2_SET   GPIOB->BSRR.H.set = 1u<<10;
#define PRO2_CLEAR GPIOB->BSRR.H.clear = 1u<<10;

#else

#define PRO1_SET
#define PRO1_CLEAR
#define PRO2_SET
#define PRO2_CLEAR

#endif //USE_PROFILING



