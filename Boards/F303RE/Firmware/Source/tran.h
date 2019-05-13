/**************************************************************************

 SLab v2
 File: transient.h

 Transient module header code

 **************************************************************************/

#ifndef TRANSIENT
#define TRANSIENT

/* Definitions */

// High frequency timer
#define TIM_FREQ     1000000
#define TIM_FREQ_F   1000000.0
// Low frequency for stime above 50ms up to 60s
//#define TIM_FREQL    1000
//#define TIM_FREQL_F  1000.0
// Threshold
//#define TIM_TH       0.05

/* Function prototypes */

void tranInit(void);
void setSampleTime(void);
void setStorage(void);
void asyncRead(void);
void stepResponse(void);
void triggeredRead(void);
void loadWaveTable(void);
void loadSecondaryWaveTable(void);
void waveResponse(void);
void dualWaveResponse(void);
void singleWaveResponse(void);
void wavePlay(void);
void dualWavePlay(void);
void loadDigitalTable(void);

#endif //TRANSIENT
