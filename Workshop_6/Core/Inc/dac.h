#ifndef INC_DAC_H_
#define INC_DAC_H_

#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_conf.h"

#define CS43L22_I2C_ADDRESS        0x94
#define I2C_TIMEOUT                10

typedef enum
{
    C4, C5, D5, E5, F5, G5, A5, B5,
    C6, D6, E6, F6, G6, A6, B6,
    C7, D7, E7, F7, G7, A7, B7,
    MAX_VALUE
} soundToneType;


void CS43L22_Init(void);
void CS43L22_Beep(soundToneType pitch, uint32_t duration_ms);
void CS43L22_StartMelody(void);
#endif /* INC_DAC_H_ */

