#ifndef INC_DAC_H_
#define INC_DAC_H_

#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_conf.h"

#define CS43L22_I2C_ADDRESS        0x94
#define I2C_TIMEOUT                10

typedef enum {
    C4 = 0x00,
    C5 = 0x11,
    D5 = 0x22,
    E5 = 0x33,
    F5 = 0x44,
    G5 = 0x55,
    A5 = 0x66,
    B5 = 0x77,
    C6 = 0x88,
    D6 = 0x99,
    E6 = 0xAA,
    F6 = 0xBB,
    G6 = 0xCC,
    A6 = 0xDD,
    B6 = 0xEE,
    C7 = 0xFF,
    MAX_VALUE
} soundToneType;

void CS43L22_Init(void);
void CS43L22_Beep(soundToneType pitch, uint32_t duration_ms);
void CS43L22_StartMelody(void);
#endif /* INC_DAC_H_ */

