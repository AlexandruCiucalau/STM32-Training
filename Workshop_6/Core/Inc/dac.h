#ifndef INC_DAC_H_
#define INC_DAC_H_

#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_conf.h"

#define CS43L22_I2C_ADDRESS        0x94
#define I2C_TIMEOUT                10

#define DO_MIDDLE				   0x00
#define RE_MIDDLE				   0x11
#define MI_MIDDLE				   0x22
#define FA_MIDDLE				   0x33
#define SO_MIDDLE				   0x44
#define LA_MIDDLE				   0X55
#define SI_MIDDLE				   0x66
#define DO_HIGH				           0x77
#define DO_HIGH_1				   0x88
#define RE_HIGH				           0x99
#define MI_HIGH				   	   0xAA
#define FA_HIGH			                   0xBB
#define SO_HIGH					   0xCC
#define LA_HIGH					   0xDD
#define SI_HIGH_2				   0xEE
#define DO_HIGH_2				   0xFF






typedef enum
{
    C4, C5, D5, E5, F5, G5, A5, B5,
    C6, D6, E6, F6, G6, A6, B6,
    C7, MAX_VALUE
} soundToneType;


void CS43L22_Init(void);
void CS43L22_Beep(soundToneType pitch, uint32_t duration_ms);
void CS43L22_StartMelody(void);
#endif /* INC_DAC_H_ */

