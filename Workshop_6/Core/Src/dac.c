#include "main.h"
#include "dac.h"
#include "stm32f4xx_it.h"
#include <stdio.h>
#include <string.h>

extern I2C_HandleTypeDef hi2c1;


// Implementation of functions
void CS43L22_Init(void) {

    // Enable chip
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);

    //
    // Initialization
    //
    uint8_t TxBuffer[2];

    TxBuffer[0] = 0x0D;
    TxBuffer[1] = 0x01;
    HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t*) &TxBuffer, 2, I2C_TIMEOUT);

    TxBuffer[0] = 0x00;
    TxBuffer[1] = 0x99;
    HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t*) &TxBuffer, 2, I2C_TIMEOUT);

    TxBuffer[0] = 0x47;
    TxBuffer[1] = 0x80;
    HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t*) &TxBuffer, 2, I2C_TIMEOUT);

    TxBuffer[0] = 0x32;
    TxBuffer[1] = 0xFF;
    HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t*) &TxBuffer, 2, I2C_TIMEOUT);

    TxBuffer[0] = 0x32;
    TxBuffer[1] = 0x7F;
    HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t*) &TxBuffer, 2, I2C_TIMEOUT);

    TxBuffer[0] = 0x00;
    TxBuffer[1] = 0x00;
    HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t*) &TxBuffer, 2, I2C_TIMEOUT);

    TxBuffer[0] = 0x04;
    TxBuffer[1] = 0xAF;
    HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t*) &TxBuffer, 2, I2C_TIMEOUT);

    TxBuffer[0] = 0x0D;
    TxBuffer[1] = 0x70;
    HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t*) &TxBuffer, 2, I2C_TIMEOUT);

    TxBuffer[0] = 0x05;
    TxBuffer[1] = 0x81;
    HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t*) &TxBuffer, 2, I2C_TIMEOUT);

    TxBuffer[0] = 0x06;
    TxBuffer[1] = 0x07;
    HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t*) &TxBuffer, 2, I2C_TIMEOUT);

    TxBuffer[0] = 0x0A;
    TxBuffer[1] = 0x00;
    HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t*) &TxBuffer, 2, I2C_TIMEOUT);

    TxBuffer[0] = 0x27;
    TxBuffer[1] = 0x00;
    HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t*) &TxBuffer, 2, I2C_TIMEOUT);

    TxBuffer[0] = 0x1A;
    TxBuffer[1] = 0x0A;
    HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t*) &TxBuffer, 2, I2C_TIMEOUT);

    TxBuffer[0] = 0x1B;
    TxBuffer[1] = 0x0A;
    HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t*) &TxBuffer, 2, I2C_TIMEOUT);

    TxBuffer[0] = 0x1F;
    TxBuffer[1] = 0x0F;
    HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t*) &TxBuffer, 2, I2C_TIMEOUT);

    TxBuffer[0] = 0x02;
    TxBuffer[1] = 0x9E;
    HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t*) &TxBuffer, 2, I2C_TIMEOUT);
}

void CS43L22_Beep(soundToneType pitch, uint32_t duration_ms) {
    uint8_t TxBuffer[2];

    // Set volume and off time
    TxBuffer[0] = 0x1D;        // Register address
    TxBuffer[1] = 0x00;        // Value (volume and off time)
    HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, TxBuffer, 2, I2C_TIMEOUT);

    // Set sound frequency
    TxBuffer[0] = 0x1C;        // Register address
    switch (pitch) {
        case C4: TxBuffer[1] = 0x00; break;
        case C5: TxBuffer[1] = 0x11; break;
        case D5: TxBuffer[1] = 0x22; break;
        case E5: TxBuffer[1] = 0x33; break;
        case F5: TxBuffer[1] = 0x44; break;
        case G5: TxBuffer[1] = 0x55; break;
        case A5: TxBuffer[1] = 0x66; break;
        case B5: TxBuffer[1] = 0x77; break;
        case C6: TxBuffer[1] = 0x88; break;
        case D6: TxBuffer[1] = 0x99; break;
        case E6: TxBuffer[1] = 0xAA; break;
        case F6: TxBuffer[1] = 0xBB; break;
        case G6: TxBuffer[1] = 0xCC; break;
        case A6: TxBuffer[1] = 0xDD; break;
        case B6: TxBuffer[1] = 0xEE; break;
        case C7: TxBuffer[1] = 0xFF; break;
        default: TxBuffer[1] = 0x00; break; // Default to maximum frequency
    }
    HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, TxBuffer, 2, I2C_TIMEOUT);

    // Enable continuous mode (SOUND STARTED)
    TxBuffer[0] = 0x1E;        // Register address
    TxBuffer[1] = 0xC0;        // Value (beep and tone configuration)
    HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, TxBuffer, 2, I2C_TIMEOUT);

    // Playing...
    HAL_Delay(duration_ms);

    // Disable continuous mode (SOUND STOPPED)
    TxBuffer[0] = 0x1E;        // Register address
    TxBuffer[1] = 0x00;        // Value (beep and tone configuration)
    HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, TxBuffer, 2, I2C_TIMEOUT);
}
void CS43L22_StartMelody(void)
{   
    const soundToneType songMelody[] = {C4, D5, E5, F5, G5, A5, B5};
    for (int i = 0; i < sizeof(songMelody) / sizeof(songMelody[0]); i++)
     {
	 CS43L22_Beep(songMelody[i], 100);
	 HAL_Delay(200);
     }
}

