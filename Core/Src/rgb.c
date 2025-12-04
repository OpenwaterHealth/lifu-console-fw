/*
 * rgb.c
 *
 *  Created on: Dec 3, 2025
 *      Author: GitHub Copilot
 */

#include "rgb.h"
#include "main.h"

/* Internal RGB state variable */
static volatile uint8_t rgb_state = RGB_GREEN;

/**
 * @brief Initialize the RGB LED module
 * @note Sets the initial RGB state to GREEN
 */
void RGB_Init(void)
{
    rgb_state = RGB_GREEN;
    RGB_Set(rgb_state);
}

/**
 * @brief Set the RGB LED state
 * @param state The desired RGB LED state (RGB_OFF, RGB_RED, RGB_GREEN, RGB_BLUE)
 * @retval 0 on success, -1 on invalid state
 */
int8_t RGB_Set(uint8_t state)
{
    /* Validate input */
    if (state > RGB_BLUE) {
        return -1;
    }

    /* Update state */
    rgb_state = state;

    /* Turn off all LEDs first */
    HAL_GPIO_WritePin(LD_R_GPIO_Port, LD_R_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LD_G_GPIO_Port, LD_G_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LD_B_GPIO_Port, LD_B_Pin, GPIO_PIN_SET);

    /* Turn on the selected LED (active low) */
    switch (state)
    {
        case RGB_RED:
            HAL_GPIO_WritePin(LD_R_GPIO_Port, LD_R_Pin, GPIO_PIN_RESET);
            break;
        case RGB_GREEN:
            HAL_GPIO_WritePin(LD_G_GPIO_Port, LD_G_Pin, GPIO_PIN_RESET);
            break;
        case RGB_BLUE:
            HAL_GPIO_WritePin(LD_B_GPIO_Port, LD_B_Pin, GPIO_PIN_RESET);
            break;
        case RGB_OFF:
        default:
            /* All LEDs already off */
            break;
    }

    return 0;
}

/**
 * @brief Get the current RGB LED state
 * @retval Current RGB LED state
 */
uint8_t RGB_Get(void)
{
    return rgb_state;
}
