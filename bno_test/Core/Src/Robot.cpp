#include "Robot.h"
#include "gpio.h"
#include "usart.h"
#include "dma.h"
#include "bno08.hpp"
#include <stdio.h>

Bno08 imu(&huart3);
uint32_t last_tick = 0;
uint32_t last_led = 0;
uint32_t callback_tick = 0;

void init_Robot()
{
    imu.init();
        printf("Robot Init Done\n");
}

void operate_Robot()
{

    uint32_t now = HAL_GetTick();

    if (now - last_tick < 10)
        return;

    last_tick = now;

        printf("yprxyz: %f %f %f %f %f %f\n", imu.get_data().yaw, imu.get_data().pitch, imu.get_data().roll, imu.get_data().accel_x, imu.get_data().accel_y, imu.get_data().accel_z);

    if (now - last_led > 100)
    {
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
        last_led = now;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
 {
    if (HAL_GetTick() - callback_tick > 100)
    {
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
        callback_tick = HAL_GetTick();
    }

    if (huart->Instance == huart3.Instance)
    {
        imu.rx_callback();
    }
 }

 void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == huart3.Instance)
    {
        imu.init();
    }
}