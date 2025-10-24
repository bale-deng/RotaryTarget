#include "bsp_can.h"
#include "main.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/// @brief 过滤器初始化
/// @param  无
/// @retval 无
void can_filter_init(void)
{
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/* 给A板的24V输出使能 */
uint8_t MotorPowerEnable(void)
{
    uint8_t PinState = HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_2) & HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_3) & HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_4) & HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_5);

    if (HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_2) == GPIO_PIN_RESET)
    {
        // 使能24V
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_3, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_SET);
        HAL_Delay(1000);
        return 1;
    }
    else
    {
        HAL_Delay(500);
        return 1;
    }
}
