#ifndef BSP_CAN_H
#define BSP_CAN_H
#include "struct_typedef.h"

extern void can_filter_init(void);
/* 给A板的24V输出使能 */
extern uint8_t MotorPowerEnable(void);

#endif
