/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_receive.h"
#include "main.h"
#include "pid.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
// motor data read
#define get_motor_measure(ptr, data)                               \
  {                                                                \
    (ptr)->last_ecd = (ptr)->ecd;                                  \
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
    (ptr)->temperate = (data)[6];                                  \
  }

#define ABS(x) ((x > 0) ? (x) : (-x))
#define MAX_CIRCLE 36

__IO float circle = 0;

/*
motor data,  0:chassis motor1 3508;1:chassis motor3 3508;2:chassis motor3 3508;3:chassis motor4 3508;
4:yaw gimbal motor 6020;5:pitch gimbal motor 6020;6:trigger motor 2006;
电机数据, 0:底盘电机1 3508电机,  1:底盘电机2 3508电机,2:底盘电机3 3508电机,3:底盘电机4 3508电机;
4:yaw云台电机 6020电机; 5:pitch云台电机 6020电机; 6:拨弹电机 2006电机
*/
static motor_measure_t motor_chassis[7];

static CAN_TxHeaderTypeDef gimbal_tx_message;
static uint8_t gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef chassis_tx_message;
static uint8_t chassis_can_send_data[8];
float vel_forword = 6.283, vel_reverse = -6.283;
int test_mode = control_position;
motor_t motor[1];
extern float target_value;
uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};

/**
 * @brief          hal CAN fifo call back, receive motor data
 * @param[in]      hcan, the point to CAN handle
 * @retval         none
 */
/**
 * @brief          hal库CAN回调函数,接收电机数据
 * @param[in]      hcan:CAN句柄指针
 * @retval         none
 */

void only_pid_struct()
{
  for (int i = 0; i < 4; i++)
  {
    PID_struct_init(&pid_spd[i], DELTA_PID, 1000, 1000,
                    10.0f, 5.0f, 0.5f); // 4 motos angular rate closeloop.
    PID_struct_init(&pid_pos, POSITION_PID, 1000, 10,
                    200.0f, 0.1f, 5000.0f); // 4 motos angular rate closeloop.
  }
}

// int postion_control()
// {
//   int current;

//   if (angle < MIN_ANGLE)
//   {
//     angle = MIN_ANGLE;
//   }
//   else if (angle > MAX_ANGLE)
//   {
//     angle = MAX_ANGLE;
//   }

//   count = angle - motor_chassis[0].ecd;

//   if (count > 0) // 目标角度大于当前角度
//   {
//     if (count <= HALF_MAX_ANGLE) // 正转更近
//     {
//       current = 1;
//     }
//     else // 反转更近
//     {
//       current = -1;
//     }
//   }
//   else if (count < 0)
//   {
//     if (count <= -HALF_MAX_ANGLE) // 正转更近
//     {
//       current = 1;
//     }
//     else // 反转更近
//     {
//       current = -1;
//     }
//   }
//   return current;
// }

// void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
// {
//   CAN_RxHeaderTypeDef rx_header;
//   HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

//   switch (rx_header.StdId)
//   {
//   case CAN_3508_M1_ID:
//   case CAN_3508_M2_ID:
//   case CAN_3508_M3_ID:
//   case CAN_3508_M4_ID:
//   case CAN_YAW_MOTOR_ID:
//   case CAN_PIT_MOTOR_ID:
//   case CAN_TRIGGER_MOTOR_ID:
//   {
//     volatile uint8_t i = 0;
//     // get motor id
//     i = rx_header.StdId - CAN_3508_M1_ID;
//     get_motor_measure(&motor_chassis[i], rx_data);

//     if (test_mode == POSITION_MODE)
//     {
//       get_current = postion_control();
//       pid_calc(&pid_pos, (float)(motor_chassis[i].ecd), (float)angle);
//       pid_calc(&pid_spd[i], motor_chassis[i].speed_rpm, pid_pos.pos_out);

//       if (get_current == 1)
//       {
//         CAN_cmd_chassis(pid_spd[0].delta_out,
//                         pid_spd[1].delta_out,
//                         pid_spd[2].delta_out,
//                         pid_spd[3].delta_out);
//       }
//       else
//       {
//         CAN_cmd_chassis(-pid_spd[0].delta_out,
//                         -pid_spd[1].delta_out,
//                         -pid_spd[2].delta_out,
//                         -pid_spd[3].delta_out);
//       }
//     }
//     else if (test_mode == SPEED_MODE)
//     {
//       for (int i = 0; i < 4; i++)
//       {
//         pid_calc(&pid_spd[i], motor_chassis[i].speed_rpm, set_spd[i]);
//       }
//       CAN_cmd_chassis(pid_spd[0].delta_out,
//                       pid_spd[1].delta_out,
//                       pid_spd[2].delta_out,
//                       pid_spd[3].delta_out);
//     }
//     vofa = (float)(motor_chassis[0].speed_rpm);
//     //			VOFA_Send(&vofa);
//     break;
//   }

//   default:
//   {
//     break;
//   }
//   }
// }

static float count_circle(motor_t *motor)
{
  if (motor->para.vel > 0)
  {
    if (motor->para.pos < motor->para.pos_last)
		{
			circle++;
		}
  }
  else
  {
    if (motor->para.pos > motor->para.pos_last)
		{
			circle--;
		}
  }
	
  return circle;
}

/// @brief 转换整形为浮点
/// @param x_int
/// @param x_min
/// @param x_max
/// @param bits
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
  /* converts unsigned int to float, given range and number of bits */
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

void dm4310_fbdata(motor_t *motor, uint8_t *rx_data)
{
  motor->para.pos_last = motor->para.pos;
  motor->para.id = (rx_data[0]) & 0x0F;
  motor->para.state = (rx_data[0]) >> 4;
  motor->para.p_int = (rx_data[1] << 8) | rx_data[2];
  motor->para.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
  motor->para.t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
  motor->para.pos = uint_to_float(motor->para.p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
  motor->para.vel = uint_to_float(motor->para.v_int, V_MIN, V_MAX, 12); // (-45.0,45.0)
  motor->para.tor = uint_to_float(motor->para.t_int, T_MIN, T_MAX, 12); // (-18.0,18.0)
  motor->para.Tmos = (float)(rx_data[6]);
  motor->para.Tcoil = (float)(rx_data[7]);
}

/// @brief 设置速度
/// @param vel 速度
void spd_control(float vel)
{
  uint8_t data[4];
  uint8_t *vbuf;
  uint32_t send_mail_box;
  CAN_TxHeaderTypeDef spd_tx_message;
  spd_tx_message.StdId = 0x201;
  spd_tx_message.IDE = CAN_ID_STD;
  spd_tx_message.RTR = CAN_RTR_DATA;
  spd_tx_message.DLC = 0x04;
  vbuf = (uint8_t *)&vel;
  data[0] = *vbuf;
  data[1] = *(vbuf + 1);
  data[2] = *(vbuf + 2);
  data[3] = *(vbuf + 3);

  HAL_CAN_AddTxMessage(&hcan1, &spd_tx_message, data, &send_mail_box);
}

/// @brief 使能DM电机
/// @param hcan       CAN句柄
/// @param motor_id   电机ID
/// @param mode_id    使能/失能
void enable_motor_mode(CAN_HandleTypeDef *hcan, uint16_t motor_id, uint16_t mode_id)
{
  uint8_t data[8];
  uint16_t id = motor_id + mode_id;
  uint32_t send_mail_box;
  CAN_TxHeaderTypeDef Enable_tx_message;
  Enable_tx_message.StdId = id;
  Enable_tx_message.IDE = CAN_ID_STD;
  Enable_tx_message.RTR = CAN_RTR_DATA;
  Enable_tx_message.DLC = 0x08;
  data[0] = 0xFF;
  data[1] = 0xFF;
  data[2] = 0xFF;
  data[3] = 0xFF;
  data[4] = 0xFF;
  data[5] = 0xFF;
  data[6] = 0xFF;
  data[7] = 0xFC;
  HAL_CAN_AddTxMessage(hcan, &Enable_tx_message, data, &send_mail_box);
  HAL_Delay(20);
}

/// @brief 失能DM电机
/// @param hcan       CAN句柄
/// @param motor_id   电机ID
/// @param mode_id    使能/失能
void disable_motor_mode(CAN_HandleTypeDef *hcan, uint16_t motor_id, uint16_t mode_id)
{
  uint8_t data[8];
  uint16_t id = motor_id + mode_id;
  uint32_t send_mail_box;
  CAN_TxHeaderTypeDef disable_tx_message;
  disable_tx_message.StdId = id;
  disable_tx_message.IDE = CAN_ID_STD;
  disable_tx_message.RTR = CAN_RTR_DATA;
  disable_tx_message.DLC = 0x08;
  data[0] = 0xFF;
  data[1] = 0xFF;
  data[2] = 0xFF;
  data[3] = 0xFF;
  data[4] = 0xFF;
  data[5] = 0xFF;
  data[6] = 0xFF;
  data[7] = 0xFD;
  HAL_CAN_AddTxMessage(hcan, &disable_tx_message, data, &send_mail_box);
}

uint8_t CAN_RX_FLAG = 0;
/// @brief 控制3519的中断回调函数
/// @param hcan
/// @retval 无
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  __HAL_CAN_DISABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  CAN_RX_FLAG = 1;
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
  dm4310_fbdata(&motor[0], rx_data);

  count_circle(&motor[0]);

  // Add Speed Valuee or Reduce Speed Value
  // When Circle is equaling to middle value(one half of MAX_CIRCLE), the Speed Value is the maximum value
  // Speed_Lower
  // Speed_Upper
  // Speed_Max
  // Speed_default

  if (circle > MAX_CIRCLE)
  {
    target_value = -target_value;
    circle = 0;
  }
  if (circle < (-MAX_CIRCLE))
  {
    target_value = -target_value;
    circle = 0;
  }
  __HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
 * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000]
 * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
 * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
 * @param[in]      rev: (0x208) reserve motor control current
 * @retval         none
 */
/**
 * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
 * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
 * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
 * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
 * @param[in]      rev: (0x208) 保留，电机控制电流
 * @retval         none
 */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
  uint32_t send_mail_box;
  gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
  gimbal_tx_message.IDE = CAN_ID_STD;
  gimbal_tx_message.RTR = CAN_RTR_DATA;
  gimbal_tx_message.DLC = 0x08;
  gimbal_can_send_data[0] = (yaw >> 8);
  gimbal_can_send_data[1] = yaw;
  gimbal_can_send_data[2] = (pitch >> 8);
  gimbal_can_send_data[3] = pitch;
  gimbal_can_send_data[4] = (shoot >> 8);
  gimbal_can_send_data[5] = shoot;
  gimbal_can_send_data[6] = (rev >> 8);
  gimbal_can_send_data[7] = rev;
  HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
 * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
 * @param[in]      none
 * @retval         none
 */
/**
 * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
 * @param[in]      none
 * @retval         none
 */
void CAN_cmd_chassis_reset_ID(void)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = 0x700;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = 0;
  chassis_can_send_data[1] = 0;
  chassis_can_send_data[2] = 0;
  chassis_can_send_data[3] = 0;
  chassis_can_send_data[4] = 0;
  chassis_can_send_data[5] = 0;
  chassis_can_send_data[6] = 0;
  chassis_can_send_data[7] = 0;

  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
 * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
 * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384]
 * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384]
 * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384]
 * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384]
 * @retval         none
 */
/**
 * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
 * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
 * @retval         none
 */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = motor1 >> 8;
  chassis_can_send_data[1] = motor1;
  chassis_can_send_data[2] = motor2 >> 8;
  chassis_can_send_data[3] = motor2;
  chassis_can_send_data[4] = motor3 >> 8;
  chassis_can_send_data[5] = motor3;
  chassis_can_send_data[6] = motor4 >> 8;
  chassis_can_send_data[7] = motor4;

  HAL_CAN_AddTxMessage(&GIMBAL_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
 * @brief          return the yaw 6020 motor data point
 * @param[in]      none
 * @retval         motor data point
 */
/**
 * @brief          返回yaw 6020电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
  return &motor_chassis[4];
}

/**
 * @brief          return the pitch 6020 motor data point
 * @param[in]      none
 * @retval         motor data point
 */
/**
 * @brief          返回pitch 6020电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
  return &motor_chassis[5];
}

/**
 * @brief          return the trigger 2006 motor data point
 * @param[in]      none
 * @retval         motor data point
 */
/**
 * @brief          返回拨弹电机 2006电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
  return &motor_chassis[6];
}

/**
 * @brief          return the chassis 3508 motor data point
 * @param[in]      i: motor number,range [0,3]
 * @retval         motor data point
 */
/**
 * @brief          返回底盘电机 3508电机数据指针
 * @param[in]      i: 电机编号,范围[0,3]
 * @retval         电机数据指针
 */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
  return &motor_chassis[(i & 0x03)];
}
