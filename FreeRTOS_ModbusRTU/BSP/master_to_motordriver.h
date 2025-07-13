/*
 * master_to_motordriver.h
 *
 *  Created on: Jul 8, 2025
 *      Author: WY
 */

#ifndef MASTER_TO_MOTORDRIVER_H_
#define MASTER_TO_MOTORDRIVER_H_

#include "main.h"
#include "modbus_rtu_base.h"


extern UART_HandleTypeDef huart2; 	// 单片机（主站）和电机驱动器（从站）通讯

#define MOTOR_DRIVER_SLAVE_NUM	4	// 从机数量

// Modbus-RTU主站结构体
typedef struct {
    uint8_t  slaveAddress;       // 从机地址
    uint8_t  RecvBuff[32];       // 接收缓存
    uint8_t  recCnt;             // 接收计数
    uint8_t  recFrameFlag;       // 接收帧标志
    uint16_t localCRCValue;      // CRC校验值
    uint16_t ReadRegsGroup[8];	 // 读寄存器组
} MotorDriverStruct;


void MotorDriverInit(void);			// 电机驱动器通讯初始化
void ProcessData_RecvFromMotorDriver(void);	// 处理收到来自电机驱动器的数据(要读的数据)
void ProcessData_TransToMotorDriver(void);	// 单片机做主站，发送控制命令

#endif /* MASTER_TO_MOTORDRIVER_H_ */
