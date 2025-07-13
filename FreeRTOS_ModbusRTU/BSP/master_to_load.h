/*
 * master_to_load.h
 *
 *  Created on: Jul 8, 2025
 *      Author: WY
 */

#ifndef MASTER_TO_LOAD_H_
#define MASTER_TO_LOAD_H_

#include "main.h"
#include "modbus_rtu_base.h"


extern UART_HandleTypeDef huart3;  // 单片机（主站）和电缸（从站）之间通讯

#define LOAD_SLAVE_NUM		 4

// Modbus-RTU主站结构体
typedef struct {
    uint8_t  slaveAddress;       // 从机地址
    uint8_t  RecvBuff[16];       // 接收缓存
    uint8_t  recCnt;             // 接收计数
    uint8_t  recFrameFlag;       // 接收帧标志
    uint16_t localCRCValue;      // CRC校验值
    uint16_t ReadRegsGroup[8];	 // 读寄存器组
} LoadStruct;

void LoadInit(void);				// 电缸通讯初始化
void LoadStatusInit(void);			// 上电后，电缸状态初始化
void ProcessData_RecvFromLoad(void);		// 处理来自读电缸的响应数据
void ProcessData_TransToLoad(void);			// 单片机控制电缸的就位与复位
void ControlLoadInTest(void);				// 测试中，根据电缸丝杆的实时位置进行电流调整


#endif /* MASTER_TO_LOAD_H_ */
