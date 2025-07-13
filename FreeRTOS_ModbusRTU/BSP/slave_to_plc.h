/*
 * slave_to_plc.h
 *
 *  Created on: Jul 8, 2025
 *      Author: WY
 */

#ifndef SLAVE_TO_PLC_H_
#define SLAVE_TO_PLC_H_


#include "main.h"
#include "modbus_rtu_base.h"


extern UART_HandleTypeDef huart1;  // 用于PLC（主站）和单片机（从站）通讯

#define SLAVE_ADDR           		 0x01    // 从机地址

#define AS_SLAVE_READ_REG_BASE_ADDR  0x8000		// 读寄存器起始地址
#define AS_SLAVE_WRITE_REG_BASE_ADDR 0x9000		// 写寄存器起始地址
#define SLAVE_MAX_READ_REG_NUMBER    40		// 作为从站时最大读寄存器数量
#define SLAVE_MAX_WRITE_REG_NUMBER   40		// 作为从站时最大写寄存器数量
#define REC_BUFF_LENGTH              128	// 接收最大字节
#define TRS_BUFF_LENGTH              128	// 发送最大字节

// Modbus-RTU从站结构体
typedef struct {
    uint8_t  slaveAddress;              // 从机地址
    uint8_t  RecvBuff[REC_BUFF_LENGTH]; // 接收缓存
    uint8_t  TranBuff[TRS_BUFF_LENGTH]; // 发送缓存
    uint8_t  recPtr;                    // 接收指针
    uint8_t  recCnt;                    // 接收计数
    uint8_t  trsPtr;                    // 发送指针
    uint8_t  trsCnt;                    // 发送计数
    uint8_t  recFrameFlag;              // 接收帧标志
    uint16_t localCRCValue;             // CRC校验值
    uint16_t ReadRegsGroup[SLAVE_MAX_READ_REG_NUMBER];	// 从机读寄存器组
    uint16_t WriteRegsGroup[SLAVE_MAX_WRITE_REG_NUMBER];// 从机写寄存器组
} ModbusRtuSlaveStruct;

void ModbusRtuSlaveInit(void);
void ProcessData_RecvFromPLC(void);		// 处理接收来自PLC的数据
void ProcessFuncNum_RecvFromPLC(void);		// 根据功能码处理接收自PLC的数据
void CommuErrorTrs(uint8_t nFuncNum, uint8_t nErrorType);	// 传输的错误处理


#endif /* SLAVE_TO_PLC_H_ */
