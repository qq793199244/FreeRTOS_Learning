/*
 * slave_to_plc.c
 *
 *  Created on: Jul 8, 2025
 *      Author: WY
 */


#include "slave_to_plc.h"

ModbusRtuSlaveStruct slaveStr;		// 定义 Modbus RTU 从站结构体实例

uint8_t REC_CTRL_MOTOR_FLAG = 0;
uint8_t REC_CTRL_LOAD_FLAG = 0;
uint8_t REC_CTRL_ARRAY_IDX = 0;

/*---------------------------------------------------------
 *
 * 初始化从站地址
 *
 *--------------------------------------------------------*/
void ModbusRtuSlaveInit(void)
{
    slaveStr.slaveAddress = SLAVE_ADDR;		// 从机地址（关联宏定义 SLAVE_ADDR ）
    slaveStr.recCnt = 0x00;        			// 接收字节计数
    slaveStr.trsCnt = 0x00;					// 发送字节计数
    slaveStr.recFrameFlag = 0x00;			// 接收帧完成标志
}

/*---------------------------------------------------------
 *
 * 单片机作为从机时，处理接收来自PLC的数据
 *
 *--------------------------------------------------------*/
void ProcessData_RecvFromPLC(void)
{
	uint16_t nFarCRC = 0x0000;

	if (slaveStr.RecvBuff[0] == slaveStr.slaveAddress) {
		slaveStr.localCRCValue = CRCCalc(slaveStr.RecvBuff, slaveStr.recCnt - 2);
		nFarCRC = slaveStr.RecvBuff[slaveStr.recCnt - 2] + (slaveStr.RecvBuff[slaveStr.recCnt - 1] << 8);

		if (nFarCRC == slaveStr.localCRCValue) {
			ProcessFuncNum_RecvFromPLC();
		}
		else {
			CommuErrorTrs(slaveStr.RecvBuff[1], 0x03);		// CRC校验出错
		}
	}
	// 处理完这一帧数据，复位参数
	slaveStr.recCnt = 0x00;
	slaveStr.recFrameFlag = 0x00;
	slaveStr.localCRCValue = 0x0000;
}

/*---------------------------------------------------------
 *
 * 单片机作为从机时，根据功能码处理接收自PLC的数据
 *
 *--------------------------------------------------------*/

void ProcessFuncNum_RecvFromPLC(void)
{
	uint8_t funcNum = 0x00;
	int16_t nRegStartIdx   = 0x0000;      // 起始寄存器数组下标
	uint16_t nRegStartAddr = 0x0000;     // 寄存器起始地址
	uint8_t  nRegNum       = 0x00;       // 寄存器数量
	uint8_t  iCpy          = 0x00;       // 循环变量

	funcNum = slaveStr.RecvBuff[1];			// 功能码
	nRegStartAddr = (slaveStr.RecvBuff[2] << 8) + slaveStr.RecvBuff[3];		// 起始地址

	switch (funcNum) {
		// -------------------- 0x03 读寄存器 --------------------
		case 0x03:
			nRegStartIdx = nRegStartAddr - AS_SLAVE_READ_REG_BASE_ADDR;		// 基于读寄存器基地址计算数组下标
			nRegNum	 = (slaveStr.RecvBuff[4] << 8) + slaveStr.RecvBuff[5];	// 计算寄存器数量

			// 校验地址和数量是否越界
			if ((nRegStartIdx + nRegNum <= SLAVE_MAX_READ_REG_NUMBER) &&
				(nRegStartAddr >= AS_SLAVE_READ_REG_BASE_ADDR)) {
				slaveStr.TranBuff[0] = slaveStr.slaveAddress;	// 从机地址
				slaveStr.TranBuff[1] = 0x03;					// 功能码
				slaveStr.TranBuff[2] = nRegNum * 2;				// 返回字节数

				// 填充寄存器数据到响应缓存
				for (iCpy = 0; iCpy < nRegNum; iCpy++) {
					slaveStr.TranBuff[3 + iCpy * 2] = slaveStr.ReadRegsGroup[nRegStartIdx + iCpy] >> 8;
					slaveStr.TranBuff[4 + iCpy * 2]= slaveStr.ReadRegsGroup[nRegStartIdx + iCpy];
				}

				// 计算发送长度（地址+功能码+字节数+数据+CRC）
				slaveStr.trsCnt = nRegNum * 2 + 5;
				// 计算 CRC
				slaveStr.localCRCValue = CRCCalc(slaveStr.TranBuff, slaveStr.trsCnt - 2);
				// 填充 CRC
				slaveStr.TranBuff[slaveStr.trsCnt - 2] = slaveStr.localCRCValue;
				slaveStr.TranBuff[slaveStr.trsCnt - 1] = slaveStr.localCRCValue >> 8;

				if (osSemaphoreAcquire(frameSendSemaphoreHandle, osWaitForever) == osOK) {
					// 发送响应
					HAL_UART_Transmit_DMA(&huart1, slaveStr.TranBuff, slaveStr.trsCnt);
					osDelay(10);
					osSemaphoreRelease(frameSendSemaphoreHandle);
				}
				else {
						// 处理信号量获取失败的情况（如记录错误）
				}

				slaveStr.trsCnt = 0x00;
			}
			else {
				// 地址越界，返回错误
				CommuErrorTrs(slaveStr.RecvBuff[1], 0x02);
			}
			break;

		// -------------------- 0x06 写单个寄存器 --------------------
		case 0x06:
			nRegStartIdx = nRegStartAddr - AS_SLAVE_WRITE_REG_BASE_ADDR;	// 基于读寄存器基地址计算数组下标
			REC_CTRL_ARRAY_IDX = nRegStartIdx;		// 方便后面控制

			// 校验地址是否越界
			if ((nRegStartIdx <= SLAVE_MAX_WRITE_REG_NUMBER) &&
					(nRegStartAddr >= AS_SLAVE_WRITE_REG_BASE_ADDR)) {

				// 更新写寄存器数据
				slaveStr.WriteRegsGroup[nRegStartIdx] = (slaveStr.RecvBuff[4] << 8) + slaveStr.RecvBuff[5];

				// 判断是给电机驱动器的命令还是给电缸的命令
				if (REC_CTRL_ARRAY_IDX % 10 < 5) {
					REC_CTRL_MOTOR_FLAG = 1;
				}
				else {
					REC_CTRL_LOAD_FLAG = 1;
				}

				// 填充响应帧
				slaveStr.TranBuff[0] = slaveStr.slaveAddress;	// 从机地址
				slaveStr.TranBuff[1] = 0x06;	// 功能码
				slaveStr.TranBuff[2] = slaveStr.RecvBuff[2];
				slaveStr.TranBuff[3] = slaveStr.RecvBuff[3];
				slaveStr.TranBuff[4] = slaveStr.WriteRegsGroup[nRegStartIdx] >> 8;
				slaveStr.TranBuff[5] = slaveStr.WriteRegsGroup[nRegStartIdx];
				// 计算 CRC
				slaveStr.localCRCValue = CRCCalc(slaveStr.TranBuff, 6);
				// 填充 CRC
				slaveStr.TranBuff[6] = slaveStr.localCRCValue;
				slaveStr.TranBuff[7] = slaveStr.localCRCValue >> 8;

				// 校验回声数据
				if ((slaveStr.TranBuff[6] == slaveStr.RecvBuff[6]) &&
					(slaveStr.TranBuff[7] == slaveStr.RecvBuff[7])) {

					if (osSemaphoreAcquire(frameSendSemaphoreHandle, osWaitForever) == osOK) {
						// 发送响应
						HAL_UART_Transmit_DMA(&huart1, slaveStr.TranBuff, slaveStr.trsCnt);
						osDelay(10);
						osSemaphoreRelease(frameSendSemaphoreHandle);
					}
					else {
							// 处理信号量获取失败的情况（如记录错误）
					}

					slaveStr.trsCnt = 0x00;
				}
				else {
					CommuErrorTrs(slaveStr.RecvBuff[1], 0x03);	// 写数据错误
				}
			}
			else {
				CommuErrorTrs(slaveStr.RecvBuff[1], 0x02);	// 地址越界，返回错误
			}
			break;

		// -------------------- 0x10 写多个寄存器 --------------------
		case 0x10:
			nRegStartIdx = nRegStartAddr - AS_SLAVE_WRITE_REG_BASE_ADDR;	// 基于读寄存器基地址计算数组下标
			REC_CTRL_ARRAY_IDX = nRegStartIdx % 10;		// 方便后面控制，起始下标为3或4

			nRegNum = (slaveStr.RecvBuff[4] << 8) + slaveStr.RecvBuff[5];	// 计算寄存器数量

			// 校验地址和数量是否越界
			if ((nRegStartIdx + nRegNum <= SLAVE_MAX_WRITE_REG_NUMBER) &&
					(nRegStartAddr >= AS_SLAVE_WRITE_REG_BASE_ADDR)) {
				// 更新写寄存器数据
				for (iCpy = 0; iCpy < nRegNum; iCpy++) {
					slaveStr.WriteRegsGroup[nRegStartIdx + iCpy] =
						(slaveStr.RecvBuff[7 + iCpy * 2] << 8) + slaveStr.RecvBuff[8 + iCpy * 2];
				}
				REC_CTRL_MOTOR_FLAG = 1;		// 去改电机驱动器的配置

				// 填充响应帧（回声数据：从机地址、功能码、起始地址、寄存器数量）
				for (iCpy = 0; iCpy < uart1_rx_len - 2; iCpy++) {
					slaveStr.TranBuff[iCpy] = slaveStr.RecvBuff[iCpy];
				}
				slaveStr.trsCnt = slaveStr.recCnt - 1 - nRegNum * 2;

				// 计算 CRC
				slaveStr.localCRCValue = CRCCalc(slaveStr.TranBuff, slaveStr.trsCnt - 2);
				// 填充 CRC
				slaveStr.TranBuff[slaveStr.trsCnt - 2] = slaveStr.localCRCValue;
				slaveStr.TranBuff[slaveStr.trsCnt - 1] = slaveStr.localCRCValue >> 8;


				if (osSemaphoreAcquire(frameSendSemaphoreHandle, osWaitForever) == osOK) {
					// 发送响应
					HAL_UART_Transmit_DMA(&huart1, slaveStr.TranBuff, slaveStr.trsCnt);
					osDelay(10);
					osSemaphoreRelease(frameSendSemaphoreHandle);
				}
				else {
						// 处理信号量获取失败的情况（如记录错误）
				}
				slaveStr.trsCnt = 0x00;
			}
			else {
				// 地址越界，返回错误
				CommuErrorTrs(slaveStr.RecvBuff[1], 0x02);
			}
			break;

		// -------------------- 非法功能码 --------------------
		default:
			// 非法功能码，返回错误
			CommuErrorTrs(slaveStr.RecvBuff[1], 0x01);
			break;
	}
}

/*---------------------------------------------------------
 * 传输的错误处理
 * [0x01:非法功能码, 0x02:非法数据地址, 0x03:非法数据值(CRC校验出错、数据值错误)]
 * @param nFuncNum:   功能码
 * @param nErrorType: 错误类型
 *--------------------------------------------------------*/
void CommuErrorTrs(uint8_t nFuncNum, uint8_t nErrorType)
{
    slaveStr.TranBuff[0] = slaveStr.slaveAddress;		// 填充从机地址
    slaveStr.TranBuff[1] = nFuncNum | 0x80;				// 功能码最高位置1（表示错误响应）
    slaveStr.TranBuff[2] = nErrorType;					// 填充错误码
    slaveStr.localCRCValue = CRCCalc(slaveStr.TranBuff, 3);	// 计算 CRC（前 3 个字节）
    slaveStr.TranBuff[3] = slaveStr.localCRCValue;				// 填充 CRC 低字节
    slaveStr.TranBuff[4] = slaveStr.localCRCValue >> 8;		// 填充 CRC 高字节

//    HAL_UART_Transmit_DMA(&huart1, slaveStr.TranBuff, 5);

	// 等待上一帧发送完成（获取信号量）
//	osStatus_t status = osSemaphoreAcquire(frameSendSemaphoreHandle, osWaitForever);
	if (osSemaphoreAcquire(frameSendSemaphoreHandle, osWaitForever) == osOK) {
		// 发送响应
		HAL_UART_Transmit_DMA(&huart1, slaveStr.TranBuff, 5);
		osDelay(5);
		osSemaphoreRelease(frameSendSemaphoreHandle);
	}
	else {
			// 处理信号量获取失败的情况（如记录错误）
	}
	slaveStr.trsCnt = 0x00;
}
