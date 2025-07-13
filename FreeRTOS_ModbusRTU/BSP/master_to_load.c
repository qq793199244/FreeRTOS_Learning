/*
 * master_to_load.c
 *
 *  Created on: Jul 8, 2025
 *      Author: WY
 */

#include "master_to_load.h"

extern ModbusRtuSlaveStruct slaveStr;
extern MotorDriverStruct motorDriverStr;
LoadStruct 		loadStr;

extern uint8_t REC_CTRL_ARRAY_IDX;

/*---------------------------------------------------------
 *
 * 电缸通讯初始化
 *
 *--------------------------------------------------------*/
void LoadInit(void)
{
	loadStr.recCnt = 0x00;
	loadStr.recFrameFlag = 0x00;
}

/*---------------------------------------------------------
 *
 * 上电后，电缸状态初始化
 * 目标电流 0x0036
 * 目标速度 0x0035
 * 目标位置 0x0034
 *
 *--------------------------------------------------------*/
void LoadStatusInit(void)
{
	uint8_t load_slave_id = 0;

	// 目标电流
	for (load_slave_id = 0; load_slave_id < LOAD_SLAVE_NUM; load_slave_id++){
		uint8_t send_data[8] = {load_slave_id + 1, 0x06, 0x00, 0x36, 0x03, 0x20,};
		SendModbusFrame(&huart3, send_data, 8);
		HAL_Delay(8);	// 刚上电，不阻塞
	}

	// 目标速度
	for (load_slave_id = 0; load_slave_id < LOAD_SLAVE_NUM; load_slave_id++){
		uint8_t send_data[8] = {load_slave_id + 1, 0x06, 0x00, 0x35, 0x03, 0xFF,};
		SendModbusFrame(&huart3, send_data, 8);
		HAL_Delay(8);	// 刚上电，不阻塞
	}

	// 目标位置
	for (load_slave_id = 0; load_slave_id < LOAD_SLAVE_NUM; load_slave_id++){
		uint8_t send_data[8] = {load_slave_id + 1, 0x06, 0x00, 0x34, 0x03, 0xE8,};
		SendModbusFrame(&huart3, send_data, 8);
		HAL_Delay(8);	// 刚上电，不阻塞
	}
}

/*---------------------------------------------------------
 *
 * 单片机处理来自读电缸的响应数据
 *
 *--------------------------------------------------------*/
void ProcessData_RecvFromLoad(void)
{
	if (loadStr.RecvBuff[1] == 0x03) {
		uint16_t nFarCRC = 0x0000;

		loadStr.localCRCValue = CRCCalc(loadStr.RecvBuff, loadStr.recCnt - 2);
		nFarCRC = loadStr.RecvBuff[loadStr.recCnt - 2] + (loadStr.RecvBuff[loadStr.recCnt - 1 ] << 8);

		if (nFarCRC == loadStr.localCRCValue) {
			uint8_t slaveId = 0x00;		// 从机地址

			if (slaveId == 0x01) {
				slaveStr.ReadRegsGroup[5] = (loadStr.RecvBuff[5] << 8) + loadStr.RecvBuff[6];	// 电缸反馈电流
				slaveStr.ReadRegsGroup[6] = (loadStr.RecvBuff[3] << 8) + loadStr.RecvBuff[4];	// 电缸实际位置
			}
			if (slaveId == 0x02) {
				slaveStr.ReadRegsGroup[15] = (loadStr.RecvBuff[5] << 8) + loadStr.RecvBuff[6];	// 电缸反馈电流
				slaveStr.ReadRegsGroup[16] = (loadStr.RecvBuff[3] << 8) + loadStr.RecvBuff[4];	// 电缸实际位置
			}
			if (slaveId == 0x03) {
				slaveStr.ReadRegsGroup[25] = (loadStr.RecvBuff[5] << 8) + loadStr.RecvBuff[6];	// 电缸反馈电流
				slaveStr.ReadRegsGroup[26] = (loadStr.RecvBuff[3] << 8) + loadStr.RecvBuff[4];	// 电缸实际位置
			}
			if (slaveId == 0x04) {
				slaveStr.ReadRegsGroup[35] = (loadStr.RecvBuff[5] << 8) + loadStr.RecvBuff[6];	// 电缸反馈电流
				slaveStr.ReadRegsGroup[36] = (loadStr.RecvBuff[3] << 8) + loadStr.RecvBuff[4];	// 电缸实际位置
			}
		} // CRC校验未通过的也不管
	} // 如果不是读的就不用管
	// 处理完这一帧，参数复位
	loadStr.recCnt = 0x00;
	loadStr.recFrameFlag = 0x00;
	loadStr.localCRCValue = 0x0000;
}

/*---------------------------------------------------------
 *
 * 单片机控制电缸的就位与复位
 *
 *--------------------------------------------------------*/
void ProcessData_TransToLoad(void)
{
	switch (REC_CTRL_ARRAY_IDX) {
	case 5:
		// 就位，电缸丝杆伸出到3000位置
		if (1 == slaveStr.WriteRegsGroup[5]) {
			uint8_t send_data[8] = {0x01, 0x06, 0x00, 0x34, 0x0B, 0x8B,};
			UART_SendDataWithSemaphore(&huart3, send_data, 8, osWaitForever, 20);
			slaveStr.WriteRegsGroup[5] = 0; 	// 避免重复写
		}
		break;

	case 6:
		// 电缸丝杆复位到1000位置
		if (1 == slaveStr.WriteRegsGroup[6]){
			uint8_t send_data[8] = {0x01, 0x06, 0x00, 0x34, 0x03, 0xE8,};
			UART_SendDataWithSemaphore(&huart3, send_data, 8, osWaitForever, 20);
			slaveStr.WriteRegsGroup[6] = 0; 	// 避免重复写
		}
		break;

	case 15:
		// 就位，电缸丝杆伸出到3000位置
		if (1 == slaveStr.WriteRegsGroup[15]) {
			uint8_t send_data[8] = {0x02, 0x06, 0x00, 0x34, 0x0B, 0x8B,};
			UART_SendDataWithSemaphore(&huart3, send_data, 8, osWaitForever, 20);
			slaveStr.WriteRegsGroup[15] = 0; 	// 避免重复写
		}
		break;

	case 16:
		// 电缸丝杆复位到1000位置
		if (1 == slaveStr.WriteRegsGroup[16]){
			uint8_t send_data[8] = {0x02, 0x06, 0x00, 0x34, 0x03, 0xE8,};
			UART_SendDataWithSemaphore(&huart3, send_data, 8, osWaitForever, 20);
			slaveStr.WriteRegsGroup[16] = 0; 	// 避免重复写
		}
		break;

	case 25:
		// 就位，电缸丝杆伸出到3000位置
		if (1 == slaveStr.WriteRegsGroup[25]) {
			uint8_t send_data[8] = {0x03, 0x06, 0x00, 0x34, 0x0B, 0x8B,};
			UART_SendDataWithSemaphore(&huart3, send_data, 8, osWaitForever, 20);
			slaveStr.WriteRegsGroup[25] = 0; 	// 避免重复写
		}
		break;

	case 26:
		// 电缸丝杆复位到1000位置
		if (1 == slaveStr.WriteRegsGroup[26]){
			uint8_t send_data[8] = {0x03, 0x06, 0x00, 0x34, 0x03, 0xE8,};
			UART_SendDataWithSemaphore(&huart3, send_data, 8, osWaitForever, 20);
			slaveStr.WriteRegsGroup[26] = 0; 	// 避免重复写
		}
		break;

	case 35:
		// 就位，电缸丝杆伸出到3000位置
		if (1 == slaveStr.WriteRegsGroup[35]) {
			uint8_t send_data[8] = {0x04, 0x06, 0x00, 0x34, 0x0B, 0x8B,};
			UART_SendDataWithSemaphore(&huart3, send_data, 8, osWaitForever, 20);
			slaveStr.WriteRegsGroup[35] = 0; 	// 避免重复写
		}
		break;

	case 36:
		// 电缸丝杆复位到1000位置
		if (1 == slaveStr.WriteRegsGroup[36]){
			uint8_t send_data[8] = {0x04, 0x06, 0x00, 0x34, 0x03, 0xE8,};
			UART_SendDataWithSemaphore(&huart3, send_data, 8, osWaitForever, 20);
			slaveStr.WriteRegsGroup[36] = 0; 	// 避免重复写
		}
		break;
	}
}
