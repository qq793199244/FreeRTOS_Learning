/*
 * master_to_motordriver.c
 *
 *  Created on: Jul 8, 2025
 *      Author: WY
 */

#include "master_to_motordriver.h"

extern ModbusRtuSlaveStruct slaveStr;
extern LoadStruct 		loadStr;
MotorDriverStruct	 motorDriverStr;

extern uint8_t REC_CTRL_ARRAY_IDX;

/*---------------------------------------------------------
 *
 * 电机驱动器通讯初始化
 *
 *--------------------------------------------------------*/
void MotorDriverInit(void)
{
	motorDriverStr.recCnt = 0x00;
	motorDriverStr.recFrameFlag = 0x00;
}

/*---------------------------------------------------------
 *
 * 单片机做主站，只关心读数据
 * 处理收到来自电机驱动器的数据，赋值到维持的读寄存器数组，方便PLC读
 *
 *--------------------------------------------------------*/
void ProcessData_RecvFromMotorDriver(void)
{
	if (motorDriverStr.RecvBuff[1] == 0x03) {
		uint16_t nFarCRC = 0x0000;

		motorDriverStr.localCRCValue = CRCCalc(motorDriverStr.RecvBuff, motorDriverStr.recCnt - 2);
		nFarCRC = motorDriverStr.RecvBuff[motorDriverStr.recCnt - 2]
										  + (motorDriverStr.RecvBuff[motorDriverStr.recCnt -1] << 8);
		if (nFarCRC == motorDriverStr.localCRCValue) {
			uint8_t slaveId = 0x00;
			uint8_t nRegNum = 0x00;

			slaveId = motorDriverStr.RecvBuff[0];		// 从机地址
			nRegNum = motorDriverStr.RecvBuff[2] / 2;	// 寄存器数量

			// 对号入座赋值
			if(slaveId == 0x01) {
				for (uint8_t idx = 0; idx < nRegNum; idx++) {
					slaveStr.ReadRegsGroup[0 + idx] = (motorDriverStr.RecvBuff[3 + idx*2] << 8)
							+ motorDriverStr.RecvBuff[4 + idx*2];
				}
			}
			if(slaveId == 0x02) {
				for (uint8_t idx = 0; idx < nRegNum; idx++) {
					slaveStr.ReadRegsGroup[10 + idx] = (motorDriverStr.RecvBuff[3 + idx*2] << 8)
							+ motorDriverStr.RecvBuff[4 + idx*2];
				}
			}
			if(slaveId == 0x03) {
				for (uint8_t idx = 0; idx < nRegNum; idx++) {
					slaveStr.ReadRegsGroup[20 + idx] = (motorDriverStr.RecvBuff[3 + idx*2] << 8)
							+ motorDriverStr.RecvBuff[4 + idx*2];
				}
			}
			if(slaveId == 0x04) {
				for (uint8_t idx = 0; idx < nRegNum; idx++) {
					slaveStr.ReadRegsGroup[30 + idx] = (motorDriverStr.RecvBuff[3 + idx*2] << 8)
							+ motorDriverStr.RecvBuff[4 + idx*2];
				}
			}
		} // 通过CRC校验的才处理
	}
	// 处理完这一帧数据，复位
	motorDriverStr.recCnt = 0x00;
	motorDriverStr.recFrameFlag = 0x00;
	motorDriverStr.localCRCValue = 0x0000;
}

/*---------------------------------------------------------
 *
 * 单片机做主站，发送控制命令
 *
 *--------------------------------------------------------*/
void ProcessData_TransToMotorDriver(void)
{
	static uint8_t prev_run_times;
	static uint16_t prev_duty_7_5_v;

	switch (REC_CTRL_ARRAY_IDX) {
	case 0:
		// 处理开始测试 当0x9000/0x900A/0x9014/0x901E = 1时
		if (1 == slaveStr.WriteRegsGroup[0]) {
			uint8_t send_data[8] = {0x01, 0x06, 0x00, 0xF3, 0x00, 0x01,};
			UART_SendDataWithSemaphore(&huart2, send_data, 8, osWaitForever, 40);
			slaveStr.WriteRegsGroup[0] = 0; // 避免重复写
		}
		slaveStr.ReadRegsGroup[0] = 1;		// 为了让电缸能顶动
		break;

	case 1:
		// 处理复位命令
		if (1 == slaveStr.WriteRegsGroup[1]) {
			// 电机驱动器自定义过程，复位0x7102=1
			uint8_t send_data[8] = {0x01, 0x06, 0x71, 0x02, 0x00, 0x01,};
			UART_SendDataWithSemaphore(&huart2, send_data, 8, osWaitForever, 40);
			slaveStr.WriteRegsGroup[1] = 0; // 避免重复写
		}
		break;

	case 2:
		// 处理停止测试
		if (1 == slaveStr.WriteRegsGroup[2]) {
			uint8_t send_data[8] = {0x01, 0x06, 0x00, 0xF3, 0x00, 0x00,};
			UART_SendDataWithSemaphore(&huart2, send_data, 8, osWaitForever, 40);
			slaveStr.WriteRegsGroup[2] = 0; // 避免重复写
		}
		slaveStr.ReadRegsGroup[0] = 0;		// 为了让电机的控制停下来
		break;

	case 10:
		// 处理开始测试 当0x9000/0x900A/0x9014/0x901E = 1时
		if (1 == slaveStr.WriteRegsGroup[10]) {
			uint8_t send_data[8] = {0x02, 0x06, 0x00, 0xF3, 0x00, 0x01,};
			UART_SendDataWithSemaphore(&huart2, send_data, 8, osWaitForever, 40);
			slaveStr.WriteRegsGroup[10] = 0; // 避免重复写
		}
		slaveStr.ReadRegsGroup[10] = 1;		// 为了让电缸能顶动
		break;

	case 11:
		// 处理复位命令
		if (1 == slaveStr.WriteRegsGroup[11]) {
			// 电机驱动器自定义过程，复位0x7102=1
			uint8_t send_data[8] = {0x02, 0x06, 0x71, 0x02, 0x00, 0x01,};
			UART_SendDataWithSemaphore(&huart2, send_data, 8, osWaitForever, 40);
			slaveStr.WriteRegsGroup[11] = 0; // 避免重复写
		}
		break;

	case 12:
		// 处理停止测试
		if (1 == slaveStr.WriteRegsGroup[12]) {
			uint8_t send_data[8] = {0x02, 0x06, 0x00, 0xF3, 0x00, 0x00,};
			UART_SendDataWithSemaphore(&huart2, send_data, 8, osWaitForever, 40);
			slaveStr.WriteRegsGroup[12] = 0; // 避免重复写
		}
		slaveStr.ReadRegsGroup[10] = 0;		// 为了让电机的控制停下来
		break;

	case 20:
		// 处理开始测试 当0x9000/0x900A/0x9014/0x901E = 1时
		if (1 == slaveStr.WriteRegsGroup[20]) {
			uint8_t send_data[8] = {0x03, 0x06, 0x00, 0xF3, 0x00, 0x01,};
			UART_SendDataWithSemaphore(&huart2, send_data, 8, osWaitForever, 40);
			slaveStr.WriteRegsGroup[20] = 0; // 避免重复写
		}
		slaveStr.ReadRegsGroup[20] = 1;		// 为了让电缸能顶动
		break;

	case 21:
		// 处理复位命令
		if (1 == slaveStr.WriteRegsGroup[21]) {
			// 电机驱动器自定义过程，复位0x7102=1
			uint8_t send_data[8] = {0x03, 0x06, 0x71, 0x02, 0x00, 0x01,};
			UART_SendDataWithSemaphore(&huart2, send_data, 8, osWaitForever, 40);
			slaveStr.WriteRegsGroup[21] = 0; // 避免重复写
		}
		break;

	case 22:
		// 处理停止测试
		if (1 == slaveStr.WriteRegsGroup[22]) {
			uint8_t send_data[8] = {0x03, 0x06, 0x00, 0xF3, 0x00, 0x00,};
			UART_SendDataWithSemaphore(&huart2, send_data, 8, osWaitForever, 40);
			slaveStr.WriteRegsGroup[22] = 0; // 避免重复写
		}
		slaveStr.ReadRegsGroup[20] = 0;		// 为了让电机的控制停下来
		break;

	case 30:
		// 处理开始测试 当0x9000/0x900A/0x9014/0x901E = 1时
		if (1 == slaveStr.WriteRegsGroup[30]) {
			uint8_t send_data[8] = {0x04, 0x06, 0x00, 0xF3, 0x00, 0x01,};
			UART_SendDataWithSemaphore(&huart2, send_data, 8, osWaitForever, 40);
			slaveStr.WriteRegsGroup[30] = 0; // 避免重复写
		}
		slaveStr.ReadRegsGroup[30] = 1;		// 为了让电缸能顶动
		break;

	case 31:
		// 处理复位命令
		if (1 == slaveStr.WriteRegsGroup[31]) {
			// 电机驱动器自定义过程，复位0x7102=1
			uint8_t send_data[8] = {0x04, 0x06, 0x71, 0x02, 0x00, 0x01,};
			UART_SendDataWithSemaphore(&huart2, send_data, 8, osWaitForever, 40);
			slaveStr.WriteRegsGroup[31] = 0; // 避免重复写
		}
		break;

	case 32:
		// 处理停止测试
		if (1 == slaveStr.WriteRegsGroup[32]) {
			uint8_t send_data[8] = {0x04, 0x06, 0x00, 0xF3, 0x00, 0x00,};
			UART_SendDataWithSemaphore(&huart2, send_data, 8, osWaitForever, 40);
			slaveStr.WriteRegsGroup[32] = 0; // 避免重复写
		}
		slaveStr.ReadRegsGroup[30] = 0;		// 为了让电机的控制停下来
		break;

	case 3:
		// 如果设置的跑合次数或者7.5V收回时的占空比有变化，则同步给电机驱动器
		if ((slaveStr.WriteRegsGroup[3] != prev_run_times) || (slaveStr.WriteRegsGroup[40] != prev_duty_7_5_v)) {
			uint8_t send_data[13] = {0x01, 0x10, 0x71, 0x00,
					0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00,};

			slaveStr.WriteRegsGroup[13] = slaveStr.WriteRegsGroup[3];
			slaveStr.WriteRegsGroup[14] = slaveStr.WriteRegsGroup[4];
			slaveStr.WriteRegsGroup[23] = slaveStr.WriteRegsGroup[3];
			slaveStr.WriteRegsGroup[24] = slaveStr.WriteRegsGroup[4];
			slaveStr.WriteRegsGroup[33] = slaveStr.WriteRegsGroup[3];
			slaveStr.WriteRegsGroup[34] = slaveStr.WriteRegsGroup[4];

			for (uint8_t i = 0; i < MOTOR_DRIVER_SLAVE_NUM; i++) {
				send_data[0] = i + 1;
				send_data[7] = slaveStr.WriteRegsGroup[3] >> 8;
				send_data[8] = slaveStr.WriteRegsGroup[3];
				send_data[9] = slaveStr.WriteRegsGroup[4] >> 8;
				send_data[10] = slaveStr.WriteRegsGroup[4];

				if (osSemaphoreAcquire(frameSendSemaphoreHandle, osWaitForever) == osOK) {
					SendModbusFrame(&huart2, send_data, 13);
					osDelay(50);
					osSemaphoreRelease(frameSendSemaphoreHandle);
				}
				else {
						// 处理信号量获取失败的情况（如记录错误）
				}

				prev_run_times = (send_data[7] << 8) + send_data[8];
				prev_duty_7_5_v = (send_data[9] << 8) + send_data[10];
			} // end for
		} // end if
		break;
	}
}
