/*
 * modbus_rtu_base.c
 *
 *  Created on: Jul 8, 2025
 *      Author: WY
 */


#include "modbus_rtu_base.h"

/*---------------------------------------------------------
 *
 * CRC16计算，用的查表法
 *
 *--------------------------------------------------------*/
uint16_t CRCCalc(uint8_t *ptr, uint16_t len)
{
    uint16_t crcTemp = 0xFFFF;

    while (len--) {
        crcTemp = (crcTemp >> 8) ^ crc16_table[(crcTemp ^ *ptr++) & 0xFF];
    }

    return (crcTemp);
}

/*---------------------------------------------------------
 *
 * 发送附上CRC16校验的数据帧
 *
 *--------------------------------------------------------*/
void SendModbusFrame(UART_HandleTypeDef *huart, uint8_t *send_data, uint8_t len)
{
    uint16_t tmp_crc16 = CRCCalc(send_data, len - 2);

    send_data[len - 2] = tmp_crc16;
    send_data[len - 1] = tmp_crc16 >> 8;

    HAL_UART_Transmit_DMA(huart, send_data, len);
}


/**
 * @brief 通过UART DMA发送数据，使用信号量进行同步控制
 * @param huart UART句柄
 * @param pData 待发送数据缓冲区指针
 * @param Size 待发送数据大小(字节)
 * @param timeout 等待信号量超时时间(ms)，设为osWaitForever表示永久等待
 * @param delayMs 发送后延时时间(ms)，用于确保第一个字节发送出去
 * @return 发送结果状态码
 *         - 0: 发送成功
 *         - -1: 获取信号量超时
 *         - -2: UART DMA发送启动失败
 */
void UART_SendDataWithSemaphore(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t timeout, uint32_t delayMs)
{
    // 尝试获取信号量
    if (osSemaphoreAcquire(frameSendSemaphoreHandle, timeout) == osOK) {
    	// 启动UART DMA发送
    	SendModbusFrame(huart, pData, Size);
		// 短暂延时确保第一个字节发送出去
		if(delayMs > 0) {
			osDelay(delayMs);
		}
    }
    // 释放信号量允许后续发送
    osSemaphoreRelease(frameSendSemaphoreHandle);
}

