/*
 * AMT21.c
 *
 *  Created on: Jan 21, 2022
 *      Author: fame
 */
#include "AMT21.h"

void AMT21_read_value(AMT21 *dev){
	HAL_GPIO_WritePin(dev->DE_port, dev->DE_pin, 1);
	HAL_UART_Transmit(dev->uartHandle, (uint8_t *) &(dev->address), sizeof(dev->address), 100);
	HAL_GPIO_WritePin(dev->DE_port, dev->DE_pin, 0);
	HAL_UART_Receive(dev->uartHandle, (uint8_t *) &(dev->uart_buf), 2, 100);
	dev->k0 = (dev->uart_buf & 0x400) == 0x400;
	dev->k1 = (dev->uart_buf & 0x800) == 0x800;
}

HAL_StatusTypeDef AMT21_check_value(AMT21 *dev){
	uint8_t k0_check = (dev->uart_buf & 0x0001);
	uint8_t k1_check = (dev->uart_buf & 0x0002) >> 1;
	for (uint8_t i = 0; i < 6; i++){
		dev->uart_buf = dev->uart_buf >> 2;
		k0_check ^= dev->uart_buf & 0x0001;
		k1_check ^= (dev->uart_buf >> 1) & 0x0001;
	}
	if ((dev->k0 == k0_check) && (dev->k1 == k1_check)){
		return HAL_OK;
	}
	else {
		return HAL_ERROR;
	}
}
