/*
 * AMT21.h
 *
 *  Created on: Jan 21, 2022
 *      Author: fame
 */

#ifndef INC_AMT21_H_
#define INC_AMT21_H_

/*
 * Include
 */
#include "stm32h7xx_hal.h"
#include "stdint.h"
/*
 * STRUCT
 */
typedef struct {
	UART_HandleTypeDef *uartHandle;
	GPIO_TypeDef *DE_port;
	uint16_t DE_pin;
	uint8_t address;

	uint16_t uart_buf;
	uint16_t position;
	uint8_t k0;
	uint8_t k1;
} AMT21;


/*
 * FUNCTIONS
 */




#endif /* INC_AMT21_H_ */
