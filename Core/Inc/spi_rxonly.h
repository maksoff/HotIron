/*
 * spi_rxonly.h
 *
 *  Created on: Aug 14, 2021
 *      Author: makso
 */

#ifndef INC_SPI_RXONLY_H_
#define INC_SPI_RXONLY_H_

HAL_StatusTypeDef HAL_SPI_ReceiveOnly(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);

#endif /* INC_SPI_RXONLY_H_ */
