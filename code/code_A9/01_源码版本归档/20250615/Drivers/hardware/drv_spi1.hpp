#pragma once

#include "spi_common.hpp"
#include <stdint.h>

SPI_Result spi1_write(
		const uint8_t *tx_data, const uint8_t tx_size, const uint8_t tx_buffer_size, 
		double TIMEOUT=-1);

SPI_Result spi1_writeNread(
    const uint8_t *tx_data, const uint8_t tx_size, const uint8_t tx_buffer_size,
    uint8_t *rx_data, const uint8_t rx_buffer_size,
    double TIMEOUT=-1);

void init_drv_spi1();