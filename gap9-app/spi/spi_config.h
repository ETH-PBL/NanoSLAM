#ifndef __SPI_CONFIG_H__
#define __SPI_CONFIG_H__

#include "pmsis.h"

// Spi Master configuration
#define SPI_MASTER_DUMMY_CYCLE          2
#define SPI_MASTER_DUMMY_CYCLE_MODE     PI_SPI_DUMMY_CLK_CYCLE_BEFORE_CS

// SPI Slave Configuration
#define SPI_SLAVE_BAUDRATE              1300000
#define SPI_SLAVE_WORDSIZE              PI_SPI_WORDSIZE_8
#define SPI_SLAVE_ENDIANESS             1
#define SPI_SLAVE_POLARITY              PI_SPI_POLARITY_0
#define SPI_SLAVE_PHASE                 PI_SPI_PHASE_0
#define SPI_SLAVE_CS                    0                   // (GPIO57) Use SPI2 CS0 on GAP9EVK's CN6(7) connector 
#define SPI_SLAVE_ITF                   2                   // SPI2 peripheral is set as slave
#define SPI_SLAVE_IS_SLAVE              1

// Spi slave pad configuration
#define SPI_SLAVE_PAD_SCK               PI_PAD_056  // Ball H1  > GAP9 EVK CN6 connector, pin 5
#define SPI_SLAVE_PAD_CS0               PI_PAD_057  // Ball H2  > GAP9 EVK CN6 connector, pin 7
#define SPI_SLAVE_PAD_CS1               PI_PAD_053  // Ball H7  > GAP9 EVK CN5 connector, pin 10 
#define SPI_SLAVE_PAD_SDO               PI_PAD_059  // Ball H4  > GAP9 EVK CN6 connector, pin 2
#define SPI_SLAVE_PAD_SDI               PI_PAD_058  // Ball H3  > GAP9 EVK CN6 connector, pin 6
#define SPI_SLAVE_PAD_FUNC              PI_PAD_FUNC2

#define SPI_NO_OPTION                   PI_SPI_CS_NONE// No option applied for SPI transfer 

#endif