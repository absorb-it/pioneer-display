#include "hal_init/spi.h"
#include "deh-x7800dab.h"

int main(void)
{
  HAL_Init();
  SystemClock_Config();

//   hspi2.Instance = SPI2;
//   hspi2.Init.Mode = SPI_MODE_SLAVE;
//   hspi2.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
//   hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
//   hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
//   hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
//   hspi2.Init.NSS = SPI_NSS_SOFT;
//   hspi2.Init.FirstBit = SPI_FIRSTBIT_LSB;
//   hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
//   hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
//   hspi2.Init.CRCPolynomial = 7;
//   hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
//   hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;

//     hdma_spi2_rx.Instance = DMA1_Channel4;
//     hdma_spi2_rx.Init.Request = DMA_REQUEST_1;
//     hdma_spi2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
//     hdma_spi2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
//     hdma_spi2_rx.Init.MemInc = DMA_MINC_ENABLE;
//     hdma_spi2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
//     hdma_spi2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
//     hdma_spi2_rx.Init.Mode = DMA_NORMAL;
//     hdma_spi2_rx.Init.Priority = DMA_PRIORITY_HIGH;

  MX_SPI2_Init();	// DEH-x7800

  deh7800_spi_errorCounter = 0;
  deh7800_spi_errorHeader = 0;

  deh7800_startSpiDMA();

  while (1)
  {
    if (HAL_GPIO_ReadPin(RADIO_ON_GPIO_Port, RADIO_ON_Pin))
      HAL_SPI_DMAResume(&hspi2);
    else
      HAL_SPI_DMAPause(&hspi2);
  }
}
