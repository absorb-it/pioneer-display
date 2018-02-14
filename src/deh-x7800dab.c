#include <deh-x7800dab.h>
#include <deh-x7800dab_icons.h>
#include "af_lcd.h"
#include "hal_init/spi.h"
#include "hal_init/dma.h"

HAL_StatusTypeDef status0x47, status0x48;
unsigned int dmaCounter0x47[4] = { 0, 0, 0, 0 };
unsigned int dmaCounter0x48[4] = { 0, 0, 0, 0 };
unsigned int dmaCounter0x43 = 0;

void deh7800_showDisplay(unsigned int rootX, unsigned int rootY,
			 uint16_t frameColor, uint16_t segmentsFgColor, uint16_t iconsFgColor,
			 uint16_t iconsInactiveColor, uint16_t sideMatrixFgColor, uint16_t infoBarFgColor) {

    LCD_drawLine(rootX, rootY, rootX + 319, rootY, frameColor);
    LCD_drawLine(rootX, rootY, rootX, rootY + 62, frameColor);
    LCD_drawLine(rootX, rootY + 62, rootX + 319, rootY + 62, frameColor);
    LCD_drawLine(rootX + 319, rootY, rootX + 319, rootY + 62, frameColor);

    deh7800_showSegments(rootX + 26, rootY + 32, segmentsFgColor);

    if (PioneerGraphicsBuffer[19] & 0b00001000)
      draw_lupeIcon(rootX + 10, rootY + 7, iconsFgColor, RGB(0,0,0));
    else
      draw_lupeIcon(rootX + 10, rootY + 7, iconsInactiveColor, RGB(0,0,0));
    deh7800_showIcons(rootX + 226, rootY + 5, iconsFgColor, RGB(0,0,0), iconsInactiveColor);

    deh7800_showInfoBar(rootX + 33, rootY + 7, infoBarFgColor);

    deh7800_showSideMatrix(rootX + 5, rootY + 34, sideMatrixFgColor);
}



void deh7800_showSegments(unsigned int rootX, unsigned int rootY, uint16_t fg_color) {
    // Segment-Data Start-Bit-Position related to continuous Data storage
    const uint16_t segDataPosition[13] = {0x006e, 0x005f, 0x0050, 0x0041, 0x0032, 0x01f0, 0x01e1, 0x01d2, 0x01c3, 0x01b4, 0x01a5, 0x0196, 0x0187};
    const uint16_t dataPosition_arrow = 0x0183;

    int index = 0;
    while (index < 13) {
	uint32_t segmentData = PioneerGraphicsBuffer[segDataPosition[index]/8 + 2] << 16 |
			      PioneerGraphicsBuffer[segDataPosition[index]/8 + 1] << 8 |
			      PioneerGraphicsBuffer[segDataPosition[index]/8];

	uint16_t segmentDrawingData = segmentData >> (segDataPosition[index] % 8);

	// special case for segment 5, grrrr
	if (index == 5) {
	    segmentDrawingData &= 0x03FF; // remove invalid data
	    segmentDrawingData |= (uint16_t)(PioneerGraphicsBuffer[5] & 0xE0) << 5 | (uint16_t)(PioneerGraphicsBuffer[6] & 0x03) << 13;
	}

	LCD_drawSegment(segmentDrawingData, index, rootX, rootY, fg_color );
	index++;
    }

    uint8_t arrow = (PioneerGraphicsBuffer[dataPosition_arrow/8] >> (dataPosition_arrow % 8)) & 0x01;
    LCD_drawSegmentArrow(arrow, rootX, rootY, fg_color);
}

void deh7800_showSideMatrix(unsigned int rootX, unsigned int rootY, uint16_t fg_color) {
  // Byte-Order in PixelBuffer (LSB) - ignore X-bit, 4bit groups
  //         23    |   22    |     21   |    20    |   19    |    18    |   17    |    16     |    15
  //      0011011 0|0011 0110|0 11110 00|000 00000 |00000 110|00 11000 1|1000 0111|0 00000 0 1|11
  //               |   X     |X     X   |  X     X |    X    | X     X  |   X     |X     X
  // the pixel-matrix is made of 6*8 pixels, strangely grouped in 5er groups with 4 relevant bits
  // containing the data for the lower and the upper half somehow continously


  int pixelBufferIndex = 15; 	// start with data in Position 15
  int currentBit = 6; 		// start with data in bit 7

  int targetMatrixXPosition = 0, targetMatrixYPosition = 0;
  int targetMatrixPixelWidth = 3, targetMatrixPixelHeight = 3;

  // first the data of the lower half (4 x 6, every fifth bit is garbage)
  // draw as the data comes, from right to left
  while (targetMatrixXPosition < 6) {
      while (targetMatrixYPosition < 4) {
	  LCD_fillRect((5-targetMatrixXPosition) * targetMatrixPixelWidth + rootX, (4+targetMatrixYPosition) * targetMatrixPixelHeight + rootY,
		       (5-targetMatrixXPosition +1) * targetMatrixPixelWidth + rootX - 2, (4+targetMatrixYPosition+1) * targetMatrixPixelHeight + rootY - 2,
		       fg_color * ((PioneerGraphicsBuffer[pixelBufferIndex] >> currentBit) & 1));
	  targetMatrixYPosition++;
	  currentBit++;
	  if (currentBit > 7) {
	      currentBit = 0;
	      pixelBufferIndex++;
	  }
      }
      targetMatrixXPosition++;
      targetMatrixYPosition = 0;
      currentBit++;	// skip one bit which is not part of the matrix
      if (currentBit > 7) {
	  currentBit = 0;
	  pixelBufferIndex++;
      }
  }

  // now the upper half (4 x6, every fifth bit is garbage)
  // draw as the data comes, from bottom up
  targetMatrixXPosition = 0;
  targetMatrixYPosition = 0;
  while (targetMatrixXPosition < 6) {
      while (targetMatrixYPosition < 4) {
	  LCD_fillRect(targetMatrixXPosition * targetMatrixPixelWidth + rootX, (3-targetMatrixYPosition) * targetMatrixPixelHeight + rootY,
		       (targetMatrixXPosition +1) * targetMatrixPixelWidth + rootX - 2, (3-targetMatrixYPosition+1) * targetMatrixPixelHeight + rootY - 2,
		       fg_color * ((PioneerGraphicsBuffer[pixelBufferIndex] >> currentBit) & 1));
	  targetMatrixYPosition++;
	  currentBit++;
	  if (currentBit > 7) {
	      currentBit = 0;
	      pixelBufferIndex++;
	  }
      }
      targetMatrixXPosition++;
      targetMatrixYPosition = 0;
      currentBit++;	// skip one bit which is not part of the matrix
      if (currentBit > 7) {
	  currentBit = 0;
	  pixelBufferIndex++;
      }
  }
}

void deh7800_showIcons(unsigned int rootX, unsigned int rootY, uint16_t fg_color, uint16_t bg_color, uint16_t inactive_color) {
    if (PioneerGraphicsBuffer[47] & 0x20)
      draw_TAIcon(rootX, rootY, fg_color, bg_color);
    else
      draw_TAIcon(rootX, rootY, inactive_color, bg_color);

    if (PioneerGraphicsBuffer[47] & 0x40)
      draw_TPIcon(rootX + 22, rootY, fg_color, bg_color);
    else
      draw_TPIcon(rootX + 22, rootY, inactive_color, bg_color);

    if (PioneerGraphicsBuffer[47] & 0x80)
      draw_SRtrvIcon(rootX + 45, rootY, fg_color, bg_color);
    else
      draw_SRtrvIcon(rootX + 45, rootY, inactive_color, bg_color);

    if (PioneerGraphicsBuffer[53] & 0x40)
      draw_repeatIcon(rootX + 11, rootY + 10, fg_color, bg_color);
    else
      draw_repeatIcon(rootX + 11, rootY + 10, inactive_color, bg_color);

    if (PioneerGraphicsBuffer[51] & 0x80)
      draw_shuffleIcon(rootX + 34, rootY + 10, fg_color, bg_color);
    else
      draw_shuffleIcon(rootX + 34, rootY + 10, inactive_color, bg_color);

    if (PioneerGraphicsBuffer[48] & 0b01000000)
      draw_bluetoothIcon(rootX + 57, rootY + 10, fg_color, bg_color);
    else
      draw_bluetoothIcon(rootX + 57, rootY + 10, inactive_color, bg_color);

    if (PioneerGraphicsBuffer[48] & 0b00000010)
      draw_notekeyIcon(rootX + 68, rootY + 10, fg_color, bg_color);
    else
      draw_notekeyIcon(rootX + 68, rootY + 10, inactive_color, bg_color);

    if (PioneerGraphicsBuffer[48] & 0b00000001)
      draw_phoneIcon(rootX + 83, rootY + 10, fg_color, bg_color);
    else
      draw_phoneIcon(rootX + 83, rootY + 10, inactive_color, bg_color);

    // I have no iPhone and will never have... no idea when this icon will be displayed
    // draw_iPhoneIcon(25, 20, inactive_color, bg_color);
}


void deh7800_showInfoBar(unsigned int rootX, unsigned int rootY, uint16_t fg_color) {
    int fieldPos;

    // this is the storage of the InfoBar Matrix - the bits have to be collected first
    // with just magic and intense monitoring I was able to figure out where are the relevant bits
    // deh7800_printDataOverview was helpful indeed
    uint8_t upperBar[20]; // 5*31 = 155 = 20Byte

    // it seems like the first 2-point-wide-bar is only stored in one bit (all 5 lines on/off)
    // after that we have 5 bits for every 2-point-wide bar spread across different positions)
    upperBar[0] = ((PioneerGraphicsBuffer[22] & 0b00010000) >> 4) * 0b00011111 |
	(PioneerGraphicsBuffer[23] & 0b00001110) << 4;
    upperBar[1] = PioneerGraphicsBuffer[23]>> 4;

    fieldPos = 1;
    while (fieldPos < 11) {
	upperBar[fieldPos] |= PioneerGraphicsBuffer[fieldPos+23] << 4;
	upperBar[fieldPos+1] = PioneerGraphicsBuffer[fieldPos+23] >> 4;
	fieldPos++;
    }

    upperBar[11] |= (PioneerGraphicsBuffer[41] & 0b01111100) | (PioneerGraphicsBuffer[40] & 0b00100000) << 2;
    upperBar[12] =  PioneerGraphicsBuffer[40] >> 6 | (PioneerGraphicsBuffer[41] & 0b00000011) << 2 |
	PioneerGraphicsBuffer[5] << 4;
    upperBar[13] = (PioneerGraphicsBuffer[5] & 0b00010000) >> 4 | PioneerGraphicsBuffer[77] << 1;

    fieldPos = 0;
    while (fieldPos < 6) {
	      upperBar[fieldPos+13] |= PioneerGraphicsBuffer[fieldPos+42] << 6;
	      upperBar[fieldPos+14] = PioneerGraphicsBuffer[fieldPos+42] >> 2;
	      fieldPos++;
    }

    fieldPos = 0;
    unsigned int targetMatrixXPosition = 0, targetMatrixYPosition = 0;
    // pixel size, but we draw two pixel side-by-side for every data-bit (thats how it is done on the display)
    // the pixel size includes a onebit frame, so actual pixel is one field smaller
    unsigned int targetMatrixPixelWidth = 3, targetMatrixPixelHeight = 3;
    int currentBit = 0;
    while (fieldPos < 20) {
	      while (currentBit < 8 && targetMatrixYPosition <= 30) {

		  unsigned int x = rootX + targetMatrixYPosition * 2 * targetMatrixPixelWidth,
		      y = rootY + targetMatrixXPosition * targetMatrixPixelHeight;
		  LCD_fillRect(x, y, x + targetMatrixPixelWidth - 2, y + targetMatrixPixelHeight - 2,
			       fg_color * ((upperBar[fieldPos] >> currentBit) & 1));
		  LCD_fillRect(x + targetMatrixPixelWidth, y, x + 2 * targetMatrixPixelWidth - 2, y + targetMatrixPixelHeight - 2,
			       fg_color * ((upperBar[fieldPos] >> currentBit) & 1));

		  currentBit++;
		  targetMatrixXPosition ++;
		  if (targetMatrixXPosition > 4) {
		      targetMatrixXPosition = 0;
		      targetMatrixYPosition ++;
		  }
		  if (targetMatrixYPosition > 30)
		    break;
	      }
	      currentBit = 0;
	      fieldPos++;
    }
}


void deh7800_printDataOverview(void) {
  char str[100];
  sprintf(str, "%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x  %.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x",
	  PioneerGraphicsBuffer[0], PioneerGraphicsBuffer[1], PioneerGraphicsBuffer[2], PioneerGraphicsBuffer[3], PioneerGraphicsBuffer[4],
	  PioneerGraphicsBuffer[5], PioneerGraphicsBuffer[6], PioneerGraphicsBuffer[7], PioneerGraphicsBuffer[8], PioneerGraphicsBuffer[9],
	  PioneerGraphicsBuffer[10], PioneerGraphicsBuffer[11],
	  PioneerGraphicsBuffer[12], PioneerGraphicsBuffer[13], PioneerGraphicsBuffer[14], PioneerGraphicsBuffer[15], PioneerGraphicsBuffer[16],
	  PioneerGraphicsBuffer[17], PioneerGraphicsBuffer[18], PioneerGraphicsBuffer[19], PioneerGraphicsBuffer[20], PioneerGraphicsBuffer[21],
	  PioneerGraphicsBuffer[22], PioneerGraphicsBuffer[23]);
  LCD_drawString (10, 50, str, RGB(255,255,255), RGB(0,0,0), FONT_SMALL);

  sprintf(str, "%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x  %.2x%.2x%.2x%.2x%.2x%.2x",
	  PioneerGraphicsBuffer[24], PioneerGraphicsBuffer[25], PioneerGraphicsBuffer[26], PioneerGraphicsBuffer[27], PioneerGraphicsBuffer[28],
	  PioneerGraphicsBuffer[29], PioneerGraphicsBuffer[30], PioneerGraphicsBuffer[31], PioneerGraphicsBuffer[32], PioneerGraphicsBuffer[33],
	  PioneerGraphicsBuffer[34], PioneerGraphicsBuffer[35],
	  PioneerGraphicsBuffer[36], PioneerGraphicsBuffer[37], PioneerGraphicsBuffer[38], PioneerGraphicsBuffer[39], PioneerGraphicsBuffer[40],
	  PioneerGraphicsBuffer[41]);
  LCD_drawString (10, 60, str, RGB(255,255,255), RGB(0,0,0), FONT_SMALL);

  sprintf(str, "%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x  %.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x",
	  PioneerGraphicsBuffer[42], PioneerGraphicsBuffer[43], PioneerGraphicsBuffer[44], PioneerGraphicsBuffer[45], PioneerGraphicsBuffer[46],
	  PioneerGraphicsBuffer[47], PioneerGraphicsBuffer[48], PioneerGraphicsBuffer[49], PioneerGraphicsBuffer[50], PioneerGraphicsBuffer[51],
	  PioneerGraphicsBuffer[52], PioneerGraphicsBuffer[53],
	  PioneerGraphicsBuffer[54], PioneerGraphicsBuffer[55], PioneerGraphicsBuffer[56], PioneerGraphicsBuffer[57], PioneerGraphicsBuffer[58],
	  PioneerGraphicsBuffer[59], PioneerGraphicsBuffer[60], PioneerGraphicsBuffer[61], PioneerGraphicsBuffer[62], PioneerGraphicsBuffer[63],
	  PioneerGraphicsBuffer[64], PioneerGraphicsBuffer[65]);
  LCD_drawString (10, 70, str, RGB(255,255,255), RGB(0,0,0), FONT_SMALL);

  sprintf(str, "%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x  %.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x",
	  PioneerGraphicsBuffer[66], PioneerGraphicsBuffer[67], PioneerGraphicsBuffer[68], PioneerGraphicsBuffer[69], PioneerGraphicsBuffer[70],
	  PioneerGraphicsBuffer[71], PioneerGraphicsBuffer[72], PioneerGraphicsBuffer[73], PioneerGraphicsBuffer[74], PioneerGraphicsBuffer[75],
	  PioneerGraphicsBuffer[76], PioneerGraphicsBuffer[77],
	  PioneerGraphicsBuffer[78], PioneerGraphicsBuffer[79], PioneerGraphicsBuffer[80], PioneerGraphicsBuffer[81], PioneerGraphicsBuffer[82],
	  PioneerGraphicsBuffer[83], PioneerGraphicsBuffer[84], PioneerGraphicsBuffer[85], PioneerGraphicsBuffer[86]);
  LCD_drawString (10, 80, str, RGB(255,255,255), RGB(0,0,0), FONT_SMALL);
}


unsigned char spiRxBuffer[deh_7800_spi_BufferSize];


void deh7800_startSpiDMA(void) {
  HAL_SPI_Receive_DMA(&hspi2, spiRxBuffer, deh_7800_spi_BufferSize);
}

// wait for first read 0x43, after that read again and always 160 (2x80 bytes)
void
HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* spiHandle)
{
  /* Read PRIMASK register, check interrupt status before you disable them */
  /* Returns 0 if they are enabled, or non-zero if disabled */
  uint32_t prim = __get_PRIMASK();

  /* Disable interrupts */
  __disable_irq();

  int foundData = 0;
  if (spiHandle->Instance == SPI2) {
      int i = 0;
      int posCounter = 0;
      while (posCounter < deh_7800_spi_BufferSize) {
	// complete body read as well, evaluate now
	int targetPosition, targetSize;
	switch (spiRxBuffer[posCounter]) {
	  case 0x47:
	    if (posCounter + 20 >= deh_7800_spi_BufferSize) {
		posCounter = deh_7800_spi_BufferSize;
		break;
	    }
	    foundData = 1;
	    i = (spiRxBuffer[posCounter + 19] & 0b11000000) >> 6;
	    switch(i) {
	      case 0:
		targetPosition = 0; targetSize = 12; break;
	      case 2:
		targetPosition = 12; targetSize = 12; break;
	      case 1:
		targetPosition = 24; targetSize = 12; break;
	      case 3:
		targetPosition = 36; targetSize = 6; break;
	    }
	    HAL_DMA_Abort(&hdma_memtomem_dma2_channel1);
	    status0x47 = HAL_DMA_Start(&hdma_memtomem_dma2_channel1, (uint32_t)(&spiRxBuffer[posCounter+1]), (uint32_t)(&PioneerGraphicsBuffer[targetPosition]), targetSize);
	    dmaCounter0x47[i]++;
	    posCounter += 20;
	    break;
	  case 0x48:
	    if (posCounter + 20 >= deh_7800_spi_BufferSize) {
		posCounter = deh_7800_spi_BufferSize;
		break;
	    }
	    foundData = 1;
	    i = (spiRxBuffer[posCounter + 19] & 0b11000000) >> 6;	// LSB
	    switch(i) {
	      case 0:
		targetPosition = 42; targetSize = 12; break;
	      case 2:
		targetPosition = 54; targetSize = 12; break;
	      case 1:
		targetPosition = 66; targetSize = 12; break;
	      case 3:
		targetPosition = 78; targetSize = 9; break;
	    }
	    HAL_DMA_Abort(&hdma_memtomem_dma2_channel1);
	    status0x48 = HAL_DMA_Start(&hdma_memtomem_dma2_channel1, (uint32_t)(&spiRxBuffer[posCounter+1]), (uint32_t)(&PioneerGraphicsBuffer[targetPosition]), targetSize);
	    dmaCounter0x48[i]++;
	    posCounter += 20;
	    break;
	  case 0x43:
	    dmaCounter0x43++;
	    posCounter += 5;
	    break;
	  default:
	    posCounter += 1;
	    break;
	}
      }
  }
  /* Enable interrupts back only if they were enabled before we disable it here in this function */
  if (!prim) {
      __enable_irq();
  }
  if (!foundData) {
      deh7800_spi_errorCounter++;
      HAL_SPI_DeInit(&hspi2);
      HAL_SPI_Init(&hspi2);
  }
  deh7800_startSpiDMA();
}

extern unsigned int dmaCounter;
void deh7800_printDebugOverview(void) {
  char str[100];
  sprintf(str, "%d 0x47: %u/%u/%u/%u 0x48: %u/%u/%u/%u", dmaCounter, dmaCounter0x47[0], dmaCounter0x47[1], dmaCounter0x47[2], dmaCounter0x47[3],
	  dmaCounter0x48[0], dmaCounter0x48[1], dmaCounter0x48[2], dmaCounter0x48[3]);
  LCD_drawString (10, 10, str, RGB(255,255,255), RGB(0,0,0), FONT_SMALL);


  sprintf(str, "0x43: %u err: %d 0x%x", dmaCounter0x43, deh7800_spi_errorCounter, deh7800_spi_errorHeader);
  LCD_drawString (10, 30, str, RGB(255,255,255), RGB(0,0,0), FONT_SMALL);

  switch(status0x47) {
    case HAL_OK:
      sprintf(str,"0x47 DMA: HAL_OK");
      break;
    case HAL_ERROR:
      sprintf(str,"0x47 DMA: HAL_ERROR");
      break;
    case HAL_BUSY:
      sprintf(str,"0x47 DMA: HAL_BUSY");
      break;
    case HAL_TIMEOUT:
      sprintf(str,"0x47 DMA: HAL_TIMEOUT");
      break;
  }
  LCD_drawString (10, 20, str, RGB(255,255,255), RGB(0,0,0), FONT_SMALL);

  switch(status0x48) {
    case HAL_OK:
      sprintf(str,"0x48 DMA: HAL_OK");
      break;
    case HAL_ERROR:
      sprintf(str,"0x48 DMA: HAL_ERROR");
      break;
    case HAL_BUSY:
      sprintf(str,"0x48 DMA: HAL_BUSY");
      break;
    case HAL_TIMEOUT:
      sprintf(str,"0x48 DMA: HAL_TIMEOUT");
      break;
  }
  LCD_drawString (200, 20, str, RGB(255,255,255), RGB(0,0,0), FONT_SMALL);
}
