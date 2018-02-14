#ifndef DEHX7800DAB_H
#define DEHX7800DAB_H

#include <stdint.h> // type definitions like uint8_t etc.

uint8_t PioneerGraphicsBuffer[88];
unsigned int deh7800_spi_errorCounter;
unsigned int deh7800_spi_errorHeader;

void deh7800_showDisplay(unsigned int rootX, unsigned int rootY,
			 uint16_t frameColor, uint16_t segmentsFgColor, uint16_t iconsFgColor,
			 uint16_t iconsInactiveColor, uint16_t sideMatrixFgColor, uint16_t infoBarFgColor);


void deh7800_showSegments(unsigned int rootX, unsigned int rootY, uint16_t fg_color);
void deh7800_showSideMatrix(unsigned int rootX, unsigned int rootY, uint16_t fg_color);
void deh7800_showIcons(unsigned int rootX, unsigned int rootY, uint16_t fg_color, uint16_t bg_color, uint16_t inactive_color);
void deh7800_showInfoBar(unsigned int rootX, unsigned int rootY, uint16_t fg_color);

void deh7800_printDataOverview(void);
void deh7800_printDebugOverview(void);

void deh7800_startSpiDMA(void);

// say how many bytes should be read by dma continously
// can be more than on 'dataset'
#define deh_7800_spi_BufferSize 200

#endif // DEHX7800DAB_H
