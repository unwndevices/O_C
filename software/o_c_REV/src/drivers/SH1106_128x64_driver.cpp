/*
*   (C) Copyright 2011, Oli Kraus
*   (C) Copyright 2013, Andrew Kroll (xxxajk)
*   (C) Copyright 2016, Patrick Dowling
*
*   Low-level driver code for SH1106 OLED with spicy DMA transfer.
*   Author: Patrick Dowling (pld@gurkenkiste.com)
*
*   Command sequences adapted from https://github.com/olikraus/u8glib/blob/master/csrc/u8g_dev_ssd1306_128x64.c
*   SPI transfer command adapted from https://github.com/xxxajk/spi4teensy3
*
*   This program is free software: you can redistribute it and/or modify
*   it under the terms of version 3 of the GNU General Public License as
*   published by the Free Software Foundation at http://www.gnu.org/licenses,
*   with Additional Permissions under term 7(b) that the original copyright
*   notice and author attibution must be preserved and under term 7(c) that
*   modified versions be marked as different from the original.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*/


#include <Arduino.h>
#include <SPI.h>
#include "SH1106_128x64_driver.h"
#include "../../OC_gpio.h"
#include "../../OC_options.h"

#define DMA_PAGE_TRANSFER
#ifdef DMA_PAGE_TRANSFER
#include <DMAChannel.h>
static DMAChannel page_dma;
#endif


static uint8_t SH1106_data_start_seq[] = {
// u8g_dev_ssd1306_128x64_data_start
  0x10, /* set upper 4 bit of the col adr to 0 */
  0x02, /* set lower 4 bit of the col adr to 0 */
  0x00  /* 0xb0 | page */  
};

static uint8_t SH1106_init_seq[] = {
// u8g_dev_ssd1306_128x64_adafruit3_init_seq
  0x0ae,          /* display off, sleep mode */
  0x0d5, 0x080,   /* clock divide ratio (0x00=1) and oscillator frequency (0x8) */
  0x0a8, 0x03f,   /* multiplex ratio, duty = 1/32 */

  0x0d3, 0x000,   /* set display offset */
  0x040,          /* start line */

  0x08d, 0x014,   /* [2] charge pump setting (p62): 0x014 enable, 0x010 disable */

  0x020, 0x000,   /* 2012-05-27: page addressing mode */ // PLD: Seems to work in conjuction with lower 4 bits of column data?
  #ifdef FLIP_180
  0x0a0,          /* segment remap a0/a1*/
  0x0c0,          /* c0: scan dir normal, c8: reverse */
  #else
  0x0a1,          /* segment remap a0/a1*/
  0x0c8,          /* c0: scan dir normal, c8: reverse */
  #endif
  0x0da, 0x012,   /* com pin HW config, sequential com pin config (bit 4), disable left/right remap (bit 5) */
  0x081, 0x0cf,   /* [2] set contrast control */
  0x0d9, 0x0f1,   /* [2] pre-charge period 0x022/f1*/
  0x0db, 0x040,   /* vcomh deselect level */
  
  0x02e,        /* 2012-05-27: Deactivate scroll */ 
  0x0a4,        /* output ram to display */
#ifdef INVERT_DISPLAY
  0x0a7,        /* inverted display mode */
#else
  0x0a6,        /* none inverted normal display mode */
#endif
  //0x0af,      /* display on */
};

static uint8_t SH1106_display_on_seq[] = {
  0xaf
};

/*static*/
void SH1106_128x64_Driver::Init() {
  pinMode(OLED_CS, OUTPUT);
  pinMode(OLED_RST, OUTPUT);
  pinMode(OLED_DC, OUTPUT);
  //SPI_init(); 

  // u8g_teensy::U8G_COM_MSG_INIT
  digitalWriteFast(OLED_RST, HIGH);
  delay(1);
  digitalWriteFast(OLED_RST, LOW);
  delay(10);
  digitalWriteFast(OLED_RST, HIGH);

  // u8g_dev_ssd1306_128x64_adafruit3_init_seq
  digitalWriteFast(OLED_CS, OLED_CS_INACTIVE); // U8G_ESC_CS(0),             /* disable chip */
  digitalWriteFast(OLED_DC, LOW); // U8G_ESC_ADR(0),           /* instruction mode */

  digitalWriteFast(OLED_RST, LOW); // U8G_ESC_RST(1),           /* do reset low pulse with (1*16)+2 milliseconds */
  delay(20);
  digitalWriteFast(OLED_RST, HIGH);
  delay(20);

  digitalWriteFast(OLED_CS, OLED_CS_ACTIVE); // U8G_ESC_CS(1),             /* enable chip */

  SPI_send(SH1106_init_seq, sizeof(SH1106_init_seq));

  digitalWriteFast(OLED_CS, OLED_CS_INACTIVE); // U8G_ESC_CS(0),             /* disable chip */

#ifdef DMA_PAGE_TRANSFER
  // todo
  // see https://github.com/manitou48/teensy4/blob/master/spidma.ino
  LPSPI4_CR &= ~LPSPI_CR_MEN;//disable LPSPI:
  LPSPI4_CFGR1 |= LPSPI_CFGR1_NOSTALL; //prevent stall from RX
  LPSPI4_FCR = 0x0; // Fifo Watermark
  LPSPI4_DER = LPSPI_DER_TDDE; //TX DMA Request Enable
  LPSPI4_CR |= LPSPI_CR_MEN; //enable LPSPI:
  page_dma.begin(true); // Allocate the DMA channel first 
  //
  page_dma.destination((volatile uint8_t&) LPSPI4_TDR);
  page_dma.transferSize(1);
  page_dma.transferCount(kPageSize);
  page_dma.disableOnCompletion();
  page_dma.triggerAtHardwareEvent(DMAMUX_SOURCE_LPSPI4_TX);
  page_dma.disable();
#endif

  Clear();
}

/*static*/
void SH1106_128x64_Driver::Flush() {
#ifdef DMA_PAGE_TRANSFER
  // Assume DMA transfer has completed, else we're doomed
  digitalWriteFast(OLED_CS, OLED_CS_INACTIVE); // U8G_ESC_CS(0)
  page_dma.clearComplete();
  page_dma.disable();
  //
  // todo: disable interupts? (was: SPI0_RSER = 0; SPI0_SR = 0xFF0F0000;
  // ???
  // LPSPI4_DER = 0x0 // // DMA no longer doing TX (or RX)
  // LPSPI4_CR = LPSPI_CR_MEN | LPSPI_CR_RRF | LPSPI_CR_RTF;   // actually clear both... (maybe not, see setup, this would never recover)
  // LPSPI4_SR = 0x3F00;  // clear out all of the other status...
#endif
}

static uint8_t empty_page[SH1106_128x64_Driver::kPageSize];

/*static*/
void SH1106_128x64_Driver::Clear() {

  memset(empty_page, 0, sizeof(kPageSize));

  SH1106_data_start_seq[2] = 0xb0 | 0;
  digitalWriteFast(OLED_DC, LOW);
  digitalWriteFast(OLED_CS, OLED_CS_ACTIVE);
  SPI_send(SH1106_data_start_seq, sizeof(SH1106_data_start_seq));
  digitalWriteFast(OLED_DC, HIGH);
  for (size_t p = 0; p < kNumPages; ++p)
    SPI_send(empty_page, kPageSize);
  digitalWriteFast(OLED_CS, OLED_CS_INACTIVE); // U8G_ESC_CS(0)

  digitalWriteFast(OLED_DC, LOW);
  digitalWriteFast(OLED_CS, OLED_CS_ACTIVE);
  SPI_send(SH1106_display_on_seq, sizeof(SH1106_display_on_seq));
  digitalWriteFast(OLED_DC, HIGH);
}

/*static*/
void SH1106_128x64_Driver::SendPage(uint_fast8_t index, const uint8_t *data) {
  SH1106_data_start_seq[2] = 0xb0 | index;

  digitalWriteFast(OLED_DC, LOW); // U8G_ESC_ADR(0),           /* instruction mode */
  digitalWriteFast(OLED_CS, OLED_CS_ACTIVE); // U8G_ESC_CS(1),             /* enable chip */
  SPI_send(SH1106_data_start_seq, sizeof(SH1106_data_start_seq)); // u8g_WriteEscSeqP(u8g, dev, u8g_dev_ssd1306_128x64_data_start);
  digitalWriteFast(OLED_DC, HIGH); // /* data mode */

#ifdef DMA_PAGE_TRANSFER
  //todo
  page_dma.sourceBuffer(data, kPageSize);
  page_dma.enable(); // go
#else
  SPI_send(data, kPageSize);
  digitalWriteFast(OLED_CS, OLED_CS_INACTIVE); // U8G_ESC_CS(0)
#endif
}

void SH1106_128x64_Driver::SPI_send(void *bufr, size_t n) {
  SPI.transfer(&bufr, n); // todo
}

/*static*/
void SH1106_128x64_Driver::AdjustOffset(uint8_t offset) {
  SH1106_data_start_seq[1] = offset; // lower 4 bits of col adr
}
