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

#define DMA_PAGE_TRANSFER_2
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
U8G_ESC_CS(0),             /* disable chip */
  U8G_ESC_ADR(0),           /* instruction mode */
  U8G_ESC_RST(1),           /* do reset low pulse with (1*16)+2 milliseconds */
  U8G_ESC_CS(1),             /* enable chip */
  
  0xfd,0x12,    /*Command Lock */
  0xae,     /*Set Display Off */
  0xd5,0xa0,    /*set Display Clock Divide Ratio/Oscillator Frequency */
  0xa8,0x3f,    /*Set Multiplex Ratio */
  0x3d,0x00,    /*Set Display Offset*/
  0x40,     /*Set Display Start Line*/
  0xa1,     /*Set Segment Re-Map*/
  0xc8,     /*Set COM Output Scan Direction*/
  0xda,0x12,    /*Set COM Pins Hardware Configuration*/
  0x81,0xdf,    /*Set Current Control */
  0xd9,0x82,    /*Set Pre-Charge Period */
  0xdb,0x34,    /*Set VCOMH Deselect Level */
  0xa4,     /*Set Entire Display On/Off */
  0xa6,     /*Set Normal/Inverse Display*/
  U8G_ESC_VCC(1), /*Power up VCC & Stabilized */
  U8G_ESC_DLY(50),
  0xaf,     /*Set Display On */
  U8G_ESC_DLY(50),
  U8G_ESC_CS(0),             /* disable chip */
  U8G_ESC_END                /* end of sequence */
};

static uint8_t SH1106_display_on_seq[] = {
  0xaf
};


/*static*/
void SH1106_128x64_Driver::Init() {
  pinMode(OLED_CS, OUTPUT);
  pinMode(OLED_RST, OUTPUT);
  pinMode(OLED_DC, OUTPUT);

  SPI.begin();

  delay(20);
  // u8g_teensy::U8G_COM_MSG_INIT
  digitalWriteFast(OLED_RST, HIGH);
  delay(1);
  digitalWriteFast(OLED_RST, LOW);
  delay(10);
  digitalWriteFast(OLED_RST, HIGH);

  // u8g_dev_ssd1306_128x64_adafruit3_init_seq
  digitalWriteFast(OLED_CS, HIGH); // U8G_ESC_CS(0),             /* disable chip */
  digitalWriteFast(OLED_DC, LOW); // U8G_ESC_ADR(0),           /* instruction mode */
  digitalWriteFast(OLED_RST, LOW); // U8G_ESC_RST(1),           /* do reset low pulse with (1*16)+2 milliseconds */
  delay(20);
  digitalWriteFast(OLED_RST, HIGH);
  delay(20);

  SPI_send(SH1106_init_seq,false, sizeof(SH1106_init_seq));
  Clear();
}

void SH1106_128x64_Driver::Flush() {

}

static uint8_t empty_page[SH1106_128x64_Driver::kPageSize];

/*static*/
void SH1106_128x64_Driver::Clear() {

  memset(empty_page, 0, sizeof(kPageSize));

  SH1106_data_start_seq[2] = 0xb0 | 0;

  SPI_send(SH1106_data_start_seq,false, sizeof(SH1106_data_start_seq));
  
  for (size_t p = 0; p < kNumPages; ++p)
    SPI_send(empty_page,true, kPageSize);

  SPI_send(SH1106_display_on_seq,false, sizeof(SH1106_display_on_seq));
}

/*static*/
void SH1106_128x64_Driver::SendPage(uint_fast8_t index, const uint8_t *data) {
  SH1106_data_start_seq[2] = 0xb0 | index;
  SPI_send(SH1106_data_start_seq,false, sizeof(SH1106_data_start_seq)); 
  SPI_send(data,true, kPageSize);
}




void SH1106_128x64_Driver::SPI_send(void *bufr,bool DATAcmd, size_t n) {

uint8_t *buf = (uint8_t *)bufr;

  if(!DATAcmd) digitalWriteFast(OLED_CS, LOW);

   SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE3));

  if(DATAcmd){
    // In SPI mode data is indicated by the state of the _dc line so no control bytes are needed
    digitalWriteFast(OLED_DC, HIGH);
    for(size_t indi = 0; indi < n; indi++){
      digitalWriteFast(OLED_CS, LOW);
      SPI.transfer(*(buf + indi));
      digitalWriteFast(OLED_CS, HIGH);
    }
  }else{
    // In SPI mode commands are indicated by the _dc line state, so no control bytes are needed
    digitalWriteFast(OLED_DC, LOW);
    for(size_t indi = 0; indi < n; indi++){
      digitalWriteFast(OLED_CS, LOW);
      SPI.transfer(*(buf + indi));
      digitalWriteFast(OLED_CS, HIGH);
    }
  }
  SPI.endTransaction();
  digitalWrite(OLED_DC, HIGH);

  if(!DATAcmd) digitalWrite(OLED_CS, HIGH);

}

/*static*/
void SH1106_128x64_Driver::AdjustOffset(uint8_t offset) {
  SH1106_data_start_seq[1] = offset; // lower 4 bits of col adr
}