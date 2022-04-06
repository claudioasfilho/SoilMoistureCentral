/***************************************************************************//**
 * @file main.c
 * @brief This project demonstrates DMA-driven use of the USART in synchronous
 * (SPI) master mode. The main loop starts the LDMA channels, which transmit the
 * specified number of bytes and receive the byte that is shifted in with each
 * outgoing one.
 *
 * The pins used in this example are defined below and are described in the
 * accompanying readme.txt file.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 *******************************************************************************
 * # Evaluation Quality
 * This code has been minimally tested to ensure that it builds and is suitable 
 * as a demonstration for evaluation purposes only. This code will be maintained
 * at the sole discretion of Silicon Labs.
 ******************************************************************************/

#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_ldma.h"
#include "em_usart.h"
#include "LedStripSPI.h"

#include <stdio.h>
#include <string.h>



// LDMA descriptor and transfer configuration structures for USART TX channel
LDMA_Descriptor_t ldmaTXDescriptor;
LDMA_TransferCfg_t ldmaTXConfig;



// Size of the data buffers
//#define BUFLEN  24+100



// Outgoing data
uint8_t outbuf[BUFLEN];

// Incoming data
uint8_t inbuf[BUFLEN];

// Data reception complete
bool rx_done;

//Led Logic - GRB - 8 bits for each color


//static const uint8_t ColorGreen[ONE_LED_BUFFER_SIZE] =
//    {LOGIC_1, LOGIC_1,LOGIC_1,LOGIC_1,LOGIC_1,LOGIC_1,LOGIC_1,LOGIC_1,LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0};
//
//
//static const uint8_t ColorRed[ONE_LED_BUFFER_SIZE] =
//    {LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0,LOGIC_1, LOGIC_1,LOGIC_1,LOGIC_1,LOGIC_1,LOGIC_1,LOGIC_1,LOGIC_1,LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0};
//
//
//static const uint8_t ColorBlue[ONE_LED_BUFFER_SIZE] =
//    {LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0,LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0,LOGIC_1, LOGIC_1,LOGIC_1,LOGIC_1,LOGIC_1,LOGIC_1,LOGIC_1,LOGIC_1};
//
//static const uint8_t ColorWhite[ONE_LED_BUFFER_SIZE] =
//    {LOGIC_1, LOGIC_1,LOGIC_1,LOGIC_1,LOGIC_1,LOGIC_1,LOGIC_1,LOGIC_1,LOGIC_1, LOGIC_1,LOGIC_1,LOGIC_1,LOGIC_1,LOGIC_1,LOGIC_1,LOGIC_1,LOGIC_1, LOGIC_1,LOGIC_1,LOGIC_1,LOGIC_1,LOGIC_1,LOGIC_1,LOGIC_1};
//

uint8_t CustomColor[ONE_LED_BUFFER_SIZE] =
    {LOGIC_0, LOGIC_0,LOGIC_0,LOGIC_0,LOGIC_0,LOGIC_0,LOGIC_0,LOGIC_1,LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0, LOGIC_0};

uint8_t LogicReset[RESET_LOGIC_BUFFER_SIZE];




/**************************************************************************//**
 * @brief
 *    GPIO initialization
 *****************************************************************************/
void initGpio(void)
{
  // Enable clock (not needed on xG21)
  CMU_ClockEnable(cmuClock_GPIO, true);

  // Configure RX pin as an input
  GPIO_PinModeSet(US1MISO_PORT, US1MISO_PIN, gpioModeInput, 0);

  // Configure TX pin as an output
  GPIO_PinModeSet(US1MOSI_PORT, US1MOSI_PIN, gpioModePushPull, 0);

  // Configure CLK pin as an output low (CPOL = 0)
  GPIO_PinModeSet(US1CLK_PORT, US1CLK_PIN, gpioModePushPull, 0);

  // Configure CS pin as an output and drive inactive high
  GPIO_PinModeSet(US1CS_PORT, US1CS_PIN, gpioModePushPull, 1);
}

/**************************************************************************//**
 * @brief
 *    USART1 initialization
 *****************************************************************************/
void initUsart1(void)
{
  // Enable clock (not needed on xG21)
  CMU_ClockEnable(cmuClock_USART1, true);

  // Default asynchronous initializer (master mode, 1 Mbps, 8-bit data)
  USART_InitSync_TypeDef init = USART_INITSYNC_DEFAULT;

  init.baudrate = 9000000;//6666666;
  //init.clockMode =

  init.msbf = true;           // MSB first transmission for SPI compatibility
  //init.autoCsEnable = true;   // Allow the USART to assert CS
  //init.autoCsSetup = 4;       // Insert 7 bit times of CS setup delay
  //init.autoCsHold = 4;        // Insert 7 bit times of CS hold delay

  /*
   * Route USART1 RX, TX, and CLK to the specified pins.  Note that CS is
   * not controlled by USART1 so there is no write to the corresponding
   * USARTROUTE register to do this.
   */
  GPIO->USARTROUTE[1].RXROUTE = (US1MISO_PORT << _GPIO_USART_RXROUTE_PORT_SHIFT)
      | (US1MISO_PIN << _GPIO_USART_RXROUTE_PIN_SHIFT);
  GPIO->USARTROUTE[1].TXROUTE = (US1MOSI_PORT << _GPIO_USART_TXROUTE_PORT_SHIFT)
      | (US1MOSI_PIN << _GPIO_USART_TXROUTE_PIN_SHIFT);
  GPIO->USARTROUTE[1].CLKROUTE = (US1CLK_PORT << _GPIO_USART_CLKROUTE_PORT_SHIFT)
      | (US1CLK_PIN << _GPIO_USART_CLKROUTE_PIN_SHIFT);
  GPIO->USARTROUTE[1].CSROUTE = (US1CS_PORT << _GPIO_USART_CSROUTE_PORT_SHIFT)
      | (US1CS_PIN << _GPIO_USART_CSROUTE_PIN_SHIFT);

  // Enable USART interface pins
  GPIO->USARTROUTE[1].ROUTEEN = GPIO_USART_ROUTEEN_RXPEN |    // MISO
                                GPIO_USART_ROUTEEN_TXPEN |    // MOSI
                                GPIO_USART_ROUTEEN_CLKPEN |
                                GPIO_USART_ROUTEEN_CSPEN;

  // Configure and enable USART1
  USART_InitSync(USART1, &init);
}

/**************************************************************************//**
 * @brief
 *    LDMA initialization
 *****************************************************************************/
void initLdma(void)
{
  // First, initialize the LDMA unit itself
  LDMA_Init_t ldmaInit = LDMA_INIT_DEFAULT;
  ldmaInit.ldmaInitIrqPriority = 0;
  LDMA_Init(&ldmaInit);

  // Source is outbuf, destination is USART1_TXDATA, and length if BUFLEN
  ldmaTXDescriptor = (LDMA_Descriptor_t)LDMA_DESCRIPTOR_SINGLE_M2P_BYTE(outbuf, &(USART1->TXDATA), BUFLEN);
  ldmaTXDescriptor.xfer.blockSize = ldmaCtrlBlockSizeUnit3;


  // Transfer a byte on free space in the USART buffer
  ldmaTXConfig = (LDMA_TransferCfg_t)LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_USART1_TXBL);


}



void NumbertoColor(uint8_t _Red,uint8_t _Green,uint8_t _Blue, uint8_t *ColorArray)
{
  Bits_t Red,Green,Blue;

  Green.fullbyte = _Green;
  Red.fullbyte = _Red;
  Blue.fullbyte = _Blue;

  //Set the Green Tone

    if (Green.bits._b7 ==1) *ColorArray = LOGIC_1;
    else *ColorArray = LOGIC_0;
    ColorArray++;
    if (Green.bits._b6 ==1) *ColorArray = LOGIC_1;
    else *ColorArray = LOGIC_0;
    ColorArray++;
    if (Green.bits._b5 ==1) *ColorArray = LOGIC_1;
    else *ColorArray = LOGIC_0;
    ColorArray++;
    if (Green.bits._b4 ==1) *ColorArray = LOGIC_1;
    else *ColorArray = LOGIC_0;
    ColorArray++;
    if (Green.bits._b3 ==1) *ColorArray = LOGIC_1;
    else *ColorArray = LOGIC_0;
    ColorArray++;
    if (Green.bits._b2 ==1) *ColorArray = LOGIC_1;
    else *ColorArray = LOGIC_0;
    ColorArray++;
    if (Green.bits._b1 ==1) *ColorArray = LOGIC_1;
    else *ColorArray = LOGIC_0;
    ColorArray++;
    if (Green.bits._b0 ==1) *ColorArray = LOGIC_1;
    else *ColorArray = LOGIC_0;
    ColorArray++;

    //Set the Red Tone

    if (Red.bits._b7 ==1) *ColorArray = LOGIC_1;
    else *ColorArray = LOGIC_0;
    ColorArray++;
    if (Red.bits._b6 ==1) *ColorArray = LOGIC_1;
    else *ColorArray = LOGIC_0;
    ColorArray++;
    if (Red.bits._b5 ==1) *ColorArray = LOGIC_1;
    else *ColorArray = LOGIC_0;
    ColorArray++;
    if (Red.bits._b4 ==1) *ColorArray = LOGIC_1;
    else *ColorArray = LOGIC_0;
    ColorArray++;
    if (Red.bits._b3 ==1) *ColorArray = LOGIC_1;
    else *ColorArray = LOGIC_0;
    ColorArray++;
    if (Red.bits._b2 ==1) *ColorArray = LOGIC_1;
    else *ColorArray = LOGIC_0;
    ColorArray++;
    if (Red.bits._b1 ==1) *ColorArray = LOGIC_1;
    else *ColorArray = LOGIC_0;
    ColorArray++;
    if (Red.bits._b0 ==1) *ColorArray = LOGIC_1;
    else *ColorArray = LOGIC_0;
    ColorArray++;

    //Set the Blue Tone

    if (Blue.bits._b7 ==1) *ColorArray = LOGIC_1;
    else *ColorArray = LOGIC_0;
    ColorArray++;
    if (Blue.bits._b6 ==1) *ColorArray = LOGIC_1;
    else *ColorArray = LOGIC_0;
    ColorArray++;
    if (Blue.bits._b5 ==1) *ColorArray = LOGIC_1;
    else *ColorArray = LOGIC_0;
    ColorArray++;
    if (Blue.bits._b4 ==1) *ColorArray = LOGIC_1;
    else *ColorArray = LOGIC_0;
    ColorArray++;
    if (Blue.bits._b3 ==1) *ColorArray = LOGIC_1;
    else *ColorArray = LOGIC_0;
    ColorArray++;
    if (Blue.bits._b2 ==1) *ColorArray = LOGIC_1;
    else *ColorArray = LOGIC_0;
    ColorArray++;
    if (Blue.bits._b1 ==1) *ColorArray = LOGIC_1;
    else *ColorArray = LOGIC_0;
    ColorArray++;
    if (Blue.bits._b0 ==1) *ColorArray = LOGIC_1;
    else *ColorArray = LOGIC_0;
    ColorArray++;




}



void populateMessageBuffer(void)
{
 volatile uint16_t LedCounter;
 volatile uint16_t offset = 0;

  for (LedCounter=0; LedCounter<RESET_LOGIC_BUFFER_SIZE;LedCounter++)
    {
      LogicReset[LedCounter] = 0;
    }

  for (LedCounter=0; LedCounter<NUMBEROFLEDS;LedCounter++)

    {
      memcpy(outbuf+offset, CustomColor, ONE_LED_BUFFER_SIZE * sizeof(uint8_t));
      offset+=ONE_LED_BUFFER_SIZE;
    }

  memcpy(outbuf+offset, LogicReset, RESET_LOGIC_BUFFER_SIZE * sizeof(uint8_t));
}


void initLedStrip(void)
{
  CMU_HFRCODPLLBandSet(cmuHFRCODPLLFreq_80M0Hz);

  // Initialize GPIO and USART1
  initGpio();
  initUsart1();
  initLdma();
}

void SetLedStriptoRGB(uint8_t _Red,uint8_t _Green,uint8_t _Blue)
{

    NumbertoColor(_Red,_Green,_Blue, &CustomColor);

    populateMessageBuffer();

    LDMA_StartTransfer(TX_LDMA_CHANNEL, &ldmaTXConfig, &ldmaTXDescriptor);


}

/**************************************************************************//**
 * @brief LDMA IRQHandler
 *****************************************************************************/
void LDMA_IRQHandler()
{
  uint32_t flags = LDMA_IntGet();



  // Clear the transmit channel's done flag if set
  if (flags & (1 << TX_LDMA_CHANNEL)){
    LDMA_IntClear(1 << TX_LDMA_CHANNEL);
    // tx done
  }

  // Stop in case there was an error
  if (flags & LDMA_IF_ERROR){
    __BKPT(0);
  }
}


