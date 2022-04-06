
#ifndef LEDSTRIPSPI_H
#define LEDSTRIPSPI_H


// Ports and pins for SPI interface
#define US1MISO_PORT  gpioPortA
#define US1MISO_PIN   5
#define US1MOSI_PORT  gpioPortA
#define US1MOSI_PIN   6
#define US1CLK_PORT   gpioPortD
#define US1CLK_PIN    2
#define US1CS_PORT    gpioPortC
#define US1CS_PIN     1

// LDMA channel for receive and transmit servicing
#define TX_LDMA_CHANNEL 4

#define ONE_LED_BUFFER_SIZE 24
#define RESET_LOGIC_BUFFER_SIZE 100//40//51//67
#define NUMBEROFLEDS 2

#define BUFLEN  (NUMBEROFLEDS*ONE_LED_BUFFER_SIZE)+RESET_LOGIC_BUFFER_SIZE

typedef union {

  struct{
    uint8_t _b0:1;
    uint8_t _b1:1;
    uint8_t _b2:1;
    uint8_t _b3:1;
    uint8_t _b4:1;
    uint8_t _b5:1;
    uint8_t _b6:1;
    uint8_t _b7:1;
  }bits;

  uint8_t fullbyte;
}Bits_t;


enum LogicState
    {
    LOGIC_0 = 0xe0,
    LOGIC_1 = 0xfc,//0xf8,
    LOGIC_RESET = 0
    };


/**************************************************************************//**
 * LedStripInit
 *****************************************************************************/
void initLedStrip(void);

void SetLedStriptoRGB(uint8_t _Red,uint8_t _Green,uint8_t _Blue);

#endif // LEDSTRIPSPI_H
