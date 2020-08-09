//  Basic read/write functions for the MB85RS64A SPI FRAM chip
//  Copyright (C) 2017  Industruino <connect@industruino.com>
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
//
//  Developed by Claudio Indellicati <bitron.it@gmail.com>
//  Added features by Vitor_Boss <vitor_boss@yahoo.com.br>

#ifndef __FRAM_H__
#define __FRAM_H__

#include <Arduino.h>
#include <SPI.h>

#define FRAM_DEFAULT_CS_PIN ((uint8_t) 16)

#if defined (ARDUINO_ARCH_AVR)
  #define FRAM_DEFAULT_CLOCK       4000000   //value in Hz
#else
  #define FRAM_DEFAULT_CLOCK       15000000  //value in Hz
#endif

#define SOFT_DELAY(x) do{for(uint32_t i=x;i>0;i--) {asm volatile("nop");}}while(0)

// MB85RS64A - 256 K (32 K x 8) bit SPI FRAM
#define FRAM_SIZE ((uint16_t) 0x8000)

#define DUMMYBYTE  0xFE	//dummy bytes to make easier to sniff

#define FRAM_CMD_WREN  0x06	//write enable
#define FRAM_CMD_WRDI  0x04	//write disable
#define FRAM_CMD_RDSR  0x05	//read status reg
#define FRAM_CMD_WRSR  0x01	//write status reg
#define FRAM_CMD_READ  0x03
#define FRAM_CMD_WRITE 0x02
//Not for all devices
#define FRAM_CMD_FSTRD  0x0B	//fast read
#define FRAM_CMD_SLEEP  0xB9	//power down
#define FRAM_CMD_RDID  0x9F	  //read JEDEC ID = Manuf+ID (suggested)
#define FRAM_CMD_SNR  0xC3	  //Reads 8-byte serial number

////////////////////////////////////////////////////////////////////////////////
class FramClass
{
  public:
    FramClass();
    #if defined(ARDUINO_ARCH_STM32)
    FramClass(uint32_t mosi, uint32_t miso, uint32_t sclk, uint32_t ssel = FRAM_DEFAULT_CS_PIN, uint32_t clockspeed = FRAM_DEFAULT_CLOCK);
    FramClass(uint32_t ssel = FRAM_DEFAULT_CS_PIN, SPIClass &_spi = SPI);
    void begin (uint32_t ssel = FRAM_DEFAULT_CS_PIN, SPIClass &_spi = SPI);
    #else
    FramClass(uint8_t mosi, uint8_t miso, uint8_t sclk, uint8_t ssel = FRAM_DEFAULT_CS_PIN, uint32_t clockspeed = FRAM_DEFAULT_CLOCK);
    FramClass(uint8_t ssel = FRAM_DEFAULT_CS_PIN, SPIClass &_spi = SPI);
    void begin (uint8_t ssel = FRAM_DEFAULT_CS_PIN, SPIClass &_spi = SPI);
    #endif
    void EnableWrite (uint8_t state);
    void setClock(uint32_t clockSpeed);
    uint8_t write (uint16_t addr, uint8_t *data, uint16_t count);
    uint8_t write (uint16_t addr, uint8_t data);
    uint8_t read (uint16_t addr, uint8_t *dataBuffer, uint16_t count);
    uint8_t read (uint16_t addr);
    uint8_t update (uint16_t addr, uint8_t data);
    uint8_t readSR ();
    uint8_t isDeviceActive();

    template< typename T > T &get( int idx, T &t )
    {
      uint16_t e = idx;
      uint8_t *ptr = (uint8_t*) &t;
      for( int count = sizeof(T) ; count ; --count, ++e )  *ptr++ = read(e);
      return t;
    }
    
    template< typename T > const T &put( int idx, const T &t )
    {
      const uint8_t *ptr = (const uint8_t*) &t;
      uint16_t e = idx;
      for( int count = sizeof(T) ; count ; --count, ++e )  write(e, *ptr++);
      return t;
    }
    
  private:
    #if defined(ARDUINO_ARCH_STM32)
      uint32_t csPin, clkPin, mosiPin, misoPin;
      volatile uint32_t mosiMask, *mosiPort;
      volatile uint32_t clkMask, *clkPort;
      volatile uint32_t csMask, *csPort;
    #else
      uint8_t csPin, clkPin, mosiPin, misoPin;
      volatile uint8_t mosiMask, *mosiPort;
      volatile uint8_t clkMask, *clkPort;
      volatile uint8_t csMask, *csPort;
    #endif
    uint32_t spiSpeed;
    SPIClass *spi;
    #define assertCS   *csPort &= ~(csMask); //FastWrite(csPort, csMask, LOW);
    #define deassertCS *csPort |= (csMask); //FastWrite(csPort, csMask, HIGH);
    uint8_t  Send(uint8_t data);
    uint16_t  Send16(uint16_t data);

    #if defined(ARDUINO_ARCH_STM32)
    inline void FastWrite(volatile uint32_t *port, uint32_t pin, int8_t state)
    #else
    inline void FastWrite(volatile uint8_t *port, uint8_t pin, int8_t state)
    #endif
    {
      if (state == LOW) *port &= ~(pin);
      else *port |= (pin);
    }

    inline void setClockPin(int8_t state)
    {
      FastWrite(clkPort, clkMask, state);
      #if defined(ARDUINO_ARCH_STM32)
        delayMicroseconds(spiSpeed);
      #else
        SOFT_DELAY(spiSpeed);
      #endif
    }

    void csPinInit()
    {
      pinMode(csPin, OUTPUT);
      csPort = portOutputRegister(digitalPinToPort(csPin));
      csMask = digitalPinToBitMask(csPin);
    }
};
////////////////////////////////////////////////////////////////////////////////


#endif   // __FRAM_H__

