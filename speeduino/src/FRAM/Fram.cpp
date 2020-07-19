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
//
//  Mod by Vitor_Boss on 01/2019
//    work with STM32
//    added option to use any SPI port
//    added software version of SPI with configurable speed

#include "Fram.h"
#ifdef SPI_HAS_TRANSACTION
  SPISettings FRAMSettings(FRAM_DEFAULT_CLOCK, MSBFIRST, SPI_MODE0);
#endif

/*-----------------------------------------------------------------------------*/

FramClass::FramClass()
{
  clkPin = mosiPin = misoPin = 255;
  csPin = FRAM_DEFAULT_CS_PIN;
  setClock(FRAM_DEFAULT_CLOCK);
  csPinInit();
}

/*-----------------------------------------------------------------------------*/

FramClass::FramClass (uint8_t ssel, SPIClass &_spi)
{
  clkPin = mosiPin = misoPin = 255;
  begin(ssel, _spi);
}

/*-----------------------------------------------------------------------------*/

FramClass::FramClass (uint8_t mosi, uint8_t miso, uint8_t sclk, uint8_t ssel, uint32_t clockspeed)
{
  csPin = ssel;
  clkPin = sclk;
  misoPin = miso;
  mosiPin = mosi;
  setClock(clockspeed);
  csPinInit();

  if (clkPin != 255)
  {
    pinMode(clkPin, OUTPUT);
    clkPort = portOutputRegister(digitalPinToPort(clkPin));
    clkMask = digitalPinToBitMask(clkPin);
  }
  if (mosiPin != 255)
  {
    pinMode(mosiPin, OUTPUT);
    mosiPort = portOutputRegister(digitalPinToPort(mosiPin));
    mosiMask = digitalPinToBitMask(mosiPin);
  }
  // Set CS pin HIGH and configure it as an output
  pinMode(csPin, OUTPUT);
  pinMode(misoPin, INPUT_PULLUP);
  deassertCS;
}

/*-----------------------------------------------------------------------------*/

void FramClass::EnableWrite (uint8_t state)
{
  assertCS;
  if (state){ Send(FRAM_CMD_WREN); }
  else { Send(FRAM_CMD_WRDI); }
  deassertCS;
}

/*-----------------------------------------------------------------------------*/

void FramClass::setClock(uint32_t clockSpeed) {
  spiSpeed = 1000000 / (clockSpeed * 2);
  #ifdef SPI_HAS_TRANSACTION
  FRAMSettings = SPISettings(clockSpeed, MSBFIRST, SPI_MODE0);
  #if defined(ARDUINO_ARCH_STM32)
  spi->beginTransaction(csPin, FRAMSettings);
  #else
  spi->beginTransaction(FRAMSettings);
  #endif
  #endif
}

/*-----------------------------------------------------------------------------*/

uint8_t FramClass::isDeviceActive() {
  uint8_t result;
  EnableWrite(1); //Best way of detecting a device
  char SR = readSR();
  result = (SR!=0) && (SR!=255);
  EnableWrite(0);
  return result;
}

/*-----------------------------------------------------------------------------*/

void FramClass::begin (uint8_t ssel, SPIClass &_spi)
{
  clkPin = mosiPin = misoPin = 255;
  csPin = ssel;
  spi = &_spi;
  
  // Set CS pin HIGH and configure it as an output
  csPinInit();
  deassertCS;
  #ifdef SPI_HAS_TRANSACTION
  spi->beginTransaction(FRAMSettings);
  #else
  #if defined(STM32F2)
    spi->setClockDivider (SPI_CLOCK_DIV4); // SPI @ 15MHz
  #elif defined(STM32F4)
    spi->setClockDivider (SPI_CLOCK_DIV16);
  #else
    spi->setClockDivider (SPI_CLOCK_DIV2); // 8 MHz
  #endif
    spi->setDataMode(SPI_MODE0);
    spi->begin();
  #endif
  delayMicroseconds(15);//>3us
}

/*-----------------------------------------------------------------------------*/

uint8_t FramClass::write (uint16_t addr, uint8_t data)
{
  EnableWrite(1);
  assertCS;
  Send(FRAM_CMD_WRITE);
  Send16(addr);
  Send(data);
  deassertCS;
  #if defined(ARDUINO_ARCH_STM32)
  delayMicroseconds(5);
  #else
  SOFT_DELAY(5);
  #endif
  EnableWrite(0);

  return 0U;
}

/*-----------------------------------------------------------------------------*/

uint8_t FramClass::write (uint16_t addr, uint8_t *data, uint16_t count)
{
  if (addr + count > FRAM_SIZE)
    return 1U;

  if (count == 0U)
    return 255;

  EnableWrite(1);
  assertCS;
  Send(FRAM_CMD_WRITE);
  Send16(addr);
  for (uint16_t i = 0; i < count; ++i)
    Send(data[i]);
  deassertCS;
  #if defined(ARDUINO_ARCH_STM32)
  delayMicroseconds(5);
  #else
  SOFT_DELAY(5);
  #endif
  EnableWrite(0);

  return 0U;
}

/*-----------------------------------------------------------------------------*/

uint8_t FramClass::read (uint16_t addr, uint8_t *dataBuffer, uint16_t count)
{
  if (addr + count > FRAM_SIZE)
    return 1U;

  if (count == 0U)
    return 255;

  assertCS;
  Send(FRAM_CMD_READ);
  Send16(addr);
  for (uint16_t i=0; i < count; ++i)
    dataBuffer[i] = Send(DUMMYBYTE);
  deassertCS;

  return 0U;
}

/*-----------------------------------------------------------------------------*/

uint8_t FramClass::read (uint16_t addr)
{
  uint8_t dataBuffer;

  assertCS;
  Send(FRAM_CMD_READ);
  Send16(addr);
  dataBuffer = Send(DUMMYBYTE);
  deassertCS;

  return dataBuffer;
}

/*-----------------------------------------------------------------------------*/

uint8_t FramClass::update (uint16_t addr, uint8_t data)
{
  if(read(addr) != data)
    write(addr, data);
  return 1U;
}

/*-----------------------------------------------------------------------------*/

uint8_t FramClass::readSR ()
{
  uint8_t dataBuffer;

  assertCS;
  Send(FRAM_CMD_RDSR);
  dataBuffer = Send(DUMMYBYTE);
  deassertCS;

  return dataBuffer;
}

/*-----------------------------------------------------------------------------*/

uint8_t FramClass::Send(uint8_t data) 
{
  uint8_t reply = 0;
  if(clkPin != 255)
  {
    for (int i=7; i>=0; i--)
    {
      reply <<= 1;
      setClockPin(LOW);
      FastWrite(mosiPort, mosiMask, (data & ((uint8_t)1<<i)));
      setClockPin(HIGH);
      reply |= digitalRead(misoPin);
    }
  }
#if defined(ARDUINO_ARCH_STM32)
  else { reply = spi->transfer(csPin, data, SPI_CONTINUE); }
#else
  else { reply = spi->transfer(data); }
#endif
  return reply;
}

/*-----------------------------------------------------------------------------*/

uint16_t FramClass::Send16(uint16_t data) 
{
  uint16_t reply = 0;
  if(clkPin != 255)
  {
    for (int i=15; i>=0; i--)
    {
      reply <<= 1;
      setClockPin(LOW);
      FastWrite(mosiPort, mosiMask, (data & ((uint16_t)1<<i)));
      setClockPin(HIGH);
      reply |= digitalRead(misoPin);
    }
  }
#if defined(ARDUINO_ARCH_STM32)
  else { reply = spi->transfer16(csPin, data, SPI_CONTINUE); }
#else
  else { reply = spi->transfer16(data); }
#endif
  return reply;
}

/*-----------------------------------------------------------------------------*/


//FramClass Fram;

