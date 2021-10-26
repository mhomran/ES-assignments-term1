#include "XantoI2C.h"

XantoI2C::XantoI2C(uint8_t clock_pin, uint8_t data_pin, uint16_t delay_time_us) : clock_pin(clock_pin), data_pin(data_pin), delay_time_us(delay_time_us)
{
  sdaHi();
  sclHi();
}

void XantoI2C::i2cDelay()
{
  delayMicroseconds(delay_time_us);
}

void XantoI2C::sclHi()
{
  pinMode(clock_pin, INPUT_PULLUP);
  i2cDelay();
}

void XantoI2C::sdaHi()
{
  pinMode(data_pin, INPUT_PULLUP);
  i2cDelay();
}

void XantoI2C::sclLo()
{
  digitalWrite(clock_pin, LOW);
  pinMode(clock_pin, OUTPUT);
  i2cDelay();
}

void XantoI2C::sdaLo()
{
  digitalWrite(data_pin, LOW);
  pinMode(data_pin, OUTPUT);
  i2cDelay();
}

uint8_t XantoI2C::readAck()
{
  sdaHi();
  return readBit() == 0 ? 0 : 1;
}

// TODO : issue a start signal
void XantoI2C::start()
{
  // Prepare the bus
  sclHi();
  sdaHi();
  i2cDelay();
  // Generate the START condition
  sdaLo();
  i2cDelay();
  sclLo();
}

// TODO : issue a stop signal
void XantoI2C::stop()
{
  //After ACK
  sdaLo();
  i2cDelay();
  // Generate the Stop condition
  sclHi();
  i2cDelay();
  sdaHi();
}

int XantoI2C::getSda(void)
{
  return digitalRead(data_pin);
}

bool XantoI2C::getAck(void)
{
  // Prepare the bus
  sdaHi();
  // Generate a clock cycle
  sclHi();

  i2cDelay();
  if (getSda() == HIGH)
  {
    sclLo();
    return true; // Error - No ack from Slave
  }
  sclLo();
  return false; // OK - Slave issued ack
}

// TODO : write a byte using SDA and SCL and return ACK
// HINT: you can use bitRead(the number from which to read, which bit to read) to read a bit of a number
bool XantoI2C::writeByte(uint8_t Data)
{
  // Sending data one bit at a time (MS bit first)
  for (uint8_t Bit = 0; Bit < 8; Bit++)
  {
    if (((Data & 0x80) >> 7) == 1)
    {
      sdaHi();
    }
    else
    {
      sdaLo();
    }

    // Generate a clock cycle
    sclHi();
    i2cDelay();
    sclLo();

    // Prepare to send next bit
    Data <<= 1;
  }

  // Make sure the Slave acknowledges
  return (getAck());
}

uint8_t XantoI2C::readByte()
{
  uint8_t out_byte = 0;

  sdaHi();
  for (uint8_t i = 0; i < 8; i++)
  {
    bitWrite(out_byte, 7 - i, readBit());
  }

  return out_byte;
}

uint8_t XantoI2C::readBit()
{
  uint8_t out_bit;

  sclHi();
  out_bit = digitalRead(data_pin);
  sclLo();

  return out_bit;
}
