#include <Wire.h>
#include "XantoI2C.h"

#define SLAVE_ADDR 120
#define WRITE_DATA 0

const uint8_t PIN_SCL = 2;
const uint8_t PIN_SDA = 3;

XantoI2C i2c(PIN_SCL, PIN_SDA, 20);

void sendSomeCommand() {
  bool ack;
  i2c.start();
  ack = i2c.writeByte((SLAVE_ADDR << 1) | WRITE_DATA);
  if(ack) ack = i2c.writeByte(0x09);
  if(ack) ack = i2c.writeByte(0xFE);
  if(ack) ack = i2c.writeByte(0x23);
  i2c.stop();
}

void setup() {
  Wire.begin(SLAVE_ADDR);       // join i2c bus with address
  Wire.onReceive(receiveEvent); // register event
  Serial.begin(9600);
  
  sendSomeCommand();
}

void loop() {
}


// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
  while(0 < Wire.available()) // loop through all but the last
  {
    char c = Wire.read(); // receive byte as a character
    Serial.print(c);         // print the character
  }
}
