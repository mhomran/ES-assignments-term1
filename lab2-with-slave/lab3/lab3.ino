#include <Arduino.h>
#include "XantoI2C.h"

#define SLAVE_SCL_PIN 2
#define SLAVE_SDA_PIN 3
#define SLAVE_ADDR 0x04

#define WRITE_DATA 0
#define MASTER_PIN_SCL A1
#define MASTER_PIN_SDA A0

#define SLAVE_BUF_SIZE 50

#define FRAME_MAX 30

XantoI2C i2c(MASTER_PIN_SCL, MASTER_PIN_SDA, 20);

typedef enum
{
  WAIT_FOR_START,
  RECEIVE_ADDRESS,
  RECEIVE_DATA,
  SEND_DATA
} SlaveState_t;

typedef enum
{
  SIGNAL_SDA,
  SIGNAL_SCL
} Signal_t;

void SDA_Handler(void);
void SCL_Handler(void);
void SM_Update(Signal_t);
void sendSomeCommand(void);
int ReadByte(void);

void setup()
{
  //SDA interrupt on both edges
  attachInterrupt(digitalPinToInterrupt(SLAVE_SDA_PIN), SDA_Handler, CHANGE);
  pinMode(SLAVE_SDA_PIN, INPUT);

  //SCL interrupt on rising edge
  attachInterrupt(digitalPinToInterrupt(SLAVE_SCL_PIN), SCL_Handler, CHANGE);
  pinMode(SLAVE_SCL_PIN, INPUT);

  Serial.begin(9600);
  Serial.print("Slave Address: ");
  Serial.println(SLAVE_ADDR);

  pinMode(13, OUTPUT);
}

void loop()
{
  sendSomeCommand();
}

void SDA_Handler(void)
{
  SM_Update(SIGNAL_SDA);
}
void SCL_Handler(void)
{
  SM_Update(SIGNAL_SCL);
}

void SM_Update(Signal_t src)
{
  static SlaveState_t state = WAIT_FOR_START;
  static uint8_t bitCount = 0;
  static uint8_t recvByte;

  static uint8_t recv[SLAVE_BUF_SIZE];
  static uint8_t recv_idx = 0;

  uint8_t sdaLevel = digitalRead(SLAVE_SDA_PIN);
  uint8_t sclLevel = digitalRead(SLAVE_SCL_PIN);

  switch (state)
  {
  case WAIT_FOR_START:
    bitCount = 0;
    recv_idx = 0;
    
    if (src == SIGNAL_SDA)
    {
      //Start condition satisfied
      if (sclLevel == HIGH && sdaLevel == LOW)
      {
        state = RECEIVE_ADDRESS;
      }
    }
    break;
  case RECEIVE_ADDRESS:
    if (src == SIGNAL_SDA)
    {
      //Stop condition satisfied
      if (sclLevel == HIGH && sdaLevel == HIGH)
      {
        state = WAIT_FOR_START;
      }
    }
    else
    {
      //Rising edge on clk
      if (sclLevel == HIGH)
      {
        if (bitCount < 8)
        {
          recvByte = recvByte << 1 | sdaLevel;
          bitCount++;
        }
        else if (bitCount == 8)
        {
          //Indicating ACK sent
          bitCount++;
        }
      }
      //Falling edge on clk
      else
      {
        if (bitCount == 8)
        {
          if ((recvByte >> 1) == SLAVE_ADDR && (recvByte & 1) == 0)
          {
            state = RECEIVE_DATA;
            digitalWrite(SLAVE_SDA_PIN, LOW);
            pinMode(SLAVE_SDA_PIN, OUTPUT);
            bitCount++;
          }
          else
          {
            state = WAIT_FOR_START;
            bitCount = 0;
          }
        }
        else if (bitCount == 9)
        {
          pinMode(SLAVE_SDA_PIN, INPUT);
          bitCount = 0;
        }
      }
    }
    break;
  case RECEIVE_DATA:
    if (src == SIGNAL_SDA)
    {
      //Stop condition satisfied
      if (sclLevel == HIGH && sdaLevel == HIGH)
      { 
        state = WAIT_FOR_START;

        Serial.println("received:");
        for (uint8_t i = 0; i < recv_idx; i++)
        {
          Serial.print("0x");
          Serial.print(recv[i], HEX);
          Serial.print(' ');
        }
        Serial.println(' ');

        recv_idx = 0;
      }
    }
    else
    {
      //Rising edge on clk
      if (sclLevel == HIGH)
      {
        if (bitCount < 8)
        {
          recvByte = recvByte << 1 | sdaLevel;
          bitCount++;
        }
        else if (bitCount == 8)
        {
          //Indicating ACK sent
          bitCount++;
        }
      }
      //Falling edge on clk
      else
      {
        if (bitCount == 8)
        {
          digitalWrite(SLAVE_SDA_PIN, LOW);
          pinMode(SLAVE_SDA_PIN, OUTPUT);

          recv[recv_idx] = recvByte;
          recv_idx++;
        }
        else if (bitCount == 9)
        {
          pinMode(SLAVE_SDA_PIN, INPUT);
          bitCount = 0;
        }
      }
    }

    break;
  default:
    //Send data from slave to master
    break;
  }
}

void sendSomeCommand(void)
{
  uint8_t N, B;
  uint8_t Bytes[FRAME_MAX];

  Serial.println("Hello! enter number of bytes to be sent:");
  N = ReadByte();

  if (N < 1)
  {
    N = 1;
  }
  else if (N > FRAME_MAX)
  {
    N = FRAME_MAX;
  }
  Serial.print("number of bytes is 0x");
  Serial.println(N, HEX);
  Serial.println("Please write the bytes to be sent:");
    
  for (int i = 0; i < N; i++)
  {
    B = ReadByte();
    Bytes[i] = B;
    Serial.print("0x");
    Serial.println(B, HEX);
  }

  bool ack;
  i2c.start();
  ack = i2c.writeByte((SLAVE_ADDR << 1) | WRITE_DATA);
  for (int i = 0; i < N; i++)
  {
    if (!ack)
      break;
    ack = i2c.writeByte(Bytes[i]);
  }
  i2c.stop();
}

int ReadByte(void)
{
  while (Serial.available() == 0);
  return Serial.read();
}
