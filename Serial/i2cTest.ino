#include <Wire.h>
#include<Queue.h>

#define SLAVE_ADDRESS 0x15

byte data_to_echo = 0;

//Led objectLed;
byte ledInfo[3];

int ledPin = 4;
int piCameraPin = 12;
int piPositionPin = 11;
int buttonPin = 2;
DataQueue<byte> queue(12);


void receiveData(int bytecount)
{
  const int NUM_BITS_IN_BYTE = 8;   /* number of bits in a byte lol*/
  int cur_bool_val_idx = 0;        /* counter for bool val array  */
  bool val[72];                      /* translated bool array vals  */
  byte byte_arr_val;               /* value of read in byte arr   */

  byte_arr_val = Wire.read();
  
  //for each byte in the byte array, run through
  for( int i = 0; i < bytecount; i++ )
  {
    byte_arr_val = Wire.read(); //read in byte of the byte array
    
    for( int i = 0; i < NUM_BITS_IN_BYTE; i++)
    {
      val[cur_bool_val_idx] = (byte_arr_val&&(1<<(7-i)));
      cur_bool_val_idx++;
    }
  }

  for(int i= 0; i < 72; i++)
  {
    Serial.print((uint8_t)val[i]);
  }
}

void sendData()
{
  unsigned int dir = 2;
  float x = 40.3;
  float y = 12.7;

  byte b[12];

  if(queue.item_count() == 0)
  {
    for(int i = 3; i >= 0; i--)
    {
      b[i] = (dir >> (8*i)) & 0xFF;
    }
  
    for(int k = 3; k >= 0; k--)
    {
      b[(k+3)] = (dir >> (8*k)) & 0xFF;
    }
  
    for(int l = 3; l >= 0; l--)
    {
      b[(l+7)] = (dir >> (8*l)) & 0xFF;
    }

    for(int z = 0; z < 12; z++)
    {
      queue.enqueue(b[z]);
    }

    Serial.print(queue.front(), HEX);
    Wire.write(queue.dequeue());
  }
  else
  {
    Serial.print(queue.front(), HEX);
    Wire.write(queue.dequeue());
  }
}

void setup() 
{

  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  pinMode(ledPin, OUTPUT);
  pinMode(piCameraPin, OUTPUT);
  pinMode(piPositionPin, OUTPUT);
  pinMode(buttonPin, INPUT);

  Serial.begin(9600);
}

void loop() 
{
  if(digitalRead(buttonPin) == HIGH)
  {
    digitalWrite(piCameraPin, HIGH);
    digitalWrite(piPositionPin, HIGH);
    
    delay(1000);
  
    digitalWrite(piCameraPin, LOW);
    digitalWrite(piPositionPin, LOW);
  }
}
