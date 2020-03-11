#include <Wire.h>
#include <Queue.h>

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
  static int num = 0;
  typedef union
  {
    float number;
    uint8_t bytes[4];
  } FLOATUNION_T;


  unsigned int dir = 2;
  float x = 23.84;
  float y = 41.20;

  FLOATUNION_T x_val;
  x_val.number = x;
  
  FLOATUNION_T y_val;
  y_val.number = y;

  FLOATUNION_T dir_val;
  dir_val.number = dir;
    
  int num_idx = num%12;
  if( num_idx >= 0 && num_idx < 4 )
  {
    Serial.print(x_val.bytes[num_idx]);
    Wire.write(x_val.bytes[num_idx]);
    Serial.print(" , ");
  }
  else if ( num_idx >= 4 && num_idx < 8 )
  {
    Serial.print(y_val.bytes[num_idx-4]);
    Wire.write(y_val.bytes[num_idx-4]);
    Serial.print(" , ");
  }
  else if( num_idx >= 8 && num_idx < 12 )
  {
    Serial.print(dir_val.bytes[num_idx-8]);
    Wire.write(dir_val.bytes[num_idx-8]);
    Serial.print(" , ");
  }
  num++;
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
