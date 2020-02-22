/*
 * Purpose is to Run an YDLidar X2 Lidar using the serial pin and a relay through an Arduino
 * This closes relay, gathers data from the lidar, opens the relay 
 * then sorts then maps the data recieved.
 * This is then repeated as necessary
 * 
 * THE LIDAR GIVES SECTORS OF 16 DEGREES
 */

//const int DATASIZE = 100;
int i = 0;
int Relay = 3; //Digital pin 7 for the relay
//int array[360][2];

void intangles(int x); // Intermediate Angle Function Definition
int angle_count(int x); // Angle Function Definition
int math(int a, int b);

//STARTING SIGNAL IS 7 BYTES

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //Create a connection with the relay module
  pinMode(Relay, OUTPUT);

}


///////////////////////////////////////////////////////////////////////////////
//The Following Loop will only be used to gather data via serial through the Lidar
//To Gather the data Comment out the Second Loop() function and uncomment this one
//////////////////////////////////////////////////////////////////////////////

/*
void loop() {
  // put your main code here, to run repeatedly:  
 //  Serial.read();
 if(Serial.available())
 { 
  for(i; i < DATASIZE; i++)
  {
    Serial.println(Serial.read()); //This lines displays the Decimal Values from the lidar
    //Serial.println(Serial.read(), HEX);  //USE THIS LINE TO FIND THE STARTING VALUES
  }
 }
}
*/
////////////////////////////////////////////////////////
/*
 * Sorting Function will consist of 
 * "INPUT_SIZE" amount of Bytes, with the 
 * first 2 Bytes denoting the beginning of the scan command (0xA55A)
 * Then find the First Packet head 0x55AA, 
 * then stepping through to gather the distant points from the point cloud
 */
////////////////////////////////////////////////////////
/* To use this loop() coment out the top loop and remove the comment here
 * ----------------------------------------------------------------------------
 */
//max size for "input is 750" due to memory size of UNO, due to change because of addition of new functions

bool processing = 0;

void loop() {
  const int INPUT_SIZE = 1000; //Amount of bytes that will be sorted each time the relay is triggered
  int storage[INPUT_SIZE];
  int counting = 0;
  while (!processing)
  {
    digitalWrite(Relay, HIGH);
    //Trigger the Relay Remove the serial availabe, once relay is triggered start storing data
    if(Serial.available() )
    {
     for(int j = 0; j < INPUT_SIZE; j++)
     {
       if(Serial.read() != -1)
       {
        counting++;
        storage[j] = Serial.read();
       }
     }
     for(int j = 0; j < (INPUT_SIZE-8); j++)
     {
       if((storage[j] == 0xA5) && (storage[j+1] == 0x5A) && (storage[j+7] == (0x81)))
       {
         processing = 1; //Starts doing "Post-processing of current lidar sample set
       }
       else
       {
      //Trigger Relay to restart lidar
          Serial.println(storage[0], HEX);
          Serial.println(storage[1], HEX);
       }
     }
    }
  }
  while(processing)
  {
    digitalWrite(Relay, LOW);
    int sample_rate = 0;
    int Start_ang = 0;
    int Start_Pos = 0;
    int End_ang = 0;
    int End_Pos = 0;
    int count = 0; //used for the actual sorting
    int sorting[counting][2];//used to store the distances assigned to there angle
    int angle[counting];//used to store intermediate angles
    for(int j=8; j<counting; j++) //J is 8 because in the first 7 Bytes only the first 2 are important
    {
      if(storage[j] == 0xAA && storage[j+1] == 0x55)
      {
        j+=3; //walks to the LSN
        sample_rate=storage[j];
        j++; //walks to the FSA
        Start_ang = math(storage[j], storage[j+1]);
        Start_ang = angle_calc(Start_ang);
        j+=2;//walks to the LSA
        End_ang = math(storage[j], storage[j+1]);
        End_ang = angle_calc(End_ang);
        j+=4;//walks to the first sample data
        Start_Pos = j;
        End_Pos = j+sample_rate;
        for(int i=2; i<sample_rate; i++) // Does intermediate angle calculations
        {
          angle[i] = (((End_ang-Start_ang)/(sample_rate-1))*(i-1)+Start_ang);
        }
      }
      for(count; count < ((End_Pos - Start_Pos)+count); (count+=2))
      {
        sorting[((count)/2)][0] = math(storage[count+Start_Pos], storage[count+Start_Pos+1]);
        sorting[((count)/2)][1] = angle[((count)/2)];
      }
      j=j+count;
    }
  }
}


///////////////////////////////////////////////////////////////////////////////
//Function used to make the "intermediate Angles"
//Pre: sample_rate, a global variable; array of int called angle of sample size
//Post: Returns nothing but fills in array with intermediate angles
//Future: Probably will merge into main loop() function, Need to inclu
//////////////////////////////////////////////////////////////////////////////
//void intangles(int x)
//{

  //return;
//}


///////////////////////////////////////////////////////////////////////////////
//Function used to calculate the start and end angles
//Pre: Requires an angle after calculated from the "math function"
//Post: Returns the angle post calculations 
//Future: will remain as is
//////////////////////////////////////////////////////////////////////////////
int angle_calc(int x)
{
  int theta = 0;
  theta = x >> 1;
  theta = theta/64;
  return theta;
}



///////////////////////////////////////////////////////////////////////////////
//Function used to place 2 Bytes in MSB, LSB order
//Pre: 2 Bytes recieved from the lidar
//Post: Returns a single int after being the bytes have been merged
//Future: No changes seen to be neccessary
//////////////////////////////////////////////////////////////////////////////
int math(int a, int b)
{
  int storage = b;
  Serial.println(b, BIN);
  storage= b<<8;
  Serial.println(storage, BIN);
  storage = storage|a;
  //Serial.println(value, BIN);
  return storage;
}