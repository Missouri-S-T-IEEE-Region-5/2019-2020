void setup() 
{

}

void loop() 
{
  // IEEE Region 5 Robotics Competition 2020 - Lidar Map Generation

  #include <math.h> 
  double sorting[][48] = {{63, 65.51, 69.46, 78.75, 88.3, 103.21, 119.89, 114.72, 110.03, 106.11, 104.14, 102.54, 102, 102.62, 104.96, 109.47, 115.07, 121.9, 130.33, 101.7, 91.94, 87.18, 83.68, 81.78, 81, 81.35, 82.12, 83.49, 29.77, 26.89, 25.32, 24.96, 27.06, 56.3, 48.01, 43.68, 42, 43.12, 46.96, 52.5, 58.35, 66.65, 75.7, 71.83, 68.54, 65.73, 64.13, 63.29},{0, 15.9, 24.9, 36.87, 44.48, 52.38, 58.3, 62.76, 67.98, 74, 78.37, 84.12, 90, 96.29, 103.64, 111.29, 117.57, 123.2, 128.5, 142.79, 151.76, 158.29, 165.47, 172.09, 180, 185.29, 189.46, 194.04, 199.12, 202.99, 207.3, 212.74, 217.94, 228.24, 241.03, 254.05, 270, 283.07, 296.57, 306.87, 313.96, 320.94, 326.3, 331.29, 336.8, 343.43, 349.22, 354.56}};
  double oppo = 0;
  double adj = 0;
  int bot1 = 0;
  int bot2 = 0;
  int countbot = 1;
  int countm = 2;
  int countc = 1;
  double senddata[][10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  double xyresult[][10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int leng = sizeof(sorting[0])/sizeof(sorting[0][0]);
  for (int i = 0; i == leng; i++) // Look at all data and determine Bot side right and left
  {
    if (i>1 && i<leng && ((sorting[1][i]-sorting[1][i+1]))>28.75) // if next instance is greater then by said amount, then its bot wall 1. Said amount is lowest distance between bot wall 1 and greatest distance between scans.
    {
      senddata[1][countbot]=sorting[1][i+1];
      senddata[2][countbot]=sorting[2][i+1];
      countbot = countbot++;
      bot1 = i++;
    }
    if (i>1 && i<leng && ((sorting[1][i]-sorting[1][i-1]))>28.75) // if previous instance is less then by said amount, then its bot wall 2.
    {
      senddata[1][countbot]=sorting[1][i-1];
      senddata[2][countbot]=sorting[2][i-1];
      countbot = countbot++;
      bot2 = i-1;  
    }    
  }
  for (int i = 0; i == leng; i++)
  {
    if (i==1 && (sorting[1][i+1]-sorting[1][i])<3) // Initial value is midpoint 1 if bot resets to nearest 90degs before scan.
    {
      senddata[1][3]=sorting[1][1];
      senddata[2][3]=sorting[2][1];
    }
    if (i>1 && i<leng && sorting[1][i]<sorting[1][i+1] && sorting[1][i]<sorting[1][i-1] && (sorting[1][i]-sorting[1][i+1])<28.75 && (sorting[1][i]-sorting[1][i-1])<28.75 && !(i>=bot1 && i<=bot2)) // If next and previous intervals are greater, then its a midpoint, exclude jumps greater then said amount and intervals in bot range.
    {
      if (sorting[2][i]<180 && sorting[2][i]>0) // Select storage locations
      {
        countm = 4;
      } 
      if (sorting[2][i]<270 && sorting[2][i]>90)
      {
        countm = 5;
      } 
      if (sorting[2][i]<360 && sorting[2][i]>180)
      {
        countm = 6;
      }
      senddata[1][countm]=sorting[1][i];
      senddata[2][countm]=sorting[2][i];
    }
    if (i>1 && i<leng && sorting[1][i]>sorting[1][i+1] && sorting[1][i]>sorting[1][i-1] && (sorting[1][i]-sorting[1][i+1])<28.75 && (sorting[1][i]-sorting[1][i-1])<28.75 && !(i>=bot1 && i<=bot2)) // If next and previous intervals are lesser, then its a corner, exclude jumps greater then said amount and intervals in bot range.
        if (sorting[2][i]<90 && sorting[2][i]>0) // Select Storage Locations
        {
            countc = 7;
        } 
        if (sorting[2][i]<180 && sorting[2][i]>90)
        {
            countc = 8;
        }
        if (sorting[2][i]<270 && sorting[2][i]>180)
        {
            countc = 9;
        }
        if (sorting[2][i]<360 && sorting[2][i]>270)
        {
            countc = 10;
        } 
        senddata[1][countc]=sorting[1][i];
        senddata[2][countc]=sorting[2][i];    
  }

  if (senddata[1][3]==0) // if data is missing, fill in using measurements from nearby cardinal or corners.
  {
    senddata[1][3]=senddata[1][7]*cos(senddata[2][7]*(PI/180));
    senddata[2][3]=0;
  }
  if (senddata[1][4]==0)
  {
    senddata[1][4]=senddata[1][8]*cos((senddata[2][8]-90)*(PI/180));
    senddata[2][4]=90;
  }
  if (senddata[1][5]==0)
  {
    senddata[1][5]=senddata[1][9]*cos((senddata[2][9]-180)*(PI/180));
    senddata[2][5]=180;
  }
  if (senddata[1][6]==0)
  {
    senddata[1][6]=senddata[1][10]*cos((senddata[2][10]-270)*(PI/180));
    senddata[2][6]=270;
  }
  if (senddata[1][7]==0)
  {
    if (!(senddata[1][4]==0))
    {
        oppo=senddata[1][4];
    }
    if (!(senddata[1][3]==0))
    {
        adj=senddata[1][3];
    }
    senddata[2][7]=atan(oppo/adj)*(PI/180);
    senddata[1][7]=oppo/sin(senddata[2][7]*(PI/180));
  }
  if (senddata[1][8]==0)
  {
    if (!(senddata[1][5]==0))
    {
        oppo=senddata[1][5];
    }
    if (!(senddata[1][4]==0))
    {
        adj=senddata[1][4];
    }
    senddata[2][8]=atan(oppo/adj)*(PI/180)+90;
    senddata[1][8]=oppo/sin((senddata[2][8]-90)*(PI/180));
  }
  if (senddata[1][9]==0)
  {
    if (!(senddata[1][6]==0))
    {
        oppo=senddata[1][6];
    }
    if (!(senddata[1][5]==0))
    {
        adj=senddata[1][5];
    }
    senddata[2][9]=atan(oppo/adj)*(PI/180)+180;
    senddata[1][9]=oppo/sin((senddata[2][9]-180)*(PI/180));
  }
  if (senddata[1][10]==0)
  {
    if (!(senddata[1][3]==0))
    {
        oppo=senddata[1][3];
    }
    if (!(senddata[1][6]==0))
    {
        adj=senddata[1][6];
    }
    senddata[2][10]=atan(oppo/adj)*(PI/180)+270;
    senddata[1][10]=oppo/sin((senddata[2][10]-270)*(PI/180));
  }

  int lengthsd = sizeof(senddata[0])/sizeof(senddata[0][0]);
  for (int i = 0; i == lengthsd; i++) // Go through stored points based on what section it is in
  {
    if (senddata[2][7]>=senddata[2][i] && senddata[2][3]<=senddata[2][i]) // Evaluate section 1
    {
        if (i==3 && !(senddata[1][i]==0)) // Convert midpoint 1 if not missing
        {
           xyresult[1][3]=senddata[1][3]+senddata[1][5];
           xyresult[2][3]=senddata[1][6];
        }
        if (i==7 && !(senddata[1][i]==0)) // Convert corner 1 if not missing
        {
           xyresult[1][7]=senddata[1][5]+(senddata[1][7]*cos(senddata[2][7]*(PI/180)));
           xyresult[2][7]=senddata[1][6]+(senddata[1][7]*sin(senddata[2][7]*(PI/180)));
        }
        if (!(i==3) && !(i==7) && !(senddata[1][i]==0)) // If bot wall present convert it if not missing
        {
           xyresult[1][i]=senddata[1][5]+(senddata[1][i]*cos(senddata[2][i]*(PI/180)));
           xyresult[2][i]=senddata[1][6]+(senddata[1][i]*sin(senddata[2][i]*(PI/180)));
        }
    }
    if (senddata[2][4]>=senddata[2][i] && senddata[2][7]<=senddata[2][i]) // Evaluate section 2
    {
        if (i==4 && !(senddata[1][i]==0)) // Convert midpoint 2 if not missing
        {
           xyresult[1][4]=senddata[1][5];
           xyresult[2][4]=senddata[1][4]+senddata[1][6];
        }
        if (!(i==4) && !(i==7) && !(senddata[1][i]==0)) // If bot wall present convert it if not missing
        {
           xyresult[1][i]=senddata[1][5]+(senddata[1][i]*sin((90-senddata[2][i])*(PI/180)));
           xyresult[2][i]=senddata[1][6]+(senddata[1][i]*cos((90-senddata[2][i])*(PI/180)));
        }
    }
   if (senddata[2][8]>=senddata[2][i] && senddata[2][4]<=senddata[2][i]) // Evaluate section 3
   {
        if (i==8 && !(senddata[1][i]==0)) // Convert corner 2 if not missing
        {
           xyresult[1][8]=senddata[1][5]-(senddata[1][8]*sin((senddata[2][8]-90)*(PI/180)));
           xyresult[2][8]=senddata[1][6]+(senddata[1][8]*cos((senddata[2][8]-90)*(PI/180)));
        }
        if (!(i==8) && !(i==4) && !(senddata[1][i]==0)) // If bot wall present convert it if not missing
        {
           xyresult[1][i]=senddata[1][5]-(senddata[1][i]*sin((senddata[2][i]-90)*(PI/180)));
           xyresult[2][i]=senddata[1][6]+(senddata[1][i]*cos((senddata[2][i]-90)*(PI/180)));
        }
   }
   if (senddata[2][5]>=senddata[2][i] && senddata[2][8]<=senddata[2][i]) // Evaluate section 4
   {
        if (i==5 && !(senddata[1][i]==0)) // Convert midpoint 3 if not missing
        {
           xyresult[1][5]=0;
           xyresult[2][5]=senddata[1][6];
        }
        if (!(i==5) && !(i==8) && !(senddata[1][i]==0)) // If bot wall present convert it if not missing
        {
           xyresult[1][i]=senddata[1][5]-(senddata[1][i]*cos((180-senddata[2][i])*(PI/180)));
           xyresult[2][i]=senddata[1][6]+(senddata[1][i]*sin((180-senddata[2][i])*(PI/180)));
        }
   }
   if (senddata[2][9]>=senddata[2][i] && senddata[2][5]<=senddata[2][i]) // Evaluate section 5
   {
        if (i==9 && !(senddata[1][i]==0)) // Convert corner 3 if not missing
        {
           xyresult[1][9]=senddata[1][5]-(senddata[1][9]*cos((senddata[2][9]-180)*(PI/180)));
           xyresult[2][9]=senddata[1][6]-(senddata[1][9]*sin((senddata[2][9]-180)*(PI/180)));
        }
        if (!(i==9) && !(i==5) && !(senddata[1][i]==0)) // If bot wall present convert it if not missing
        {
           xyresult[1][i]=senddata[1][5]-(senddata[1][i]*cos((senddata[2][i]-180)*(PI/180)));
           xyresult[2][i]=senddata[1][6]-(senddata[1][i]*sin((senddata[2][i]-180)*(PI/180)));
        }
   }
   if (senddata[2][6]>=senddata[2][i] && senddata[2][9]<=senddata[2][i]) // Evaluate section 6
   {
        if (!(i==6) && !(i==9) && !(senddata[1][i]==0)) // If bot wall present convert it if not missing
        {
           xyresult[1][i]=senddata[1][5]-(senddata[1][i]*sin((270-senddata[2][i])*(PI/180)));
           xyresult[2][i]=senddata[1][6]-(senddata[1][i]*cos((270-senddata[2][i])*(PI/180)));
        }
   }
   if (senddata[2][10]>=senddata[2][i] && senddata[2][6]<=senddata[2][i]) // Evaluate section 7
   {
        if (i==6 && !(senddata[1][i]==0)) // Convert midpoint 4 if not missing
        {
           xyresult[1][6]=senddata[1][5];
           xyresult[2][6]=0;
        }
        if (!(i==10) && !(i==6) && !(senddata[1][i]==0)) // If bot wall present convert it if not missing
        {
           xyresult[1][i]=senddata[1][5]+(senddata[1][i]*sin((senddata[2][i]-270)*(PI/180)));
           xyresult[2][i]=senddata[1][6]-(senddata[1][i]*cos((senddata[2][i]-270)*(PI/180)));
        }
   }
   if (360>=senddata[2][i] && senddata[2][10]<=senddata[2][i]) // Evaluate section 8
   {
        if (i==10 && !(senddata[1][i]==0)) // Convert corner 4 if not missing
        {
           xyresult[1][10]=senddata[1][5]+(senddata[1][10]*sin((senddata[2][10]-270)*(PI/180)));
           xyresult[2][10]=senddata[1][6]-(senddata[1][10]*cos((senddata[2][10]-270)*(PI/180)));
        }
        if (!(i==10) && !(senddata[1][i]==0)) // If bot wall present convert it if not missing
        {
           xyresult[1][i]=senddata[1][5]+(senddata[1][i]*cos((360-senddata[2][i])*(PI/180)));
           xyresult[2][i]=senddata[1][6]-(senddata[1][i]*sin((360-senddata[2][i])*(PI/180)));
        }
   }
  }

  int lengthxy = sizeof(xyresult[0])/sizeof(xyresult[0][0]);
  for(int i = 0; i <= lengthsd ; i++)
  {
    Serial.println(xyresult[1][i]);
    Serial.println(xyresult[2][i]);
  }
}
