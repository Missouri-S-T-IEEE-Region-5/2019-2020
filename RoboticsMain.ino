const float maxDistance = 108;
const int fieldX = 72;
const int fieldY = 72;
bool mapData[fieldX][fieldY]; //1 ft x 1 ft squares
int botPos[2];
int binsPos[3][2];/* 0: Trash
                     1: Plastic
                     2: Metal
                  */
int trashPos[2];

void setup() {
  // put your setup code here, to run once:
  
}

void loop() {
  // put your main code here, to run repeatedly:
  //Update mapData and botPos

  //Look for closest trash
  findTrash(botPos, trashPos);

  /*

  if(//TrashFound)
  {
    //Beeline
      //Rotate in the direction of the trash //considers the degrees of rotation of
                                           //the bot when looking over this
      //Roatate so that the brush side of the bot is facing the trash
      //Move distance of the trash
      //update botPos while moving and stop when the bot is at the position of the trash
  
    //pick up and test trash
      /*if pan is not already lowered lower pan
      //position bot over trash
      //use brushes until trash in pan 
        //* there could be a case where the trash doesn't go in the pan
      //Once trash is in pan check for trash type
      //set trash bin 

    //move trash to the set bin
    //determine where the bot and the bin is
      //bot turns around to position itself facing away from the bin 
        //have the bot turn onto a line perpendicular to the bin facing away from the bin
            //If the bot is already on the perpendicular line but facing towards the bin turn around then reverse into the bin
            //If the bot is already facing away from the bin reverse into the line perpendicular to the bins
      //Reverse into the bin and stops at bin
      //update bot position
      

    //dump trash 
      //rise pan over bot until trash is no longer in pan
      //lower pan

   //Find the next cloest trash from the map and repeat

  }
  else
    //Roam
      //Rotate and move the bot to collect more data from the field

  */

}

void findTrash(const int botPosition[], int trashArr[])
{

  int trashPos[2]; 
  float minTrashDistance = maxDistance;
  float trashDistance;
  //is there a way to check the positions closest to the bot first
  for(int x = 0; 0 < fieldX; x++)
  {
    for(int y = 0; 0 < fieldY; y++)
    {
      trashDistance = sqrt(sq(botPos[0] - x) + sq(botPos[1] - y));
      if(mapData[x][y] && minTrashDistance > trashDistance)
      {
        trashArr[0] = x;
        trashArr[1] = y;
      }
    }
  }
  
}
