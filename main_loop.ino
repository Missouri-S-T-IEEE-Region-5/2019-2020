void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  while(start_button)
  {
    //If trash is not detected roam based on pre-determined path
    if(!trash_Detected())
    {
      roam();
    }
    //Once trash is detected beeline to the nearest piece of trash
    beeline();

    //Attempt to spin the trash into the pan and lift to the pan 
    //so that the trash doesn't fall out
    eat();

    //Tests to see if it trash got into the pan 
    if(test_Trash())
    {
      haul_Booty();
      dump();
    }
  }
}
