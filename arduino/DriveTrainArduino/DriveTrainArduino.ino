#include <Wire.h>

void setup()
{
 
  Wire.begin(4);                // join i2c bus with address #4
  Wire.onReceive(receiveEvent); // register event
}

void loop()
{
  delay(100);
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
  String data = "";
 
  while ( Wire.available() > 0 )
  {
    char n=(char)Wire.read();
    if(((int)n)>((int)(' ')))
   data += n; 
  }
  
  if (data == "CN:ALL") 
  {
    
  }

  if (data == "CN:1")
  {
    
  }

  if (data == "CN:2")
  {
    
  }

  if (data == "CN:3")
  {
    
  }

  if (data == "CN:4")
  {

  }

  if (data == "CN:5")
  {

  }

  if (data == "CN:6")
  }

  }


}
