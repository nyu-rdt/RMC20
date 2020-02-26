/* NYU RDT
 * Drive Control code
 * 7    - bit string will be sent from the topic that we the esp is subscribed to
FOR DRIVE
7bits
0 = mode
  0- forwards/backwards
  1- rotating
  2- arm movement
1-3 = speed
  0- forwards/backwards
  1- percentage
  2- percentage
4-7 = degrees
  0-neg/pos
  1-1st dig
  2-2nd dig
  3-3th dig
 */
#include <Servo.h>

Servo Spark;

bool writetest = false;

int val;
int val2;
int comms;

void setup (){
  val = 0;
  Spark.attach(3);
  Serial1.begin(115200);
  Serial.begin(9600);
}

void loop() {
  
  if(Serial1.available() > 0){
     comms = Serial1.read();
     //][----Serial1.println(comms);
  }
  //if(Serial.available() > 0){
  //   val = (Serial.parseInt());
  //}
  interpret(comms);
//  delay(15);

  //if (writetest) {
    //digitalWrite(1, HIGH);
  }
}


void interpret(int& commandUpdate){
  switch(commandUpdate[0].toInt()){
    case '0':
      drive(commandUpdate.substring(1,7));
      break;
    case  '1':
      //keep one side stationary, the other rotates
      rotate(commandUpdate.substring(1,3));
      break;
    case '3':
      //slowly rotate wheels in different directions slicing like a hamburger
      rollUp(commandUpdate.substring(1,3));
      break; 
  }
}
void drive(String commandUpdate){
  //if [0] is 0, go forwards, if 1, go backwards
  //translates degrees into different percentages for the wheels divided like a hotdog
   val = (commandUpdate.substring(1,4)).toInt();

  if((commandUpdate[0].toInt())==1){
        val*=-1;
     }

   val2 = map(val,-100,100,1000,2000);
   Spark.writeMicroseconds(val2);
  
}
void rotate(String comm){
  //rotates based upon speed passed, one side of the robot remains stationary, while the other spins
  //if [0] is 0, go forwards, if 1, go backwards
}
void rollUp(String Comm){
  // arm stuff
}
