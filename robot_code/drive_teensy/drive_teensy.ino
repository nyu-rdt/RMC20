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
}

void loop() {
  
  if(Serial1.available() > 0){
     comms = Serial1.read();
//     Serial1.println(comms);
  }
  interpret(comms);
//  delay(15);

  if (writetest) {
    digitalWrite(1, HIGH);
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
//   val = (commandUpdate.substring(2,4)).toInt();

  if((commandUpdate.substring(2,3).toInt())==1){
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
