#include <iostream>
#include <stack>
#include <queue>

class EchoLocation{
private:
    unsigned short xcoord;        //Question: the data from the robot would be shorts of the xy position,
    unsigned short ycoord;        //so would the variables in the private part be expressed as shorts?
    std::queue<unsigned short> x; //or since I want to store the history of the data recieved in a queue
    std::queue<unsigned short> y; //would the variables in private be queues
public:
    //unwritten optional future functions: change in degrees, velocity

    //is the robot changing in x?
    bool ChangeX(unsigned short x1){ //x1 is the newest x coordinate
        if(x.back() == NULL){
            x.push(x1);
            return true;
        }
        else if (x1 != x.back()){
            x.push(x1);
            return true;
        } else{
            return false;
        }
    }
    //is the robot changing directions in y?
    bool ChangeY(unsigned short y1){
        if(y.back() == NULL){
            y.push(y1);
            return true;
        }
        else if (y1 != y.back()){
            y.push(y1);
            return true;
        } else{
            return false;
        }
    }
    //is the robot responding?
    bool ConnectionEstablished(){
        if (ChangeX(xcoord) || ChangeY(ycoord)){
            return true;
        }else{
            return false;
        }
    }
    //removes all values except for the most recent 5
    void removeOld(){
        if (x.size() > 5){
            for(unsigned int i = 0; i < x.size()-5; i++){
                x.pop();
            }
        }
        if (y.size() > 5){
            for(unsigned int i = 0; i < y.size()-5; i++){
                y.pop();
            }
        }
    }
}
;
