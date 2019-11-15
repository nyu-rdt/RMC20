#include <iostream>
#include <set>

#define SIZE 7
/*
The Heartbeat of Dan, just acity boy. Born and raised in South Detroit.
He took the midnight train going any where
*/
Class HeartBeat{
Private:
	std::set<unsigned long> Dan;
Public:
	void DanTryingToFindLove(unsigned long lover){
		Dan.insert(lover); //when sending love letters Dan keeps track of whom is on the list 
	}
	bool DanGetsLove(unsigned long Thiviya){ //Dan found out one of his love letters was send back
		std::set<unsigned long >::iterator broken = Dan.find(Thiviya);
		if(broken == Dan.end()){
			return false;
		}
		return true;
	}
	bool DanChecker(unsigned long threshhold){//Dan checks on how many people left him on seen/not send back any reponse
		if(Dan.size() > SIZE){ //if he's too big
			Dan.clear();
			return true;		
		}
		
		for(unsigned long pieces: Dan){
			if(pieces)		
		}
		return false;	
	}
}


//Alexandra Lachman al614@nyu.edu 516-640-1294
//For more about this love story ask me!
