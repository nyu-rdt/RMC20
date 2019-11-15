#include "JumpTable.h"

rdt::FunctionTable::FunctionTable()
{
	curr = 0;
}

rdt::FunctionTable::~FunctionTable()
{
}

void rdt::FunctionTable::insert(
        std::vector<unsigned char>(*encoder)(std::vector<bool>),
		void(*decoder)(std::vector<unsigned char>),
		void(*setup)(),
		std::vector<Keyboard> keyboard,
		unsigned int num_bytes
	)
{
    insert(Functions(encoder,decoder,setup,keyboard,num_bytes));
}

//insert function for function table class
void rdt::FunctionTable::insert(Functions f)
{
	//inputs
	/*
	Functions f				- struct containing command functions
	std::vector<char>		- pressed keyboard keys
	uint bytes				- something else i guess
	*/

	functable[curr] = f;
	unsigned int tot = 0;
    for(Keyboard k:f.keyboard){
        tot |= 1 << (int)k;
    }
    funcVector.push_back(std::pair<unsigned char, std::pair<unsigned int, int>>(curr,std::pair<unsigned int, int>(tot,0)));
	curr++;
}

void rdt::FunctionTable::parse(std::vector<unsigned char> keyVal) //you need to implement recv algo
{
    curr = 6;
    while(curr<keyVal.size()){
        Functions& f =functable[keyVal[curr]];
        ++curr;
        if(f.decoder!=nullptr){
            std::vector<unsigned char> data(f.num_bytes);
            for(int i=0;i<f.num_bytes;++i){
                data[i]=keyVal[curr+i];
            }
            curr+=f.num_bytes;
            f.decoder(data);
        }
    }
}

std::vector<unsigned char> rdt::FunctionTable::update(unsigned int compressed, unsigned int ticks)
{
    std::vector<unsigned char> datapack; //need to implement sending algo
    datapack.resize(6);
    memcpy(&datapack[0],&ticks,4);
    datapack[4]=datapack[5]=0;
    for(const std::pair<unsigned char, std::pair<unsigned int, int>>& data: funcVector){
        if(data.second.first & compressed){
            Functions& f =functable[data.first];
            if(f.encoder!=nullptr){
                std::vector<bool> inputs;
                for(Keyboard k: f.keyboard){
                    inputs.push_back(compressed& (1<<(int)k));
                }
                std::vector<unsigned char> out = f.encoder(inputs);
                if(out.size()!= f.num_bytes){
                    printf("wrong number of bytes");
                    continue;
                }
                datapack.push_back(data.first);
                for(unsigned char uc: out){
                    datapack.push_back(uc);
                }
            }
        }
    }

}
