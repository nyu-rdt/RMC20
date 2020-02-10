#include "JumpTable.h"

rdt::FunctionTable::FunctionTable()
{
	currIndex = 0;
	compressedUsed = 0U;
}

rdt::FunctionTable::~FunctionTable()
{
}

void rdt::FunctionTable::insert(
        unsigned int num_bytes,
        std::vector<Keyboard> keyboard,
        std::vector<char>(*encoder)(const std::vector<bool>&),
		void(*decoder)(const std::vector<char>&),
		void(*setup)(bool),
		void(*cleanup)(bool)
	)
{
    insert(Functions(num_bytes, keyboard, encoder, decoder, setup, cleanup));
}

//insert function for function table class
void rdt::FunctionTable::insert(Functions f)
{
	functable[currIndex] = f;
	unsigned int tot = 0;
    for(Keyboard k:f.keyboard){
        tot |= 1 << (int)k;
    }

    compressedUsed |= tot;

    funcVector.push_back(std::pair<unsigned char, unsigned int>(currIndex, tot));
	++currIndex;
}

void rdt::FunctionTable::setup(bool sender){
    for (const std::pair<unsigned char, unsigned int>& data: funcVector){
        Functions& f = functable[data.first];
        if (f.setup != nullptr){
            f.setup(sender);
        }
    }
}

void rdt::FunctionTable::parse(const std::vector<char>& keyVal, int curr)
{
    while(curr < keyVal.size()){
        Functions& f = functable[keyVal[curr]];
        ++curr;
        if(f.decoder != nullptr){
            std::vector<char> data(f.num_bytes);
            for(int i = curr+f.num_bytes; curr < i; ++curr){
                data[f.num_bytes - i + curr]=keyVal[curr];
            }
            f.decoder(data);
        }
    }
}

void rdt::FunctionTable::encode(unsigned int compressed, unsigned int compressedprev, std::vector<char>& datapack)
{
    //figure out if this is necessary later on
    unsigned int delta = compressedprev ^ compressed;
    std::vector<bool> expanded;
    expanded.resize((int)Keyboard::LAST);
    for(int i=0;i<(int)Keyboard::LAST;++i){
        expanded[i] = (compressed & (1<<i)) > 0;
    }
    for(const std::pair<unsigned char, unsigned int>& data: funcVector){
        if(data.second & compressed || data.second & delta){
            Functions& f = functable[data.first];
            if(f.encoder!=nullptr){
                std::vector<bool> inputs;
                for(Keyboard k: f.keyboard){
                    inputs.push_back(expanded[(int)k]);
                }
                std::vector<char> out = f.encoder(inputs);
                if(out.size()!= f.num_bytes){
                    printf("wrong number of bytes");
                    continue;
                }
                datapack.push_back(data.first);
                for(char uc: out){
                    datapack.push_back(uc);
                }
            }
        }
    }
}

bool rdt::FunctionTable::hasUpdate(unsigned int compressed){
    return (compressedUsed & compressed) > 0;
}
