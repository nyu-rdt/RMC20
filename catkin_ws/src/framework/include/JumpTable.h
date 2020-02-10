#pragma once
#include "Keyboard.h"

#include <vector>
#include <string>
#include <map>

class Manager;
class FunctionTable;
struct Functions;

namespace rdt{
    struct Functions
    {
    public:
        std::vector<char>(*encoder)(const std::vector<bool>&);
        void(*decoder)(const std::vector<char>&);
        void(*setup)(bool);
        void(*cleanup)(bool);
        //keyboard combination for encoding
        std::vector<Keyboard> keyboard;
        //number of bytes used for encoding and decoding (makes life easier for some stuff)
        unsigned int num_bytes;

        Functions():
            num_bytes(0),
            keyboard(),
            encoder(nullptr),
            decoder(nullptr),
            setup(nullptr),
            cleanup(nullptr)
        {};

        Functions(
            unsigned int num_bytes,
            std::vector<Keyboard> keyboard,
            std::vector<char>(*encoder)(const std::vector<bool>&),
            void(*decoder)(const std::vector<char>&),
            void(*setup)(bool) = nullptr,
            void(*cleanup)(bool) = nullptr
        ) :
            num_bytes(num_bytes),
            keyboard(keyboard),
            encoder(encoder),
            decoder(decoder),
            setup(setup),
            cleanup(cleanup)
        {};
    };

    class FunctionTable {
    friend class Manager;
    public:
        FunctionTable();
        ~FunctionTable();

        void insert(
            unsigned int num_bytes,
            std::vector<Keyboard> keyboard,
            std::vector<char>(*encoder)(const std::vector<bool>&),
            void(*decoder)(const std::vector<char>&),
            void(*setup)(bool) = nullptr,
            void(*cleanup)(bool) = nullptr
        );

        void insert(
            Functions f
        );

    private:
        unsigned int compressedUsed;
        unsigned char currIndex;
        //pair of indexes and keyboard inputs
        std::vector<std::pair<unsigned char, unsigned int>> funcVector;
        Functions functable[256];

        void setup(bool sender);

        void parse(
            const std::vector<char>& keyVal,
            int start
        );

        void encode(
            unsigned int compressed,
            unsigned int compressedprev,
            std::vector<char>& header
        );

        bool hasUpdate(unsigned int compressed);
    };
}
