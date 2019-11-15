#include <vector>
#include "Keyboard.h"
class FunctionTable;
struct Functions;

namespace rdt{
    struct Functions
    {
    public:
        std::vector<unsigned char >(*encoder)(std::vector<bool>);
        void(*decoder)(std::vector<unsigned char>);
        void(*setup)();

        //keyboard combination for encoding
        std::vector<Keyboard> keyboard;
        //number of bytes used for encoding and decoding (makes life easier for some stuff could change it on the next sprint)
        unsigned int num_bytes;

        Functions(
            std::vector<unsigned char>(*encoder)(std::vector<bool>) = nullptr,
            void(*decoder)(std::vector<unsigned char>) = nullptr,
            void(*setup)() = nullptr,
            std::vector<Keyboard> keyboard = {},
            unsigned int num_bytes = 0
        ) :
            encoder(encoder),
            decoder(decoder),
            setup(setup),
            keyboard(keyboard),
            num_bytes(num_bytes)
        {
        };
    };

    class FunctionTable {
    public:
        FunctionTable();
        ~FunctionTable();

        void insert(
            std::vector<unsigned char>(*encoder)(std::vector<bool>) = nullptr,
            void(*decoder)(std::vector<unsigned char>) = nullptr,
            void(*setup)() = nullptr,
            std::vector<Keyboard> keyboard = {},
            unsigned int num_bytes = 0
        );

        void insert(
            Functions f
        );

        void parse(
            std::vector<unsigned char> keyVal
        );

        std::vector<unsigned char> update(
            unsigned int compressed,
            unsigned int ticks
        );


    /*
    need to track active commands
        to deactivate commands if packet not recieved
        kill function to stop all functions
    */
    private:
        //some description
        /*
        use a vector of <key, paired_Keyboard> - implement a vector
            key					identifies index (of what?)
            paired_eyboard		Determines if the key is used?

        */
        //convert keyboard to bits
        //map characters to bits in an integer


        unsigned char curr;
        //funcvector - vector with three values
        std::vector<std::pair<unsigned char, std::pair<unsigned int, int>>> funcVector;
        Functions functable[256];

    };
}
