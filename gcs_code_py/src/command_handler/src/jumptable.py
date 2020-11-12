from KeyMap import Key_library

class Functions:
    # constructor
    def __init__(self, keyboard, encoder, decoder, setup, cleanup, num_bytes=0):
        self.num_bytes = num_bytes
        self.keyboard = keyboard
        
        # func(lst(chr)) -> lst(bool)
        self.encoder = encoder
        
        # func(lst(chr)) -> None
        self.decoder = decoder
        
        # func(bool) -> None
        self.setup = setup
        
        # func(bool) -> None
        self.cleanup = cleanup

        # lst(tuple(chr, int))
        self.func_vector = None

    
class FunctionTable:
    # constructor
    def __init__(self):
        self.curr_index = 0
        self.compressed_used = 0

        # holds a tuple of a character and an integer
        # (char, int)
        self.func_vector = []

        # lst(Functions)
        self.func_table = []


    # destructor
    def __del__(self):
        self.curr_index = None
        self.compressed_used = None
        self.manage = None
        self.func_vector = None


    """
    Function used to insert a function into func_table by providing its components
    returns None
    parameters:
        num_bytes: int
        keyboard: Keyboard
        encoder: func(lst(bool)) -> lst(chr)
        decoder: func(lst(chr)) -> None
        setup: func(bool) -> None
        cleanup: func(bool) -> None
    """
    def insert(self, num_bytes, keyboard, encoder, decoder, setup, cleanup):
        func = Functions(keyboard, encoder, decoder, setup, cleanup, num_bytes)
        self.insert_function(func)


    '''
    Function used to insert a function into func_table by providing the function
    returns None
    parameters:
        f: Functions
    '''
    def insert_function(self, function):
        self.func_table.append(function)
        tot = 0

        # casting k to int isn't allowed in python
        # easy solution would be to change KeyMap from Enum to IntEnum, see how it affects other files first
        for k in function.keyboard:
            tot |= 1 << int(k)
    
        self.compressed_used |= tot

        self.func_vector.append((tot, self.curr_index))

        self.curr_index += 1


    def setup(self, sender):
         for data in self.func_vector:
            '''
            Following line may not work in python
            We are casting the first field of data which is a char as an index
            We may need to do something special to avoid issues in python
            '''
            f = self.func_table[data[0]] 
            if f.setup != None:
                f.setup(sender)


    '''
    Taking a snippet of key_val that is length f.num_bytes and then passing it to the decode function

    returns None
    parameters:
        key_val: lst(chr)
        curr: int
    '''
    def parse(self, key_val, curr):
        while curr < len(key_val):
            f = self.func_table[key_val[curr]]
            curr += 1

            if f.decoder != None:
                data = [0] * f.num_bytes

                '''
                charles did very interesting math for some reason

                Original:  
                for(int i = curr+f.num_bytes; curr < i; ++curr) {
                    data[f.num_bytes - i + curr] = keyVal[curr];
                
                curr -> curr + f.num_bytes
                data[f.num_bytes - f.num_bytes - curr + curr(this is the incremented value)]

                this is the same as going from 0 -> f.num_bytes

                key_val[curr -> curr + f.num_bytes]
                }
                '''
                for i in range(0, f.num_bytes-curr):
                    data[i] = key_val[i + curr]

                f.decode(data)


    '''
    returns None
    parameters:
        compressed: int
        compressed_prev: int
        header: lst(chr)
    '''
    def encode(self, compressed, compressed_prev, header):
        delta = compressed ^ compressed_prev

        expanded = [False] * len(Key_library)
        for i in range(len(expanded)):
            expanded[i] = (compressed & (1<<i)) > 0
    
        for data in self.func_vector:
            if data[1] & compressed or data[1] & delta:
                '''
                Following line may not work in python
                We are casting the first field of data which is a char as an index
                We may need to do something special to avoid issues in python
                '''
                f = self.func_table[data[0]]
                if f.encoder != None:
                    inputs = []
                    for k in f.keyboard:
                        inputs.append(expanded[int(k)])
                    
                    out = f.encoder(inputs)
                    if len(out) != f.num_byte:
                        print('wrong number of bytes')
                        continue
                    
                    header.append(data[0])
                    for uc in out:
                        header.append(uc)

    
    '''
    Does a bitwise and to find changes in the keyboard

    returns bool
    parameters:
        compressed: int
    '''
    def has_update(self, compressed):
        return (self.compressed_used & compressed) > 0