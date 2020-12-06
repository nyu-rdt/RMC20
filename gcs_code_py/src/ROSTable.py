from threading import Thread
from mutex import *

class RosFunction: 
    def __init__(self, data, setup_func_ptr, decoder_func_ptr):
        self.data = ""
        self.setup_func_ptr = setup_func_ptr
        self.decoder_func_ptr = decoder_func_ptr
        self.lock = mutex()

class RosTable:
    def __init__(self):
        # maps string to int
        # str -> int
        self.idMap = {}

        # stores instances of RosFunction
        # list[RosFunction]
        self.data = []

        # stores chars
        self.nextSend = []

    def addBroadcast(self, identifier, setup, decoder):
        if identifier in self.idMap: 
            # exit if the identifier has already been added
            print identifier +  "is already in use"
            return

        self.data.append(RosFunction(identifier, setup, decoder))

        # i think this was only present in c++ because of the nature of 
        # having multiple constructors, probably not needed in the python implementation
        # if not data[-1].mutex: 
        #   data[-1].mutex = Lock()

        self.idMap[identifier] = len(self.data) - 1

        # check to see if we need to add more bytes
        curr = len(self.data)
        tmp = curr//8
        if tmp * 8 < len(self.data)-1: tmp += 1
        if tmp > len(self.nextSend):
            print "adding ros buffer"
            self.nextSend.append(0)

    def setup(self, sender):
        #if not callable(setup_func_ptr):
        #    raise Exception("Setup is not a function pointer")
        #if not callable(decoder_func_ptr):
        #    raise Exception("Decoder is not a function pointer")

        # if our function pointer doesn't have a setup value, we assign sender to it
        for func in self.data:
            # go through our funciton pointers
            if callable(func.setup_func_ptr):
                # if function.setup != None
                func.setup_func_ptr(sender)

    def valAt(self, index):
        # return blank string if index out of bounds
        if index < 0 or index >= len(self.data): return ""

        # c++: std::lock_guard<std::mutex> lock(*data[index].lock);
        # "start" lock
        self.data[index].lock.testandset()
        try: # returns a string with index label and value at index
            return "".join([str(index), str(self.data[index].data)])
        finally:
            # "end" lock
            self.data[index].lock.unlock()

    def decode(self, index, msg):
        # check if index is out of bounds
        if index < 0 or index >= len(self.data): return

        #checks if RosFunction has a decoder, then run with msg
        if (self.data[index].decoder):
            self.data[index].decoder(msg)

    def GetID(self, reqID, resID):
        # if reqID in idMap dictionary
        if reqID in self.idMap:
            # set resID.data = idMap[reqID]
            resID.data = idMap[reqID]
        else: # if not found
            # set resID.data = -1
            resID.data = -1
        return True

    def SetData(self, req, res):
        # depends on how framework::IntStr::Request will be implemented, need further testing

        # if reqID is not a valid ID in self.data return False
        if req.ID < 0 or req.ID >= len(self.data): return False

        self.data[req.ID].lock.testandset()
        try: #else set index reqId in self.data to req.data
            self.data[req.ID].data = req.data
            res.success = True
            return True
        finally:
            self.data[req.ID].lock.unlock()
  

    def GetData(self, req, res):
        # return False for out of bounds
        if req.ID < 0 or req.ID >= len(self.data):
            return False
        tmp1 = req.ID
        tmp2 = tmp1 >> 3
        tmp1 -= tmp2 << 3
        #set the data that will be sent next
        self.nextSend[tmp2] |= chr(1 << tmp1)
        res.success = True
        return True

# temp debug classes
class IntStr_request():
    def __init__(self, ID, data):
        self.ID = ID
        self.data = data

class IntStr_response():
    def __init__(self, success):
        self.success = success
  


def a_func():
    print("my func")

try:
    r = RosTable()
    # r.addBroadcast("print identifier", print, print)
    r.addBroadcast("my function identifier", a_func, a_func)
    print('id map', r.idMap)
    print('next send', r.nextSend)
    print r.valAt(0)
    print r.valAt(0)
    r.SetData(IntStr_request(0,123), IntStr_response(False))
    print('after set')
    print(r.valAt(0))



    # r.SetData(0, )
    # r.GetData(1, )

except Exception as e:
    print(e)