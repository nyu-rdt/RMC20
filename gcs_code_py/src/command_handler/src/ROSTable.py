from threading import Thread, Lock 

class RosFunction: 
  def __init__(self, data:str  = "", setup_func_ptr:callable=None, decoder_func_ptr:callable=None):
    if not callable(setup_func_ptr):
      raise Exception("Setup is not a function pointer")
    if not callable(decoder_func_ptr):
      raise Exception("Decoder is not a function pointer")
    self.data = ""
    self.setup_func_ptr = setup_func_ptr
    self.decoder_func_ptr = decoder_func_ptr

    # lock not implemented
    # self.lock = Lock()

class RosTable:

  def __init__(self):
    # maps string to int
    self.idMap = {}
    
    # stores instances of RosFunction
    self.data = []

    # stores chars
    self.nextSend = []

  def addBroadcast(self, identifier: str, setup:callable=None, decoder:callable=None) -> None:
    if identifier in self.idMap: 
      # exit if the identifier has already been added
      print(f"{identifier} is already in use")
      return

    self.data.append(RosFunction(identifier, setup, decoder))
    # self.data[-1].setup = setup
    # self.data[-1].decoder = decoder

    # add lock and mutex if needed

    self.idMap[identifier] = len(self.data) - 1

    # check to see if we need to add more bytes
    curr = len(self.data)
    tmp = curr//8
    if tmp * 8 < len(self.data)-1: tmp += 1
    if tmp > len(self.nextSend):
      print("adding ros buffer")
      self.nextSend.append(chr(0))
    
  def decode(self, index: int, msg: str) -> None:
    # check if index is out of bounds
    if index < 0 or index >= len(self.data): return

    #checks if RosFunction has a decoder, then run with msg
    if (self.data[index].decoder):
      self.data[index].decoder(msg)

  def setup(self, sender: bool) -> None:
    # if our function pointer doesn't have a setup value, we assign sender to it
    for func in self.data:
      # go through our funciton pointers
      if not func.setup:
        # if function.setup != None
        func.setup(sender)

  def valAt(self, index: int) -> str:
    # return blank string if index out of bounds
    if index < 0 or index >= len(self.data): return ""

    # add lock/mutex, if needed

    # returns a string with index label and value at index
    return "".join([str(index), self.data[index].data])

  def GetID(self, reqID:int, resID) -> bool:
    # if reqID in idMap dictionary
    if reqID in self.idMap:
      # set resID.data = idMap[reqID]
      resID.data = idMap[reqID]
    else: # if not found
      # set resID.data = -1
      resID.data = -1
    return True

  def GetData(self, req: int, res) -> bool:
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

  def SetData(self, req: int, res) -> bool:
    #if reqID is not a valid ID in self.data return False 
    if req.ID < 0 or req.ID >= len(self.data): return False
    # add lock, mutex
    #else set index reqId in self.data to req.data
    self.data[req.ID].data = req.data
    res.success = True
    return True


# testing 
# ✅ Add broadcast 
# Get broadcast id (Also ensure all ids are unique)
# Set data using broadcast id (waiting for NUC)
# Get data using broadcast id (waiting for NUC)


def a_func():
  print("my func")

try:
  r = RosTable()
  r.addBroadcast("print identifier", print, print)
  r.addBroadcast("my function identifier", a_func, a_func)
  print('id map', r.idMap)
  print('next send', r.nextSend)


  # r.SetData(0, )
  # r.GetData(1, )

except Exception as e:
  print(e)