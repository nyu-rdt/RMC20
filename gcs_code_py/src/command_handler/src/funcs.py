import  rospy 
keyboard = ['W','A','S','D', ' '] # this will be taken from the keyboard class  
keyboard_move_status = [False for _ in range(0,4)] 

def encoder(a):  
    # a is array of booleans; a: list[bool]
    # presumably you're actually going to search for the specific key that you need to use to call a function  
    
    
    # right now this is pushing all of the keys it get of the in     
    #  7 -> 0b0111 -> [f, t, t, t] 
    return [sum([int(a[i]) << i for i in range(len(a))])]
 

def decoder(a):
    global keyboard

    # 7 -> 0b0111 -> [f, t, t, t] 
    # inverse [f, t, t, t] 
    # [ t , t , t , f ] 
    # ['W','A','S','D'] 
    # decoder([7]) -> WAS 


    # printing a statement for demonstration purposes 
    # actual functions should just call the relevant subsystems 
    print("".join(
        [keyboard[i] if (2**i) & a[0] > 0 else "" for i in range(len(keyboard))]
    ))


def perms(n):
    for i in range(2 ** n):
        yield [((2**j) & i) > 0 for j in range(n)]


def test_func(keyboard, encoder, decoder):
    for perm in perms(len(keyboard)):
        print("".join([str(keyboard[i]) if perm[i] else '' for i in range(len(keyboard))]))
        decoder(encoder(perm))
        print("-------")


# for perm in perms(len(keyboard)):
#     #print(perm)
#     print("".join([keyboard[i] if perm[i] else '' for i in range(len(keyboard))]))
#     #   print(encoder(perm))
#     decoder(encoder(perm))
#     print("-------")

# encoders and decoders 

def encoder_emergency_stop(a):
    # emergency stop is at 0th index
    if a[0]:
        return [1]
    return [0]
    

def decoder_emergency_stop(d):
    # if d[0] is 1, emergency stop
    # if not, then don't emergency stop
    # replace print with function calls
    if d[0]:
        return "stop"
        #print("emergency stop")
    #else:
        #print("no emergency stop")
        

test_func([1], encoder_emergency_stop, decoder_emergency_stop)

def encoder_door(a):
    # first bit to open, second bit to close
    if not a[0] ^ a[1]:
        return [0]
    if a[0]:
        return [0b01]
    if a[1]:
        return [0b10]


def decoder_door(d):
    # 0b01 means open, 0b10 means close
    # replace print with function calls
    if d[0] & 0b01:
        print("open door")
    elif d[0] & 0b10:
        print("close door")
    else:
        print('No door action')

test_func([1,1], encoder_door, decoder_door)

def encoder_actuator(a):
    # first bit to move forward, second bit to move backward
    if not a[0] ^ a[1]:
        return [0]
    if a[0]:
        return [0b01]
    if a[1]:
        return [0b10]


def decoder_actuator(d):
    # 0b01 means forward, 0b10 means backward
    # replace print with function calls
    if d[0] & 0b01:
        print("Move linear actuator forward")
    elif d[0] & 0b10:
        print("Move linear actuator backward")
    else:
        print('Stop actuator')
        
test_func([1,1], encoder_actuator, decoder_actuator)

#manual_drive_topic = "TOPIC_TO_MANUAL_DRIVE"
#manual_drive_sendHandler = rospy.Publisher(manual_drive_topic, Keyboard, queue_size=10)

def encoder_manual_drive(IIII):
    # should recieve wasd
    # a=0 d=1 s=2 w=3   
    
    # don't go forward and backwards at the same time
    I = 1
    II = 1
    if IIII[3]: I += 1
    if IIII[2]: I -= 1
    # same for left and right
    if IIII[0]: II += 1
    if IIII[1]: II -= 1
    II = II << 3
    I = I | II
    print([I])
    print("decoder manual drive:", decoder_manual_drive([I]))
    return [I]


def decoder_manual_drive(d):
    # will recieve forward, left, back, reverse (WASD)
    # TODO: change I variables to something normal
    global keyboard_move_status
#    result = Keyboard()
    I = d[0] & 0b111000
    I = I >> 3
    II = d[0] & 0b000111
    # result.a = False 
    # result.d = False 
    # result.s = False 
    # result.w = False
    # if I == 0: result.a = True  #print( "go leftwards" )
    # elif I == 2: result.d = True  #print( "go rightwards" )

    # if II == 0: result.s = True #print("go backwards" )
    # elif II == 2: result.w = True   #print( "go frontwards")
    # # print("decoder manual drive")
    if I == 0:
        #A 
        keyboard_move_status[0] = True if keyboard_move_status[0] == False else False #print( "go leftwards")
    elif I == 2: 
        #D
        keyboard_move_status[1] = True if keyboard_move_status[1] == False else False #print( "go rightwards")
        #W
    if II == 0: 
        keyboard_move_status[2] = True if keyboard_move_status[2] == False else False #print( "go forward")
        #S
    elif II == 2:
        keyboard_move_status[3] = True if keyboard_move_status[3] == False else False #print( "go frontwards")
    # print("decoder manual drive")
#    result.a = keyboard_move_status[0]
#    result.d = keyboard_move_status[1]
#    result.s = keyboard_move_status[2]
#    result.w = keyboard_move_status[3]
#    manual_drive_sendHandler.publish(result)
#    return result
    print(keyboard_move_status)
    

test_func(['w','a','s','d'], encoder_manual_drive, decoder_manual_drive)



def encode_drum_speeds(a):
    #recieves drum speed 
    # index 0 of a is positive rotation, index 1 is neg
    d = 0
    if a[0]: d += 0b1000
    elif [1]: d += 0b0010
    print("drum rotating")


def decode_drum_speeds(d):
    if d[0] & 0b1000: pass #rotate positively
    if d[0] & 0b0010: pass #rotate in the negative direction

def encoder_arm(a):
    d = 0
    if a[0]: d += 0b0100  #arm moves in positive direction
    elif a[1]: d += 0b0001 #arm moves in negative direction
    
    
def decoder_arm(d):
    if d[0] & 0b1000: pass
    elif d[0] & 0b0010: pass
