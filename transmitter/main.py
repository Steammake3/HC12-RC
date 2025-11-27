from servo import Servo
from machine import Pin, SPI, UART, Timer
from utime import sleep_ms
from MCP3008 import MCP3008
from buzzer import Buzzer

#LX Rudder LY Flaps RX Ailerons RY Elevator POT Throttle
#<Throttle><Rudder><Elevator><Left Flaperon><Right Flaperon>
# __  __         __ ___ ____
#|   |  | |\  | |    |  |   |
#|   |  | | \ | |~~  |  |  ____
#|__ |__| |  \| |   _|_ |___| |
#

trims = [0,0,0,0]

aileron_reverse = False
rudder_reverse = False
elev_reverse = False

#---------------------------------

BASE64 = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"
debug_LED = Pin("LED",Pin.OUT);

radio = UART(1, baudrate=19200, tx=Pin(4), rx=Pin(5))

mySPI = SPI(0, sck=Pin(18),mosi=Pin(19),miso=Pin(16), baudrate=100000)

cs = Pin(17, Pin.OUT)
cs.value(1)

myMCP = MCP3008(mySPI, cs)

bizzer = Buzzer(10, led_pin=21, flipped=True)
bizz_timer = Timer(); btact=False

trim_start = Pin(15, Pin.IN, Pin.PULL_UP)
trim_cut = Pin(14, Pin.IN, Pin.PULL_UP)

can_trim = True

prevals = [32,32,32,32]

normalite = True

def based64(points, checksum=False):
    finale = ""
    for p in points:
        finale += BASE64[p]
    
    if checksum:
        finale += BASE64[sum(points)%64]
    
    return finale

def clamp(val, lb=0, ub=63):
    return lb if val<lb else (ub if val>ub else val)

def adjust_trims():
    global can_trim, trims
    
    #Rudder
    if (myMCP.read(1)>>6)<4:
        trims[0] -= (-1 if rudder_reverse else 1)
        can_trim = False
        bizzer.play(659 if rudder_reverse else 293, 0.2)
    if (myMCP.read(1)>>6)>12:
        trims[0] += (-1 if rudder_reverse else 1)
        can_trim = False
        bizzer.play(293 if rudder_reverse else 659, 0.2)
    
    #Elevator
    if (myMCP.read(2)>>6)<4:
        trims[1] -= (-1 if elev_reverse else 1)
        can_trim = False
        bizzer.play(659 if elev_reverse else 293, 0.2)
    if (myMCP.read(2)>>6)>12:
        trims[1] += (-1 if elev_reverse else 1)
        can_trim = False
        bizzer.play(293 if elev_reverse else 659, 0.2)
        
    #TODO
    #MAKE SURE THAT THE TRIMS ARE MIXED RATHER THAN FED; OTHERWISE, HALF OF THE TRIMMING FUNCTIOALITY IS UNFUNCIONABLE
    
    flats=[0,0] 
    
    if (myMCP.read(3)>>6)<4:
        flats[0] += (-1 if aileron_reverse else 1)
        flats[1] -= (-1 if aileron_reverse else 1)
        can_trim = False
        bizzer.play(659 if aileron_reverse else 293, 0.2)
    if (myMCP.read(3)>>6)>12:
        flats[0] -= (-1 if aileron_reverse else 1)
        flats[1] += (-1 if aileron_reverse else 1)
        can_trim = False
        bizzer.play(293 if aileron_reverse else 659, 0.2)
    
    if (myMCP.read(0)>>6)<4:
        flats = [flats[0]-1, flats[1]-1]
        can_trim = False
        bizzer.play(293, 0.2)
    elif (myMCP.read(0)>>6)>12:
        flats = [flats[0]+1, flats[1]+1]
        can_trim = False
        bizzer.play(659, 0.2)
    
    trims[2]+=flats[0]
    trims[3]+=flats[1]

def calcit():
    vals = []
    #Here we go
    
    #BTW this is weird but since my joysticks are plugged in breadboard
    #What looks like X+ turns out to be Y-, Y+ becomes X+
    vals.append(myMCP.read(4)>>4) #Throttle
    sleep_ms(2)
    
    vals.append(63-myMCP.read(1)>>4 if rudder_reverse else myMCP.read(1)>>4) #Rudder
    sleep_ms(2)
    
    vals.append(63-myMCP.read(2)>>4 if elev_reverse else myMCP.read(2)>>4) #Eleva  tor
    sleep_ms(2)
    
    zagreb = myMCP.read(3)>>4 #Input
    flaperons = [zagreb,63-zagreb] #Taking up the flaperon duty
    if aileron_reverse:
        flaperons = [flaperons[1], flaperons[0]]
    
    if (myMCP.read(0)>>6)<4:
        flaperons = [clamp(flaperons[0]-16), clamp(flaperons[1]-16)]
        bizzer.stop(); bizzer.play(523, 0.5)
    elif (myMCP.read(0)>>6)>12:
        flaperons = [clamp(flaperons[0]+16), clamp(flaperons[1]+16)]
        bizzer.stop(); bizzer.play(1046, 0.5)
    else:
        bizzer.stop()
    
    vals.append(flaperons[0])
    vals.append(flaperons[1])
    
    #End that & start THE TRIMMING
    
    for i in range(1, 5):
        vals[i] = clamp(vals[i]+trims[i-1])
    
    return vals

while True:
    vals = []
    
    if trim_start.value()==0:
        normalite = False
        if not btact:
            bizz_timer = Timer(period=250, mode=Timer.PERIODIC, callback=(lambda x : 0 if bizzer.is_playing() else bizzer.play(440,0.1)))
        btact = True
    if trim_cut.value()==0:
        normalite = True
        btact = False
        bizzer.stop()
        bizz_timer.deinit()
    
    if normalite:
        vals=calcit()
        prevals=vals[1:]
    else:
        vals = [myMCP.read(4)>>4]+prevals
        
        zebra_in_deadzone = True
        for i in range(0,4):
            zebra_in_deadzone &= (myMCP.read(i)>>6)>4 and (myMCP.read(i)>>6)<12
        if not can_trim and zebra_in_deadzone: can_trim = True
        
        for i in range(1, 5):
            vals[i] = clamp(vals[i]+trims[i-1])
        
        if can_trim:
            adjust_trims()
    #Debug & send
    print(based64(vals, checksum=True))
    radio.write(based64(vals, checksum=True))
    
    sleep_ms(20)
