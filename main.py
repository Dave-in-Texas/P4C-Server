import time
from utime import localtime
import random
import _thread
import network
import socket
import math
from machine import ADC, Pin
from machine import RTC, reset
import rp2
import sys
import utime as time
import usocket as socket
import ustruct as struct
import secrets

#import uasyncio as asyncio

############################################
#Manual Mode Types

class SwimModeType():
    def __init__ (enabled, speed,aux,wfalls):
        self.enabled =  enabled = 0
        self.speed =  speed = 0
        self.aux =  aux = 0
        self.wfalls =  wfalls = 0
Swim = SwimModeType
Swim.enabled = 0
Swim.speed = 0
Swim.aux = 0
Swim.wfalls = 0

class CleanModeType():
    def __init__ (enabled, speed,aux,wfalls):
        self.enabled =  enabled = 0
        self.speed =  speed = 0
        self.aux =  aux = 0
        self.wfalls =  wfalls = 0
Clean = CleanModeType
Clean.enabled = 0
Clean.speed = 0
Clean.aux = 0
Clean.wfalls = 0

############################################
#Hardware Type
class MainPumpType():
    def __init__ (enabled, speed):
        self.enabled =  enabled = 0
        self.speed =  speed = 0
MainPump = MainPumpType
MainPump.speed = 0
MainPump.enabled = 0

class AuxPumpType():
    def __init__ (enabled, speed):
        self.enabled =  enabled = 0
        self.speed =  speed = 0
AuxPump = AuxPumpType
AuxPump.speed = 0
AuxPump.enabled = 0

class WaterfallsType():
    def __init__ (enabled, speed):
        self.enabled =  enabled = 0
        self.speed =  speed = 0
Waterfalls = WaterfallsType
Waterfalls.speed = 0
Waterfalls.enabled = 0

############################################
#Control Variable Type
class ControlVariableType():
    def __init__ (AirTemp, WaterTemp,FPL,CurrentTime):
        self.AirTemp =  AirTemp = 0
        self.WaterTemp =  WaterTemp = 0
        self.FreezeProtectLevel =  FPL = 0
        self.CurrentTime =  CurrentTime = 0
        self.TimerinOperation =  CurrentTimer = 0
        self.TimerinOperation =  NextTimer = 0
        self.SocketTime =  SocketTime = 0

#Control Variable Declaration
CV = ControlVariableType
CV.AirTemp = 0
CV.WaterTemp = 0
CV.FPL = 0
CV.CurrentTime = 0
CV.CurrentTimer = 0
CV.SocketTime = 9999

############################################
#Timer Table Type
class TimerProgram :
    def __init__ (self, name, enabled, start, stop, speed, aux, wfalls):
        self.name = name =""
        self.enabled = enabled = 0
        self.start =  start = 0
        self.stop =  stop = 0
        self.speed =  speed = 0
        self.aux =  aux = 0
        self.wfalls =  wfalls = 0
        self.TimeTill = TT = 0
        self.TimetoGo = TTG = 0

#Timer Table Variable

Timer = []

for i in range (9):
    Timer.append(TimerProgram("",0,0,0,0,0,0))

############################################
    
#Freeze Table Type
class FreezeProgram():
    def __init__ (self, mode, temp, speed, aux):
        self.mode = mode = 0
        self.temp = temp = 0
        self.speed =  speed = 0
        self.aux =  aux = 0

#Freeze Table Variable

FreezeProtectLevel = 0

Freeze = []

for i in range (5):
    Freeze.append(FreezeProgram('test', 0,0,0))


############################################
    
#Pump Speed Table Type
class PumpSpeedType():
    def __init__ (self, speed):
        self.speed =  speed

#Pump Speed Variable

SpeedTable = []

for i in range (9):
    SpeedTable.append(PumpSpeedType(0))
    
############################################
    
DST_OFFSET = 3600 * 1
GMT_OFFSET = 3600 * (-6) # 3600 = 1 h (Central Standard Time)

# NTP-Host
NTP_HOST = 'pool.ntp.org'
rtc = RTC()
    
#################### END TYPE DECLARATIONS ####################
    
#################### FUNCTION DECLARATIONS ####################
    
########## FILE READING FUNCTIONS ##########

def readTimerTable(Timer):
    csvFile = open("Timers.csv","r")
    a,b,c,d,e,f,g,h = csvFile.readline().strip().split(",")
    for index in range(1,9):
        a,b,c,d,e,f,g,h = csvFile.readline().strip().split(",")
        #Timer[index].prog=int(a)
        Timer[index].name=b
        Timer[index].enabled=int(c)
        Timer[index].start=int(d)
        Timer[index].stop=int(e)
        Timer[index].speed=int(f)
        Timer[index].aux=int(g)
        Timer[index].wfalls=int(h)
    csvFile.close()

############################################
    
def readManualModesTable(Swim,Clean):
    csvFile = open("ManualModes.csv","r")
    a,b,c,d,e = csvFile.readline().strip().split(",") #read & ignore header row
    a,b,c,d,e = csvFile.readline().strip().split(",")
    Swim.enabled=0
    Swim.speed=c
    Swim.aux=d
    Swim.wfalls=e

    a,b,c,d,e = csvFile.readline().strip().split(",")
    Clean.enabled=0
    Clean.speed=c
    Clean.aux=d
    Clean.wfalls=e

    Waterfalls.speed = 0
    
    csvFile.close()

############################################

def readFreezeTable(Freeze):
    csvFile = open("Freeze.csv","r")
    a,b,c,d = csvFile.readline().strip().split(",")
    for index in range(5):
        a,b,c,d = csvFile.readline().strip().split(",")
        Freeze[index].mode=a
        Freeze[index].temp=b
        Freeze[index].speed=c
        Freeze[index].aux=d
    csvFile.close()

############################################

def readPumpSpeedTable(Speed):
    csvFile = open("PumpSpeeds.csv","r")
    for index in range(9):
        a,b = csvFile.readline().strip().split(",")
        SpeedTable[index].number=a
        SpeedTable[index].speed=b

    SpeedTable[1].speed = "0"
    csvFile.close()

########## END FILE READING FUNCTIONS ##########

def clock():
   # used to retrieve current data,time
    global Dyear, Dmonth, Dday, Dhour, Dmin, Dsec, Dweekday, Dyearday
    dateTimeObj = localtime()
    Dyear, Dmonth, Dday, Dhour, Dmin, Dsec, Dweekday, Dyearday = (dateTimeObj)
    if Dmin < 10:
        stringtime = (str(Dhour)+"0"+str(Dmin))
    else:
        stringtime = (str(Dhour)+str(Dmin))
    CV.CurrentTime = int(stringtime)
    print(Dhour,":",Dmin,":",Dsec)

############################################


def Data_xfer_Thread(HOST, PORT):
    HOST = '192.168.1.244'
    PORT = 9090
    ClockReset = 0
    CV.SocketTime = 9999
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(1)

    while True:
        communication_socket, address = server.accept()
        CV.SocketTime = CV.CurrentTime
        print("CV.SocketTime=",CV.SocketTime)
        
        Received_string = communication_socket.recv(1024).decode('utf-8')
        Swim.enabled = int(Received_string)
        Transmit_String = str(CV.AirTemp)
        communication_socket.send(Transmit_String.encode('utf-8'))
        
        Received_string = communication_socket.recv(1024).decode('utf-8')
        Clean.enabled = int(Received_string)
        Transmit_String = str(CV.WaterTemp)
        communication_socket.send(Transmit_String.encode('utf-8'))
        
        Received_string = communication_socket.recv(1024).decode('utf-8')
        Transfer_file = Received_string
        Transmit_String = str(CV.FPL)
        communication_socket.send(Transmit_String.encode('utf-8'))
        
        Received_string = communication_socket.recv(1024).decode('utf-8')
        Year = 2000 + int(Received_string)
        Transmit_String = str(CV.CurrentTimer)
        communication_socket.send(Transmit_String.encode('utf-8'))
        
        Received_string = communication_socket.recv(1024).decode('utf-8')
        Month = int(Received_string)
        Transmit_String = str(CV.NextTimer)
        communication_socket.send(Transmit_String.encode('utf-8'))
        
        Received_string = communication_socket.recv(1024).decode('utf-8')
        Day = int(Received_string)
        Transmit_String = str(MainPump.enabled)
        communication_socket.send(Transmit_String.encode('utf-8'))
        
        Received_string = communication_socket.recv(1024).decode('utf-8')
        Hour = int(Received_string)
        Transmit_String = str(MainPump.speed)
        communication_socket.send(Transmit_String.encode('utf-8'))
        
        Received_string = communication_socket.recv(1024).decode('utf-8')
        Minutes = int(Received_string)
        Transmit_String = str(AuxPump.enabled)
        communication_socket.send(Transmit_String.encode('utf-8'))
        
        Received_string = communication_socket.recv(1024).decode('utf-8')
        Seconds = int(Received_string)
        Transmit_String = str(AuxPump.speed)
        communication_socket.send(Transmit_String.encode('utf-8'))
        
        Received_string = communication_socket.recv(1024).decode('utf-8')
        dayofweek = Received_string
        Transmit_String = str(Waterfalls.enabled)
        communication_socket.send(Transmit_String.encode('utf-8'))

        Received_string = communication_socket.recv(1024).decode('utf-8')
        spare = Received_string
        Transmit_String = str(Waterfalls.speed)
        communication_socket.send(Transmit_String.encode('utf-8'))
        
        Received_string = communication_socket.recv(1024).decode('utf-8')
        spare = Received_string
        if Dsec < 10:
            stringsec = "0" + str(Dsec)
        else:
            stringsec = str(Dsec)
        if Dmin < 10:
            stringmin = "0" + str(Dmin)
        else:
            stringmin = str(Dmin)
        if Dhour < 10:
            stringhour = "0" + str(Dhour)
        else:
            stringhour = str(Dhour)
            
        Transmit_String = stringhour+":"+stringmin+":"+stringsec
        #print(Transmit_String)
        communication_socket.send(Transmit_String.encode('utf-8'))
        
        #print(Dyear, Dmonth, Dday, Dhour, Dmin, Dsec, Dweekday, Dyearday)

        if ".csv" in Transfer_file:
          
            file = open(Transfer_file, "wb")

            file_bytes = b""

            done = False

            while not done:
                data = communication_socket.recv(1024)
                if file_bytes[-5:] == b"<END>":
                    done = True
                else:
                    file_bytes += data
                    
            file.write(file_bytes)
            file.close()
            
            if Transfer_file == "Timers.csv":
                readTimerTable(Timer)
            elif Transfer_file == "Freeze.csv":
                readFreezeTable(Freeze)
            elif Transfer_file == "ManualModes.csv":
                readManualModesTable(Swim,Clean)
            elif Transfer_file == "PumpSpeeds.csv":
                readPumpSpeedTable(SpeedTable)
            Transfer_file = "nil"

        communication_socket.close()
        CV.SocketTime = 9999
        
        if CV.CurrentTime == 0200 and ClockReset == 0:
            setTimeRTC()
            ClockReset = 1
        else:
            if CV.CurrentTime > 0200 and ClockReset == 1:
                ClockReset = 0
        
    
############################################

def Freeze_Protect_Logic():
    
    DoMainPart=1

    if CV.FPL==4 and CV.AirTemp <= (int(Freeze[4].temp) + 1):
        DoMainPart = 0

    if CV.FPL==3 and CV.AirTemp <= (int(Freeze[3].temp) + 1) and CV.AirTemp > int(Freeze[4].temp):
        DoMainPart = 0

    if CV.FPL==2 and CV.AirTemp <= (int(Freeze[2].temp) + 1) and CV.AirTemp > int(Freeze[3].temp):
        DoMainPart = 0

    if CV.FPL==1 and CV.AirTemp <= (int(Freeze[1].temp) + 1) and CV.AirTemp > int(Freeze[2].temp):
        DoMainPart = 0

    if CV.FPL==0 and CV.AirTemp >= (int(Freeze[0].temp)):
        DoMainPart = 0

    if DoMainPart == 1:

        if CV.AirTemp >= int(Freeze[0].temp):
            CV.FPL=0
            
        if CV.AirTemp <= int(Freeze[1].temp) and CV.AirTemp > int(Freeze[2].temp):
            CV.FPL=1

        if CV.AirTemp <= int(Freeze[2].temp) and CV.AirTemp > int(Freeze[3].temp):
            CV.FPL=2

        if CV.AirTemp <= int(Freeze[3].temp) and CV.AirTemp > int(Freeze[4].temp):
            CV.FPL=3

        if CV.AirTemp <= int(Freeze[4].temp):
            CV.FPL=4

    
    if CV.FPL > 0:
        MainPump.speed = Freeze[CV.FPL].speed
        if AuxPump.enabled == 1:
            AuxPump.speed = Freeze[CV.FPL].aux
        else:
            AuxPump.speed = 0
        Waterfalls.speed = 0
        
        Swim.enabled = 0
        Clean.enabled = 0
    
############################################

def Timer_Logic():
    clock()
    
    if CV.FPL == 0 and Swim.enabled == 1 and Clean.enabled == 0:
        MainPump.speed  = Swim.speed
        AuxPump.speed = Swim.aux
        Waterfalls.speed = Swim.wfalls

    if CV.FPL == 0 and Swim.enabled == 0 and Clean.enabled == 1:
        MainPump.speed = Clean.speed
        AuxPump.speed = Clean.aux
        Waterfalls.speed = Clean.wfalls

    if CV.FPL == 0 and Swim.enabled == 0 and Clean.enabled == 0:
        CV.CurrentTimer = 0
        MainPump.speed = 0
        AuxPump.speed = 0
        MinutesToNextTimer=2400
        CV.NextTimer=99
        FallBack = 0
        
        for i in range (8,0,-1):
            if Timer[i].enabled == 1:
                ###  find the Timer which should be in use now
                if Timer[i].stop > Timer[i].start:  #'easy' case - timer does not run past midnight
                    ### calculate TimeTill for each
                    if Timer[i].start > CV.CurrentTime:
                        Timer[i].TT = Timer[i].start - CV.CurrentTime
                    else:
                        Timer[i].TT = 2400 - (CV.CurrentTime - Timer[i].start)

                    ## check if it's within the start/stop time range
                    if CV.CurrentTime >= Timer[i].start and CV.CurrentTime < Timer[i].stop:
                        #calculate TimeToGo
                        Timer[i].TTG = Timer[i].stop - CV.CurrentTime
                        if AuxPump.enabled == 1:
                            AuxPump.speed = Timer[i].aux
                        else:
                            AuxPump.speed = 0
                        if Waterfalls.enabled == 1:
                             Waterfalls.speed = Timer[i].wfalls
                        #set parameters for "found a timer in range"
                        FallBack = CV.CurrentTimer
                        CV.CurrentTimer = i
                        MainPump.speed = Timer[i].speed
                        Timer[FallBack].TT = Timer[i].TTG
                    else:   #if this timer not in range
                        Timer[i].TTG = 0
                            
                if Timer[i].start > Timer[i].stop:  # timer runs past midnight
                     ### calculate TimeTill for each
                    if Timer[i].start > CV.CurrentTime:
                        Timer[i].TT = Timer[i].start - CV.CurrentTime
                    else:
                        Timer[i].TT = 2400 - (CV.CurrentTime - Timer[i].start)

                    ## check if it's within the start/stop time range (note the OR rather than AND              
                    if CV.CurrentTime >= Timer[i].start or CV.CurrentTime < Timer[i].stop:
                        #calculate TimeToGo
                        if CV.CurrentTimer >= Timer[i].stop:
                            Timer[i].TTG = (2400-CV>CurrentTime) + Timer[i].start
                        else:
                            Timer[i].TTG = Timer[i].stop - CV.CurrentTime
                        if AuxPump.enabled == 1:
                            AuxPump.speed = Timer[i].aux
                        else:
                            AuxPump.speed = 0
                        if Waterfalls.enabled == 1:
                            Waterfalls.speed = Timer[i].wfalls
                        else:
                            Waterfalls.speed = 0
                        #set parameters for "found a timer in range"
                        FallBack = CV.CurrentTimer
                        CV.CurrentTimer = i
                        MainPump.speed = Timer[i].speed
                        Timer[FallBack].TT = Timer[i].TTG
                    else:
                        Timer[i].TTG = 0
                
        ####  Find which Timer will be the next to run
        for i in range (8,0,-1):
            if Timer[i].enabled == 1:
                #Look for an 'upward' / higher priority case
                if Timer[i].TT < MinutesToNextTimer:
                    MinutesToNextTimer = Timer[i].TT
                    CV.NextTimer= i

                if i != CV.CurrentTimer and Timer[i].TTG != 0 and Timer[i].TTG < MinutesToNextTimer:
                    MinutesToNextTimer = Timer[i].TTG
                    CV.NextTimer = i
   
        ###  the case where a Timer is running
        if CV.CurrentTimer != 0:
            CV.NextTimerTime = Timer[CV.CurrentTimer].start
                
        ###  the case where no Timer is running
        else:
            MainPump.speed = 0
            AuxPump.speed = 0

    print("Current Timer: ", CV.CurrentTimer," FPL: ", CV.FPL,"   MP: ",MainPump.speed, "   Aux: ", AuxPump.speed, "   WF: ", Waterfalls.speed)


######################  Set Outputs   ######################

######## Hardware definitions  ########

AirADCpin = 26
WaterADCpin = 27
AirThermistor = ADC(AirADCpin)
WaterThermistor = ADC(WaterADCpin)

# Voltage Divider
Vin = 3.3
Ro = 10000  # 10k Resistor

# Steinhart Constants
A = 0.001129148
B = 0.000234125
C = 0.0000000876741

def Read_Air_Temp():
    #CV.AirTemp=random.randrange(25,70)
    # Get Voltage value from ADC   
    adc = AirThermistor.read_u16()
    Vout = (3.3/65535)*adc
    
    # Calculate Resistance
    Rt = (Vout * Ro) / (Vin - Vout) 
    #Rt = random.randrange(9000, 37000)
    #Rt = 10000  # Used for Testing. Setting Rt=10k should give TempC=25
    
    # Steinhart - Hart Equation
    TempK = 1 / (A + (B * math.log(Rt)) + C * math.pow(math.log(Rt), 3))

    # Convert from Kelvin to Celsius
    TempC = TempK - 273.15
    
    #Convert to Farenheit
    CV.AirTemp = round((TempC *9/5)+32,1)

############################################
    
def Read_Water_Temp():
   #CV.WaterTemp=random.randrange(25,70)
   # Get Voltage value from ADC   
   adc = WaterThermistor.read_u16()
   Vout = (3.3/65535)*adc
    
   # Calculate Resistance
   Rt = (Vout * Ro) / (Vin - Vout) 
   #Rt = 10000  # Used for Testing. Setting Rt=10k should give TempC=25    
   # Steinhart - Hart Equation
   TempK = 1 / (A + (B * math.log(Rt)) + C * math.pow(math.log(Rt), 3))

   # Convert from Kelvin to Celsius
   TempC = TempK - 273.15

   #Convert to Farenheit
   CV.WaterTemp = round((TempC *9/5)+32,1)

############################################
#############  HARDWARE SETUP  #############

pin_MP0 = Pin(16, mode=Pin.OUT)
pin_MP1 = Pin(17, mode=Pin.OUT)
pin_MP2 = Pin(18, mode=Pin.OUT)
pin_WF  = Pin(19, mode=Pin.OUT)
pin_AUX = Pin(20, mode=Pin.OUT)
switch_MP =  Pin(15, mode=Pin.IN)
switch_AUX = Pin(14, mode=Pin.IN)
switch_WF =  Pin(13, mode=Pin.IN)

def Read_Switches():
    if switch_MP.value() == 1:
        MainPump.enabled = 1
    else:
        MainPump.enabled = 0
        
    if switch_AUX.value() == 1:
        AuxPump.enabled = 1
    else:
        AuxPump.enabled = 0
        
    if switch_WF.value() ==1:
        Waterfalls.enabled = 1
    else:
        Waterfalls.enabled = 0

def Set_Outputs():
  #AuxPump
    if int(AuxPump.enabled) == 1 and int(AuxPump.speed) == 1:
        pin_AUX.high()
    else:
        pin_AUX.low()
    
    #Waterfalls
    if int(Waterfalls.enabled) == 1 and int(Waterfalls.speed) == 1:
        pin_WF.high()
    else:
       pin_WF.low()
           
    #MainPump
        
    diff = 5000
    found = 99
    for index in range(1,9):
        delta = abs(int(SpeedTable[index].speed) - int(MainPump.speed))
        if delta < diff :
            found = index
            diff = delta
            command_speed = found - 1

    if MainPump.enabled == 0:
        command_speed = 0

    if command_speed == 0:
        pin_MP0.low()
        pin_MP1.low()
        pin_MP2.low()
    elif command_speed == 1:
        pin_MP0.high()
        pin_MP1.low()
        pin_MP2.low()
    elif command_speed == 2:
        pin_MP0.low()
        pin_MP1.high()
        pin_MP2.low()
    elif command_speed == 3:
        pin_MP0.high()
        pin_MP1.high()
        pin_MP2.low()
    elif command_speed == 4:
        pin_MP0.low()
        pin_MP1.low()
        pin_MP2.high()
    elif command_speed == 5:
        pin_MP0.high()
        pin_MP1.low()
        pin_MP2.high()
    elif command_speed == 6:
        pin_MP0.low()
        pin_MP1.high()
        pin_MP2.high()
    elif command_speed == 7:
        pin_MP0.high()
        pin_MP1.high()
        pin_MP2.high()

############################################
def setTimeRTC():
    NTP_DELTA = 2208988800
    NTP_QUERY = bytearray(48)
    NTP_QUERY[0] = 0x1B
    print(NTP_HOST)
    addr = socket.getaddrinfo(NTP_HOST, 123)[0][-1]
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    success = 0
    while success!= 1:
        try:
            s.settimeout(1)
            res = s.sendto(NTP_QUERY, addr)
            msg = s.recv(48)
            success = 1
        except:
            print('NTP failure')
        #finally:
    s.close()
    ntp_time = struct.unpack("!I", msg[40:44])[0]
    
    tm = time.gmtime(ntp_time - NTP_DELTA + GMT_OFFSET)
    yearmonthday = tm[0]*10000+tm[1]*100+tm[2]
    
    #print("yearmonthday= ",yearmonthday)
    #print(tm)
    
    if 20230312<=yearmonthday < 20231105:
        tm = time.gmtime(ntp_time - NTP_DELTA + GMT_OFFSET+DST_OFFSET)
        
    elif 20240310<=yearmonthday < 20241103:
        tm = time.gmtime(ntp_time - NTP_DELTA + GMT_OFFSET+DST_OFFSET)
        
    elif 20250309<=yearmonthday < 20251102:
        tm = time.gmtime(ntp_time - NTP_DELTA + GMT_OFFSET+DST_OFFSET)
        
    elif 20260308<=yearmonthday < 20261101:
        tm = time.gmtime(ntp_time - NTP_DELTA + GMT_OFFSET+DST_OFFSET)
        
    elif 20270314<=yearmonthday < 20271107:
        tm = time.gmtime(ntp_time - NTP_DELTA + GMT_OFFSET+DST_OFFSET)
        
    elif 20280312<=yearmonthday < 20271105:
        tm = time.gmtime(ntp_time - NTP_DELTA + GMT_OFFSET+DST_OFFSET)
        
    elif 20290311<=yearmonthday < 20291104:
        tm = time.gmtime(ntp_time - NTP_DELTA + GMT_OFFSET+DST_OFFSET)
        
    elif 20300310<=yearmonthday < 20301103:
        tm = time.gmtime(ntp_time - NTP_DELTA + GMT_OFFSET+DST_OFFSET)
        
    elif 20310309<=yearmonthday < 20311102:
        tm = time.gmtime(ntp_time - NTP_DELTA + GMT_OFFSET+DST_OFFSET)
        
    elif 20320314<=yearmonthday < 20321107:
        tm = time.gmtime(ntp_time - NTP_DELTA + GMT_OFFSET+DST_OFFSET)
        
    elif 20330313<=yearmonthday < 20331106:
        tm = time.gmtime(ntp_time - NTP_DELTA + GMT_OFFSET+DST_OFFSET)

    elif 20340312<=yearmonthday < 20341105:
        tm = time.gmtime(ntp_time - NTP_DELTA + GMT_OFFSET+DST_OFFSET)
        
    elif 20350311<=yearmonthday < 20351104:
        tm = time.gmtime(ntp_time - NTP_DELTA + GMT_OFFSET+DST_OFFSET)
        
    elif 20360309<=yearmonthday < 20361102:
        tm = time.gmtime(ntp_time - NTP_DELTA + GMT_OFFSET+DST_OFFSET)
        
    elif 20370308<=yearmonthday < 20371101:
        tm = time.gmtime(ntp_time - NTP_DELTA + GMT_OFFSET+DST_OFFSET)
        
    elif 20380314<=yearmonthday < 20381107:
        tm = time.gmtime(ntp_time - NTP_DELTA + GMT_OFFSET+DST_OFFSET)
        
    elif 20390313<=yearmonthday < 20391106:
        tm = time.gmtime(ntp_time - NTP_DELTA + GMT_OFFSET+DST_OFFSET)
        
    elif 20400311<=yearmonthday < 20401104:
        tm = time.gmtime(ntp_time - NTP_DELTA + GMT_OFFSET+DST_OFFSET)
        
    rtc.datetime((tm[0], tm[1], tm[2], tm[6] + 1, tm[3], tm[4], tm[5], 0))
    MDHM_ntp = tm[1]*1000000+tm[2]*10000+tm[3]*100+tm[4]


############## CONNECT TO WLAN ##################

def Connect_to_WLAN(HOST):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(secrets.SSID, secrets.PASSWORD)
    # Wait for connect or fail
    max_wait = 10
    while max_wait > 0:
      if wlan.status() < 0 or wlan.status() >= 3:
        break
      max_wait -= 1
      print('waiting for connection...')
      time.sleep(1)
    # Handle connection error
    if wlan.status() != 3:
       raise RuntimeError('network connection failed')
    else:
      print('connected')
      status = wlan.ifconfig()
      HOST = status[0]
      print('ip = ' + HOST)


##############  MAIN PROGRAM  ##############################    
    
# Setup the variables
HOST = '192.168.1.244'
PORT = 9090

#### Read in the files  ####
  
readManualModesTable(Swim,Clean)
readTimerTable(Timer)
readFreezeTable(Freeze)
readPumpSpeedTable(SpeedTable)

# Connect to the WLAN
Connect_to_WLAN(HOST)
print(HOST)

setTimeRTC()

def Main_Logic_Thread():
    
    while True:
        Read_Air_Temp()
        Read_Water_Temp()
        Read_Switches()
        Freeze_Protect_Logic()
        Timer_Logic()
        Set_Outputs()
        if CV.SocketTime != 9999:
            socketdiff = CV.CurrentTime - CV.SocketTime
            print("socketdiff=",socketdiff)
            if  socketdiff > 1:
                reset()
        time.sleep(1)


# Start the second Thread        
_thread.start_new_thread(Main_Logic_Thread,())

#################################
# HTML portion?
#################################

Data_xfer_Thread(HOST, PORT)




