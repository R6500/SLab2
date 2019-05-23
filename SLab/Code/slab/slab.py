'''
@package slab
Base SLab Python module
It contains all interaction to the Hardware Board through Base Commands

History:

Version 1.0 : First version

17/2/2017 : It seems that DAC output is more acurate
            than ADC input. 
            ADC can be calibrated against DAC.
            
10/2/2018 : Addition to halt response code   

Version 1.1 : Added Python 3 compatibility

 1/3/2018 : Added compatibility with python 3

19/3/2018 : Added pause flags for the calibration methods 
            Corrected error saving DAC calibration global list
            
 8/4/2018 : Add of the gdata dictionary  

Version 1.3 : Support for firmware 2.0
 
24/05/2018 : Support for firmware 2.0 
30/05/2018 : Added axes limits for plot11, plot1n and plotnn
             Added interactivePlots() 
             Eliminated help(). Uses normal python help() now
 5/06/2018 : Use of text for mode in tranTriggered and dioMode
10/06/2018 : Add of transient digital signals
             Add dioWriteAll and dioReadAll 
			
Version 1.4 : Small modificatons	
		
13/03/2018 : Added parameters na,nm to checkcalibration
19/03/2019 : Calibration change to restrict the number of ADCs channels
02/04/2019 : Added calibration log
             Set exceptions on setVoltage and readVoltage if not connected
04/05/2019 : Added plot3D y plotFunc3D 
20/05/2019 : Added osd option to connect for Debian Linux            
'''

from __future__ import print_function
 
###################### INFO FOR THE HELP FILE ###################################

'''@package slab
Documentation for this module.
More details.
'''

'''
@slab
This is the main page of the SLab help
List of command category topics:

   manage : System management
     file : File commands
     base : Base DC voltage operations
   plotdc : DC curves
     plot : Generic plot commands
     tran : Transient commands
     wave : Wave commands
     util : Utility commands
      dio : Digital I/O
      cal : Board calibration
      var : Internal variables
      
List of SLab submodule category topics:

       dc : DC submodule
       ac : AC submodule
     meas : Measure submodule
      fft : FFT submodule
       ez : Easy submodule
	  
You can also input the name of a particular command
@manage@  
List of management command topics:

  connect
  disconnect
  softReset
  printBoardInfo
  wait
  pause
  setVerbose
  setPlotReturnData
@file@
  save
  load  
  setFilePrefix
  setCalPrefix  
@base@
List of base DC voltage command topics:

  setVoltage
  readVoltage
  rCurrent
  dcPrint
  dcLive
  zero
  dcSweep
  dcSweepPlot
  writeDAC
  readADC  
  setDCreadings
@plot@
List of generic plotting command topics:

  plot11
  plot1n
  plotnn
@tran@
List of transient command topics:

  setSampleTime
  setTransientStorage
  tranStore (alias)
  tranAsyncPlot
  tranTriggeredPlot
  stepPlot
  transientAsync
  transientTriggered
  stepResponse
@wave@
List of wave command topics:

   waveSquare
   waveTriangle
   waveSawtooth
   waveSine
   waveCosine
   wavePulse
   waveNoise
   waveRandom
   loadWavetable
   setWaveFrequency
   wavePlot
   waveResponse
   singleWavePlot
   singleWaveResponse
   wavePlay
@util@  
List of utility command topics:

   highPeak
   lowPeak
   peak2peak
   halfRange
   mean
   std
   rms
@dio@
List of digital I/O command topics:

   dioMode
   dioWrite
   dioRead   
@cal@
Full board calibration is a four stage procedure
List of calibration command and alias topics:

  Stage 1 : manualCalibrate   | cal1
  Stage 2 : adcCalibrate      | cal2
  Stage 3 : dacCalibrate      | cal3
  Stage 4 : checkCalibration  | cal4
 
 setVdd
 setVref
@var@
List of internal variables topics:

  vdd
  vref
  sampleTime
  linux  
'''

'''
Help information about internal variables
@vdd@
vdd
Voltage supply in Volt
Do not modify this variable
@vref@
vref
Voltage reference in Volt for DACs and ADCs
Do not modify this variable
@sampleTime@
Current sample time in seconds
Do not modify this variable
@linux@
True if system is detected as Linux
Modify before connect if autodetect fails
'''

################################################################################# 
 
import sys            # System module
import time           # Time module for wait
import serial         # Serial connection module
import pickle         # Savedata for calibration
import math           # Math module
import numbers        # Numbers module
import glob
import warnings       # Warnings module
import inspect        # Inspect module for getVar
import datetime       # Date and Time for log information
import os             # Operating system module

################# PYTHON VERSION CHECK ###########################

'''
Check if we are running on Python 2.x or 3.x
'''

PY3 = False
if sys.version_info >= (3,0):
    PY3 = True
    print("Python 3.0 or later detected")
    
'''
Set input both in python2 and python3
'''    
try:
   input = raw_input
except NameError:
   pass    

##################### LINUX CHECK ################################

'''
We need to check if we are in Linux because the connection
procedure has specific requirements in Linux

If Linux detection does not work ok, we can force this
variable using:

slab.linux = True    # Linux system
slab.linux = False   # Non Linux system
'''

if sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
    linux = True
    print('Linux detected')
else:
    linux = False

#################### SCIPY LOAD ##################################    
    
# Try to load the SciPy modules
# The "TkAgg" backend is selected out of linux because in default
# operation the console goes very slow after plotting
try:		
    if not PY3: # Needed so it does not crash Python 3.x
        if not linux:
            import matplotlib
            matplotlib.use("TkAgg")    
    import numpy as np                # Numpy for math calculations
    import pylab as pl                # Pylab and Mathplotlib for plotting
    import matplotlib.pyplot as plt
    from matplotlib import cm                     # Colormaps
    from mpl_toolkits.mplot3d import Axes3D       # For 3D graphs

except:
    scipy = False
else:
    scipy = True
	   
# Internal Constants ###############################################################       
       
# Version information
version_major = 1
version_minor = 4
version_date  = '4/5/2019'
version = str(version_major) + '.' + str(version_minor) + ' (' + version_date + ')'

# Default baud rate
BAUD_RATE = 38400

# Serial constants
ACK = 181     # Command Ok
NACK = 226    # Command Error
ECRC = 37     # Error in CRC

# External files
HELP_FILE = "SLab_Help.dat"

LAST_COM_FILE = "Last_COM.dat"
ADC_CAL_FILE = "Cal_ADC.dat"
DAC_CAL_FILE = "Cal_DAC.dat"
VDD_CAL_FILE = "Cal_Vdd.dat"
LOG_CAL_FILE = "Cal_Log.txt"

# Internal global variables #######################################################
        
# Interactive plot
iplots = False        
        
# Warn information    
nwarns = 0  # No warnings yet        
                    
adcNames = ["ADC1","ADC2","ADC3","ADC4"]  # Names of ADCs
dacNames = ["DAC1","DAC2","DAC3"]         # Names of DACs
        
magic = [56,41,18,1]  # Magic identification code

dcroundings = 10      # Number of measurements to average in DC

# Wavetable information

w_idle = -1  # No wave loaded
w_points = 0 # Number of wave points

# Secondary wavetable information
w_idle2 = -1  # No secondary table loaded
w_points2 = 0 # Number of wave points

# Digital wavetable information
w_idleD = -1  # No digital wave loaded
w_pointsD = 0 # Number of wave points
w_mask = 0    # Wave mask

##################### GLOBAL DATA ################################

opened = 0            # Connection not opened yet

vdd = 3.3             # By default vdd is 3.3V
vref = 3.3            # By default vref is 3.3V

fprefix = ""          # External files prefixes
calprefix = ""        # File calibration prefix

# Variable that indicates if plot functions shall return data
plotReturnData = False 

# Verbose level 0 = None, 1 = Warn, 2 = Normal, 3 = Extensive
verbose = 2 

# Board specific data        
ndacs = 2     # Default number of available DACs
nadcs = 4     # Default number of available ADCs

sampleTime = 0.001    # Current sample time

# Default values to be loaded from board
min_sample = 50e-6
max_sample = 1.0

'''
Data required for other modules
It is stored on a gdata dictionary
Elements in the dictionary
  'opened'         : True if there is a connection to the board
  'maxSFfresponse' : Max sample frequency for frequency response 
  'min_sample'     : Min sample time
  'sampleTime'     : Last set sample time
'''
gdata = {}
gdata['opened'] = opened
gdata['vdd'] = vdd
gdata['vref'] = vref
gdata['fprefix'] = fprefix
gdata['calprefix'] = calprefix
gdata['maxSFfresponse'] = None
gdata['plotReturnData'] = plotReturnData
gdata['verbose'] = verbose
gdata['ndacs'] = ndacs
gdata['nadcs'] = nadcs
gdata['sampleTime'] = sampleTime
gdata['min_sample'] = min_sample
gdata['max_sample'] = max_sample

##################### PUBLIC CONSTANTS ###########################

# Trigger Modes
tmodeRise = 0
tmodeFall = 1

# DIO Modes
mInput     = 10
mPullUp    = 11
mPullDown  = 12
mOutput    = 20
mOpenDrain = 21

# Dictionary for public constants
cDict = {}

cDict['rise'] = tmodeRise
cDict['fall'] = tmodeFall

cDict['input'] = mInput
cDict['pullUp'] = mPullUp
cDict['pullDown'] = mPullDown
cDict['output'] = mOutput
cDict['openDrain'] = mOpenDrain

# Exception and warn code ###########################################################

class SlabEx(Exception):
    # Exception Methods ----------------------------------
    def __init__(self, msg=""):
        self.msg = msg
        print("\n** SLab exception")
        print('** ' + msg)
        print("\n")    
        if not interactive:
            input("Hit RETURN to end the script")
            
#    def __str__(self):
#        return 

# Warn code
def warn(message):
    global nwarns
    if verbose > 0:
        print("\n** WARNING")
        print("** " + message)
        print()
    nwarns = nwarns + 1

######################## OBJECTS #################################

'''
bunch (Not used yet)
Generate a new namespace
Included in slab.py 

Example:
  data = bunch(a=1,b=[2,3])
  data.a
  data.b.append(7)
  data.c = "New element"
  data
      
class bunch(dict):
    def __init__(self,**kw):
        dict.__init__(self,kw)
        self.__dict__.update(kw)
'''   
        

####################### PRIVATE ##################################        

'''
Creates a list of n empty lists
'''
def listOfEmptyLists(n):
    list = []
    for i in range(0,n):
        list.append([])
    return list    
        
####################### PRIVATE SERIAL ###########################

'''
Compose a u16 value from two byes: low and high
Parameters:
  low  : Low byte
  high : High byte
Returns composed valuie
'''    
def composeU16(low,high):
    value = low + high*256
    return value     
   
'''
Split a u16 value in two byes: low and high
Parameters:
  data : value to convert
Returns low,high
'''    
def splitU16(data):
    high = data//256
    low = data % 256
    return low,high   
   
'''
Start of a Tx transmission
'''   
def startTx():
    global crcTx
    crcTx = 0
    
'''
Send the crc
Usually that ends the Tx transmission
'''    
def sendCRC():
    global crcTx
    if PY3:
        ser.write([crcTx])
    else:
        ser.write(chr(crcTx))
    
'''
Send one byte and computes crc
'''    
def sendByte(byte):
    if byte < 0 or byte > 255:
        raise SlabEx("Byte value out of range")
    global crcTx
    if PY3:
        ser.write([byte])
    else:
        ser.write(chr(byte))
    crcTx = crcTx ^ byte
   
'''
Send one uint16 and computes crc
'''   
def sendU16(value):
    if value < 0 or value > 65535:
        raise SlabEx("Unsigned 16 bit integer out of range")
    low,high = splitU16(value)
    sendByte(low)
    sendByte(high)  
  
'''
Send one float and computes crc
Floats are sent as an offset 128 exponent (1 byte)
followed by an offset 20000 4 digit mantissa (2 byte U16)
Returns the sent float value
'''
def sendFloat(value):
    # Convert to mantissa/exponent
    exp  = int(math.floor(math.log10(value)))-3;
    mant = int(value/math.pow(10,exp))
    sendByte(exp+128)
    sendU16(mant+20000)
    # Recalculate value
    value2 = 1.0 * mant * math.pow(10,exp)
    return value2
    
'''
Start of a Rx reception
'''   
def startRx():
    global crcRx
    crcRx = 0
    
'''
Get CRC anc check it
It usually ends the Rx reception
'''    
def checkCRC():
    global crcRx
    crc = ord(ser.read())
    if crc != crcRx:
        raise SlabEx("CRC Error in Board to PC link")
   
'''
Get one byte and computes crc
'''   
def getByte():
    global crcRx
    byte = ord(ser.read())
    crcRx = crcRx ^ byte    
    return byte
    
'''
Get one uint16 and computes crc
'''    
def getU16():
    low  = getByte()
    high = getByte()
    value = composeU16(low,high)
    return value 
 
'''
Get one float value and computes crc
''' 
def getFloat():
    # Get exponent and mantissa
    exp = getByte() - 128
    mant = getU16() - 20000
    # Compute value
    value = 1.0 * mant * math.pow(10,exp)
    return value    
   
'''
Check ACK, NACK and ECRC
'''   
def checkACK():
    # Get response
    response = getByte();    
   
    if (response == NACK) or (response == ECRC):
        # Check also CRC
        checkCRC()
        # Exceptions
        if (response == NACK):
            raise SlabEx("Remote Error : Bad command parameters")
        
        if response == ECRC:
            raise SlabEx("CRC Error in PC to Board link")
        
    if response != ACK:
        raise SlabEx("Unknown Board Response")   
    
'''
Start command
Parameters:
   code : Code of command
'''
def startCommand(code):
    startTx()
    startRx()
    sendByte(ord(code))
    

'''
Check Magic 
Check magic code in an opened serial connection
returns 1 if board is correct or 0 otherwise
'''
def checkMagic():
    global ser
    
    # First we flush
    ser.flushInput()
    
    startCommand('M') # Magic request
    sendCRC()         # End of command
        
    # Check that it responds if not Linux
    if not linux:
        time.sleep(0.1)
      
        if ser.inWaiting() < 5:
            return 0
        
    read = getByte()
    if read != ACK:
        return 0
    
    # Check all magic bytes
    for char in magic:
        # Obtain the byte value of received character 
        read = getByte()
        # Exit if the magic does not match
        if read != char:
            return 0
       
    # Check CRC
    checkCRC()
       
    # If we arrive here, magic is good
    return 1    

'''
Open a serial connection with the given port
Includes the Linux especific operations
'''    
def openSerial(com_port):
    global ser
    ser = serial.Serial(port=com_port,baudrate=BAUD_RATE)
    # Settings required for Linux in the Nucleo boards
    if linux: 
        ser.setDTR(False)
        ser.setRTS(True)
    
'''
Detect and open COM port
Only returns if the board is detected
'''
def detectCom():
    global ser
    global com_port
    
    # Check if there is a saved last port
    try:
        with open(fprefix + LAST_COM_FILE,'rb') as f:
            com_port = pickle.load(f) 
    except:
        pass
    else:
        try:
            message(1,"Trying last valid port " + str(com_port))
            openSerial(com_port)
        except:
            pass
        else:    
            if checkMagic():
                message(1,"Board detected")
                return   
        message(1,"Last used port is not valid")
        message(1,"Searching for port")
    
    # Get a list of ports to tests
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise SlabEx('Platform not supported in autodetect')
            
    # Test each port    
    for p in ports:
        try:
            message(2,"Testing port " +str(p))
            # Port to try
            openSerial(p)
            
            if checkMagic():
                message(1,"Board detected at port " + str(p))
                com_port = p
                return
            ser.close()
        except (OSError, serial.SerialException):
            pass
    raise SlabEx('COM Autodetect Fail')    

    
#################### PRIVATE FUNCTIONS ###########################

'''
Convert a U16 value in float
Truncates to the limits
Parameter:
  value : U16 value between 0 an 65535
Returns float between 0.0 and 9.9998
'''   
def u16toFloat(value):
    if value < 0:
        value = 0
    if value > 65535:
        value = 65535
    return value / 65536.0
   
'''
Convert a float value to U16
Truncates in the limits
Parameter:
  value : Float value between 0.0 and 9.9998
Returns u16 between 
'''   
def floatToU16(fvalue):
    value = int(fvalue*65536.0)
    if value < 0:
        value = 0
    if value > 65535:
        value = 65535
    return value    

'''
Send a message to screen
Parameters:
    level : 1 Normal for verbose 2
            2 Extensive for verbose 3
'''
def message(level,message):
    # Check level errors    
    if level < 1 or level > 2:
        raise SlabEx("Internal Error : Bad message level")
    # Print message depending on level    
    if level < verbose:
        print(message)
        
'''
Calibrates one reading 0.0...1.0
Returns real ratiometric from read ADC ratiometric 
If data is outside of the calibration table, it returns 
the data without any calibration.
Parameters:
  input  : Float value to calibrate in table2 domain
  list1 : List of correct values
  list2 : List of incorrect values
Returns the calibrated value
'''  
def dc_cal(input,list1,list2):  
    if len(list1) < 2:
        return input
    prevx = -1.0   # Lower limit for x (calibrated)
    prevy = -1.0   # Lower limit for y (uncalibrated)
    for x,y in zip(list1,list2):
        if input <= y:         # Locate upper limit
            if prevx == -1:    # If out of table...
                return input   # ...don't calibrate
            # Calbrate the value    
            alpha = (input - prevy)/(y - prevy)
            value = prevx + alpha*(x - prevx)
            return value            
        else:
            prevx = x    # New lower limit
            prevy = y
    # Don't calibrate if we are out of the table        
    return input     
      
'''
Get firmware string
This command don't use CRC
'''    
def getFirmwareString():
    startCommand('F')

    cad = "" 

    # Check that it responds if not Linux
    if not linux:
        time.sleep(0.2);
        nchar = ser.inWaiting()
        if nchar < 1 :
            return "Unknown"
   
        for i in range(0,nchar):
            car = ser.read()
            if (not car == '\n') and (not car == '\r'):
                if PY3:
                    cad = cad + car.decode("utf-8")
                else:
                    cad = cad + str(car)
        return cad        
    else:
        car = ser.read()
        while not car == '\n':
            cad = cad + str(car) 
            car = ser.read()
        ser.read() # Flush '\r'
        return cad     
   
'''
Read one pin name
'''
def readPinName():
    name = ""
    while True:
        car = chr(getByte())
        if car == '$':
            raise SlabEx("Unexpected end of pin list")
        if car == '|':
            return name        
        name = name + str(car)    
   
'''
Identifies the connected board
'''
def getBoardData():
    global gdata
    global board_name
    global ndacs,nadcs,buff_size,max_sample,min_sample,vdd
    global dacPinList,adcPinList,dioPinList
    global maxSFfresponse
    global vref,dac_bits,adc_bits
    global ndio
    
    print("Getting board data")
    
    # Get firmware string
    board_name = getFirmwareString()
    message(1,"Connected to " + board_name)
    gdata['board_name']=board_name
    
    startCommand('I')  # Get board capabilities
    sendCRC()          # End of command
    
    # Check response
    time.sleep(0.1)
    flush = 0
    if not ser.inWaiting() == 25:   # Data size + 2 (ACK and CRC)
        message(1,"Unexpected Board Data")
        flush = 1
    # Check ACK    
    checkACK()    
    # Get data
    ndacs = getByte()
    gdata['ndacs']=ndacs
    nadcs = getByte()
    gdata['nadcs']=nadcs
    buff_size = getU16()
    gdata['buff_size']=buff_size
    max_sample = getFloat()
    gdata['max_sample']=max_sample
    min_sample = getFloat()
    gdata['min_sample']=min_sample    
    vdd  =  getFloat()
    gdata['vdd']=vdd
    maxSFfresponse = getFloat()
    gdata['maxSFfresponse']=maxSFfresponse
    vref = getFloat()
    gdata['vref']=vref
    dac_bits = getByte()
    gdata['dac_bits']=dac_bits
    adc_bits = getByte()
    gdata['adc_bits']=adc_bits
    ndio = getByte()
    gdata['ndio']=ndio
    rState = getByte()
   
    if flush:
        # Flush buffer
        ser.flushInput()
    else:
        checkCRC()
    
    if rState:
        message(1,"Board at reset state")
    else:
        message(1,"Board out of reset state")
    
    # Get pin list
    dacPinList=[]
    adcPinList=[]
    dioPinList=[]
    startCommand('L')
    sendCRC()          # End of command
    
    checkACK()
    for i in range(0,ndacs):
        dacPinList.append(readPinName())
    for i in range(0,nadcs):
        adcPinList.append(readPinName()) 
    for i in range(0,ndio):
        dioPinList.append(readPinName())         
            
    # Get the final $
    car = chr(getByte())
    if car != '$':
        raise SlabEx("[P] Bad termination in pin list")
           
    checkCRC()
    # Flush buffer
    ser.flushInput()
   
'''
Ratiometric read of one analog ADC channel
Does not perform any calibration
Parameters:
   n :Channel can be a number 1,2,3,4
Returns the ratiometric reading between 0.0 and 1.0   
'''    
def readChannel(n):
    if not opened:
        raise SlabEx("Not connected to board")
    if n > nadcs:
        raise SlabEx("Invalid ADC number")        
    acum = 0.0     
    
    startCommand('A')
    sendByte(n);
    sendCRC()          # End of command
        
    checkACK()
    value = getU16()
    checkCRC() 
    
    fvalue = u16toFloat(value)
    
    return fvalue  
	
'''
Ratiometric write of one analog DAC channel
Does not perform any calibration
Parameters:
      n : DAC to set 1,2 (or 3 if three DACs)
  value : Value to set 0.0 to 1.0
'''    
def writeChannel(n,value):
    if not opened:
        raise SlabEx("Not connected to board")
    if n > ndacs:    
        raise SlabEx("Invalid DAC number")        
    data = ratio2counts(value)
    
    startCommand('D')
    sendByte(n)
    sendU16(data) 
    sendCRC()
    
    checkACK()
    checkCRC()

'''
Convert ratiometric level to counts
Generate exception if out of range
Parameter:
   ratio : Value between 0.0 and 1.0
Returns uint 16   
'''  
def ratio2counts(ratio):
    if ratio < -0.001:
        raise SlabEx("Ratiometric value cannot be below 0.0")
    if ratio > 1.001:
        raise SlabEx("Ratiometric value cannot be above 1.0")

    data = floatToU16(ratio)
            
    return data    

'''
Convert voltage value to ratiometric value
'''    
def voltage2ratio(value):
    if value < -0.001:
        raise SlabEx("Voltage value cannot be below 0 V")
    if value > vref*1.001:
        raise SlabEx("Voltage value cannot be above Vref")
    return value/vref;
    
'''
Convert voltage value to counts 
'''  
def voltage2counts(value):
    return ratio2counts(voltage2ratio(value))
       
'''
Message when SciPy is not loaded and we cannot plot
'''       
def cannotPlot(exception=False):
    if not exception:
        message(1,"")
        message(1,"SciPy not loaded. Cannot plot")
        message(1,"")
    else:
        raise SlabEx("SciPy not loaded. Cannot plot")
       
'''
Generate an exception if scipy is not loaded
'''  
def checkSciPy():
    if not scipy:
        raise SlabEx("SciPy not loaded. Cannot execute")
              
############# PUBLIC NO BOARD RELATED FUNCTIONS #################
    
def wait(t):
    '''
    ----------------------------------------
    wait(t)
    Wait the indicated time in seconds

    Required parameter:
        t : Time to wait in float seconds
    Returns nothing
    Included in slab.py 
    ----------------------------------------
    '''   
    time.sleep(t)
    

def pause(message="Script Paused. Hit RETURN to continue"):
    '''
    --------------------------------------------
    pause(message)
    Pause the script untill return is hit
    
    Optional parameter:
    message : Message to show 
                (Use default if not provided)
    Returns nothing 
    Included in slab.py
    --------------------------------------------    
    '''    
    input(message)
    
    
    
def setVerbose(level):
    '''
    -------------------------------------------- 
    setVerbose(level)
    Sets the verbose level

    Required parameter:
       level : Verbose level 0, 1, 2 or 3
               0 : No messages
               1 : Only warnings
               2 : Basic information
               3 : Detailed information
    Returns previous verbose level 
    Included in slab.py   
    -------------------------------------------- 
    '''
    global verbose,gdata
    # Check
    if level < 0 or level > 3:
        raise SlabEx("Invalid verbose level")
    # Set level
    last_verbose = verbose
    verbose = level
    gdata['verbose'] = verbose    
    return last_verbose
    
    
def setFilePrefix(prefix=""):
    '''
    -------------------------------------------- 
    setFilePrefix(prefix)
    Set file prefix for all external files

    Optional parameter:
      prefix : Prefix to use (Defaults to none)
    Returns nothing  
    Included in slab.py 

    See also setCalPrefix
    -------------------------------------------- 
    ''' 
    global fprefix,gdata
    fprefix = prefix
    gdata['fprefix'] = fprefix
    

def setCalPrefix(prefix=""):
    '''
    -------------------------------------------- 
    setCalPrefix(prefix)
    Set file prefix for calibration files
    It adds after file prefix if present

    Optional parameter:
      prefix : Prefix to use (Defaults to none)
    Returns nothing  
    Included in slab.py 

    See also setFilePrefix
    -------------------------------------------- 
    ''' 
    global calprefix,gdata
    calprefix = prefix   
    gdata[calprefix] = calprefix    
    
    
def save(filename,data):
    '''
    ---------------------------------------------------
    save(filename,data)
    Saves a variable on a file
    Adds .sav extension to filename

    Parameters:
      filename : Name of the file (with no extension)
      data : Variable to store
      
    Returns nothing  
    Included in slab.py 
    ---------------------------------------------------
    '''    
    with open(filename+".sav",'wb') as f:
        pickle.dump(data,f)
    message(1,"Data saved")

  
def load(filename):
    '''
    ---------------------------------------------------
    load(filename)
    Loads a variable from a file
    Adds .sav extension to filename

    Parameters:
      filename : Name of the file (with no extension)
      
    Returns variable contained in the file
    Included in slab.py 
    ---------------------------------------------------
    ''' 
    with open(filename+".sav",'rb') as f:
        data = pickle.load(f)
    message(1,"Data loaded")    
    return data
    
'''
Interpolates in a function defined by two vectors
interpolate(value,iv,ov,extrapolate)
Required parameters:
  value : Value in the input domain
  iv  : Input vector (sorted and rising)
  ov : Output vector
Optional parameters:
  extrapolate : Extrapolates if needed (Defaults to True)
Returns value on the output domain
'''  
def interpolate(value,iv,ov,extrapolate=True):
    lv = len(iv)  
    if lv < 2:
        raise SlabEx('Vectors shall have at least two elements')
    if lv != len(ov):
        raise SlabEx('Vectors shall have the same length')
     
    # Locate the first point of to use in interpolation     
    n = -2    
    # Check if we are below te curve        
    if value < iv[0]:    
        if not extrapolate:
            raise SlabEx('Value out of range')
        n = 0
    else:
        for i in range(1,lv-1):
            if value <= iv[i]:
                n = i-1
        if n == -2:
            if not extrapolate:
                raise SlabEx('Value out of range')

    # Define linear function                 
    A = (ov[n+1]-ov[n])/(iv[n+1]-iv[n])
    B = ov[n] - A*iv[n]    
     
    res = A*value+B

    return res    
        
################ PUBLIC BASIC DC FUNCTIONS ####################


def printBoardInfo():
    '''
    -----------------------------------
    printBoardInfo()
    Shows board information on screen
    Returns nothing
    Included in slab.py 
    -----------------------------------
    '''
    if not opened:
        raise SlabEx("No board connected")
    print("")
    print("Board : " + board_name)
    print("  COM port : " + str(com_port))
    print("  Reference Vref voltage : " + str(vref) + " V")
    print("  Power Vdd voltage : " + str(vdd) + " V")
    print("  " + str(ndacs) + " DACs with " + str(dac_bits) + " bits")
    print("  " + str(nadcs) + " ADCs with " + str(adc_bits) + " bits")
    print("  " + str(ndio) + " Digital I/O lines")
    print("  DAC Pins " + str(dacPinList))
    print("  ADC Pins " + str(adcPinList))
    print("  DIO Pins " + str(dioPinList))
    print("  Buffer Size : " + str(buff_size) + " samples")
    print("  Maximum Sample Period : " + str(max_sample) + " s")
    print("  Minimum Sample Period for Transient Async: " + str(min_sample) + " s")
    print("  Maximum Sample Frequency for Frequency Response : " + str(maxSFfresponse) + " Hz")
    print("")
 
  
  
def disconnect():
    '''
    ----------------------------
    disconnect()
    Disconnect from the board
    Returns nothing
    Included in slab.py 
    ----------------------------
    '''
    global gdata
    global opened
    if not opened:
        raise SlabEx("Not connected to board")
    ser.close()
    opened = 0
    gdata['opened'] = opened
    message(1,"Disconnected from the board")
    if nwarns > 0:
        message(1,str(nwarns) + " warnings since connect")
  
 

def connect(portIdent=-1,osd=True):
    '''
    -----------------------------------------------------------
    connect(portIdent,osd)
    Open the connection with the hardware board
    Must be called before any other function that uses it

    Optional parameter:
       portIdent : Identifier of the COM port 
                   In windows it is COMx where x is a number
                   (Defaults to Autodetect)
       osd : Operating system detection (Defaults to true)       
    Returns nothing
    Included in slab.py 
    -----------------------------------------------------------
    '''
    global gdata
    global ser,opened
    global xcal,adcCalData
    global dacx,dacCalData
    global com_port
    global vdd,vref
    global nwarns
    global linux
    
    # Check OSD flag
    if not osd:
        linux = False
    
    # Check if already connected
    if opened:
        message(1,"Already connected")
        disconnect()
    
    com_port = portIdent
        
    if portIdent == -1:
        detectCom()
    else:    
        try:    
            openSerial(com_port)
        except:
            raise SlabEx("Cannot open connection. Check port")
    
    if not ser.isOpen():
        raise SlabEx("Cannot open connection. Check port")
         
    if not checkMagic(): 
        raise SlabEx("Bad magic from board. Check Firmware")
            
    # Set board as opened
    opened = 1    
    gdata['opened'] = opened
    
    # Save good com port
    with open(fprefix + LAST_COM_FILE,'wb') as f:
        pickle.dump(com_port, f)
    
    # Get information about the board
    getBoardData()
                    
    # Try to load ADC calibration data
    try:
        with open(fprefix + calprefix + ADC_CAL_FILE,'rb') as f:
            xcal,adcCalData = pickle.load(f) 
    except:
        message(1,"No ADC calibration data found")
        # Empty ADC Calibration tables (for maximum of 8 ADCs)
        xcal  = []
        adcCalData = listOfEmptyLists(nadcs)
    else:
        message(1,"ADC Calibration data loaded")
      
    # Try to load DAC calibration data
    try:
        with open(fprefix + calprefix + DAC_CAL_FILE,'rb') as f:
            dacx,dacCalData = pickle.load(f) 
    except:
        message(1,"No DAC calibration data found")
        # Empty DAC Calibration tables (for maximum of 4 ADCs)
        dacx  = []
        dacCalData = listOfEmptyLists(ndacs)
    else:
        message(1,"DAC Calibration data loaded")
        
    # Try to load Vdd and Vref calibration data
    try:
        with open(fprefix + calprefix + VDD_CAL_FILE,'rb') as f:
            vdd,vref = pickle.load(f)
            gdata['vdd'] = vdd
            gdata['vref'] = vref
    except:
        pass
    else:
        message(1,"Vdd loaded from calibration as " + str(vdd) + " V")
        message(1,"Vref loaded as " + str(vref) + " V")
  
    message(1,"")
    
    # Erase warn count
    nwarns = 0


def setVdd(value,persistent=False):
    '''
    -------------------------------------------------
    setVdd(value,persistent)
    Set Vdd supply voltage

    Required parameter:
      value : Value to set (in Volt)
      
    Optional parameter:
      persistent : Makes value persistent in a file
                   (Defaults to False)

    Returns nothing  
    Included in slab.py   
    -------------------------------------------------    
    '''  
    global vdd,gdata
    # Check connection
    if not opened:
        raise SlabEx("Not connected to board")
    # Check vdd value
    if value < 3.0:
        raise SlabEx("Vdd value too low")
    # Removed to expand functionality at high voltages    
    # if value > 4.0:
    #    raise SlabEx("Vdd value too high")
    vdd = value
    gdata['vdd'] = vdd
    
    # Save if persistent
    if persistent:
        message(1,"Saving Vdd and Vref data to " + fprefix + calprefix + VDD_CAL_FILE)
        with open(fprefix + calprefix + VDD_CAL_FILE,'wb') as f:
            pickle.dump([vdd,vref], f)
            
 
def setVref(value,persistent=False):
    '''
    -------------------------------------------------
    setVref(value,persistent)
    Set Vref reference value for DACs and ADCs

    Required parameter:
      value : Value to set (in Volt)
      
    Optional parameter:
      persistent : Makes value persistent in a file
                   (Defaults to False)

    Returns nothing
    Included in slab.py  
    -------------------------------------------------
    ''' 
    global vref,gdata
    # Check connection
    if not opened:
        raise SlabEx("Not connected to board")
    # Check vref value
    if value < 3.0:
        raise SlabEx("Vref value too low")
    # Removed to expand functionality at high voltages    
    # if value > 4.0:
    #    raise SlabEx("Vref value too high")
    vref = value
    gdata['vref'] = vref
    
    # Save if persistent
    if persistent:
        message(1,"Saving Vdd and Vref data to " + fprefix + calprefix + VDD_CAL_FILE)
        with open(fprefix + calprefix + VDD_CAL_FILE,'wb') as f:
            pickle.dump([vdd,vref], f)            
  
  
def writeDAC(channel,value):  
    '''
    --------------------------------------------
    writeDAC(channel,value)
    Write a ratiometric value to one DAC
    Performs calibration if available

    Required parameteres:
     channel : DAC number
       value : Value to set from 0.0 to 1.0 
    Returns nothing
    Included in slab.py  
    --------------------------------------------
    ''' 
    # Calibrate value    
    value = dc_cal(value,dacx,dacCalData[channel-1])    
    
    # Send to board
    writeChannel(channel,value)  

 
  
def setVoltage(channel,value):
    '''
    -------------------------------------
    setVoltage(channel,value)
    Sets the voltage value of one DAC
    Performs calibration if available

    Required parameters:
     channel : DAC to write
       value : Voltage to set
    Returns nothing
    Included in slab.py
    -------------------------------------    
    ''' 
    if not opened:
        raise SlabEx("Not connected to board")
    
    value = voltage2ratio(value)
    writeDAC(channel,value)
 

           
def readADC(channel):
    '''
    ---------------------------------------------------
    readADC(channel)
    Read the ratiometric value at one ADC
    Uses calibration data if available

    Rquired parameter:
      n : ADC number
    Returns a ratiometric value between 0.0 and 1.0
    Included in slab.py
    ---------------------------------------------------    
    ''' 
    fvalue = readChannel(channel)
    return dc_cal(fvalue,xcal,adcCalData[channel-1])  
    
    
   
def readVoltage(ch1,ch2=0):
    '''
    --------------------------------------------------------------
    readVoltage(ch1,ch2)
    Reads a differential voltage between two ADCs at ch1 and ch2
    If ch2 is ommited, returns voltage between ch1 and GND
    If any channel is zero, it is considered as GND
    Return the voltage
    Included in slab.py
    --------------------------------------------------------------    
    ''' 
    if not opened:
        raise SlabEx("Not connected to board")
        
    if ch1 == 0:
        pvalue = 0.0
    else:    
        pvalue = vref*readADC(ch1)
    if ch2 == 0:
        nvalue = 0.0
    else:
        nvalue = vref*readADC(ch2)
    return pvalue - nvalue 

    
def rCurrent(r,ch1,ch2=0):
    '''
    --------------------------------------------------------------
    rCurrent(r,ch1,ch2)
    Reads the voltage on a resistor and computes current from it
    If any channel is zero, it is considered as GND

    Parameters:
       r : Resistor value
      n1 : Positive terminal ADC
      n2 : Negative terminal ADC
           If omited it is considered to be GND
    Returns the resistor current 
    Included in slab.py
    --------------------------------------------------------------    
    '''
    v = readVoltage(ch1,ch2)
    i = v/r
    return i
    

def setDCreadings(number):
    '''
    --------------------------------------------------------------
    setDCreadings(number)
    Sets the number of readings to average on each DC mesurement
    Parameter:
      number : Number of values to read
    Returns last value of this number
    Included in slab.py
    --------------------------------------------------------------    
    '''
    global dcroundings
    lastValue = dcroundings
    # Check
    if number < 1:
        raise SlabEx("Number of readings must be greater or equal than 1")
    if number > 65535:
        raise SlabEx("Number of readings too big")
    dcroundings = int(number)
    
    # Send command
    startCommand('N')
    sendU16(dcroundings) 
    sendCRC()
    
    checkACK()
    checkCRC()
    
    return lastValue
  
def getVariable(name):
    '''
    --------------------------------------------
    getVariable(name)
    Get a SLab internal variable from its name
    Parameter:
       name : Name of the variable
    Returns the varible contents
    Generates an exception if not found    
    --------------------------------------------
    '''    
    return gdata[name]
  
######################## CLASSIC CALIBRATION ###########################################
  
 
def adcCalibrate(pause=True):
    '''
    ---------------------------------------------------------------------
    adcCalibrate(pause)
    Second stage of board calibration
    Calibrates ADCs against DAC1
    Stores calibration data on ADC_CAL_FILE file
    Optional parameter:
       pause : Pauses after explaining the procedure (Defaults to True)
    Returns nothing
    Included in slab.py
    This function is kept only for backward compatibility. 
    See newCalibrate1 and newCalibrate2
    ---------------------------------------------------------------------
    '''  
    global xcal,adcCalData
    
    print()
    print("Calibration of ADCs")
    print()
    print("Connect the DAC 1 output to all ADC inputs")
    print()   
    if pause:
        input("Press [Return] to continue")
    
    # Increase number of readings for better calibration
    lastDCR=setDCreadings(1000)
    
    # Define input calibration range and steps
    xcal = []
    for x in range(0,11):
        xcal.append(x/10.0)
    
    # Output range is now empty
    adcCalData = listOfEmptyLists(nadcs)
    
    # Obtain calibration data
    # We use readChannel because it does not depend on
    # previous calibrations
    message(1,"Performing ADC calibration")
    for x in xcal:
        message(2,"  Calibrate at " + str(x))
        writeDAC(1,x)        # Set DAC value
        time.sleep(0.1)      # Wait a little
        for i in range(1,nadcs+1):
            a = readChannel(i)          # ADC read
            adcCalData[i-1].append(a)   # Append value
    
    # Check monotony
    for i in range(0,len(xcal)-1):
        for ch in range(0,nadcs):
            if adcCalData[ch][i]>adcCalData[ch][i+1]:
                raise SlabEx("Channel " + str(ch+1) + " non monotonous")
    
    # Show graph if we have SciPy
    if not scipy:
        cannotPlot()
    else:    
        plt.figure(facecolor="white")     # White border
        for ch in range(0,nadcs): 
            pl.plot(xcal,adcCalData[ch],label="ADC"+str(ch+1))  # Show curves

        pl.xlabel('DAC1 Value')                 # Set X label
        pl.ylabel('ADC Values')                 # Set Y label
        pl.title('ADC Ratiometric Calibration Curves')  # Set title

        pl.legend(loc='lower right')
        pl.grid()
        if not iplots:
            pl.show() 
            pl.close()

    # Save of calibration data
    message(1,"Saving calibration data to " + fprefix +  calprefix + ADC_CAL_FILE)
    with open(fprefix + calprefix +  ADC_CAL_FILE,'wb') as f:
        pickle.dump([xcal,adcCalData], f)  
            
    # Restore the number of ADC readings
    setDCreadings(lastDCR)
            
    print() 
    print("Calibration of ADCs completed")
    print()    
   
'''
Stores DAC calibration data
Internal function
Parameter:
 n : Number of DACs to show
'''   
def _storeAndShowDACcalibration(n):
    # Plot if we have SciPy
    if not scipy:
        cannotPlot()
    else:    
        plt.figure(facecolor="white")     # White border
        for i in range(0,n):
            print('i=',i)
            print('len dacx=',len(dacx))
            print('len cal=',len(dacCalData[i]))
            pl.plot(dacx,dacCalData[i],label="DAC"+str(i))  # Show curve
           
        pl.xlabel('DAC Value')                          # Set X label
        pl.ylabel('Real Ratiometric Values')            # Set Y label
        pl.title('DAC Ratiometric Calibration Curves')  # Set title

        pl.legend(loc='lower right')
        pl.grid()
        if not iplots:
            pl.show()   
            pl.close()        

    message(1,"Saving calibration data to "+ fprefix + calprefix + DAC_CAL_FILE)
    with open(fprefix + calprefix + DAC_CAL_FILE,'wb') as f:
        pickle.dump([dacx,dacCalData], f)
           
   
  
def dacCalibrate(pause=True):
    '''
    ---------------------------------------------------------------------
    dacCalibrate(pause)
    Third stage of board calibration
    Calibrates DAC(i) against ADC(i)
    Stores calibration data on DAC_CAL_FILE file
    Optional parameter:
       pause : Pauses after explaining the procedure (Defaults to True)
    Returns nothing
    Included in slab.py 
    This function is kept only for backward compatibility. 
    See newCalibrate1 and newCalibrate2
    ---------------------------------------------------------------------
    ''' 
    global dacx,dacCalData
    
    print()
    print("Calibration of DACs")
    print()
    print("Connect the DAC outputs to ADC inputs with same number")
    print("DAC 1 to ADC 1 and DAC2 to ADC2 and son on...")
    print()    
    if pause:
        input("Press [Return] to continue")
    
    # Increase number of readings for better calibration
    lastDCR=setDCreadings(1000)
    
    # Define input calibration range and steps
    # dacx = np.arange(0.0,1.1,0.1)
    dacx = []
    for x in range(0,11):
        dacx.append(x/10.0)
    
    # Output range is now empty
    dacCalData = listOfEmptyLists(ndacs)
    
    # Obtain calibration data
    # We use previusly calibrated ADC channels
    message(1,"Performing DAC calibration")
    for x in dacx:
        message(2,"  Calibrate at " + str(x))
        for ch in range(1,ndacs+1):
            writeChannel(ch,x)       # Set DAC values without calibration
        time.sleep(0.1)    # Wait a little
        for ch in range(1,ndacs+1):
            a = readADC(ch)              # ADC ratiometric read (with calibration)
            dacCalData[ch-1].append(a)   # Append read values
    
    # Check monotony
    for i in range(0,len(dacx)-1):
        for ch in range(0,ndacs):
            if dacCalData[ch][i] > dacCalData [ch][i+1]:
                raise SlabEx("Channel " + str(ch+1) + " non monotonous")
        
    # Show and save calibration data
    _storeAndShowDACcalibration(ndacs)
     
    # Restore the number of ADC readings
    setDCreadings(lastDCR) 
     
    print() 
    print("Calibration of DACs completed")
    print()     
   
 
def manualCalibrateDAC1():  
    '''
    ---------------------------------------------------------------------
    manualCalibrateDAC1()
    First stage of board calibration
    Performs a manual calibration of DAC 1 against a voltage meter
    Also calibrates Vdd and Vref
    Returns nothing
    Included in slab.py 
    This function is kept only for backward compatibility. 
    See newCalibrate1 and newCalibrate2
    ---------------------------------------------------------------------
    '''  
    global vdd,vref,dacx,dacCalData,gdata
    print()
    print("Manual calibration of DAC 1")
    print("You will need a voltage measurement instrument (VM)" )
    print()
    print("Put VM between the Vdd terminal and GND")
    print("Write down the voltage value and press enter")
    print()
    vdd = float(input("Voltage value [Volt]: "))
    gdata['vdd'] = vdd
    print()
    print("Put VM between the buffered DAC 1 output and GND")
    print("Write down the voltage value and press enter each time it is asked")
    print()
    
    # Increase number of readings for better calibration
    lastDCR=setDCreadings(1000)
    
    dacx  = [0.0,0.02,0.1,0.5,0.9,0.98,1.0]
    voltages = []
    
    prevv = -1.0
    for x in dacx:
        writeChannel(1,x)
        y = float(input("Voltage value [Volt]: "))
        voltages.append(y)
        if y < prevv:
            raise SlabEx("Non monotonous. Cannot calibrate")
        prevv = y
        
    # Stores vdd and vref calibration
    if voltages[3]*2.0 > voltages[6] :
        setVref(voltages[3]*2.0, persistent=True)
    else:    
        setVref(voltages[-1], persistent=True) 

    # Convert to ratiometric    
    dacCalData = listOfEmptyLists(ndacs)
    for v in voltages:
        dacCalData[0].append(v/vref)
        
    # Store calibration
    _storeAndShowDACcalibration(1)  

    # Restore the number of ADC readings
    setDCreadings(lastDCR) 
    
    print() 
    print("Manual calibration of DAC 1 completed")
    print()    
      
######################## NEW CALIBRATION ###############################################    

  
def newCalibrate1(na=-1):  
    '''
    newCalibrate1()
    -----------------------------------------------------------------------------
    First stage of new board calibration
    Performs a manual calibration of DAC 1 and all ADCs against a voltage meter
    Also calibrates Vdd and Vref
	Optional parameter na indicates the number of ADCs to calibrate
    Returns nothing
    Included in slab.py
    -----------------------------------------------------------------------------
    ''' 
    
    global vdd,vref,dacx,dacCalData,xcal,adcCalData,gdata
    
	# Check if na is not provided
    if na<0: na = nadcs
	
    # Increase number of readings for better calibration
    lastDCR=setDCreadings(1000)
    
    # Clear calibration data
    dacCalData = listOfEmptyLists(ndacs)
    adcCalData = listOfEmptyLists(nadcs)
    
    print()
    print('Manual calibration of DAC 1 and all ADCs')
    print('You will need a voltage measurement instrument (VM)')
    print()
    print('Connect together all ADCs on the same node')
    print('Put the VM between this node and GND')
    print()
    print('Now, connect this node to Vdd ')
    print('Write down the voltage value and press enter')
    print()
    
    # Get real Vdd voltage and set Vdd and Vref to this value
    vdd = float(input("Vdd Voltage value [Volt]: "))
    vref = vdd
    gdata['vdd'] = vdd   # Set Vdd
    gdata['vref'] = vdd  # Set Vref equal to Vdd
    
    # Store ADC readings at Vdd
    ADCvddRead = []
    for ch in range(0,nadcs):
        if ch<na:
            ADCvddRead.append(readChannel(ch+1))
        else:
            ADCvddRead.append(1.0)
        
    print()
    print('Keep the ADCs connected together')
    print('Disconnect Vdd from the ADCs node')
    print('Conect the ADCs node to GND')
    print('Write down the voltage value and press enter')
    print()
    
    # Get shorted to gnd voltage
    vgnd = float(input("GND Voltage value [Volt]: "))
    xcal = [vgnd/vref]
    for ch in range(0,nadcs):
        if ch<na:
            adcCalData[ch].append(readChannel(ch+1))
        else:
            adcCalData[ch].append(0.0) # Non calibrated channed
    
    print()
    print('Keep the ADCs connected together')
    print('Disconnect GND from the ADCs node')
    print('Connect DAC1 to the same node')
    print("Write down the voltage value and press enter each time it is asked")
    print()
    
    # Set range of values to calibrate
    dacx  = [0.1,0.2,0.35,0.5,0.65,0.8,0.9]
    
    prevv = -1.0
    for x in dacx:
        writeChannel(1,x)                             # Set DAC
        volt = float(input("Voltage value [Volt]: ")) # Ask for real value
        if volt < prevv:
            raise SlabEx("DAC1 non monotonous. Cannot calibrate")
        prevv = volt     
        n = volt/vref                                 # Convert real voltage to ratiometric
        dacCalData[0].append(n)                       # Store DAC1 calibration
        xcal.append(n)                                # Store ADC cal x data
        for ch in range(0,nadcs):
            if ch<na:
                read = readChannel(ch+1)
                if read < adcCalData[ch][-1]:   
                    raise SlabEx("ADC"+str(ch+1)+" non monotonous. Cannot calibrate")
                adcCalData[ch].append(read)  # Store ADC meas data
            else:
                adcCalData[ch].append(x) # Non calibrated channed
            
    # Add the last Vdd point 
    xcal.append(vdd/vref)    
    for ch in range(0,nadcs):
        if ADCvddRead[ch] < adcCalData[ch][-1]:   
                raise SlabEx("ADC"+str(ch+1)+" non monotonous. Cannot calibrate")
        adcCalData[ch].append(ADCvddRead[ch])
       
    # Set DACs to zero
    zero()    
       
    # Create legends
    legend = []
    for i in range(1,nadcs+1):
        legend.append('ADC'+str(i))

    # Plot curves
    plot11(dacx,dacCalData[0],'DAC1 Ratiometric Curve','Set value','Real value')
    plot1n(xcal,adcCalData,'ADC Ratiometric Curves','Real value','Measured value',legend)
       
    message(1,"Saving Vdd and Vref data to " + fprefix + calprefix + VDD_CAL_FILE)
    os.makedirs(os.path.dirname(fprefix + calprefix + VDD_CAL_FILE), exist_ok=True)
    with open(fprefix + calprefix + VDD_CAL_FILE,'wb+') as f:
        pickle.dump([vdd,vref], f)   
         
    message(1,"Saving DAC calibration data to "+ fprefix + calprefix + DAC_CAL_FILE)
    os.makedirs(os.path.dirname(fprefix + calprefix + DAC_CAL_FILE), exist_ok=True)     
    with open(fprefix + calprefix + DAC_CAL_FILE,'wb+') as f:
        pickle.dump([dacx,dacCalData], f)       

    # Save of calibration data
    message(1,"Saving ADC calibration data to " + fprefix +  calprefix + ADC_CAL_FILE)
    os.makedirs(os.path.dirname(fprefix + calprefix + ADC_CAL_FILE), exist_ok=True)
    with open(fprefix + calprefix +  ADC_CAL_FILE,'wb+') as f:
        pickle.dump([xcal,adcCalData], f)  
       
    # Create LOG Entry
    message(1,"Logging Calibration to "	+ fprefix +  calprefix + LOG_CAL_FILE)
    os.makedirs(os.path.dirname(fprefix + calprefix + LOG_CAL_FILE), exist_ok=True) 
    with open(fprefix +  calprefix + LOG_CAL_FILE,'a+') as f:
        now = datetime.datetime.now()
        text = str(now.day) + '/' + str(now.month) + '/' + str(now.year) + ' : '
        text += 'First Stage Calibration with '+str(na)+' ADC channels'
        f.write(text +'\n')
        
    # Restore the number of ADC readings
    setDCreadings(lastDCR) 
    
    print() 
    print("Calibration stage 1 completed")
    print("Remeber to perform calibration stage 2")
    print()   

    
   
def newCalibrate2():  
    '''
    ------------------------------------------------------
    newCalibrate2()
    Second stage of new board calibration
    Performs automatic calibration of DACs gainst ADCs
    Returns nothing
    Included in slab.py
    ------------------------------------------------------    
    '''
    global dacx,dacCalData
    print()
    print("Automatic calibration of DACs using ADCs")
    print()
    for i in range(1,ndacs+1):
        s = str(i)
        print("Connect DAC" + s + " to ADC" + s)
    print()
     
    # Increase number of readings for better calibration
    lastDCR=setDCreadings(1000)
    
    # Clear calibration data for DACs 
    dacCalData = listOfEmptyLists(ndacs)
    dacx = []
     
    prevn = []
    for ch in range(0,ndacs):
        prevn.append(-1)
    for x in range(0,31):
        value = x/30.0
        dacx.append(value)
        for ch in range(1,ndacs+1):
            writeChannel(ch,value)      # Set DAC
            n = readADC(ch)             # Read ADC
            if n < prevn[ch-1]:
                n = prevn[ch-1]
            prevn[ch-1] = n    
            dacCalData[ch-1].append(n)
               
    # Set DACs to zero
    zero()    
               
    # Create legends
    legend = []
    for i in range(1,ndacs+1):
        legend.append('DAC'+str(i))
     
    # Plot curves
    plot1n(dacx,dacCalData,'DAC Ratiometric Curves','Set value','Real value',legend)
       
    message(1,"Saving DAC calibration data to "+ fprefix + calprefix + DAC_CAL_FILE)
    os.makedirs(os.path.dirname(fprefix + calprefix + DAC_CAL_FILE), exist_ok=True)     
    with open(fprefix + calprefix + DAC_CAL_FILE,'wb+') as f:
        pickle.dump([dacx,dacCalData], f)       
       
    # Create LOG Entry
    message(1,"Logging Calibration to "	+ fprefix +  calprefix + LOG_CAL_FILE)
    os.makedirs(os.path.dirname(fprefix + calprefix + LOG_CAL_FILE), exist_ok=True)   
    with open(fprefix +  calprefix + LOG_CAL_FILE,'a+') as f:
        now = datetime.datetime.now()
        text = str(now.day) + '/' + str(now.month) + '/' + str(now.year) + ' : '
        text += 'Second Stage Calibration for DACs'
        f.write(text+'\n')     
        f.write('----------------------------------------------------------\n')
       
    # Restore the number of ADC readings
    setDCreadings(lastDCR) 
    
    print() 
    print("Calibration stage 2 completed")
    print("Calibration is now complete")
    print()       
    
def checkCalibration(pause=True,na=-1,nm=-1):
    '''
    ---------------------------------------------------------------------
    checkCalibration(pause)
    Checks the board calibration
    Shows the curves of DACs connected to ADCs
    Optional parameter:
       pause : Pauses after explaining the procedure (Defaults to True)
	   na    : Number of ADCs to tests (Defaults to all)
	   nm    : Number of measurements to average (Defaults to 400)  
    Returns nothing
    Included in slab.py 
    See newCalibrate1 and newCalibrate2
    ---------------------------------------------------------------------
    '''    

    print()
    print("Calibration check")
    print()
    
    if na==-1: na = nadcs
	
    if na<ndacs:
	    raise SlabEx("na cannot be lower that the number of DACs")
    if na>nadcs:
	    raise SlabEx("na is bigger than the number of ADCs")
	
    if nm<1: nm = 400
	
    for i in range(2,ndacs+1):
        s = str(i)
        print("Connect DAC" + s + " to ADC" + s)
    print("Connect the rest of ADCs also to DAC1")
    print()
    if pause:    
        input("Press [Return] to continue")

    # Increase number of readings for better calibration
    lastDCR=setDCreadings(nm)
    
    vmin = 0.05
    vmax = vref - 0.05

    R = listOfEmptyLists(ndacs)
    for i in range(0,ndacs):
        R[i] = dcSweep(i+1,vmin,vmax,0.05)

    x = R[0][0]
    ylist = []
    ylegend = []
    for i in range(0,ndacs):
        ylist.append(R[i][1+i])
        ylegend.append('ADC'+str(i+1))
    if na > ndacs:    
        for i in range(ndacs,na):
            ylist.append(R[0][1+i])
            ylegend.append('ADC'+str(i+1))

    for i in range(1,ndacs+1):
        setVoltage(i,1.0)
    
    message(1,"")
    message(1,"DAC outputs shall be now 1V")
    message(1,"They will be zero after closing the plot")
    message(1,"")
    
    plot1n(x,ylist,"Calibration check","DAC (V)","ADC (V)",ylegend)
    
    zero()
    
    # Restore the number of ADC readings
    setDCreadings(lastDCR)     
    
    
def showADCcal():
    '''
    showADCcal()
    Shows ADC calibration curves
    Returns nothing
    '''
    labels = []
    for i in range(0,len(adcCalData)):
        labels.append('ADC'+str(i+1))
    plot1n(xcal,adcCalData,'ADC Calibration Curves'
               ,'Ratiometric Real','Ratiometric Measured',labels)
    print('Vref =',vref,'V')
    print('Vdd =',vdd,'V')    
    vin = np.array(xcal)*vref
    vout = []
    for list in adcCalData:
        vout.append(np.array(list)*vref)
    plot1n(vin,vout,'ADC Voltage Curves'
               ,'Real Voltage [V]','ADC Read Voltage[V]',labels)
  
def showDACcal():
    '''
    showDACcal()
    Shows DAC calibration curves
    Returns nothing
    '''
    labels = []
    for i in range(0,len(dacCalData)):
        labels.append('DAC'+str(i+1))
    plot1n(dacx,dacCalData,'DAC Calibration Curves'
               ,'Ratiometric Programmed','Ratiometric Real',labels)
    print('Vref =',vref,'V')
    print('Vdd =',vdd,'V')    
    vin = np.array(dacx)*vref
    vout = []
    for list in dacCalData:
        vout.append(np.array(list)*vref)
    plot1n(vin,vout,'DAC Voltage Curves'
               ,'Programmed Voltage [V]','DAC Real Voltage [V]',labels)    
     
    
########### CALIBRATE ALIAS COMMANDS ############################


def cal1():
    '''
    -------------------------------------------
    cal1()
    Alias for the manualCalibrateDAC1 command
    Included in slab.py
    -------------------------------------------
    '''
    manualCalibrateDAC1()

   
def cal2():
    '''
    -------------------------------------------
    cal2()
    Alias for the adcCalibrate command
    Included in slab.py
    -------------------------------------------
    ''' 
    adcCalibrate()
  
    
def cal3():
    '''
    -------------------------------------------
    cal3()
    Alias for the dacCalibrate command
    Included in slab.py
    -------------------------------------------
    ''' 
    dacCalibrate()
 
      
def cal4():
    '''
    -------------------------------------------
    cal4()
    Alias for the checkCalibration command
    Included in slab.py
    -------------------------------------------
    '''
    checkCalibration()     
    
########################################################################################


def dcPrint():
    '''
    ------------------------------------------
    dcPrint()
    Show readings all ADC channels on screen
    Returns nothing
    Included in slab.py 
    ------------------------------------------    
    '''
    print("ADC DC Values")
    for i in range(1,nadcs+1):
        a = readVoltage(i);
        print("  ADC"+str(i)+" = "+"{0:.3f}".format(a)+" V")
    print()
    
   
def zero():
    '''
    -----------------------------------
    zero()
    Set all DACs to ratiometric zero
    Does not use calibration
    Returns nothing
    Included in slab.py
    -----------------------------------    
    ''' 
    for i in range(1,ndacs+1):
        writeChannel(i,0.0);  
    message(2,"All DACs at zero")        
  

def dcLive(n=None,wt=0.2,single=False,returnData=False):
    '''
    -----------------------------------------------------------------------------
    dcLive(n,wt,single,returnData)
    Prints live values of ADC voltages
    Use CTRL+C to exit

    Optional parameters:
              n : Number of ADCs to show (Defaults to 4)
             wt : Wait time, in seconds, between measurements (Defaults to 0.2)
         single : Read only the ADC number n
     returnData : Return obtained data

    Included in slab.py
    -----------------------------------------------------------------------------
    '''
    # Check n
    if n == None: n = nadcs

    # Initial checks
    if not opened:
        raise SlabEx("Not connected to board")
    if n < 1 or n > nadcs:
        raise SlabEx("Invalid number of ADCs")
    
    # Generate output lists
    data=[]
    if not single:
        for i in range(0,n):
            data.append([])
    
    print("Live voltage readings:")
    print()
    try:
        while True:
            sys.stdout.write("\r")
            if single:
                a = readVoltage(n)
                sys.stdout.write(" ADC%d: " % n)
                sys.stdout.write("%f V" % a)
                if returnData:
                    data.append(a)
            else:
                for i in range(1,n+1):
                    a = readVoltage(i)
                    sys.stdout.write(" ADC%d: " % i)
                    sys.stdout.write("%f V" % a)
                    if returnData:
                        data[i-1].append(a)
            sys.stdout.write("    ")
            wait(wt)
    except:
        print()
        print("End of live measurements")
        print()
        
    # Compose return data if enabled    
    if returnData:
        if not scipy:
            return data
        else:    
            if single:
                return np.array(data)
            elif n == 1:
                return np.array(data[0])
            else:
                ret = []
                for i in range(0,n):
                    ret.append(np.array(data[i]))
                return ret    
        
   
################ PUBLIC COMPLEX DC FUNCTIONS ####################

def dcSweep(ndac,v1,v2,vi=0.1,wt=0.1):
    '''
    ----------------------------------------------------------------
    dcSweep(ndac,v1,v2,vi,wt)
    Performs a DC Sweep on a DAC and reads all ADCs at each point

    Required parameters:
      ndac : DAC to sweep
        v1 : Start voltage
        v2 : End voltage
        
    Optional parameters    
        vi : Increment (Defaults to 0.1 V)
        wt : Wait time at each step (Defaults to 0.1 s)
        
    Returns a list of vectors
       Vector 0 is the DAC value
       Vectorns 1 onward are ADC values    

    Included in slab.py
    ----------------------------------------------------------------    
    '''
    # Checks
    if not opened:
        raise SlabEx("Not connected to board")
    if v1 < 0.0:
        raise SlabEx("Voltage cannot be below 0 volts")
    if v1 > vref:
        raise SlabEx("Voltage cannot be over Vref")
    if v2 < 0.0:
        raise SlabEx("Voltage cannot be below 0 volts")
    if v2 > vref:
        raise SlabEx("Voltage cannot be over Vref")        
    if ndac < 1 or ndac > ndacs:
        raise SlabEx("Invalid DAC number")

    checkSciPy()
            
    message(1,"Performing mesurements...")        
         
    # Determine DAC range       
    xrange = np.arange(v1,v2,vi)
         
    # Initialize outputs
    out = [xrange]
    for i in range(0,nadcs):
        out.append([])   
           
    # Perform the measurements       
    for x in xrange:
        message(2,"  DAC at " + str(x) + " V")
        setVoltage(ndac,x)
        wait(wt)
        for i in range(0,nadcs):
            out[i+1].append(readVoltage(i+1))
    
    # Convert no np arrays
    for i in range(0,nadcs):
        out[i] = np.array(out[i])
    
    message(1,"Measurement ends")    
    
    # Return results
    return tuple(out)
    
################## PUBLIC PLOTTING FUNCTIONS ####################

def getVar(name,level=1):
    """
    Get a variable from its name
    Level indicates stack level
    Level 1 is appropiate if this function is called from the target context
    """
    # Get caller globals and locals
    caller_globals = dict(inspect.getmembers(inspect.stack()[level][0]))["f_globals"]
    caller_locals = dict(inspect.getmembers(inspect.stack()[level][0]))["f_locals"]
    # Get variable
    var = eval(name,caller_globals,caller_locals)
    # Return variable
    return var 

def interactivePlots(flag=True):
    """
    ----------------------------------------------------------------
    interactivePlots(flag)
    Activates interactive plotting on Jypyter
    Requires also setting the magic '%matplotlib notebook'
    Using the optinal parameter to False, return to inline mode
    
    Optional parametres:
        flag : Set interactive mode (defaults to True)
    
    Included in slab.py
    ----------------------------------------------------------------
    """
    global iplots
    if flag:
        iplots = True
        message(1,"Plots are now interactive")
        message(1,"Remember to also set '%matplotlib notebook' in Jupyter")
    else:
        iplots = False
        message(1,"Plots are static now")
        message(1,"Remember to also set '%matplotlib inline' in Jupyter")

'''
Plot two magnitudes using log if needed
Used by the plot11, plot1n and plotnn commands
'''
def plotXY(x,y,label="",logx=False,logy=False):
    if not logx and not logy:
        pl.plot(x,y,label=label)
        return
    if logx and not logy:
        pl.semilogx(x,y,label=label)
        return
    if logy and not logx:
        pl.semilogy(x,y,label=label)
        return
    if logx and logy:
        pl.loglog(x,y,label=label)
        return


def setPlotReturnData(value=False):
    '''
    -----------------------------------------------------------
    setPlotReturnData(value)
    Configures if plot commands shoud return the plotted data

    Optional parameters:
      value : By default is False
    Returns nothing 
    Included in slab.py
    -----------------------------------------------------------    
    '''
    global plotReturnData,gdata
    plotReturnData = value
    gdata['plotReturnData'] = plotReturnData


def plot11(x,y,title="",xt="",yt="",logx=False,logy=False,grid=True,hook=None,xlim=None,ylim=None):
    '''
    -----------------------------------------------------------
    plot11(x,y,title,xt,yt,logx,logy,grid,hook,xlim,ylim)
    Plot one input against one output
    If x is an empty list [], a sequence number
    will be used for the x axis

    Required parameters:
      x : Horizontal vector (string calls eval)
      y : Vertical vector (string calls eval)
      
    Optional parameters:
      title : Plot title (Defaults to none)
         xt : Label for x axis (Defaults to none or x string)
         yt : Label for y axis (Defaults to none or y string)
       logx : Use logarithmic x axis (Defaults to False)
       logy : Use logarithmic x axis (Defaults to False)
       grid : Draw a grid (Defaults to true) 
       hook : Function to be executed before showing the graph
       xlim : Tuple (min,max) with Limits for x axis
       ylim : Tuple (min,max) with Limits for y axis     

    Returns nothing
    -----------------------------------------------------------
    '''
    # Check for x, y given as strings
    if type(x)==str:
        if xt=='': xt=x
        x = getVar(x,level=2)
    if type(y)==str:
        if yt=='': yt=y
        y = getVar(y,level=2)
 
    # Generate sequence if x is not provided
    if x == []:
        x = np.arange(0,len(y))      
 
    plt.figure(facecolor="white")   # White border
    if xlim != None:
        plt.xlim(xlim[0],xlim[1])
    if ylim != None:
        plt.ylim(ylim[0],ylim[1])    
    plotXY(x,y,logx=logx,logy=logy)
    pl.xlabel(xt)
    pl.ylabel(yt)
    pl.title(title)
    if grid:
        pl.grid()
    if not hook is None:
        hook()    
    if not iplots:    
        pl.show()
        pl.close()
    

def plot1n(x,ylist,title="",xt="",yt="",labels=[],location='best'
            ,logx=False,logy=False,grid=True,hook=None,xlim=None,ylim=None):
    '''
    -----------------------------------------------------------
    plot1n(x,ylist,title,xt,yt,labels,location,logx,logy
            ,grid,hook,xlim,ylim)
    Plot one input against several outputs
    If x is an empty list [], a sequence number
    will be used for the x axis

    Required parameters:
          x : Horizontal vector (string calls eval)
      ylist : List of vertical vectors (list of strings calls eval)
      
    Optional parameters:
        title : Plot title (Defaults to none)
           xt : Label for x axis (Defaults to none or x string)
           yt : Label for y axis (Defaults to none)
       labels : List of legend labels (Defaults to none or ylist strings)
     location : Location for legend (Defaults to 'best')
         logx : Use logarithmic x axis (Defaults to False)
         logy : Use logarithmic x axis (Defaults to False)
         grid : Draw a grid (Defaults to true)
         hook : Function to be executed before showing the graph     
         xlim : Tuple (min,max) with Limits for x axis
         ylim : Tuple (min,max) with Limits for y axis 

    Returns nothing
    -----------------------------------------------------------
    '''            
    # Check for x, y given as strings
    if type(x)==str:
        if xt=='': xt=x
        x = getVar(x,level=2)
    if type(ylist[0])==str and (labels==[]): 
        labels=ylist
    ylist2 = []
    for element in ylist:
        if type(element) == str:
            ylist2.append(getVar(element,level=2))
        else:
            ylist2.append(element)
    ylist=ylist2        

    # Generate sequence is x is not provided
    if x == []:
        x = np.arange(0,len(ylist[0]))          
        
    plt.figure(facecolor="white")   # White border
    if xlim != None:
        plt.xlim(xlim[0],xlim[1])
    if ylim != None:
        plt.ylim(ylim[0],ylim[1])      
    if labels == []:
        for y in ylist:
            plotXY(x,y,logx=logx,logy=logy)
            #pl.plot(x,y)
    else:
        for y,lbl in zip(ylist,labels):
            plotXY(x,y,label=lbl,logx=logx,logy=logy)
            #pl.plot(x,y,label=lbl)
    pl.xlabel(xt)
    pl.ylabel(yt)
    pl.title(title)
    if grid:
        pl.grid()
    if not labels == []:
        pl.legend(loc=location)
    if not hook is None:
        hook()    
    if not iplots:    
        pl.show() 
        pl.close()    
  

def plotnn(xlist,ylist,title="",xt="",yt="",labels=[],location='best'
           ,logx=False,logy=False,grid=True,hook=None,xlim=None,ylim=None):
    '''
    -----------------------------------------------------------
    plotnn(xlist,ylist,title,xt,yt,labels,location,logx
                ,logy,grid,hook,xlim,ylim)
    Plot several curves with different inputs and outputs

    Required parameters:
      xlist : List of horizontal vector (list of strings calls eval)
      ylist : List of vertical vectors (list of strings calls eval)
      
    Optional parameters:
        title : Plot title (Defaults to none)
           xt : Label for x axis (Defaults to none or first string of xlist)
           yt : Label for y axis (Defaults to none)
       labels : List of legend labels (Defaults to none or ylist strings)
     location : Location for legend (Defaults to 'best')
         logx : Use logarithmic x axis (Defaults to False)
         logy : Use logarithmic x axis (Defaults to False)
         grid : Draw a grid (Defaults to true)
         hook : Function to be executed before showing the graph     
         xlim : Tuple (min,max) with Limits for x axis
         ylim : Tuple (min,max) with Limits for y axis  

    Returns nothing
    -----------------------------------------------------------
    '''           
    # Check for x, y given as strings
    if type(xlist[0])==str:
        if xt=='': xt=xlist[0]
    if type(ylist[0])==str and (labels==[]): 
        labels=ylist
        
    xlist2 = []
    for element in xlist:
        if type(element) == str:
            xlist2.append(getVar(element,level=2))
        else:
            xlist2.append(element)
    xlist=xlist2  
    
    ylist2 = []
    for element in ylist:
        if type(element) == str:
            ylist2.append(getVar(element,level=2))
        else:
            ylist2.append(element)
    ylist=ylist2    

    plt.figure(facecolor="white")   # White border
    if xlim != None:
        plt.xlim(xlim[0],xlim[1])
    if ylim != None:
        plt.ylim(ylim[0],ylim[1])      
    if labels == []:
        for x,y in zip(xlist,ylist):
            plotXY(x,y,logx=logx,logy=logy)
    else:
        for x,y,lbl in zip(xlist,ylist,labels):
            plotXY(x,y,label=lbl,logx=logx,logy=logy)
    pl.xlabel(xt)
    pl.ylabel(yt)
    pl.title(title)
    pl.grid()
    if not labels == []:
        pl.legend(loc=location)
    if not hook is None:
        hook()
    if not iplots:    
        pl.show()  
        pl.close()    
  
def plot3D(Xmesh,Ymesh,Zmesh,title='',Xlabel='x',Ylabel='y',Zlabel='z',angles=None):
    '''
    -----------------------------------------------------------
    plot3D(Xmesh,Ymesh,Zmesh,title,Xlabel,Ylabel,Zlabel)    
    Plot projection in 3D
    
    Required parameters:
        Xmesh : Point mesh in X
        Ymesh : Point mesh in Y
        Zmesh : Point mesh in Z
        
    Optional parameters:
        title  : Plot title
        Xlabel : Label for x axis
        Ylabel : Label for y axis
        Zlabel : Label for z axis   
        angles : (elev,azim) angles        
    
    Returns nothing
    -----------------------------------------------------------
    '''
    fig=plt.figure(figsize=(14,12),facecolor="white")   # White border
    ax = fig.gca(projection='3d')
    surf = ax.plot_surface(Xmesh, Ymesh, Zmesh, rstride=1, cstride=1
                , cmap=cm.coolwarm,linewidth=0, antialiased=False)

    ax.xaxis.pane.set_edgecolor('black')
    ax.yaxis.pane.set_edgecolor('black')
    ax.zaxis.pane.set_edgecolor('black')
    ax.xaxis.pane.fill = True
    ax.yaxis.pane.fill = True
    ax.zaxis.pane.fill = True
    
    # Set pane colors
    ax.xaxis.set_pane_color((0.8, 0.9, 0.9, 1.0))
    ax.yaxis.set_pane_color((0.9, 0.8, 0.9, 1.0))
    ax.zaxis.set_pane_color((0.9, 0.9, 0.8, 1.0))
       
    # Improve ticks and axes legend
    '''
    [t.set_va('center') for t in ax.get_yticklabels()]
    [t.set_ha('left') for t in ax.get_yticklabels()]
    [t.set_va('center') for t in ax.get_xticklabels()]
    [t.set_ha('right') for t in ax.get_xticklabels()]
    [t.set_va('center') for t in ax.get_zticklabels()]
    [t.set_ha('left') for t in ax.get_zticklabels()]
    '''
    
    ax.contour(Xmesh, Ymesh, Zmesh)
          
    if angles is not None:      
        ax.view_init(angles[0],angles[1])
    ax.set_title(title,fontsize=20)
    ax.set_xlabel(Xlabel,fontsize=20)
    ax.set_ylabel(Ylabel,fontsize=20)
    ax.set_zlabel(Zlabel,fontsize=20,rotation='vertical')
    
    pl.show() 
    pl.close()
 
def plotFunc3D(f,xrange,yrange,title='',Xlabel='x',Ylabel='y',Zlabel='z'
                ,naxis=200,clip=None,fpost=None,angles=None): 
    '''
    -----------------------------------------------------------
    plotFunc3D(f,xrange,yrange,title,Xlabel,Ylabel,Zlabel
                ,naxis=200)    
    Plot a function in 3D
    
    Required parameters:
        f      : Function f(x,y) to obtain z
        xrange : Tuple or list with (minX,maxX)
        yrange : Tuple or list with (minY,maxY)
        
    Optional parameters:
        title  : Plot title (defaults to none)
        Xlabel : Label for x axis (defaults to none)
        Ylabel : Label for y axis (defaults to none)
        Zlabel : Label for z axis  (defaults to none)
        naxis  : Number of points in X and Y axes (defaults to 200)
        clip   : Tuple of list with Z clipping (minZ,maxZ)
        fpost  : Postprocessing function (defaults to none)      
    
    Returns nothing
    -----------------------------------------------------------
    '''
    # Create a vector with naxis points for X
    xr = np.linspace(xrange[0],xrange[1],naxis)
    # Create a vector with naxis points for Y
    yr = np.linspace(yrange[0],yrange[1],naxis)
    # Create a mesh for generating a 3D plot
    X,Y = np.meshgrid(xr,yr)
    # Evaluate the function in all points in the mesh
    Z = f(X,Y)
    # Check post processing function
    if fpost is not None:
        Z = fpost(Z)
    # Check clipping
    if clip is not None:
        Z = np.clip(Z,clip[0],clip[1])
    # Perform the plotting    
    plot3D(X,Y,Z,title,Xlabel,Ylabel,Zlabel,angles)
    
######################### DC SWEEP PLOT ############################


def dcSweepPlot(ndac,v1,v2,vi=0.1,na=None,wt=0.1,returnData=False):
    '''
    -----------------------------------------------------------
    dcSweepPlot(ndac,v1,v2,vi,na,wt,returnData)
    Plots the results of a DC sweep

    Required parameters: 
     ndac : DAC to sweep
       v1 : Initial value (in Volt)
       v2 : End of range (in Volt)
      
    Optional parameters:  
      vi : Step (defaults to 0.1V)  
      na : Number of ADCs to show (defaults to all)
      wt : Waiting time between steps (defaults to 0.1s)
      returnData : Enable return of plot data (Defaults to False)
      
    Returns plot data if enabled (see also setPlotReturnData)
    Included in slab.py
    -----------------------------------------------------------    
    ''' 

    # Check if na was provided
    if na == None: na = nadcs

    # Check if SciPy is loaded
    if not scipy:
        cannotPlot(exception=True)
        return

    # Perform a DC sweep
    res = dcSweep(ndac,v1,v2,vi,wt)
    
    if na == 1:
        # Plot result
        message(1,"Drawing curve")
        plot11(res[0],res[1],"V(V) Plot","DAC"+str(ndac)+" (V)","ADC1 (V)")
        ret = [res[0],res[1]]
    else:
        ylist=[]
        labels=[]
        ret=[res[0]]
        for i in range(0,na):
            ylist.append(res[i+1])
            ret.append(res[i+1])
            labels.append("ADC"+str(i+1))
        plot1n(res[0],ylist,"DC Sweep Plot","DAC"+str(ndac)+" (V)","ADC(V)",labels)
    
    if plotReturnData or returnData:
        return ret
   
   
######################### REALTIME PLOT ############################   

'''
def realtimePlot(nadc=1,wt=0.2,n=0,returnData=False):

    -----------------------------------------------------------
    realtimePlot(nadc,wt,returnData)
    Plots ADC data in real time

    Optional parameters:
            nadc : Number of ADCs to read (Defaults to 1)
              wt : Wait time between read (Defaults to 0.2s)
               n : Number of points to show (Defaults to All)
      returnData : Returns captured data if true (Defaults to False)
      
    Returns a time+nadc list if returnData is true  
    Included in slab.py   
    Currently does not work inside Jupyter
    -----------------------------------------------------------


    nn=int(n)

    # Checks
    if not opened:
        raise SlabEx("Not connected to board")  
    if nadc < 1 or nadc > nadcs:
        raise SlabEx("Invalid number of ADCs")
    
    # Initialize the data arrays    
    vt = []
    va = []
    for i in range(0,nadc):
        va.append([])
        
    message(1,"Entering realtime plot")
    message(1,"Close the graph window to exit")

    #plt.ion()    
    
    fig=plt.figure(facecolor="white")   # White border
    ax=fig.add_subplot(1,1,1)
    pl.xlabel("time (s)")
    pl.ylabel("Value (V)")
    pl.title("Realtime Plot")
    pl.grid()
    
    labels=[]
    for i in range(0,nadc):
        labels.append("ADC"+str(i+1))
        
    toffs=-1.0
    try:    
        while True:
            t = time.time()
            if toffs < 0.0:
                toffs=t
            t=t-toffs    
            vt.append(t)
            
            for i in range(0,nadc):
                value = readVoltage(i+1)
                va[i].append(value)
                
            ax.cla()    
            ax.grid()
            pl.xlabel("time (s)")
            pl.ylabel("Value (V)")
            pl.title("Realtime Plot") 
            
            for i in range(0,nadc):
                if nn:
                    ax.plot(vt[-nn:-1], va[i][-nn:-1], label=labels[i])
                else:
                    ax.plot(vt, va[i], label=labels[i])
                
            if nadc>1:
                pl.legend(loc='lower right')   
            
            fig.canvas.draw()
            plt.pause(wt)
               
    except:
        print()
        print("End of realtime measurements")
        print()

    plt.close()

    plot1n(vt,va,"Measurement Plot","time (s)","Value (V)",labels)   

    if plotReturnData or returnData:
        ret = vt
        ret.extend(va)
        return ret 
'''    
    
def realtimePlot(nadc=1,wt=0.2,n=0,returnData=False):
    '''
    -----------------------------------------------------------
    realtimePlot(nadc,wt,returnData)
    Plots ADC data in real time

    Optional parameters:
            nadc : Number of ADCs to read (Defaults to 1)
              wt : Wait time between read (Defaults to 0.2s)
               n : Number of points to show (Defaults to All)
      returnData : Returns captured data if true (Defaults to False)
      
    Returns a time+nadc list if returnData is true  
    Included in slab.py   
    Currently does not work inside Jupyter
    -----------------------------------------------------------
    '''

    nn=int(n)

    # Checks
    if not opened:
        raise SlabEx("Not connected to board")  
    if nadc < 1 or nadc > nadcs:
        raise SlabEx("Invalid number of ADCs")
    
    # Initialize the data arrays    
    vt = []
    va = []
    for i in range(0,nadc):
        va.append([])
        
    message(1,"Entering realtime plot")
    message(1,"Interrupt to exit")

    #plt.ion()    
    
    fig=plt.figure(facecolor="white")   # White border
    ax=fig.add_subplot(1,1,1)
    pl.xlabel("time (s)")
    pl.ylabel("Value (V)")
    pl.title("Realtime Plot")
    pl.grid()
    
    labels=[]
    for i in range(0,nadc):
        labels.append("ADC"+str(i+1))
        
    toffs=-1.0
    try:    
        while True:
            t = time.time()
            if toffs < 0.0:
                toffs=t
            t=t-toffs    
            vt.append(t)
            
            for i in range(0,nadc):
                value = readVoltage(i+1)
                va[i].append(value)
                
            ax.cla()    
            ax.grid()
            pl.xlabel("time (s)")
            pl.ylabel("Value (V)")
            pl.title("Realtime Plot") 
            
            for i in range(0,nadc):
                if nn:
                    ax.plot(vt[-nn:-1], va[i][-nn:-1], label=labels[i])
                else:
                    ax.plot(vt, va[i], label=labels[i])
                
            if nadc>1:
                pl.legend(loc='lower right')   
            
            if iplots:
                fig.canvas.draw()
                fig.canvas.draw_idle()
                wait(wt)
            else:
                fig.canvas.draw()
                plt.pause(wt)
               
    except:
        print()
        print("End of realtime measurements")
        print()

    plt.close()
    
    if not iplots:
        plot1n(vt,va,"Measurement Plot","time (s)","Value (V)",labels)   

    if plotReturnData or returnData:
        ret = [vt]
        ret.extend(va)
        return ret 
    
   
################ PUBLIC BASIC TRANSIENT FUNCTIONS ####################    


def setSampleTime(st):
    '''
    ----------------------------------------------------
    setSampleTime(st)
    Set sample time for time measurements
    Resolution on sample time is limited so value set 
    can be different of the input value
    Check your hardware board limits

    Required parameter:
      st : Sample time (float seconds)
      
    Returns real sample time set
    Included in slab.py
    ----------------------------------------------------    
    '''
    global sampleTime,gdata
    
    # Check
    if st > 100.0:
        raise SlabEx("Sample time too high")
    if st < 0.000001:  
        raise SlabEx("Sample time too low") 
        
    startCommand('R')
    sampleTime = sendFloat(st)
    gdata['sampleTime'] = sampleTime
    sendCRC()
    
    checkACK()
    checkCRC()
    
    return sampleTime
 

'''
Check of storage space
Gives exception if there is not enough space
'''
def checkBuffSpace(samples,na,nd=0):
    # Calculate space    
    space = buff_size - w_points - w_points2
    if nd:
        required = samples * (na+1)
    else:        
        required = samples * na 
    if space < required:
        raise SlabEx("Not enough buffer space. Only " + str(space) + " samples free")
 
    
def setTransientStorage(samples,na=1,nd=0):
    '''
    ------------------------------------------------------------------
    setTransientStorage(samples,na,nd)
    Set storage for samples in transient time measurements
    Check your hardware board limits

    Required parameters:
      samples : Number of samples to obtain
      
    Optional parameters:  
      na : Number of ADC analog signals to record (Defaults to 1)
      nd : Number of digital lines to record (Defaults to 0)
           
    Returns nothing
    Included in slab.py 

    This command has an alias tranStore
    ------------------------------------------------------------------
    '''
    if (nd < 0) or (nd >= ndio):
        raise SlabEx("Ilegal number of dio lines")
    
    # Check space
    checkBuffSpace(samples,na,nd)
    
    startCommand('S')
    sendByte(na)
    sendByte(nd)
    sendU16(samples)
    sendCRC()
    
    checkACK()
    checkCRC()
   
 
def tranStore(samples,na=1,nd=0):
    '''
    -------------------------------------------
    tranStore(samples,na)
    Alias for the command setTransientStorage
    Included in slab.py
    -------------------------------------------    
    '''  
    setTransientStorage(samples,na,nd)
   
  
def transientAsync():
    '''
    ------------------------------------------------
    transientAsync()
    Performs an asynchronous transient measurement

    Returns a list of vectors
      Vector 0 is time
      Vectors 1 onward are ADC readings
      
    Included in slab.py  
    See also setSampleTime and setTransientStorage
    ------------------------------------------------    
    ''' 

    global na,nd
    
    message(1,"Performing transient measurement..." )
   
    startCommand('Y')
    sendCRC()
    
    checkACK()
    
    # Check for overrun or other errors
    code = getByte()
    if code == 1:
        checkCRC()
        raise SlabEx("Sample overrun")
    if code == 3:
        checkCRC()
        raise SlabEx("Halt from board")
    if code != 0: 
        raise SlabEx("Unknown transient response code")
    
    message(1,"Mesurement ends. Receiving data")
    
    na = getByte()       # Get number of analog channels
    nd = getByte()       # Get number of digital channels
    samples = getU16()   # Get number of samples
    
    result = []  # Initialize result vector
    
    # Set time scale
    vector = []
    for s in range(0,samples):
        vector.append(s*sampleTime)
    if scipy:    
        result.append(np.array(vector))
    else:
        result.append(vector)    
        
    # Read analog channels   
    if (na):    
        for i in range(0,na):
            vector = []
            for s in range(0,samples):
                fvalue = dc_cal(getU16()/65536.0,xcal,adcCalData[i])
                fvalue = fvalue * vref
                vector.append(fvalue)
            if scipy:    
                result.append(np.array(vector))
            else:
                result.append(vector)
                
    # Read digital channels
    if (nd):
        vector = []
        for s in range(0,samples):
            vector.append(getU16())
        if scipy:    
            result.append(np.array(vector))
        else:
            result.append(vector)    

    checkCRC()    
        
    message(1,"Data received")
        
    return result    
        

def transientTriggered(level,mode=tmodeRise,timeout=0):
    '''
    --------------------------------------------------------
    transientTriggered(level,mode,timeout)
    Performs a triggered transient measurement
    Mesuremenst will be centered at the trigger point

    Required parameters:
      level : Trigger level (float voltage)
      
    Optional parameters:  
       mode : Trigger mode (tmodeRise or tmodeFall)
              (Defaults to tmodeRise)
       timeout : Timeout in integer seconds (Defaults to no timeout)       
       
    Returns a list of vectors
      Vector 0 is time
      Vectors 1 onward are ADC readings   
       
    Mode can also be defined with the strings 'rise' and 'fall'   
       
    Included in slab.py    
    See also setSampleTime and setTransientStorage
    --------------------------------------------------------    
    ''' 
    
    global na,nd
    
    # Check timeout
    timeout = int(timeout)
    if timeout > 255:
        raise SlabEx("Timeout limited to 255 seconds")
    if timeout < 0:
        raise SlabEx("Timeout cannot be negative")

    # Check mode string
    if type(mode) is str:
        mode = cDict[mode]
        
    # Convert level to uint16 considering calibration
    ratio = voltage2ratio(level)
    cal_ratio = dc_cal(ratio,adcCalData[0],xcal) # Reverse calibration
    counts = ratio2counts(cal_ratio)
        
    message(1,"Performing transient triggered measurement...")
        
    startCommand('G')
    sendU16(counts)
    sendByte(mode)
    sendByte(timeout)
    sendCRC()
    
    # Receive data
    checkACK()
    
    # Check for overrun or other errors
    code = getByte()
    if code == 1:
        checkCRC()
        raise SlabEx("Sample overrun")
    if code == 2:
        checkCRC();
        raise SlabEx("Timeout error")
    if code == 3:
        checkCRC()
        raise SlabEx("Halt from board")
    if code != 0: 
        raise SlabEx("Unknown transient response code")        

    
    message(1,"Mesurement ends. Receiving data")
    
    na = getByte()
    nd = getByte()
    samples = getU16()
    
    # Initialize results list
    result = []
    
    # Determine the trigger sample
    tsample = (samples//2) - 1
    
    # Set time scale
    vector = []
    for s in range(0,samples):
        vector.append((s - tsample)*sampleTime)
    if scipy:    
        result.append(np.array(vector))
    else:
        result.append(vector)
        
    # Read analog channels    
    if na:    
        for i in range(0,na):
            vector = []
            for s in range(0,samples):
                fvalue = dc_cal(getU16()/65536.0,xcal,adcCalData[i])
                fvalue = fvalue * vref
                vector.append(fvalue)
            if scipy:    
                result.append(np.array(vector))
            else:
                result.append(vector)
                
    # Read digital channels
    if (nd):
        vector = []
        for s in range(0,samples):
            vector.append(getU16())
        if scipy:    
            result.append(np.array(vector))
        else:
            result.append(vector)                    

    checkCRC()    
        
    message(1,"Data received")
        
    return result          
    
  
def stepResponse(v1,v2,tinit=1.0):
    '''
    -----------------------------------------------------------
    stepResponse(v1,v2,t)
    Obtains the Step Response for a circuit
      1/5 of measurement time will be before the step
      4/5 of measurement time will be after the step
      
    Required parameters:
           v1 : Start voltage
           v2 : End voltage
           
    Optional parameters:       
        tinit : Time before start in seconds (defaults to 1 s)
        
    Returns a list of vectors
      Vector 0 is time
      Vectors 1 onward are ADC readings   

    Included in slab.py   
    See also setSampleTime and setTransientStorage
    -----------------------------------------------------------    
    ''' 
    
    global na,nd
    
    message(1,"Performing step response...")
   
    setVoltage(1,v1)
    time.sleep(tinit)
    v2cal = dc_cal(v2,dacx,dacCalData[0])  
    counts = voltage2counts(v2)
    
    readADC(1);  # Precharge ADC inputs
    readADC(2);  # and discard the reading
    readADC(3);
    readADC(4);
    
    startCommand('P')
    sendU16(counts)
    sendCRC()
    
    # Receive data
    checkACK()
    
    # Check for overrun or other errors
    code = getByte()
    if code == 1:
        checkCRC()
        raise SlabEx("Sample overrun")
    if code == 3:
        checkCRC()
        raise SlabEx("Halt from board")
    if code != 0: 
        raise SlabEx("Unknown transient response code")           
    
    message(1,"Mesurement ends. Receiving data")
    
    na = getByte()
    nd = getByte()
    samples = getU16()
    result = []
    
    # Determine the trigger sample
    tsample = samples / 5
    
    # Time vector
    vector = []
    for s in range(0,samples):
        vector.append((s - tsample)*sampleTime)
    if scipy:    
        result.append(np.array(vector))
    else:
        result.append(vector)
        
    # Read analog channels    
    if na:    
        for i in range(0,na):
            vector = []
            for s in range(0,samples):
                fvalue = dc_cal(getU16()/65536.0,xcal,adcCalData[i])
                fvalue = fvalue * vref
                vector.append(fvalue)
            if scipy:    
                result.append(np.array(vector))
            else:
                result.append(vector)
                
    # Read digital channels
    if (nd):
        vector = []
        for s in range(0,samples):
            vector.append(getU16())
        if scipy:    
            result.append(np.array(vector))
        else:
            result.append(vector) 
        
    checkCRC()

    setVoltage(1,v1)
    
    message(1,"Data received")
        
    return result      


############ PRIVATE TRAN MEASURE PLOT #########################

def getDigital(vector,pos):
    out = []
    for data in vector:
        if data & (1<<pos):
            out.append(1)
        else:
            out.append(0)
    return np.array(out)

def tranPlot(na,nd,data,title):
    # Initialize data to plot
    yvectors = []
    labels = []
    # Horizontal axis
    time = data[0]
    # Analog data
    for i in range(0,na):
        yvectors.append(data[1+i])
        labels.append('ADC'+str(i+1))
    # Digital data
    for i in range(0,nd):
        vector = getDigital(data[1+na],i)
        vector = -0.5*(i+1)+0.4*vector
        yvectors.append(vector)
        labels.append('DIO'+str(i))
    # Plot 
    plot1n(time,yvectors,title,'time (s)','ADC (V) or Digital',labels)    
    
################## PUBLIC AC PLOT FUNCTIONS ####################        


def tranAsyncPlot(returnData=False):
    '''
    -----------------------------------------------------------
    tranAsyncPlot(returnData)
    Plots an asynchronous transient measurement

    Optional parameter:
      returnData : Enable return of plot data (Defaults to False)

    Returns plot data if enabled
      Vector 0 is time
      Vectors 1 onward are ADC readings

    Included in slab.py   
    See also setSampleTime, setTransientStorage and setPlotReturnData
    -----------------------------------------------------------
    '''

    if not scipy:
        cannotPlot(exception=True)
        return

    # Perform measurement
    res = transientAsync()
    
    # Plot result
    message(1,"Drawing curves")
    
    tranPlot(na,nd,res,"Async Transient Plot")
    
    if plotReturnData or returnData:
        return res    


def tranTriggeredPlot(level,mode=tmodeRise,timeout=0,returnData=False):
    '''
    -----------------------------------------------------------
    tranTriggeredPlot(level,mode,timeout,returnData)
    Plots a triggered transient measurement
    Mesuremenst will be centered at the trigger point

    Required parameters:
      level : Trigger level (float voltage)
      
    Optional parameters:  
       mode : Trigger mode (tmodeRise or tmodeFall)
              (Defaultst to tmodeRise)
       timeout : Timeout in integer seconds (Defaults to no timeout)           
       returnData : Enable return of plot data (Defaults to False)
       
    Returns plot data if enabled
      Vector 0 is time
      Vectors 1 onward are ADC readings   

    Mode can also be defined with the strings 'rise' and 'fall'  
      
    Included in slab.py   
    See also setSampleTime, setTransientStorage and setPlotReturnData
    -----------------------------------------------------------
    '''

    if not scipy:
        cannotPlot(exception=True)
        return

    # Perform measurement
    res = transientTriggered(level,mode,timeout)
    
    # Plot result
    message(1,"Drawing curves")
    
    tranPlot(na,nd,res,"Transient Triggered Plot")    
    
    if plotReturnData or returnData:
        return res    


def stepPlot(v1,v2,tinit=1.0,returnData=False):
    '''
    -----------------------------------------------------------
    stepPlot(v1,v2,tinit,returnData)  
    Plots the Step Response for a circuit
      1/5 of measurement time will be before the step
      4/5 of measurement time will be after the step
      
    Required parameters:
      v1 : Start voltage
      v2 : End voltage
           
    Optional parameters:       
      tinit : Time before start in seconds (defaults to 1 s)
      returnData : Enable return of plot data (Defaults to False)
        
    Returns plot data if enabled (see setPlotReturnData) 
      Vector 0 is time
      Vectors 1 onward are ADC readings   

    Included in slab.py   
    See also setSampleTime, setTransientStorage and setPlotReturnData
    -----------------------------------------------------------
    '''

    if not scipy:
        cannotPlot(exception=True)
        return

    # Perform measurement
    res = stepResponse(v1,v2,tinit)
    
    # Plot result
    message(1,"Drawing curves")
    
    tranPlot(na,nd,res,"Step Response Plot")
    
    if plotReturnData or returnData:
        return res    
 
################## WAVETABLE COMMANDS #################


def loadWavetable(list,second=False): 
    '''
    -----------------------------------------------------------
    loadWavetable(list,second=False)
    Load one wavetable on the hardware board
    Loading a primary wavetable erases the secondary if present

    Required parameters:
      list : List of values of the wavetable
             If empty [] the wavetable will be erased
        
    Optional parameters:
      second : Load secondary wavetable for DAC2
               (Defaults to false)    

    Included in slab.py            
    Returns nothing
    -----------------------------------------------------------
    '''
    global w_idle,w_idle2,w_points,w_points2

    # Get list size
    size = len(list)
        
    # Checks
    if not opened:
        raise SlabEx("Not connected to board")    
    if size > buff_size :
        raise SlabEx("Wavetable too big")
        
    # Additional check for secondary wavetable
    if second:
        if size > buff_size - w_points:
            raise SlabEx("Not enough space for secondary wavetable")
        
    # Send data
    if not second:
        w_points = size      # Size of main wavetable
        if size > 0:         # Iddle value (Volt) 
            w_idle = list[0]    
        else:
            w_idle = -1        
        w_idle2 = -1         # Eliminate secondary wavetable
        w_points2 = 0
        startCommand('W')    # Start
    else:
        w_points2 = size     # Size of secondary wavetable
        if size > 0:
            w_idle2 = list[0]    # Iddle value (Volt)
        else:
            w_idle2 = -1
        startCommand('w')    # Start
        
    w_idleD = -1    # Eliminate digital wavetable
    w_pointsD = 0    
        
    sendU16(size)
    if size > 0:
        for value in list:
            ratio = value/vref
            if not second:
                rCal = dc_cal(ratio,dacx,dacCalData[0]) 
            else:
                rCal = dc_cal(ratio,dacx,dacCalData[1])
            counts = ratio2counts(rCal)
            sendU16(counts)
        
    sendCRC()
    
    checkACK()
    checkCRC()    
        
    if not second and size > 0:    
        # Inform on frequency only on main wave
        fmax = 1.0/(w_points * min_sample)
        fmin = 1.0/(w_points * max_sample)
        message(1,str(w_points) + " point wave loaded")
        message(1,"Wave frequency must be between " + "{:.6f}".format(fmin) + " and " + "{:.2f}".format(fmax) + " Hz")
        frequency = 1.0 /(sampleTime * w_points)
        message(1,"Current frequency is " + str(frequency) + " Hz")
        
    # Inform on space
    space = buff_size - w_points - w_points2
    message(1,"Remaining buffer space is " + str(space) + " samples")
 
 
def loadDigitalWavetable(list,mask=0): 
    '''
    -----------------------------------------------------------
    loadDigitalWavetable(list,mask)
    Load a digital wavetable on the hardware board

    Required parameters:
      list : List of values of the wavetable
             If empty [] the wavetable will be erased
        
    Optional parameters:
      mask : Mask of lines to change (Defaults to all)    

    Included in slab.py            
    Returns nothing
    -----------------------------------------------------------
    '''
    global w_pointsD,w_idleD
    
    global w_idle,w_idle2,w_points,w_points2

    # Get list size
    size = len(list)
        
    # Checks
    if not opened:
        raise SlabEx("Not connected to board")    
        
    # Check for size
    if size > buff_size - w_points - w_points2:
        raise SlabEx("Not enough space for digital wavetable")
        
    # Set iddle
    if size > 0:
        w_idleD = list[0]  # Iddle value (Volt)
        w_mask = mask
    else:
        w_idleD = -1
        w_pointsD = 0
        w_mask = 0
        
    w_pointsD = size
    
    startCommand('O') # Start
    
    # Send table size     
    sendU16(size)
    
    # Send table mask
    sendU16(mask)
    
    # Send samples
    if size > 0:
        for value in list:
            sendU16(value)
        
    sendCRC()
    
    checkACK()
    checkCRC()    
        
    # Inform on space
    space = buff_size - w_points - w_points2 - w_pointsD
    message(1,"Remaining buffer space is " + str(space) + " samples") 

def waveSquare(v1,v2,np,returnList=False,second=False):
    '''
    -----------------------------------------------------------
    waveSquare(v1,v2,np,returnList,second)
    Loads square wavetable omn the hardware board

    Required parameters:
      v1 : Start value
      v2 : End value
      np : Number of points for a full wave
        
    Optional parameters:    
     returnList : Request a return list (Default False)
         second : Load on secondary table
                  (Defaults to false) 
                   
    If returnList is True, returns the table of loaded values
    Included in slab.py
    -----------------------------------------------------------
    ''' 
    # Check
    if not opened:
        raise SlabEx("Not connected to board")
    if np < 4:
        raise SlabEx("Not enough points for wave")
        
    # Create wave
    list = []
    for point in range(0,np):
        if point < np/2.0:
            list.append(v1)
        else:
            list.append(v2)
            
    # Program wave 
    loadWavetable(list,second)
    
    # Return list
    if returnList:
        return list


def wavePulse(v1,v2,np,n1,returnList=False,second=False):
    '''
    -----------------------------------------------------------
    wavePulse(v1,v2,np,n1,returnList,second)
    Loads a pulse wavetable on the hardware board

    Parameters:
      v1 : Start value
      v2 : End value
      np : Number of points for a full wave 
      n1 : Number of points at v1
               
    Optional parameters:    
     returnList : Request a return list (Default False)
         second : Load on secondary table
                  (Defaults to false)   
                  
    If returnList is True, returns the table of loaded values
    Included in slab.py
    -----------------------------------------------------------    
    ''' 
    # Check
    if not opened:
        raise SlabEx("Not connected to board")
    if np < 4:
        raise SlabEx("Not enough points for wave")
        
    # Create wave
    list = []
    for point in range(0,np):
        if point < n1:
            list.append(v1)
        else:
            list.append(v2)
            
    # Program wave 
    loadWavetable(list,second)    
    
    # Return list
    if returnList:
        return list
    

def waveTriangle(v1,v2,np,returnList=False,second=False):
    '''
    -----------------------------------------------------------
    waveTriangle(v1,v2,np,n1,returnList,second)
    Loads a triangle wavetable on the hardware board

    Parameters:
       v1 : Minimum value
       v2 : Maximum value
       np : Number of points for a full wave 
               
    Optional parameters:    
     returnList : Request a return list (Default False)
         second : Load on secondary table
                  (Defaults to false)   
                  
    If returnList is True, returns the table of loaded values
    Included in slab.py 
    -----------------------------------------------------------
    '''
    # Check
    if not opened:
        raise SlabEx("Not connected to board")
    if np < 4:
        raise SlabEx("Not enough points for wave")

    # Create wave
    list = []
    for point in range(0,np):
        point = (point + np//4) % np
        if point < np/2.0:
            value = v1 + 2.0*(v2-v1)*point/np
        else:
            value = v1 + 2.0*(v2-v1)*(np - point)/np
        list.append(value)
        
    # Program wave 
    loadWavetable(list,second)  

    # Return list
    if returnList:
        return list
    

def waveSawtooth(v1,v2,np,returnList=False,second=False):
    '''
    -----------------------------------------------------------
    waveSawtooth(v1,v2,np,returnList,second)
    Loads a sawtooth wavetable on the hardware board

    Parameters:
        v1 : Start value
        v2 : End value
        np : Number of points for a full wave 
               
    Optional parameters:    
     returnList : Request a return list (Default False)
         second : Load on secondary table
                  (Defaults to false)     
       
    If returnList is True, returns the table of loaded values
    Included in slab.py 
    -----------------------------------------------------------
    '''
    # Check
    if not opened:
        raise SlabEx("Not connected to board")
    if np < 4:
        raise SlabEx("Not enough points for wave")
        
    # Create wave
    list = []
    for point in range(0,np):
        value = v1*1.0 + (v2*1.0-v1*1.0)*point/np
        list.append(value)
        
    # Program wave 
    loadWavetable(list,second) 

    # Return list
    if returnList:
        return list   


def waveSine(v1,v2,np,phase=0.0,returnList=False,second=False):
    '''
    -----------------------------------------------------------
    waveSine(v1,v2,np,phase,returnList,second)

    Generates a sine wavetable
    Parameters:
      v1 : Minimum value
      v2 : Maximum value
      np : Number of points for a full wave 
               
    Optional parameters:    
          phase : Phase of the signal (deg) (Defaults to 0)
     returnList : Request a return list (Default False)
         second : Load on secondary table
                  (Defaults to false)     
       
    If returnList is True, returns the table of loaded values
    Included in slab.py 
    -----------------------------------------------------------
    '''
    # Check
    if not opened:
        raise SlabEx("Not connected to board")
    if np < 4:
        raise SlabEx("Not enough points for wave")

    # Create wave
    phase = phase*math.pi/180.0
    list = []
    mean = (v1 + v2)/2.0
    amplitude = (v2 - v1)/2.0
    for point in range(0,np):
        value = mean + amplitude*math.sin(2.0*math.pi*point/np+phase)
        list.append(value)
        
    # Program wave 
    loadWavetable(list,second)  

    # Return list
    if returnList:
        return list     
 

def waveCosine(v1,v2,np,phase=0.0,returnList=False,second=False):
    '''
    -----------------------------------------------------------
    waveCosine(v1,v2,np,returnList,second)

    Generates a cosine wavetable
    Parameters:
      v1 : Minimum value
      v2 : Maximum value
      np : Number of points for a full wave 
               
    Optional parameters:    
           phase : Phase of the signal (deg) (Defaults to 0)
      returnList : Request a return list (Default False)
          second : Load on secondary table
                   (Defaults to false)     
       
    If returnList is True, returns the table of loaded values
    Included in slab.py 
    -----------------------------------------------------------
    '''
    # Check
    if not opened:
        raise SlabEx("Not connected to board")
    if np < 4:
        raise SlabEx("Not enough points for wave")

    # Create wave
    phase = phase*math.pi/180.0
    list = []
    mean = (v1 + v2)/2.0
    amplitude = (v2 - v1)/2.0
    for point in range(0,np):
        value = mean + amplitude*math.cos(2.0*math.pi*point/np+phase)
        list.append(value)
        
    # Program wave 
    loadWavetable(list,second)  

    # Return list
    if returnList:
        return list  
        

def waveNoise(vm,vstd,n,returnList=False,second=False):
    '''
    -----------------------------------------------------------
    waveNoise(vm,vstd,n,returnList,second)

    Generates a noise wavetable
    Based on a normal distribution
    Samples are truncated between 0 and Vref

    Parameters:
        vm : Mean value
      vstd : Standard deviation
         n : Number of points
               
    Optional parameters:    
       returnList : Request a return list (Default False)
           second : Load on secondary table
                    (Defaults to false)     
       
    If returnList is True, returns the table of loaded values
    Included in slab.py 
    -----------------------------------------------------------
    '''
    # Check
    if not opened:
        raise SlabEx("Not connected to board")
    if n < 4:
        raise SlabEx("Not enough points for wave")

    # Create wave
    list = np.random.normal(loc=vm,scale=vstd,size=n)
    for i in range(0,n):
        if list[i] > vref:
            list[i] = vref
        if list[i] < 0.0:
            list[i] = 0
        
    # Program wave 
    loadWavetable(list,second)  

    # Return list
    if returnList:
        return list          
 

def waveRandom(v1,v2,n,returnList=False,second=False):
    '''
    -----------------------------------------------------------
    waveRandom(v1,v2,n,returnList,second)

    Generates a random wavetable
    Based on a uniform distribution
    Samples will be random values between v1 and v2

    Parameters:
       v1 : Minimum voltage
       v2 : Maximum voltage
        n : Number of points
               
    Optional parameters:    
       returnList : Request a return list (Default False)
           second : Load on secondary table
                    (Defaults to false)     
       
    If returnList is True, returns the table of loaded values
    Included in slab.py
    -----------------------------------------------------------    
    '''
    # Check
    if not opened:
        raise SlabEx("Not connected to board")
    if n < 4:
        raise SlabEx("Not enough points for wave")
    if v1 >= v2 :
        raise SlabEx("v1 must be lower than v2")

    # Create wave
    list = v1 + (v2-v1)*np.random.random(size=n)
        
    # Program wave 
    loadWavetable(list,second)  

    # Return list
    if returnList:
        return list   
 
 
def setWaveFrequency(freq):
    '''
    -------------------------------------------------
    setWaveFrequency(freq)
    Set wave frequency by changing sample frequency 

    Required parameters:
       freq : Wave frequency in Hz
       
    Return sampleTime set
    Included in slab.py
    -------------------------------------------------    
    '''  
    # Checks
    if freq <= 0.0:
        raise SlabEx("Frequency cannot be negative or zero")
    if w_idle == -1:
        raise SlabEx("No wave loaded")
    # Calculate sample time
    st = 1.0/(w_points * freq)
    if st < min_sample:
        raise SlabEx("Frequency too high")
    if st > max_sample:
        raise SlabEx("Frequency too low")
    # Change sample time
    st = setSampleTime(st)
    frequency = 1/(st * w_points)
    message(1,"Sample time set to " +str(st) + " s")
    message(1,"Wave frequency set to " +str(frequency) + " Hz")
    return st
    
    
 
################## WAVE RESPONSE COMMANDS ################# 
 

def waveResponse(npre = 0,tinit = 1.0,dual=False):
    '''
    -------------------------------------------------
    waveResponse(npre,tinit,dual)
    Obtain the response of a circuit against a wave

    Measurement sequence:
      1) Set DAC1 to first wave sample during tinit
      2) Send npre waves to DAC1
      3) Start measurement as set on setTransientStorage
         During this time wave continues to be generated

    Optional parameters:  
      npre : Number of waves before measurement (default to zero)
     tinit : Time iddle before first wave (default to 1s)
      dual : Use dual DAC generation (defaults to False)
     
    Returns a list of vectors:
      Vector 0 is time
      Vectors 1 onward are ADC and DIO readings          
          
    Included in slab.py       
    See also setWaveFrequency and setTransientStorage
    -------------------------------------------------
    ''' 
    
    global na,nd
    
    # Checks
    if not opened:
        raise SlabEx("Not connected to board")  
    if npre < 0:
        raise SlabEx("Invalid number of waves")
    if w_idle < 0:
        if w_idleD < 0:
            raise SlabEx("Wavetable not loaded")
    if dual and w_idle2 < 0:
        raise SlabEx("Secondary wavetable not loaded")

    message(1,"Performing wave measurement...")
        
    # Idle start
    if tinit > 0.0:
        if w_idle >= 0:
            setVoltage(1,w_idle)
        if dual and w_idle2 >= 0:
            setVoltage(2,w_idle2) 
        if w_idleD >= 0:
            dioWriteAll(w_idleD,w_mask)
        time.sleep(tinit)
      
    # Send command
    if not dual:
        startCommand('V')
    else:
        startCommand('v')
    sendU16(npre)
    sendCRC()
    
    checkACK()
    
    # Check for overrun or other errors
    code = getByte()
    if code == 1:
        checkCRC()
        raise SlabEx("Sample overrun")
    if code == 3:
        checkCRC()
        raise SlabEx("Halt from board")
    if code != 0: 
        raise SlabEx("Unknown transient response code")          
        
    message(1,"Mesurement ends. Receiving data")    
        
    na = getByte()
    nd = getByte()
    samples = getU16()
    result = []
    
    # Time vector
    vector = []
    for s in range(0,samples):
        vector.append(s*sampleTime)
    if scipy:    
        result.append(np.array(vector))
    else:
        result.append(vector)
        
    # Read analog channels    
    if na:    
        for i in range(0,na):
            vector = []
            for s in range(0,samples):
                fvalue = dc_cal(getU16()/65536.0,xcal,adcCalData[i])
                fvalue = fvalue * vref
                vector.append(fvalue)
            if scipy:    
                result.append(np.array(vector))
            else:
                result.append(vector)
                
    # Read digital channels
    if (nd):
        vector = []
        for s in range(0,samples):
            vector.append(getU16())
        if scipy:    
            result.append(np.array(vector))
        else:
            result.append(vector) 
        
    checkCRC()    

    # Return to iddle
    if w_idle >=0:
        setVoltage(1,w_idle)  
    if w_idle2 >=0:
        setVoltage(1,w_idle2)  
    if w_idleD >= 0:
        dioWriteAll(w_idleD,w_mask)        
    
    message(1,"Data received")
        
    return result    
 

def singleWaveResponse(channel = 1,npre = 0,tinit = 1.0):
    '''
    ---------------------------------------------------------
    singleWaveResponse(channel,npre,tinit)
    Obtain the response of a circuit against a wave
    Response is obtained only on the selected channel
    regardless of the setting on setTransientStorage

    Measurement sequence:
      1) Set DAC1 to first wave sample during tinit
      2) Send npre waves to DAC1
      3) Start measurement as set on setTransientStorage
         During this time wave continues to be generated

    Optional parameters: 
     channel : ADC channel to read (default to 1)
        npre : Number of waves before measurement (default to zero)
       tinit : Time iddle before first wave (default to 1s)
     
    Returns a list of two:
      Vector 0 is time
      Vectors 1 is ADC readings          
          
    Included in slab.py       
    See also setWaveFrequency and setTransientStorage
    
    Deprecated. Not used on new board firmwares
    ---------------------------------------------------------
    ''' 

    global na,nd
    
    # Checks
    if not opened:
        raise SlabEx("Not connected to board")  
    if npre < 0:
        raise SlabEx("Invalid number of waves")
    if channel < 1 or channel > nadcs:
        raise SlabEx("Invalid channel number")
    if w_idle < 0:
        raise SlabEx("Wavetable not loaded")

    message(1,"Performing wave measurement at ADC " + str(channel) + " ...")
            
    # Idle start
    if tinit > 0.0:
        setVoltage(1,w_idle)
        time.sleep(tinit)
      
    # Send command
    startCommand('X')
    sendByte(channel)
    sendU16(npre)
    sendCRC()
    
    checkACK()
    
    # Check for overrun or other errors
    code = getByte()
    if code == 1:
        checkCRC()
        raise SlabEx("Sample overrun")
    if code == 3:
        checkCRC()
        raise SlabEx("Halt from board")
    if code != 0: 
        raise SlabEx("Unknown transient response code")          
        
    message(1,"Mesurement ends. Receiving data")    
        
    na = getByte()
    nd = getByte()
    if nd!=0:
        raise SlabEx("Digital transient is not supported")
    if na!=1:
        raise SlabEx("Internal Error: Only one ADC should be read")    
                
    samples = getU16()
    result = []
    vector = []
    for s in range(0,samples):
        vector.append(s*sampleTime)
    if scipy:    
        result.append(np.array(vector))
    else:
        result.append(vector)
    
    vector = []
    for s in range(0,samples):
        fvalue = dc_cal(getU16()/65536.0,xcal,adcCalData[channel-1])
        fvalue = fvalue * vref
        vector.append(fvalue)
    if scipy:    
        result.append(np.array(vector)) 
    else:
        result.append(vector)
        
    checkCRC()    

    # Return to iddle
    setVoltage(1,w_idle)  
    
    message(1,"Data received")
        
    return result  
 

def wavePlay(n = 1,tinit = 1.0,dual=False):
    '''
    ---------------------------------------------------------
    wavePlay(n,tinit,dual)
    Generates wave withou measuring

    Generation sequence:
      1) Set DAC1 to first wave sample during tinit
      2) Send n waves to DAC1

    Optional parameters:  
         n : Number of waves to send (default to one)
               Zero means infinite (Use HALT to end)
     tinit : Time iddle before first wave (default to 1s)
      dual : Use dual DAC generation (defaults to False)

    Returns nothing  
    Included in slab.py 
          
    See also setWaveFrequency
    ---------------------------------------------------------
    ''' 
    # Checks
    if not opened:
        raise SlabEx("Not connected to board")  
    if n < 0:
        raise SlabEx("Invalid number of waves")
    if w_idle < 0:
        if w_idleD < 0:
            raise SlabEx("Wavetable not loaded")
    if dual and w_idle2 < 0:
        raise SlabEx("Secondary wavetable not loaded")

    message(1,"Sending wave...")
        
    # Idle start
    if tinit > 0.0:
        if w_idle >= 0:
            setVoltage(1,w_idle)
        if dual:
            setVoltage(2,w_idle2)
        if w_idleD >= 0:
            dioWriteAll(w_idleD,w_mask)    
        time.sleep(tinit)
      
    # Send command
    if not dual:
        startCommand('Q')
    else:
        startCommand('q')
    sendU16(n)
    sendCRC()
    
    checkACK()
    
    # Check for overrun or other errors
    code = getByte()
    if code == 1:
        checkCRC()
        raise SlabEx("Sample overrun")     
    if code == 3:
        checkCRC()
        raise SlabEx("Halt from board")
    if code != 0: 
        raise SlabEx("Unknown transient response code")          
        
    message(1,"Wave play ends")    
            
    checkCRC()    

    # Return to iddle   
    if w_idle >= 0:
        setVoltage(1,w_idle)
    if dual:
        setVoltage(2,w_idle2)
    if w_idleD >= 0:
        dioWriteAll(w_idleD,w_mask)  
  
 
     
def wavePlot(n = 0,tinit = 1.0,dual=False,returnData=False):
    '''
    ---------------------------------------------------------
    wavePlot(npre,tinit,dual,returnData)
    Plot the response of a circuit against a wave

    Measurement sequence:
      1) Set DAC1 to first wave sample during tinit
      2) Send npre waves to DAC1
      3) Start measurement as set on setTransientStorage
         During this time wave continues to be generated

    Optional parameters:  
     npre : Number of waves before measurement (default to zero)
     tinit : Time iddle before first wave (default to 1s)
     dual : Generate waves on both dacs (defaults to False)
     returnData : Enables return of plot data (defaults to False)
     
    Returns plot data if enabled
      Vector 0 is time
      Vectors 1 onward are ADC and DIO readings          
          
    Included in slab.py       
    See also setWaveFrequency, setTransientStorage and setPlotReturnData
    ---------------------------------------------------------
    '''

    if not scipy:
        cannotPlot(exception=True)
        return

    # Perform measurement
    res = waveResponse(n,tinit,dual)
    
    # Plot result
    message(1,"Drawing curves")
    
    tranPlot(na,nd,res,"Wave Response Plot")
    
    if plotReturnData or returnData:
        return res           
        
    
def singleWavePlot(channel=1,n=0,tinit = 1.0,returnData=False):
    '''
    ---------------------------------------------------------
    singleWavePlot(channel,npre,tinit,returnData)
    Plot the response of a circuit against a wave
    Response is obtained only on the selected channel
    regardless of the setting on setTransientStorage

    Measurement sequence:
      1) Set DAC1 to first wave sample during tinit
      2) Send npre waves to DAC1
      3) Start measurement as set on setTransientStorage
         During this time wave continues to be generated

    Optional parameters:  
     channel : ADC channel to use (defaults to 1)
     npre : Number of waves before measurement (default to zero)
     tinit : Time iddle before first wave (default to 1s)
     returnData : Enables return of plot data (defaults to False)
     
    Returns plot data if enabled 
      Vector 0 is time
      Vectors 1 is the ADC reading   
          
    Included in slab.py       
    See also setWaveFrequency, setTransientStorage and setPlotReturnData
    
    Deprecated. Not used on new firmwares
    ---------------------------------------------------------
    ''' 

    if not scipy:
        cannotPlot(exception=True)
        return

    # Perform measurement
    res = singleWaveResponse(channel,n,tinit)

    tranPlot(na,nd,res,"Single Wave Response Plot")    
    
    if plotReturnData or returnData:
        return res              
     
################## CALCULATIONS WITH VECTORS ###################


def highPeak(vector):
    '''
    ---------------------------------
    highPeak(vector)
    Returns the maximum of a vector
    Included in slab.py 
    ---------------------------------
    '''
    value = max(vector)
    return value
    
  
def lowPeak(vector):
    '''
    ---------------------------------
    lowPeak(vector)
    Returns the minimum of a vector
    Included in slab.py 
    ---------------------------------
    ''' 
    value = min(vector)    
    return value

    
def peak2peak(vector):
    '''
    -------------------------------------------------------
    peak2peak(vector)
    Returns the maximum to minimum difference of a vector
    Included in slab.py
    -------------------------------------------------------    
    ''' 
    value = max(vector)-min(vector)
    return value
    
if scipy:   
    def mean(vector):
        '''
        ------------------------------------
        mean(vector)
        Returns the mean value of a vector
        Included in slab.py
        ------------------------------------
        '''     
        return np.mean(vector)    
    
 
def halfRange(vector):  
    '''
    -------------------------------------------------------------
    halfRange(vector)
    Returns the average between maximum and minimum of a vector
    Included in slab.py
    -------------------------------------------------------------
    ''' 
    return (max(vector)+min(vector))/2.0
    

if scipy:   
    def rms(vector):
        '''
        ----------------------------------
        rms(vector)
        Returns the RMS value of a vector
        Included in slab.py
        ----------------------------------
        ''' 
        return np.sqrt(np.mean(np.array(vector)*np.array(vector)))
    

if scipy:   
    def std(vector):
        '''
        --------------------------------------------
        std(vector)
        Returns the standard deviation of a vector
        Included in slab.py 
        --------------------------------------------
        ''' 
        return np.std(vector)
   
  

def softReset():
    '''
    --------------------------------------------
    softReset()
    Generates a soft reset on the hardware board
    Board state is set to reset condition
    Returns nothing
    Included in slab.py 
    --------------------------------------------
    '''   
    global sampleTime,gdata
    global w_idle,w_points,w_idle2,w_points2

    # Send Command
    startCommand('E')
    sendCRC()
    checkACK()
    checkCRC()
    
    # Syncronize state
    sampleTime = 0.001  # Default sample time of 1ms
    gdata['sampleTime'] = sampleTime
    
    w_idle = -1         # No wave loaded
    w_points = 0 
    w_idle2 = -1        # No secondary table loaded
    w_points2 = 0
    w_idleD = -1        # No digital wavetable loaded
    w_pointsD = 0
    
    dcroundings = 10    # Default of 10 readings on DC
    
    # Generate message
    message(1,"Hardware board at reset state")
   
################## DC DIGITAL IO ##############################


def dioMode(line,mode=mInput):
    '''
    --------------------------------------------------
    dioMode(line,mode)
    Configures a digital I/O line mode

    Possible modes are:
          mInput : Normal input mode
         mPullUp : Input with Pull Up
       mPullDown : Input with Pull Down
         mOutput : Output Push Pull
      mOpenDrain : Output with Open Drain
      
    Modes use the slab namespace

    Required parameter:
      line : Line to configure
      
    Optional parameter:
      mode : Mode to configure (Defaults to mInput)

    Returns nothing  
    Included in slab.py
    --------------------------------------------------    
    '''
    
    # Check mode string
    if type(mode) is str:
        mode = cDict[mode]
    
    if not opened:
        raise SlabEx("Not connected to board")
    if line < 0 or line >= ndio:
        raise SlabEx("Invalid digital I/O line")
    startCommand('H')
    sendByte(line)
    sendByte(mode)
    sendCRC()
    checkACK()
    checkCRC()
    

def dioWrite(line,value):
    '''
    --------------------------------------------
    dioWrite(line,value)
    Writes on a digital I/O line

    Required parameters:
       line : Line to write
      value : Value to write (True of False)

    Returns nothing 
    Included in slab.py 
    --------------------------------------------    
    '''
    if not opened:
        raise SlabEx("Not connected to board")
    if line < 0 or line >= ndio:
        raise SlabEx("Invalid digital I/O line")
    startCommand('J')
    sendByte(line)
    if value:
        sendByte(1)
    else:
        sendByte(0)
    sendCRC()
    checkACK()
    checkCRC()    
   

def dioRead(line):
    '''
    -------------------------------
    dioRead(line)
    Reads a digital I/O line

    Required parameter:
       line : Line to read

    Returns state (True of False)
    Included in slab.py 
    -------------------------------    
    '''
    if not opened:
        raise SlabEx("Not connected to board")
    if line < 0 or line >= ndio:
        raise SlabEx("Invalid digital I/O line")
    startCommand('K')
    sendByte(line)
    sendCRC()
    checkACK()
    value = getByte()
    checkCRC()    
    if value:
        return True
    else:
        return False
    
def dioWriteAll(value,mask=0):
    '''
    -----------------------------------------------------
    dioWriteAll(value)
    Writes on all digital I/O lines

    Required parameter:
      value : Value to write
    Optional parameter
       mask : Mask of lines to change (Defaults to all)
      
    The value to write shall be between 0 and 2^NDIO - 1  
    If mask is zero, it won't be used

    Returns nothing 
    Included in slab.py 
    -----------------------------------------------------    
    '''
    if not opened:
        raise SlabEx("Not connected to board")
    startCommand('j')
    sendU16(value)
    sendU16(mask)
    sendCRC()
    checkACK()
    checkCRC()        
    
def dioReadAll():
    '''
    --------------------------------------------
    dioReadAll()
    Reads all digital I/O lines
    Returns state of all lines
    
    The state will be between 0 and 2^NDIO - 1
    
    Included in slab.py 
    --------------------------------------------    
    '''
    if not opened:
        raise SlabEx("Not connected to board")
    startCommand('k')
    sendCRC()
    checkACK()
    value = getU16()
    checkCRC()    
    return value    
    
################## CODE EXECUTED AT IMPORT ####################
 
# Remove specific warnings if scipy was loaded 
if scipy:
    warnings.filterwarnings("ignore",".*GUI is implemented.*")
 
# Show version information upon load
message(1,"SLab Module")
message(1,"Version "+str(version_major)+"."+str(version_minor)+" ("+version_date+")")
    
# Indicate if we are running in script or interactive mode    
if hasattr(sys, 'ps1'):
    interactive = True
    message(1,"Running interactively")
else:
    message(1,"Running from script")
    interactive = False
   
# Message if we are in Linxu
if linux:
    message(2,"")
    message(2,"System is Linux")

# Give message if SciPy modules could not be loaded   
if not scipy:
    message(1,"")
    message(1,"Cannot load SciPy modules")
    message(1,"Functionality will be reduced")
    
message(1,"")    
    
    
    
