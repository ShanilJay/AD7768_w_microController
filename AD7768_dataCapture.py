'''
    Continuous read on Analog Devices AD7768 24-bit ADC
    tested with RPi 4B (legacy Raspbian OS), EVAL-AD7768 board, and Teensy 4.0
'''

'''LIBRARIES'''
import spidev
import time
import RPi.GPIO as GPIO
import signal
import sys
import numpy as np

nChannels = 8
nBytes    = 4

# Initialise data ready flag
DRDY_flag = False

addrlist = [
    0x00,                           # Channels
    0x01, 0x02, 0x03,               # Channel configuration
    0x04, 0x05, 0x06, 0x07, 0x08,   # General configuration
    0x09, 0x0a,                     # Device status
    0x0b, 0x0c, 0x0d,               # Reserved registers    
    0x0e, 0x0f, 0x10,               # GPIO registers
    0x11, 0x12, 0x13, 0x14,         # Prechrage registers
    0x56, 0x57, 0x58, 0x59          # Misc registers
]

'''PREAMBLE'''
# Initialise spi bus
bus = 0
device0 = 0
device1 = 1

# Initialise SPI bus for ADC
spi_AD7768 = spidev.SpiDev(bus, device0)
spi_AD7768.max_speed_hz = 2000000
spi_AD7768.mode = 0b00
spi_AD7768.lsbfirst = False

# Initialise SPI bus for Teensy
spi_Teensy = spidev.SpiDev(bus, device1)
spi_Teensy.max_speed_hz = 2000000
spi_Teensy.mode = 0b00
spi_Teensy.lsbfirst = False

# GPIO Pins
START_pin = 11
DRDY_for_RPi = 13
GPIO.setmode(GPIO.BOARD)
GPIO.setup(START_pin, GPIO.OUT)
GPIO.setup(DRDY_for_RPi, GPIO.IN)

# Define function to close when keyboard interrupt
def signal_handler(sig, frame):
    GPIO.cleanup()
    spi_AD7768.close()
    spi_Teensy.close()
    sys.exit(0)

def resetADC():
    # Write to Data Control Register
    writeRegADC(0x06, 0x03)
    writeRegADC(0x06, 0x02)

    # Allow to settle
    time.sleep(1/10)

'''ADC configuration'''
def setupConfig():

    # Channel Standby Register
    writeRegADC(0x00, 0b00000000)   # Enable channels (need to have channel 4 open for crystal excitiation)

    # Channel Mode A Register
    writeRegADC(0x01, 0b00001101)   # Sinc5 filter & 1024 decimation rate

    # Channel Mode B Register
    writeRegADC(0x02, 0b00001101)   # Sinc5 filter & 1024 decimation rate | UNUSED

    # Channel Mode Select Register
    writeRegADC(0x03, 0b00000000)   # All channels mode A

    # Power Mode Select Register
    writeRegADC(0x04, 0b00000000)   # Lower power mode

    # General Device Configuration Register
    writeRegADC(0x05, 0x08)         #  Leave as default

    # Data Control Register
    # writeRegADC(0x06, 0x80)         # Leave as default

    # Interface Configuration Register
    writeRegADC(0x07, 0b00000000)   # No CRC, DCLK_DIV=MCLK/8

    # Digital Filter Ram Built In Self-Test Register
    writeRegADC(0x08, 0b00000000)   # Off

    # GPIO Control Register
    writeRegADC(0x0E, 0x00000000)   # Off

    # Analog Input Precharge Buffer Enable Register Ch0-Ch3
    writeRegADC(0x11, 0xff)         # Off (default - write inverse)

    # Analog Input Precharge Buffer Enable Register Ch0-Ch3
    writeRegADC(0x12, 0xff)         # Off (default - write inverse)

    # Positive Reference Precharge Buffer Enable Register
    writeRegADC(0x13, 0x00)         # Off (default)
    writeRegADC(0x14, 0x00)         # Off (default)

    # Allow things to settle
    time.sleep(1/10)

# Define function to handle data ready indication
def dataReady_callback(channel):

    global DRDY_flag

  # Loop through all channels
    if DRDY_flag:
        DRDY_flag = False
        for i in range(nChannels):
            # If first transaction in block - throw away 8 bits
            if i == 0:
                spi_Teensy.writebytes([0x00])

            # Perform SPI transaction for channel
            ret = spi_Teensy.xfer2([0x00]*nBytes)

            # Format data and append to array
            header[i].append(ret[0])
            data[i].append(ret[1]<<16 | ret[2]<<8 | ret[3])

        DRDY_flag = True

''' AUX FUNCTIONS'''
# Read values from register
def readRegADC(reg):
    word = [reg | 0x80] + [0x00]
    spi_AD7768.writebytes2(word)
    return spi_AD7768.xfer2([0x00, 0x00])

# Write values to register
def writeRegADC(reg, values):
    word = [reg] + [values]
    spi_AD7768.writebytes2(word)

def showRegsADC(registerList):
    for i in registerList:
        word = readRegADC(i)
        print(["0x" + hex(i)[2:].zfill(2), "0x" + hex(word[1])[2:].zfill(2)])

# Allow quit command and sleep the main thread
signal.signal(signal.SIGINT, signal_handler)

# Setup AD7768
resetADC()
writeRegADC(0x06, 0x00)    # Sync command
setupConfig()
# showRegsADC(addrlist)

# Initialise data array
data =   [[] for i in range(nChannels)]
header = [[] for i in range(nChannels)]

# Let everything settle
time.sleep(0.5)

# Allow interrupts on falling edge
GPIO.add_event_detect(DRDY_for_RPi, GPIO.FALLING, callback=dataReady_callback)

# Start sampling
DRDY_flag = True
writeRegADC(0x06, 0x80)    # Sync command

# Time for test
time.sleep(2)

DRDY_flag = False

# Save data to file
for i in range(nChannels):
    np.savetxt("data" + str(i) + ".csv", data[i], delimiter=",", fmt='%.8i')
    np.savetxt("headers" + str(i) + ".csv", header[i], delimiter=",", fmt='%.3i')

spi_AD7768.close()
spi_Teensy.close()
GPIO.cleanup()
