#!/usr/bin/python

import time
import smbus

i2c = smbus.SMBus(1);
WM8731_SLAVE_ADDR = 0x1a;

def WM8731_write(reg_addr, data):
    i2c.write_byte_data(WM8731_SLAVE_ADDR, reg_addr, data)
    # time.sleep(0.1)

# Write functions grouped by common register address
# Control bits are listed in the order they appear in the data sheet

def setLeftLineIn(LRINBOTH, LINMUTE, LINVOL):
    # LINVOL is a 5 bit number to set volume from 0x00 = -34.5 dB to ox1F = +12dB, 1.5dB steps
    # LINMUTE, LRINBOTH are single bit
    val = ((LRINBOTH & 0x01) << 8) | ((LINMUTE & 0x01) << 7) | (LINVOL & 0x1F)
    WM8731_write(0x00<<1, val)

def setRightLineIn(RLINBOTH, RINMUTE, RINVOL):
    # RINVOL is a 5 bit number to set volume from 0x00 = -34.5 dB to ox1F = +12dB, 1.5dB steps
    # RINMUTE, RLINBOTH are single bit
    val = ((RLINBOTH & 0x01) << 8) | ((RINMUTE & 0x01) << 7) | (RINVOL & 0x1F)
    WM8731_write(0x01<<1, val)

def setLeftHeadphoneOut(LRHPBOTH, LZCEN, LHPVOL):

    val = ((LRHPBOTH & 0x01) << 8) | ((LZCEN & 0x01) << 7) | (LHPVOL & 0x3F)
    WM8731_write(0x02<<1, val)

def setRightHeadphoneOut(RLHPBOTH, RZCEN, RHPVOL):

    val = ((RLHPBOTH & 0x01) << 8) | ((RZCEN & 0x01) << 7) | (RHPVOL & 0x3F)
    WM8731_write(0x03<<1, val)

def setAnalogueAudioPathControl(SIDEATT, SIDETONE, DACSEL, BYPASS, INSEL, MUTEMIC, MICBOOST):
    # Insert description
    val = ((SIDEATT & 0x03) << 6) | ((SIDETONE & 0x01) << 5) | ((DACSEL & 0x01) << 4) | ((BYPASS & 0x01) << 3) | ((INSEL & 0x01) << 2) | ((MUTEMIC & 0x01) << 1) | ((MICBOOST & 0x01) << 0)
    WM8731_write(0x04<<1, val)

def setDigitalAudioPathControl(HPOR, DACMU, DEEMP, ADCHPD):
    # Description
    val =  ((HPOR & 0x01) << 4)  | ((DACMU & 0x01) << 3) | ((DEEMP & 0x03) << 1) | ((ADCHPD & 0x01) << 0)
    WM8731_write(0x05<<1, val)

def setPowerDownControl(POWEROFF, CLKOUTPD, OSCPD, OUTPD, DACPD, ADCPD, MICPD, LINEINPD):

    val = ((POWEROFF & 0x01) << 7) | ((CLKOUTPD & 0x01) << 6) | ((OSCPD & 0x01) << 5) | ((OUTPD & 0x01) << 4) | ((DACPD & 0x01) << 3) | ((ADCPD & 0x01) << 2) | ((MICPD & 0x01) << 1) | (LINEINPD & 0x01)
    WM8731_write(0x06<<1, val)

def setDigitalAudioInterfaceFormat(BCLKINV, MS, LRSWAP, LRP, IWL, FORMAT):

    val = ((BCLKINV & 0x01) << 7) | ((MS & 0x01) << 6) | ((LRSWAP & 0x01) << 5) | ((LRP & 0x01) << 4) | ((IWL & 0x03) << 2) | (FORMAT & 0x03)
    WM8731_write(0x07<<1, val)

def setSamplingControl(CLKODIV2, CLKIDIV2, SR, BOSR, USBNORM):

    val = ((CLKODIV2 & 0x01) << 7) | ((CLKIDIV2 & 0x01) << 6) | ((SR & 0x0F) << 2) | ((BOSR & 0x01) << 1) | (USBNORM & 0x01)
    WM8731_write(0x08<<1, val)

def setActiveControl(ACTIVE):

    val = (ACTIVE & 0x01)
    WM8731_write(0x09<<1, val)

def resetDevice():
    # Reset device when 0x00 is written
    WM8731_write(0x0F<<1, 0x00)

def WM8731_init():
    # Modify the initialization functions as necessary for your application
    resetDevice()

    # Following recommended power up sequence:
    # Set all bits in Power Down reg to 0 except OUTPD
    # Don't need Line In or Clk Out right now
    setPowerDownControl(0, 1, 0, 1, 0, 0, 0, 0)

    # Configure others registers as desired
    setLeftLineIn(0, 0, 0x17)
    setRightLineIn(0, 0, 0x17)
    setLeftHeadphoneOut(0, 0, 0x79)
    setRightHeadphoneOut(0, 0, 0x79)

    setAnalogueAudioPathControl(0x0, 0, 1, 0, 1, 0, 0)
    setDigitalAudioPathControl(0, 0, 0x3, 0)

    # Use I2S format, 24-bit width, and Master mode
    setDigitalAudioInterfaceFormat(0, 1, 0, 0, 0x2, 0x2)
    setSamplingControl(0, 0, 0x0, 0, 0)

    # Set Active bit
    setActiveControl(1)

    # Set OUTPD to 0
    setPowerDownControl(0, 1, 0, 0, 0, 0, 0, 0)


WM8731_init()
print("Codec initialized.")
