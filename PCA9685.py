#!/usr/bin/python

import time
import math
import smbus

# ============================================================================
# Raspi PCA9685 16-Channel PWM Servo Driver
# ============================================================================

class PCA9685:

  # Registers/etc.
  __SUBADR1            = 0x02
  __SUBADR2            = 0x03
  __SUBADR3            = 0x04
  __MODE1              = 0x00
  __MODE2              = 0x01
  __PRESCALE           = 0xFE
  __LED0_ON_L          = 0x06
  __LED0_ON_H          = 0x07
  __LED0_OFF_L         = 0x08
  __LED0_OFF_H         = 0x09
  __ALLLED_ON_L        = 0xFA
  __ALLLED_ON_H        = 0xFB
  __ALLLED_OFF_L       = 0xFC
  __ALLLED_OFF_H       = 0xFD


  def __init__(self, address=0x40, debug=False):
    self.bus = smbus.SMBus(1)
    self.address = address
    self.debug = debug
    if (self.debug):
      print("Reseting PCA9685")
    self.write(self.__MODE1, 0x00)
	
  def write(self, reg, value):
    "Writes an 8-bit value to the specified register/address"
    self.bus.write_byte_data(self.address, reg, value)
    if (self.debug):
      print("I2C: Write 0x%02X to register 0x%02X" % (value, reg))
	  
  def read(self, reg):
    "Read an unsigned byte from the I2C device"
    result = self.bus.read_byte_data(self.address, reg)
    if (self.debug):
      print("I2C: Device 0x%02X returned 0x%02X from reg 0x%02X" % (self.address, result & 0xFF, reg))
    return result
	
  def setPWMFreq(self, freq):
    "Sets the PWM frequency"
    prescaleval = 25000000.0    # 25MHz
    prescaleval /= 4096.0       # 12-bit
    prescaleval /= float(freq)
    prescaleval -= 1.0
    if (self.debug):
      print("Setting PWM frequency to %d Hz" % freq)
      print("Estimated pre-scale: %d" % prescaleval)
    prescale = math.floor(prescaleval + 0.5)
    if (self.debug):
      print("Final pre-scale: %d" % prescale)

    oldmode = self.read(self.__MODE1);
    newmode = (oldmode & 0x7F) | 0x10        # sleep
    self.write(self.__MODE1, newmode)        # go to sleep
    self.write(self.__PRESCALE, int(math.floor(prescale)))
    self.write(self.__MODE1, oldmode)
    time.sleep(0.005)
    self.write(self.__MODE1, oldmode | 0x80)
    self.write(self.__MODE2, 0x04)

  def setPWM(self, channel, on, off):
    "Sets a single PWM channel"
    self.write(self.__LED0_ON_L+4*channel, on & 0xFF)
    self.write(self.__LED0_ON_H+4*channel, on >> 8)
    self.write(self.__LED0_OFF_L+4*channel, off & 0xFF)
    self.write(self.__LED0_OFF_H+4*channel, off >> 8)
    if (self.debug):
      print("channel: %d  LED_ON: %d LED_OFF: %d" % (channel,on,off))
	  
  def setServoPulse(self, channel, pulse):
    "Sets the Servo Pulse,The PWM frequency must be 50HZ"
    pulse = pulse*4096/20000        #PWM frequency is 50HZ,the period is 20000us
    self.setPWM(channel, 0, int(pulse))
    
  def setRotationAngle(self, channel, Angle): 
    if(Angle >= 0 and Angle <= 180):
        temp = Angle * (2000 / 180) + 501
        self.setServoPulse(channel, temp)
    else:
        print("Angle out of range")
        
  def start_PCA9685(self):
    self.write(self.__MODE2, 0x04)
    #Just restore the stopped state that should be set for exit_PCA9685
    
  def exit_PCA9685(self):
    self.write(self.__MODE2, 0x00)#Please use initialization or __MODE2 =0x04

    ########################################################################################
    #selbst geschrieben
    ######################################################################################

  def moveToAngle(self, channel, start_angle, end_angle, steps, delay):
    """Move the servo from start_angle to end_angle in a smooth manner."""
    if type(start_angle) == list:
        start_angle = start_angle[channel]
    if type(end_angle) == list:
        end_angle = end_angle[channel]
    
    if start_angle < 0 or start_angle > 180 or end_angle < 0 or end_angle > 180:
        print("Angle out of range")
        return False
    step_size = (end_angle - start_angle) / steps
    for i in range(steps + 1):
        angle = start_angle + step_size * i
        self.setRotationAngle(channel, angle)
        time.sleep(delay)
    return True

  def moveToAngle2d(self, channel1, channel2, start_angle1, end_angle1, start_angle2, end_angle2, steps, delay):
      """Move two servos simultaneously from their start angles to their end angles in a smooth manner."""
      # Validate the input angles
      if not (0 <= start_angle1 <= 180 and 0 <= end_angle1 <= 180 and 0 <= start_angle2 <= 180 and 0 <= end_angle2 <= 180):
          print("Angle out of range")
          return

      # Calculate the step size for both servos
      step_size1 = (end_angle1 - start_angle1) / steps
      step_size2 = (end_angle2 - start_angle2) / steps

      # Move both servos in sync
      for i in range(steps + 1):
          angle1 = start_angle1 + step_size1 * i
          angle2 = start_angle2 + step_size2 * i

          self.setRotationAngle(channel1, angle1)
          self.setRotationAngle(channel2, angle2)
          
          time.sleep(delay)


