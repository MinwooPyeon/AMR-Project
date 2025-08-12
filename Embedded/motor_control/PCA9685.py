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
  __PRESCALE           = 0xFE
  __LED0_ON_L          = 0x06
  __LED0_ON_H          = 0x07
  __LED0_OFF_L         = 0x08
  __LED0_OFF_H         = 0x09
  __ALLLED_ON_L        = 0xFA
  __ALLLED_ON_H        = 0xFB
  __ALLLED_OFF_L       = 0xFC
  __ALLLED_OFF_H       = 0xFD

  def __init__(self, address, debug=True, i2c_bus=None):
    self.address = address
    self.debug = debug
    
    # I2C 버스 자동 감지 또는 지정된 버스 사용
    if i2c_bus is None:
      self.bus = self._find_available_bus()
    else:
      self.bus = smbus.SMBus(i2c_bus)
    
    if (self.debug):
      print(f"PCA9685 초기화 - 주소: 0x{address:02X}, 버스: {self.bus.fileno() if hasattr(self.bus, 'fileno') else 'unknown'}")
    self.write(self.__MODE1, 0x00)

  def _find_available_bus(self):
    for bus_num in range(10): 
      try:
        bus = smbus.SMBus(bus_num)
        try:
          bus.read_byte_data(self.address, 0x00)  
          if self.debug:
            print(f"PCA9685 발견 - I2C 버스 {bus_num}")
          return bus
        except:
          bus.close()
          continue
      except:
        continue
    
    # 사용 가능한 버스가 없으면 기본값 사용
    if self.debug:
      print("사용 가능한 I2C 버스를 찾을 수 없어 기본값 사용")
    return smbus.SMBus(1)  # 기본값으로 버스 1 사용

  def write(self, reg, value):
    "Writes an 8-bit value to the specified register/address"
    self.bus.write_byte_data(self.address, reg, value)
    if (self.debug):
      print("I2C: Write 0x%02X to register 0x%02X" % (value, reg))

  def read(self, reg):
    "Read an unsigned byte from the I2C device"
    result = self.bus.read_byte_data(self.address, reg)
    if (self.debug):
      print("I2C: Device 0x%X returned 0x%X from reg 0x%X" % (self.address, result & 0xFF, reg))
    return result

  def setPWMFreq(self, freq):
    "Sets the PWM frequency"
    prescaleval = 25000000.0    # 25MHz
    prescaleval //= 4096.0       # 12-bit
    prescaleval //= float(freq)
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
    self.read(self.__MODE1);


  def setPWM(self, channel, on, off):
    "Sets a single PWM channel"
    self.write(self.__LED0_ON_L + 4*channel, on & 0xFF)
    self.write(self.__LED0_ON_H + 4*channel, on >> 8)
    self.write(self.__LED0_OFF_L + 4*channel, off & 0xFF)
    self.write(self.__LED0_OFF_H + 4*channel, off >> 8)
    if (self.debug):
      print("channel: %d  LED_ON: %d LED_OFF: %d" % (channel,on,off))


  def setDutycycle(self, channel, pulse):
    self.setPWM(channel, 0, int(pulse * (4096 // 100)))

  def setLevel(self, channel, value):
    if (value == 1):
      self.setPWM(channel, 0, 4095)
    else:
      self.setPWM(channel, 0, 0)

# pwm = PCA9685(0x5f, debug=False)
# pwm.setPWMFreq(50)
# pwm.setDutycycle(0,100)
# pwm.setLevel(1,0)
# pwm.setLevel(2,1)
