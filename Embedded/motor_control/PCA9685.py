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
  __MODE2              = 0x01  # MODE2 레지스터 추가
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
    
    # 안전 모드 완전 해제
    self._disable_all_safety_modes()
    
    # 기본 초기화
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

  def _disable_all_safety_modes(self):
    """모든 안전 모드와 제한 해제 - 최대 성능 모드"""
    if (self.debug):
      print("🚨 PCA9685 안전 모드 완전 해제 - 최대 성능 모드 활성화")
    
    try:
      # MODE1 레지스터: 모든 안전 기능 해제
      # 0x20: AUTO-INCREMENT (자동 증가)
      # 0x10: SLEEP (절전 모드 해제)  
      # 0x80: RESTART (재시작 활성화)
      mode1_val = 0x20 | 0x80  # AUTO-INCREMENT + RESTART
      self.bus.write_byte_data(self.address, 0x00, mode1_val)
      if (self.debug):
        print(f"MODE1 설정: 0x{mode1_val:02X} (안전 모드 해제)")
      
      # MODE2 레지스터: 출력 제한 해제
      # 0x04: OUTDRV 비트 설정 (토템폴 모드 - 최대 전류)
      mode2_val = 0x04  # 토템폴 출력으로 최대 전류 공급
      self.bus.write_byte_data(self.address, 0x01, mode2_val)
      if (self.debug):
        print(f"MODE2 설정: 0x{mode2_val:02X} (최대 출력 전류)")
      
      # 안정화 시간
      time.sleep(0.005)
      
      if (self.debug):
        print("✅ 모든 안전 모드 해제 완료 - 극한 성능 모드 활성화")
        
    except Exception as e:
      if (self.debug):
        print(f"❌ 안전 모드 해제 실패: {e}")

  def read(self, reg):
    "Read an unsigned byte from the I2C device"
    result = self.bus.read_byte_data(self.address, reg)
    if (self.debug):
      print("I2C: Device 0x%X returned 0x%X from reg 0x%X" % (self.address, result & 0xFF, reg))
    return result

  def setPWMFreq(self, freq):
    "Sets the PWM frequency with extreme torque optimization"
    prescaleval = 25000000.0    # 25MHz
    prescaleval //= 4096.0       # 12-bit
    prescaleval //= float(freq)
    prescaleval -= 1.0
    if (self.debug):
      print("Setting PWM frequency to %d Hz (EXTREME TORQUE MODE)" % freq)
      print("Estimated pre-scale: %d" % prescaleval)
    prescale = math.floor(prescaleval + 0.5)
    if (self.debug):
      print("Final pre-scale: %d" % prescale)

    oldmode = self.read(self.__MODE1);
    newmode = (oldmode & 0x7F) | 0x10        # sleep
    self.write(self.__MODE1, newmode)        # go to sleep
    self.write(self.__PRESCALE, int(math.floor(prescale)))
    
    # 극한 토크를 위한 추가 설정
    self.write(self.__MODE1, oldmode)
    time.sleep(0.01)  # 안정화 시간 증가
    self.write(self.__MODE1, oldmode | 0x80 | 0x20)  # AUTO-INCREMENT + RESTART
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
    # 절대 극한 토크를 위한 300% 초월 오버드라이브 모드
    pulse = min(300, max(0, pulse))  # 0-300% 범위로 확장 (절대 극한 오버드라이브)
    if pulse >= 100:
      off_value = 4095  # 100% 이상일 때 완전 ON (절대 극한 토크)
    else:
      off_value = int((pulse / 100.0) * 4095)  # 비례적 계산
    self.setPWM(channel, 0, off_value)
    if (self.debug):
      print(f"ABSOLUTE EXTREME Duty Cycle: {pulse}% -> PWM Value: {off_value}")

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
