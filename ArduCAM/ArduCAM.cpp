// Simplethings ("COMPANY") CONFIDENTIAL
// Unpublished Copyright (c) 2014-2015 Simplethings, Inc, All Rights Reserved.

#include "ArduCAM.h"
#include <SPI.h>
#include "HardwareSerial.h"
#include "memorysaver.h"
#include "Arduino.h"

// Assert CS signal
void ArduCAM::CS_LOW(void) {
  cbi(P_CS, B_CS);
}

// Disable CS signal
void ArduCAM::CS_HIGH(void) {
  sbi(P_CS, B_CS);
}

// Set corresponding bit
void ArduCAM::set_bit(uint8_t addr, uint8_t bit) {
  uint8_t temp;
  temp = read_reg(addr);
  write_reg(addr, temp | bit);
}

// Clear corresponding bit
void ArduCAM::clear_bit(uint8_t addr, uint8_t bit) {
  uint8_t temp;
  temp = read_reg(addr);
  write_reg(addr, temp & (~bit));
}

// Get corresponding bit status
uint8_t ArduCAM::get_bit(uint8_t addr, uint8_t bit) {
  uint8_t temp;
  temp = read_reg(addr);
  temp = temp & bit;
  return temp;
}

// Low level SPI write operation
void ArduCAM::bus_write(int address, int value) {
  // take the SS pin low to select the chip:
  cbi(P_CS, B_CS);
  //  send in the address and value via SPI:
  SPI.transfer(address);
  SPI.transfer(value);
  // take the SS pin high to de-select the chip:
  sbi(P_CS, B_CS);
}

// Low level SPI read operation
uint8_t ArduCAM::bus_read(int address) {
  uint8_t value = 0;
  // take the SS pin low to select the chip:
  cbi(P_CS, B_CS);
  //  send in the address and value via SPI:
  #if defined (OV5640_CAM)
    SPI.transfer(address);
    value = SPI.transfer(0x00);
    // take the SS pin high to de-select the chip:
    sbi(P_CS, B_CS);
    return value;
  #else
    SPI.transfer(address);
    value = SPI.transfer(0x00);
    // take the SS pin high to de-select the chip:
    sbi(P_CS, B_CS);
    return value;
  #endif
}

// Write ArduChip internal registers
void ArduCAM::write_reg(uint8_t addr, uint8_t data) {
  bus_write(addr | 0x80, data);
}

// Read ArduChip internal registers
uint8_t ArduCAM::read_reg(uint8_t addr) {
  uint8_t data;
  data = bus_read(addr & 0x7F);
  return data;
}

// I2C Array Write 16bit address, 8bit data
int ArduCAM::wrSensorRegs16_8(const struct sensor_reg reglist[]) {
  // int err = 0;
  unsigned int reg_addr;
  unsigned char reg_val;
  const struct sensor_reg *next = reglist;
  while ((reg_addr != 0xffff) | (reg_val != 0xff)) {
    reg_addr = pgm_read_word(&next->reg);
    reg_val = pgm_read_word(&next->val);
    // err = wrSensorReg16_8(reg_addr, reg_val);
    next++;
  }
  return 1;
}

// I2C Write 8bit address, 8bit data
byte ArduCAM::wrSensorReg8_8(int regID, int regData) {
  byte write_sensor_addr = sensor_addr_ << 1;

  /* phase 1: slave_id is 7 bits + 1 bit 0 = w, 1 = r, + dont care */
  StartSCCB();
  if (WriteSCCB(write_sensor_addr) == 0) {
    Serial.println("wrSensorReg8_8 failed phase 1");
    StopSCCB();
    return(0);
  }
  delayMicroseconds(SIO_CLKDELAY);

  /* phase 2 : sub address is just 8 bits, + don't care */
  if (WriteSCCB(regID) == 0) {
    Serial.println("wrSensorReg8_8 failed phase 2");
    StopSCCB();
    return(0);
  }
  delayMicroseconds(SIO_CLKDELAY);

  /* phase 3 : data writing */
  if (WriteSCCB(regData) == 0) {
    Serial.println("wrSensorReg8_8 failed phase 3");
    StopSCCB();
    return(0);
  }
  StopSCCB();

  delay(1);
  return(1);
}

// I2C Write 16bit address, 8bit data
byte ArduCAM::wrSensorReg16_8(int regID, int regData) {
  byte write_sensor_addr = sensor_addr_ << 1;

  /* phase 1: slave_id is 7 bits + 1 bit 0 = w, 1 = r, + dont care */
  StartSCCB();
  if (WriteSCCB(write_sensor_addr) == 0) {
    Serial.println("wrSensorReg16_8 failed phase 1");
    StopSCCB();
    return(0);
  }
  delayMicroseconds(SIO_CLKDELAY);

  /* phase 2 : sub-address MSbyte first then LSbyte */
  if (WriteSCCB((regID >> 8) & 0xFF) == 0) {
    Serial.println("wrSensorReg16_8 failed phase 2");
    StopSCCB();
  }
  if (WriteSCCB(regID & 0xFF) == 0) {
    Serial.println("wrSensorReg16_8 failed phase 2");
    StopSCCB();
  }
  delayMicroseconds(SIO_CLKDELAY);

  /* phase 3 : data writing */
  if (WriteSCCB(regData) == 0) {
    Serial.println("wrSensorReg16_8 failed phase 3");
    StopSCCB();
    return(0);
  }
  StopSCCB();

  delay(1);
  return(1);
}

// I2C Read 8bit address, 8bit data
byte ArduCAM::rdSensorReg8_8(uint8_t regID, uint8_t* regData) {
  /* phase 1: slave_id is 7 bits + 
  1 bit 0 = w, 1 = r, + dont care */
  StartSCCB();
  byte write_sensor_addr = sensor_addr_ << 1;
  if (WriteSCCB(write_sensor_addr) == 0) {
    Serial.println("rdSensorReg8_8 failed phase 1-1");
    StopSCCB();
    return(0);
  }
  /* phase 2 : sub-address */
  if (WriteSCCB(regID) == 0) {
    Serial.println("rdSensorReg8_8 failed phase 1-2");
    StopSCCB();
    return(0);
  }
  /* need to stop and restart
  between write and read parts */
  StopSCCB();

  /* 2-phase read transmission cycle */
  byte read_sensor_addr = write_sensor_addr | 0x01;
  StartSCCB();
  /* phase 1 : ID address */
  if (WriteSCCB(read_sensor_addr) == 0) {
    Serial.println("rdSensorReg8_8 failed phase 2-1");
    StopSCCB();
    return(0);
  }
  /*phase 2 : data reading*/
  if (ReadSCCB(regData) == 0) {
    Serial.println("rdSensorReg8_8 failed phase 2-2");
    StopSCCB();
    return(0);
  }
  StopSCCB();

  delay(1);
  return(1);
}

// I2C Read 16bit address, 8bit data
byte ArduCAM::rdSensorReg16_8(uint16_t regID, uint8_t* regData) {
  /* phase 1: slave_id is 7 bits + 
  1 bit 0 = w, 1 = r, + dont care */
  StartSCCB();
  byte write_sensor_addr = sensor_addr_ << 1;
  if (WriteSCCB(write_sensor_addr) == 0) {
    Serial.println("rdSensorReg16_8 failed phase 1-1");
    StopSCCB();
    return(0);
  }
  /* phase 2 : sub-address MSbyte first then LSbyte */
  if (WriteSCCB((regID >> 8) & 0xFF) == 0) {
    Serial.println("rdSensorReg16_8 failed phase 1-2");
    StopSCCB();
    return(0);
  }
  if (WriteSCCB(regID & 0xFF) == 0) {
    Serial.println("rdSensorReg16_8 failed phase 1-2");
    StopSCCB();
    return(0);
  }
  /* need to stop and restart
  between write and read parts */
  StopSCCB();

  /* 2-phase read transmission cycle */
  byte read_sensor_addr = write_sensor_addr | 0x01;
  StartSCCB();
  /* phase 1 : ID address */
  if (WriteSCCB(read_sensor_addr) == 0) {
    Serial.println("rdSensorReg16_8 failed phase 2-1");
    StopSCCB();
    return(0);
  }
  /*phase 2 : data reading*/
  if (ReadSCCB(regData) == 0) {
    Serial.println("ReadSensor failed phase 2-2");
    StopSCCB();
    return(0);
  }
  StopSCCB();

  delay(1);
  return(1);
}

// Set ArduCAM working mode
// MCU2LCD_MODE: MCU writes the LCD screen GRAM
// CAM2LCD_MODE: Camera takes control of the LCD screen
// LCD2MCU_MODE: MCU read the LCD screen GRAM
void ArduCAM::set_mode(uint8_t mode) {
  switch (mode) {
    case MCU2LCD_MODE:
      write_reg(ARDUCHIP_MODE, MCU2LCD_MODE);
      break;
    case CAM2LCD_MODE:
      write_reg(ARDUCHIP_MODE, CAM2LCD_MODE);
      break;
    case LCD2MCU_MODE:
      write_reg(ARDUCHIP_MODE, LCD2MCU_MODE);
      break;
    default:
      write_reg(ARDUCHIP_MODE, MCU2LCD_MODE);
      break;
  }
}

ArduCAM::ArduCAM(int SIO_C, int SIO_D, int CS) {
  P_CS = (volatile uint32_t*)portOutputRegister(
    digitalPinToPort(CS));
  B_CS = digitalPinToBitMask(CS);
  pinMode(CS, OUTPUT);
  sbi(P_CS, B_CS);  // must initialize bus default status

  SIO_C_ = SIO_C;
  SIO_D_ = SIO_D;
}

// Reset the FIFO pointer to ZERO
void ArduCAM::flush_fifo(void) {
  write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

// Send capture command
void ArduCAM::start_capture(void) {
  write_reg(ARDUCHIP_FIFO, FIFO_START_MASK);
}

// Clear FIFO Complete flag
void ArduCAM::clear_fifo_flag(void) {
  write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

// Read FIFO single
uint8_t ArduCAM::read_fifo(void) {
  uint8_t data;
  data = bus_read(SINGLE_FIFO_READ);
  return data;
}

// Read Write FIFO length
// Support ArduCAM Mini only
uint32_t ArduCAM::read_fifo_length(void) {
  uint32_t len1, len2, len3, length = 0;
  len1 = read_reg(FIFO_SIZE1);
  len2 = read_reg(FIFO_SIZE2);
  len3 = read_reg(FIFO_SIZE3) & 0x7f;
  length = ((len3 << 16) | (len2 << 8) | len1) & 0x07fffff;
  return length;
}

// Send read fifo burst command
// Support ArduCAM Mini only
void ArduCAM::set_fifo_burst() {
  SPI.transfer(BURST_FIFO_READ);
}

void ArduCAM::OV5642_set_JPEG_size(uint8_t size) {
  #if defined (OV5642_CAM) ||defined (OV5642_CAM_BIT_ROTATION_FIXED)
  // uint8_t reg_val;
  wrSensorRegs16_8(ov5642_dvp_fmt_global_init);
  delay(100);
  switch (size) {
    case OV5642_320x240:
      wrSensorRegs16_8(ov5642_dvp_fmt_jpeg_qvga);
      wrSensorReg16_8(0x4407, 0x04);
      wrSensorReg16_8(0x3818, 0xA8);
      wrSensorReg16_8(0x3621, 0x10);
      wrSensorReg16_8(0x3801 , 0xC8);
      break;
    case OV5642_640x480:
      wrSensorRegs16_8(ov5642_dvp_fmt_jpeg_vga);
      wrSensorReg16_8(0x3818, 0xA8);
      wrSensorReg16_8(0x3621, 0x10);
      wrSensorReg16_8(0x3801 , 0xC8);
      break;
    case OV5642_1280x720:
      wrSensorRegs16_8(ov5642_dvp_fmt_jpeg_qvga);
      wrSensorRegs16_8(ov5642_res_720P);
      wrSensorReg16_8(0x3818, 0xA8);
      wrSensorReg16_8(0x3621, 0x10);
      wrSensorReg16_8(0x3801 , 0xC8);
      break;
    case OV5642_1920x1080:
      wrSensorRegs16_8(ov5642_dvp_fmt_jpeg_qvga);
      wrSensorRegs16_8(ov5642_res_1080P);
      wrSensorReg16_8(0x3818, 0xA8);
      wrSensorReg16_8(0x3621, 0x10);
      wrSensorReg16_8(0x3801 , 0xC8);
      break;
    case OV5642_2048x1563:
      wrSensorRegs16_8(ov5642_dvp_fmt_jpeg_qxga);
      wrSensorReg16_8(0x3818, 0xA8);
      wrSensorReg16_8(0x3621, 0x10);
      wrSensorReg16_8(0x3801 , 0xC8);
      break;
    case OV5642_2592x1944:
      wrSensorRegs16_8(ov5642_dvp_fmt_jpeg_5M);
      wrSensorReg16_8(0x4407, 0x08);
      wrSensorReg16_8(0x3818, 0xA8);
      wrSensorReg16_8(0x3621, 0x10);
      wrSensorReg16_8(0x3801 , 0xC8);
      break;
    default:
      wrSensorRegs16_8(ov5642_dvp_fmt_jpeg_qvga);
      break;
  }
  #endif
}

void ArduCAM::set_format(byte fmt) {
  if (fmt == BMP)
    m_fmt = BMP;
  else
    m_fmt = JPEG;
}

void ArduCAM::InitCAM() {
  byte reg_val;
  switch (sensor_model) {
    case OV5642:
    {
      #if defined (OV5642_CAM) || defined (OV5642_CAM_BIT_ROTATION_FIXED)
      wrSensorReg16_8(0x3008, 0x80);
      delay(100);
      if (m_fmt == JPEG) {
        wrSensorRegs16_8(ov5642_dvp_fmt_global_init);
        delay(100);
        wrSensorRegs16_8(ov5642_dvp_fmt_jpeg_qvga);
        wrSensorReg16_8(0x4407, 0x0C);
      } else {
        wrSensorRegs16_8(OV5642_RGB_QVGA);
        rdSensorReg16_8(0x3818, &reg_val);
        wrSensorReg16_8(0x3818, (reg_val | 0x60) & 0xff);
        rdSensorReg16_8(0x3621, &reg_val);
        wrSensorReg16_8(0x3621, reg_val & 0xdf);
      }
      #endif
      break;
    }
    default:
      break;
  }
}

void ArduCAM::InitSCCB(void) {
  // no M_CLK for arducam breakout
  /*
  pinMode(M_CLK, OUTPUT);
  analogWriteFrequency(M_CLK, 12000000); // 15 MHz
  analogWrite(M_CLK, 128);

  pinMode(P_CLK, INPUT);

  pinMode(RESET, OUTPUT);
  digitalWrite(RESET, HIGH);
  */
  pinMode(SIO_C_, OUTPUT);
  pinMode(SIO_D_, OUTPUT);
  digitalWrite(SIO_C_, HIGH);
  digitalWrite(SIO_D_, HIGH);
  Serial.println("InitSCCB - done");
}

int ArduCAM::ScanSCCB(void) {
  int found_address = -1;
  for (int j = 0; j < 128; ++j) {
    // Serial.println(j);
    StartSCCB();
    byte temp = WriteSCCB(j << 1);
    StopSCCB();
    if (temp > 0) {
      Serial.print("found sccb device: ");
      Serial.println(j, HEX);
      found_address = j;
    }
  }
  return found_address;
}

void ArduCAM::StartSCCB(void) {
  /* put HIGH in order to avoid
  propagating unknown bus state. */
  digitalWrite(SIO_D_, HIGH);
  delayMicroseconds(SIO_CLKDELAY);
  digitalWrite(SIO_C_, HIGH);
  delayMicroseconds(SIO_CLKDELAY);
  digitalWrite(SIO_D_, LOW);
  // dox: assert SIO_D low while SIO_C high
  delayMicroseconds(SIO_CLKDELAY);
  digitalWrite(SIO_C_, LOW);
}

void ArduCAM::StopSCCB(void) {
  digitalWrite(SIO_D_, LOW);
  delayMicroseconds(SIO_CLKDELAY);
  digitalWrite(SIO_C_, HIGH);
  delayMicroseconds(SIO_CLKDELAY);
  digitalWrite(SIO_D_, HIGH);
  delayMicroseconds(SIO_CLKDELAY);
}

byte ArduCAM::WriteSCCB(byte m_data) {
  byte temp;
  for (int j = 0; j < 8; j++) {
    /*write the MSB not written yet :*/
    if ((m_data << j) & 0x80) {
      digitalWrite(SIO_D_, HIGH);
    } else {
      digitalWrite(SIO_D_, LOW);
    }
    delayMicroseconds(SIO_CLKDELAY);
    digitalWrite(SIO_C_, HIGH);
    delayMicroseconds(SIO_CLKDELAY);
    digitalWrite(SIO_C_, LOW);
    delayMicroseconds(SIO_CLKDELAY);
  }
  /* eight bits have been send
  deal with the ninth don't-care one*/
  /* we put the SIO_D pin at low in order
  to avoid propagating any unknown state.*/
  /* why input ? because it enables 
  a high impedance, I suppose...?*/
  pinMode(SIO_D_, INPUT);
  digitalWrite(SIO_D_, LOW);

  delayMicroseconds(SIO_CLKDELAY);
  digitalWrite(SIO_C_, HIGH);
  delayMicroseconds(SIO_CLKDELAY);

  /*there is something to read to check the transmission :*/
  if (digitalRead(SIO_D_) == HIGH) {
    temp = 0;
  } else {
    temp = 1;
  }

  digitalWrite(SIO_C_, LOW);
  delayMicroseconds(SIO_CLKDELAY);

  /* clean that up */
  pinMode(SIO_D_, OUTPUT);

  return temp;
}

byte ArduCAM::ReadSCCB(byte* m_data) {
  /* Let's make things readable */
  pinMode(SIO_D_, INPUT);
  digitalWrite(SIO_D_, LOW);

  for (unsigned int j = 0; j < 8; j++) {
    delayMicroseconds(SIO_CLKDELAY);
    digitalWrite(SIO_C_, HIGH);

    /*let's read in the middle of the SIO_C cycle :*/
    /*read the MSB not read yet :*/
    if (digitalRead(SIO_D_) != LOW) {
      *m_data = 0x01 | (*m_data << 1);
    } else {
      *m_data = 0xFE & (*m_data << 1);
    }

    delayMicroseconds(SIO_CLKDELAY);
    digitalWrite(SIO_C_, LOW);
    delayMicroseconds(SIO_CLKDELAY);
  }

  /*eight bits have been read, let's deal with the ninth Don't-care one*/
  /*the master is responsible for driver SIO_D at logical 1 during the NA bit.*/
  pinMode(SIO_D_, OUTPUT);
  digitalWrite(SIO_D_, HIGH);

  delayMicroseconds(SIO_CLKDELAY);
  digitalWrite(SIO_C_, HIGH);
  delayMicroseconds(SIO_CLKDELAY);

  /*there is something/nothing to read to check the transmission ???*/
  digitalWrite(SIO_C_, LOW);
  delayMicroseconds(SIO_CLKDELAY);

  /*let's clean that up : reset as usual as if we had written...*/
  pinMode(SIO_D_, OUTPUT);
  digitalWrite(SIO_D_, LOW);

  return 1;
}
