// Simplethings ("COMPANY") CONFIDENTIAL
// Unpublished Copyright (c) 2014-2015 Simplethings, Inc, All Rights Reserved.

#ifndef ARDUCAM_ARDUCAM_H_
#define ARDUCAM_ARDUCAM_H_

#include "Arduino.h"
#include <pins_arduino.h>

#if defined (__AVR__)

#define cbi(reg, bitmask) *reg &= ~bitmask
#define sbi(reg, bitmask) *reg |= bitmask
#define pulse_high(reg, bitmask) sbi(reg, bitmask); cbi(reg, bitmask);
#define pulse_low(reg, bitmask) cbi(reg, bitmask); sbi(reg, bitmask);
#define cport(port, data) port &= data
#define sport(port, data) port |= data
#define swap(type, i, j) {type t = i; i = j; j = t;}
#define fontbyte(x) pgm_read_byte(&cfont.font[x])
#define regtype volatile uint8_t
#define regsize uint8_t
#endif

#if defined(__arm__)
#define cbi(reg, bitmask) *reg &= ~bitmask
#define sbi(reg, bitmask) *reg |= bitmask
#define pulse_high(reg, bitmask) sbi(reg, bitmask); cbi(reg, bitmask);
#define pulse_low(reg, bitmask) cbi(reg, bitmask); sbi(reg, bitmask);
#define cport(port, data) port &= data
#define sport(port, data) port |= data
#define swap(type, i, j) {type t = i; i = j; j = t;}
#define fontbyte(x) cfont.font[x]
#define regtype volatile uint32_t
#define regsize uint32_t
#define PROGMEM
  #if defined F
    #undef F
  #endif
  #define F(X) (X)
#endif

/* Sensor related definition */
#define BMP 0
#define JPEG 1

#define OV7670 0
#define MT9D111_A 1
#define OV7675 2
#define OV5642 3
#define OV3640 4
#define OV2640 5
#define OV9655 6
#define MT9M112 7
#define OV7725 8
#define OV7660 9
#define MT9M001 10
#define OV5640 11
#define MT9D111_B 12
#define OV9650 13
#define MT9V111 14
#define MT9T112 15
#define MT9D112 16

#define OV5642_320x240 0  // 320x240
#define OV5642_640x480 1  // 640x480
#define OV5642_1280x720 2  // 1280x720
#define OV5642_1920x1080 3  // 1920x1080
#define OV5642_2048x1563 4  // 2048x1563
#define OV5642_2592x1944 5  // 2592x1944

#define OV5640_320x240 		0	// 320x240 
#define OV5640_352x288		1	// 352x288
#define OV5640_640x480 	  2	// 640x480
#define OV5640_800x480	  3	// 800x480
#define OV5640_1024x768	  4	// 1024x768
#define OV5640_1280x960	  5	// 1280x960	
#define OV5640_1600x1200	6	 // 1600x1200
#define OV5640_2048x1536	7  // 2048x1536
#define OV5640_2592x1944	8	 // 2592x1944

// I2C, SCCB
#define I2C_ADDR_8BIT 0
#define I2C_ADDR_16BIT 1
#define I2C_REG_8BIT 0
#define I2C_REG_16BIT 1
#define I2C_DAT_8BIT 0
#define I2C_DAT_16BIT 1
#define SIO_CLKDELAY 100

/* Register initialization tables for SENSORs */
/* Terminating list entry for reg */
#define SENSOR_REG_TERM_8BIT                0xFF
#define SENSOR_REG_TERM_16BIT               0xFFFF
/* Terminating list entry for val */
#define SENSOR_VAL_TERM_8BIT                0xFF
#define SENSOR_VAL_TERM_16BIT               0xFFFF

// ArduChip
#define RWBIT 0x80  // READ AND WRITE BIT IS BIT[7]

#define ARDUCHIP_TEST1       	0x00  //TEST register
#define ARDUCHIP_TEST2      	0x01  //TEST register

#define ARDUCHIP_FRAMES			  0x01  //Bit[2:0]Number of frames to be captured

#define ARDUCHIP_MODE      		0x02  //Mode register
#define MCU2LCD_MODE       		0x00
#define CAM2LCD_MODE       		0x01
#define LCD2MCU_MODE       		0x02

#define ARDUCHIP_TIM       		0x03  //Timming control
#define HREF_LEVEL_MASK    		0x01  //0 = High active , 		1 = Low active
#define VSYNC_LEVEL_MASK   		0x02  //0 = High active , 		1 = Low active
#define LCD_BKEN_MASK      		0x04  //0 = Enable, 			1 = Disable
#define DELAY_MASK         		0x08  //0 = no delay, 			1 = delay one clock
#define MODE_MASK          		0x10  //0 = LCD mode, 			1 = FIFO mode
#define FIFO_PWRDN_MASK	   		0x20  //0 = Normal operation, 	1 = FIFO power down
#define LOW_POWER_MODE			  0x40  //0 = Normal mode, 		1 = Low power mode

#define ARDUCHIP_FIFO      		0x04  //FIFO and I2C control
#define FIFO_CLEAR_MASK    		0x01
#define FIFO_START_MASK    		0x02
#define FIFO_RDPTR_RST_MASK     0x10
#define FIFO_WRPTR_RST_MASK     0x20

#define ARDUCHIP_GPIO 0x06  // GPIO Write Register
#define GPIO_RESET_MASK 0x01  // 0 = default state, 1 = Sensor reset IO value
#define GPIO_PWDN_MASK 0x02  // 0 = power down, 1 = Sensor power enable IO value

#define BURST_FIFO_READ 0x3C  // Burst FIFO read operation
#define SINGLE_FIFO_READ 0x3D  // Single FIFO read operation

#define ARDUCHIP_REV 0x40  // ArduCHIP revision
#define VER_LOW_MASK 0x3F
#define VER_HIGH_MASK 0xC0

#define ARDUCHIP_TRIG 0x41  // Trigger source
#define VSYNC_MASK 0x01
#define SHUTTER_MASK 0x02
#define CAP_DONE_MASK 0x08

#define FIFO_SIZE1 0x42  // Camera write FIFO size[7:0] for burst to read
#define FIFO_SIZE2 0x43  // Camera write FIFO size[15:8]
#define FIFO_SIZE3 0x44  // Camera write FIFO size[18:16]

/****************************************************/

struct sensor_reg {
  uint16_t reg;
  uint16_t val;
};

class ArduCAM {
 public:
  ArduCAM(int SIO_C, int SIO_D, int CS);
  void InitCAM();

  void CS_HIGH(void);
  void CS_LOW(void);

  void flush_fifo(void);
  void start_capture(void);
  void clear_fifo_flag(void);
  uint8_t read_fifo(void);

  uint8_t read_reg(uint8_t addr);
  void write_reg(uint8_t addr, uint8_t data);

  uint32_t read_fifo_length(void);
  void set_fifo_burst(void);
  void set_bit(uint8_t addr, uint8_t bit);
  void clear_bit(uint8_t addr, uint8_t bit);
  uint8_t get_bit(uint8_t addr, uint8_t bit);
  void set_mode(uint8_t mode);

  void OV5642_set_JPEG_size(uint8_t size);
  void set_format(byte fmt);

  void transferBytes_(uint8_t * out, uint8_t * in, uint8_t size);
  void transferBytes(uint8_t * out, uint8_t * in, uint32_t size);
  inline void setDataBits(uint16_t bits);

  void bus_write(int address, int value);
  uint8_t bus_read(int address);

  int wrSensorRegs(const struct sensor_reg*);
  int wrSensorRegs8_8(const struct sensor_reg*);
  int wrSensorRegs16_8(const struct sensor_reg*);

  byte wrSensorReg8_8(int regID, int regData);
  byte wrSensorReg16_8(int regID, int regData);

  byte rdSensorReg8_8(uint8_t regID, uint8_t* regData);
  byte rdSensorReg16_8(uint16_t regID, uint8_t* regData);

  // sccb code
  int SIO_C_;
  int SIO_D_;
  byte sensor_addr_;

  void InitSCCB(void);
  int ScanSCCB(void);
  void StartSCCB(void);
  void StopSCCB(void);
  byte WriteSCCB(byte m_data);
  byte ReadSCCB(byte* m_data);

 protected:
  regtype *P_CS;
  regsize B_CS;
  byte m_fmt;
  byte sensor_model;
};

#endif  // ARDUCAM_ARDUCAM_H_
