/*
  M24LR.c - Library for STMicroelectronics M24LR RFID/NFC Energy harvesting chip.
  Created by Colin Brahmstedt, July 8, 2018.
  Released into the public domain.

  Modified by pmst, 27/11/2022, to support present password, read lock bits etc

*/

//I had this thing working nicely with the UNO, but it borked when trying to use the Wire
//library for the ATtiny. This is an initial hack to get it working, definitely needs to be cleaned up.

#include <Arduino.h>

#if defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny45__)
#include <TinyWireM.h>
#define lib TinyWireM
#define _i2c TinyWireM
void i2cwrite(uint8_t data){
  TinyWireM.send(data);
}
static uint8_t i2cread(void){
return TinyWireM.receive();
}

#elif defined(__AVR_ATmega328P__)
#include <Wire.h>
#define lib Wire
#define _i2c Wire
void i2cwrite(byte data){
  Wire.write(data);
}
static uint8_t i2cread(void){
return  Wire.read();
}
#endif
#include "Arduino.h"
#include "M24LR.h"


M24LR::M24LR(uint8_t M24LRaddress)
{
  _deviceaddress = M24LRaddress;
  _i2c.begin();
}

uint8_t M24LR::read_byte(uint16_t eeaddress)
{

  uint8_t rdata = 0xFF;
  _i2c.beginTransmission(_deviceaddress);
  i2cwrite((int)(eeaddress >> 8));
  i2cwrite((int)(eeaddress & 0xFF)); 
  _i2c.endTransmission();

  _i2c.requestFrom(_deviceaddress,1);
  if (_i2c.available()) {
    rdata = i2cread();
  }
  return rdata;
}

int16_t M24LR::read(uint16_t eeaddress, void *buf, uint16_t len) 
{
  uint8_t *ptr = (uint8_t *)buf;
  int16_t rlen, tlen = 0;
  int bsize;

  _i2c.beginTransmission(_deviceaddress);
  i2cwrite((int)(eeaddress >> 8)); // MSB
  i2cwrite((int)(eeaddress & 0xFF)); // LSB
  _i2c.endTransmission();

  rlen = len;
  while(rlen > 0)
  {
    if (rlen > 32) 
    {
      rlen -= 32;
      bsize = 32;
    } 
    else 
    {
      bsize = rlen;
      rlen = 0;
    }
    _i2c.requestFrom(_deviceaddress, bsize);
    while (_i2c.available() && bsize > 0) 
    {
      *ptr++ = i2cread();
      tlen++; bsize--;
    }
  }
  return tlen;
}

void M24LR::write_byte(uint16_t eeaddress, uint8_t data){
  int rdata = data;
  _i2c.beginTransmission(_deviceaddress);
  i2cwrite((int)(eeaddress >> 8)); // MSB
  i2cwrite((int)(eeaddress & 0xFF)); // LSB
  i2cwrite(rdata);
  _i2c.endTransmission();
  delay(50);
}

int16_t M24LR::write(uint16_t eeaddress, void* buf, uint16_t len)
{
  uint8_t *ptr = (uint8_t *)buf;
  int16_t rlen, tlen = 0;
  int bsize;


  rlen = len;
  while (rlen > 0)
  {
    _i2c.beginTransmission(_deviceaddress);
    i2cwrite((int)(eeaddress >> 8)); // MSB
    i2cwrite((int)(eeaddress & 0xFF)); // LSB
    
    bsize = (4 - (eeaddress % 4));

    bsize = min(rlen, bsize);
    
    tlen += _i2c.write(ptr, bsize);
    char b[64];snprintf(b, 64, "rl %d, bs %d, t %d, ptr %p, eeadress %04X\n", rlen, bsize, tlen, ptr, eeaddress); Serial.println(b);

    rlen -= bsize;
    eeaddress += bsize;
    ptr += bsize;
    
    _i2c.endTransmission();
  }
  return tlen;
}

uint8_t read_system_byte(uint8_t devaddr, uint16_t eeaddress)
{

  uint8_t rdata = 0xFF;
  _i2c.beginTransmission(devaddr | 0x04);
  i2cwrite((int)(eeaddress >> 8)); // MSB
  i2cwrite((int)(eeaddress & 0xFF)); // LSB
  _i2c.endTransmission();

  _i2c.requestFrom(devaddr| 0x04,1);
  if (_i2c.available()) {
    rdata = i2cread();
  }
  return rdata;
}

uint32_t read_system_long(uint8_t devaddr, uint16_t eeaddress)
{
  uint32_t l = 0xFFFFFFFF;

  _i2c.beginTransmission(devaddr | 0x04);
  i2cwrite((int)(eeaddress >> 8)); // MSB
  i2cwrite((int)(eeaddress & 0xFF)); // LSB
  _i2c.endTransmission();

  _i2c.requestFrom(devaddr|0x04, 4);

  delay(1);

  if (_i2c.available())
    _i2c.readBytes((uint8_t*)&l, 4);

   
   return l;
}

void write_system_byte(uint8_t devaddr, uint16_t eeaddress, uint8_t data)
{
  int rdata = data;
  _i2c.beginTransmission(devaddr | 0x04);
  i2cwrite((int)(eeaddress >> 8)); // MSB
  i2cwrite((int)(eeaddress & 0xFF)); // LSB
  i2cwrite(rdata);
  _i2c.endTransmission();
  delay(50);
}

void write_system_long(uint8_t devaddr, uint16_t eeaddress, uint32_t data)
{
  uint8_t *rdata = (uint8_t*)&data;
  _i2c.beginTransmission(devaddr | 0x04);
  i2cwrite((int)(eeaddress >> 8)); // MSB
  i2cwrite((int)(eeaddress & 0xFF)); // LSB
  i2cwrite(rdata[3]);
  i2cwrite(rdata[2]);
  i2cwrite(rdata[1]);
  i2cwrite(rdata[0]);
  _i2c.endTransmission();
  delay(50);
}

void M24LR::EH_enable()
{
    byte temp=read_system_byte(_deviceaddress, 2336);
     write_system_byte(_deviceaddress, 2336, temp|0x01);
}


void M24LR::EH_disable()
{
    byte temp=read_system_byte(_deviceaddress, 2336);
     write_system_byte(_deviceaddress, 2336, temp&0xFE);
}

void M24LR::EH_config(uint8_t config)
{
     write_system_byte(_deviceaddress, 2320, config);
}

boolean M24LR::FIELD_PRESENT(){
    return((read_system_byte(_deviceaddress, 2336)>>1)&0x01);
}

uint32_t M24LR::read_lock_bits()
{
  uint32_t lock_bits = 0;

  _i2c.beginTransmission(_deviceaddress | 0x04);
  i2cwrite((int)(2048 >> 8)); // MSB
  i2cwrite((int)(2048 & 0xFF)); // LSB
  _i2c.endTransmission();

  _i2c.requestFrom(_deviceaddress|0x04, 4);

  delay(1);

  _i2c.readBytes((uint8_t*)&lock_bits, 4);
   
   return lock_bits;
}

uint64_t M24LR::read_uuid()
{
  uint64_t uuid = 0;

  _i2c.beginTransmission(_deviceaddress|0x04);
  i2cwrite((int)(2324 >> 8)); // MSB
  i2cwrite((int)(2324 & 0xFF)); // LSB
  _i2c.endTransmission();

  _i2c.beginTransmission(_deviceaddress|0x04);
  _i2c.requestFrom(_deviceaddress|0x04, 8);

  delay(1);

  _i2c.readBytes((uint8_t*)&uuid, 8);
    
    return uuid;
}

uint8_t M24LR::read_sectory_security_status(uint8_t sector)
{
    return read_system_byte(_deviceaddress, (uint16_t)sector);
}

void M24LR::write_sectory_security_status(uint8_t sector, uint8_t mask)
{
  return write_system_byte(_deviceaddress, (uint16_t)sector, mask);
}

void M24LR::present_password(uint8_t* passWd) {

    _i2c.beginTransmission(_deviceaddress|0x04);
    i2cwrite(0x09);
    i2cwrite(0x00);

    for (int i = 0; i < PASSWORD_LENGTH; i++) {
        i2cwrite(passWd[i]);
    }
    i2cwrite(0x09);
    for (int i = 0; i < PASSWORD_LENGTH; i++) {
        i2cwrite(passWd[i]);
    }
    _i2c.endTransmission();
    delay(50);
}

void M24LR::write_password(byte* passWd) {

 _i2c.beginTransmission(_deviceaddress|0x04);
    i2cwrite(0x09);
    i2cwrite(0x00);

    for (int i = 0; i < PASSWORD_LENGTH; i++) {
        i2cwrite(passWd[i]);
    }
    i2cwrite(0x07);
    for (int i = 0; i < PASSWORD_LENGTH; i++) {
        i2cwrite(passWd[i]);
    }
    _i2c.endTransmission();
    delay(50);
}

