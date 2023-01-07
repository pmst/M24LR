/*
  M24LR.h - Library for STMicroelectronics M24LR RFID/NFC Energy harvesting chip.
  Created by Colin Brahmstedt, July 8, 2018.
  Released into the public domain.
*/
#ifndef M24LR_h
#define M24LR_h

#if defined(__AVR_ATtiny45__) || defined(__AVR_ATmega2560__)
#include <TinyWireM.h>
#else
#include <Wire.h>
#endif

#include <Arduino.h>

#define PASSWORD_LENGTH 4

class M24LR
{
public:
  M24LR(uint8_t M24LRaddress);
  uint8_t read_byte(uint16_t eeaddress );
  int16_t read(uint16_t eeaddresse, void* buf, uint16_t len);

  void write_byte(uint16_t eeaddress, uint8_t data);
  int16_t write(uint16_t eeaddress, void* buf, uint16_t len);

  uint32_t read_lock_bits();
  uint64_t read_uuid();

  uint8_t read_sectory_security_status(uint8_t sector);
  void write_sectory_security_status(uint8_t sector, uint8_t mask);

  void present_password(uint8_t *password);
  void write_password(uint8_t *password);

  void EH_enable();
  void EH_disable();
  void EH_config(uint8_t eh_config); 
  boolean FIELD_PRESENT();

private:
  uint8_t _deviceaddress;

  

};
#endif