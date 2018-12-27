/*
   Copyright (c) 2016 Thomas Roell.  All rights reserved.

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to
   deal with the Software without restriction, including without limitation the
   rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
   sell copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

    1. Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimers.
    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimers in the
       documentation and/or other materials provided with the distribution.
    3. Neither the name of Thomas Roell, nor the names of its contributors
       may be used to endorse or promote products derived from this Software
       without specific prior written permission.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
   CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
   FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
   WITH THE SOFTWARE.
*/

#include "Arduino.h"
#include "stm32l4_wiring_private.h"
#include "WireSlave.h"

TwoWireSlave::TwoWireSlave(struct _stm32l4_i2c_t *i2c, unsigned int instance, const struct _stm32l4_i2c_pins_t *pins, unsigned int priority, unsigned int mode)
{
  _i2c = i2c;
  _pins = pins;

  _clock = TWI_CLOCK;
  _option = 0;

  stm32l4_i2c_create(i2c, instance, pins, priority, mode);

  _rx_read = 0;
  _rx_write = 0;

  _tx_write = 0;
  _tx_active = false;

  _events = 0;

  _completionCallback = NULL;
  _requestCallback = NULL;
  _receiveCallback = NULL;
}

void TwoWireSlave::begin(uint8_t address)
{
  _option = I2C_OPTION_RESET | (address << I2C_OPTION_ADDRESS_SHIFT) | I2C_OPTION_ALTERNATE;

  stm32l4_i2c_enable(_i2c, _clock, _option, TwoWireSlave::_eventCallback, (void*)this,
    (I2C_EVENT_RECEIVE_REQUEST | I2C_EVENT_RECEIVE_DONE | I2C_EVENT_TRANSMIT_REQUEST | I2C_EVENT_ADDRESS_MATCH | I2C_EVENT_TRANSMIT_DONE));
}

size_t TwoWireSlave::write(uint8_t data)
{
  if (!_tx_active) {
    return 0;
  }

  if (_tx_write >= BUFFER_LENGTH) {
    return 0;
  }

  _tx_data[_tx_write++] = data;

  return 1;
}

int TwoWireSlave::read(void)
{
  if (_rx_read >= _rx_write) {
    return -1;
  }

  return _rx_data[_rx_read++];
}

uint8_t TwoWireSlave::state(void)
{
  return _i2c->state;
}

uint8_t TwoWireSlave::status(void)
{
  unsigned int status = stm32l4_i2c_status(_i2c);

  if (status == 0) {
    return 0;
  }

  if (status & I2C_STATUS_ADDRESS_NACK) {
    return 2;
  }

  else if (status & I2C_STATUS_DATA_NACK) {
    return 3;
  }

  else {
    return 4;
  }
}

int TwoWireSlave::xf_count(void)
{
  return _i2c->xf_count;
}

int TwoWireSlave::tx_count(void)
{
  return _i2c->tx_count;
}

int TwoWireSlave::get_cr1()
{
  return _i2c->I2C->CR1;
}

int TwoWireSlave::get_cr2()
{
  return _i2c->I2C->CR2;
}

int TwoWireSlave::get_isr()
{
  return _i2c->I2C->ISR;
}

int TwoWireSlave::get_icr()
{
  return _i2c->I2C->ICR;
}

int TwoWireSlave::get_oar1()
{
  return _i2c->I2C->OAR1;
}

int TwoWireSlave::get_sda_pin()
{
  return _pins->sda;
}

int TwoWireSlave::get_scl_pin()
{
  return _pins->scl;
}

void TwoWireSlave::onOverrideHandler(void(*callback)(struct _stm32l4_i2c_t *))
{
  stm32l4_i2c_override_irq_handler(_i2c, callback);
}

void TwoWireSlave::onReceive(void(*callback)(int))
{
  _receiveCallback = callback;
}

void TwoWireSlave::onRequest(void(*callback)(void))
{
  _requestCallback = callback;
}

void TwoWireSlave::EventCallback(uint32_t events)
{
  _events = events;
  unsigned int status;

  Serial.println("EventCallback");

  if (events & I2C_EVENT_ADDRESS_MATCH)
  {
    Serial.println("Address match");
  }

  if (events & I2C_EVENT_RECEIVE_DONE) {
    Serial.println("Receive done");
    _rx_read = 0;
    _rx_write = stm32l4_i2c_count(_i2c);

    if (_receiveCallback) {
      (*_receiveCallback)(_rx_write);
    }

    stm32l4_i2c_service(_i2c, &_rx_data[0], BUFFER_LENGTH);
  }

  if (events & I2C_EVENT_RECEIVE_REQUEST) {
    Serial.println("Receive request");
    stm32l4_i2c_service(_i2c, &_rx_data[0], BUFFER_LENGTH);
  }

  if (events & I2C_EVENT_TRANSMIT_DONE)
  {
    Serial.println("Transmit done");
  }

  if (events & I2C_EVENT_TRANSMIT_REQUEST) {
    Serial.println("Transmit request");
    _tx_write = 0;
    _tx_active = true;

    if (_requestCallback) {
      (*_requestCallback)();
    }

    stm32l4_i2c_service(_i2c, &_tx_data[0], _tx_write);

    _tx_active = false;
  }

  _events = 0;
}

void TwoWireSlave::_eventCallback(void *context, uint32_t events)
{
  reinterpret_cast<class TwoWireSlave*>(context)->EventCallback(events);
}

extern const stm32l4_i2c_pins_t g_WirePins;
extern const unsigned int g_WireInstance;
extern const unsigned int g_WireMode;

static stm32l4_i2c_t _Wire;

TwoWireSlave WireSlave(&_Wire, g_WireInstance, &g_WirePins, STM32L4_I2C_IRQ_PRIORITY, g_WireMode);

