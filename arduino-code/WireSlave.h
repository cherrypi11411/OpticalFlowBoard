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

#ifndef _WIRESLAVE_HEADER
#define _WIRESLAVE_HEADER

#include "Stream.h"
#include "variant.h"

#define BUFFER_LENGTH 32

// WIRE_HAS_END means Wire has end()
#define WIRE_HAS_END 1

class TwoWireSlave
{
  public:
    TwoWireSlave(struct _stm32l4_i2c_t *i2c, unsigned int instance, const struct _stm32l4_i2c_pins_t *pins, unsigned int priority, unsigned int mode);
    void begin(uint8_t address);

    size_t write(uint8_t data);

    int read(void);
    void onReceive(void(*callback)(int));
    void onRequest(void(*callback)(void));

    uint8_t status(void);

    uint8_t state(void);
    int xf_count(void);
    int tx_count(void);
    int get_isr(void);
    int get_cr1(void);
    int get_cr2(void);
    int get_icr(void);
    int get_oar1(void);
    int get_sda_pin(void);
    int get_scl_pin(void);

    uint32_t get_events(void) { return _events; }

    void onOverrideHandler(void(*callback)(struct _stm32l4_i2c_t *));

  private:
    struct _stm32l4_i2c_t *_i2c;
    const struct _stm32l4_i2c_pins_t *_pins;
    uint32_t _clock;
    uint32_t _option;

    uint8_t _rx_data[BUFFER_LENGTH];
    uint8_t _rx_read;
    uint8_t _rx_write;

    uint8_t _tx_data[BUFFER_LENGTH];
    uint8_t _tx_write;
    uint8_t _tx_address;
    bool _tx_active;

    uint32_t _events;

    void (*_completionCallback)(uint8_t);
    void (*_requestCallback)(void);
    void (*_receiveCallback)(int);

    static void _eventCallback(void *context, uint32_t events);
    void EventCallback(uint32_t events);

    static const uint32_t TWI_CLOCK = 100000;
};

extern TwoWireSlave WireSlave;

#endif // _WIRESLAVE_HEADER

