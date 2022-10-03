// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

/*
 * This Source Code Form is subject to the terms of the MIT License which
 * is included in the root directory of this project and copyrighted to
 * James Pearman
 */

#pragma once

// Based on https://github.com/jpearman/V5_addr_led_demo/blob/master/include/vex_addrled.h
#include "vex.h"

extern "C" {
  // private addressable LED API
  int32_t  vexAdiAddrLedSet( uint32_t index, uint32_t port, uint32_t *pData, uint32_t nOffset, uint32_t nLength, uint32_t options );
}
using namespace vex;

// A class for the addressable led
class AddressableLed {
    public:
        static const int32_t MAX_LEDS = 64;

    private:      
        uint32_t maxled = MAX_LEDS;
        uint32_t ledbuffer[MAX_LEDS];
        uint32_t tmpbuffer[MAX_LEDS];
        int32_t _index;
        int32_t _id;
  
    public:
      AddressableLed( triport::port &port, int32_t max = MAX_LEDS ) {
        port.type( triportType::digitalOutput );
        _index = port.index();
        _id = port.id();
        maxled = max <= MAX_LEDS ? max : MAX_LEDS;
      }

      ~AddressableLed(){}

      void Clear( color col = color(0x000000) ) {
        for(int i=0;i<maxled;i++) {
          ledbuffer[i] = col;
        }
        Set( ledbuffer, 0, maxled, 0);
      }

      void Set( uint32_t *pData, uint32_t nOffset, uint32_t nLength, uint32_t options ) {
        vexAdiAddrLedSet( _index, _id, pData, nOffset, nLength, options );

        // make copy if different buffer
        if( pData != ledbuffer ) {
          uint32_t *p = pData;
          for(int i=nOffset;i<nLength && i<maxled;i++)
            ledbuffer[i] = *p++;
        }
      }

      void Set( color col = color(0x000000) ) {
        for(int i=0;i<maxled;i++) {
          ledbuffer[i] = col;
        }
      }

      void Rotate( int n ) {
          if( abs(n) >= maxled )
            return;

          if( n > 0 ) {
            // make a copy
            memcpy( tmpbuffer, ledbuffer, sizeof(ledbuffer));
            memcpy( &ledbuffer[n], &tmpbuffer[0], (maxled-n)*sizeof(uint32_t) );
            memcpy( &ledbuffer[0], &tmpbuffer[maxled-1-n], (n)*sizeof(uint32_t) );          
          }
          else {
            n = -n;
            memcpy( tmpbuffer, ledbuffer, sizeof(ledbuffer));
            memcpy( &ledbuffer[0], &tmpbuffer[n], (maxled-n)*sizeof(uint32_t) );
            memcpy( &ledbuffer[maxled-n], &tmpbuffer[0], (n)*sizeof(uint32_t) );          
          }
      }

      void Flush() {
        Set( ledbuffer, 0, maxled, 0);
      }

      int32_t Max() {
        return maxled;
      }
};