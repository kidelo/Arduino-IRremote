/*
 * IRremote
 * Version 0.1 July, 2009
 * Copyright 2009 Ken Shirriff
 * For details, see http://arcfn.com/2009/08/multi-protocol-infrared-remote-library.htm http://arcfn.com
 *
 * Interrupt code based on NECIRrcv by Joe Knapp
 * http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1210243556
 * Also influenced by http://zovirl.com/2008/11/12/building-a-universal-remote-with-an-arduino/
 */

#ifndef IRremote_h
#define IRremote_h

#include <stdint.h>

// The following are compile-time library options.
// If you change them, recompile the library.
// If DEBUG is defined, a lot of debugging output will be printed during decoding.
// TEST must be defined for the IRtest unittests to work.  It will make some
// methods virtual, which will be slightly slower, which is why it is optional.
// #define DEBUG
// #define TEST

// Results returned from the decoder
class decode_results {
public:
  int8_t     decode_type; // NEC, SONY, RC5, UNKNOWN
  uint32_t   value;       // Decoded value
  uint8_t    bits;        // Number of bits in decoded value
};

// Values for decode_type
#define NEC 1
#define UNKNOWN -1

// Decoded value for NEC when a repeat code is received
#define REPEAT 0xffffffff



// main class for receiving IR
class IRrecv
{
public:
  IRrecv(int recvpin);
  static bool decode(decode_results *results);
  static void enableIRIn();
private:
  static bool decodeNEC(decode_results *results);
} 
;
// Some useful constants

#define USECPERTICK 100  // microseconds per clock interrupt tick
#define RAWBUF 76        // Length of raw duration buffer

// Marks tend to be 100us too long, and spaces 100us too short
// when received due to sensor lag.
#define MARK_EXCESS 100

#endif
