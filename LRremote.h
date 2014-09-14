/*
 * LRremote.h
 * Version 0.1 September 2014
 * Copyright 2014 by D. L. Ehnebuske
 *
 * Based heavily on IRRemote v0.11 by Ken Shirriff. See http://arcfn.com/2009/08/multi-protocol-infrared-remote-library.html
 *
 * See LRremote.cpp for documentation and details.
 *
 */

#ifndef LRremote_h
#define LRremote_h

// The following are compile-time library options.
// If you change them, recompile the library.
// If DEBUG is defined, a lot of debugging output will be printed during decoding.
// #define DEBUG

// Values for decode_type
#define NEC 1
#define SONY 2
#define RC5 3
#define RC6 4
#define DISH 5
#define SHARP 6
#define PANASONIC 7
#define JVC 8
#define SANYO 9
#define MITSUBISHI 10
#define SAMSUNG 11
#define LG 12
#define UNKNOWN -1

// Some useful constants

#define RAWBUF 100			// Length of raw duration buffer
#define REPEAT 0xffffffff	// Decoded value for NEC when a repeat code is received
#define REPEAT_PAUSE (3)	// Number of repeat codes to ignore before deciding the user means it

// Marks tend to be 100us too long, and spaces 100us too short
// when received due to sensor lag.
#define MARK_EXCESS 100

// main class for receiving IR
class LRremote
{
public:
	LRremote(int rpin);												// Constructor
	void enable();													// Enable timer interrupts
	bool onButton(long code[], void (*fButton[])(), int codeCount);	// Button function invoker

private:
	// Instance variables
	int decode_type;							// NEC, SONY, RC5, etc.
	unsigned int panasonicAddress;				// This is only used for decoding Panasonic data
	unsigned long value;						// Decoded value
	int bits;									// Number of bits in decoded value
	long lastValue;								// Value last time a button pushed (for REPEAT processing)
	int repeat;									// How many times the REPEAT code was received in a row

	// Methods
	void resume();								// Resume collecting transmitted values
	bool decode();								// Decode collected MARKs and SPACEs and place in value
	bool decodeNEC();							//   Decoders and helpers for various types of remotes
	bool decodeSony();
	bool decodeSanyo();
	bool decodeMitsubishi();
	int getRClevel(int *offset, int *used, int t1);
	bool decodeRC5();
	bool decodeRC6();
	bool decodePanasonic();
	bool decodeLG();
	bool decodeJVC();
	bool decodeSAMSUNG();
	int compare(unsigned int oldval, unsigned int newval);
	bool decodeHash();
} 
;

#endif
