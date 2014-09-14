/*****
 * LRremote.cpp
 * Version 0.1 September 2014
 * Copyright 2014 by D. L. Ehnebuske
 *
 * Based heavily on IRRemote v0.11 by Ken Shirriff. See http://arcfn.com/2009/08/multi-protocol-infrared-remote-library.html
 *
 * Restructured and cut down for the simple IR receiver use case of an action function for each button of interest. 
 *
 * Normal use consists of instantiating a LRremote object in the static part of the sketch, invoking enable() in the 
 * sketch's setup() function and then invoking onButton() in the sketch's loop() function to get input from the IR remote.
 *
 *****/

#include "LRremote.h"
#include "LRremoteInt.h"

// Provides ISR macros
#include <avr/interrupt.h>

/*
 *
 * ISR state data
 *
 */
int recvpin;							// Pin that the IR receiver is attached to
volatile int rcvstate;					// The state of the ISR state machine
volatile unsigned int timer;			// State timer, counts 50uS ticks.
volatile unsigned int rawbuf[RAWBUF];	// Raw data
volatile unsigned int rawlen;			// Counter of entries in rawbuf

/*
 * Constructor for LRremote object
 *
 * rpin is the pin to which the IR receiver is attached.
 *
 */
LRremote::LRremote(int rpin) {
	recvpin = rpin;						// Remember which pin the IR receiver is on
  
	lastValue = repeat = 0;				// Init lastValue and repeat
	rcvstate = STATE_IDLE;				// Initialize state machine variables
	rawlen = 0;
	pinMode(recvpin, INPUT);			// Set pin mode so we can read the IR receiver

}

/*
 * enable() -- Enable timer interrupts.
 *
 * Needs to be a separate method invoked during setup(). Can't do it in the constructor because the Arduino
 * environment messes with the timer during reset.
 *
 */

void LRremote::enable() {
	cli();								// Disable interrupts
	TIMER_CONFIG_NORMAL();				// Set clock interrupt interval to 50ms
	TIMER_ENABLE_INTR;					// Enable clock interrupt
	TIMER_RESET;						// Reset timer
	sei();								// Enable interrupts
}

/*
 * Timer interrupt service routine (ISR) to collect raw data.
 *
 * Durations, measured in 50 microsecond ticks, of alternating SPACE, MARK are recorded in rawbuf[].
 * The count of entries recorded so far is in rawlen. The first entry is the long SPACE between transmissions.
 *
 * The ISR is a state machine driven by data received through the IR receiver. It starts in STATE_IDLE. At
 * each clock tick, the state of the IR receiver is sampled and, based on the current state of the state 
 * machine and the IR receiver state, duration of each MARK and SPACE in the sequence is recorded. Normally,
 * a sequence ends with a long SPACE. It can also end -- abnormally -- by filling up the buffer. In either case
 * when the sequence ends, recording stops and the machine remains in STATE_STOP until it's reset to 
 * STATE_IDLE by LRremote::resume().
 *
 */
ISR(TIMER_INTR_NAME) {
	TIMER_RESET;

	uint8_t irdata = (uint8_t)digitalRead(recvpin);			// Sample the state of the IR receiver

	timer++;												// Count one more 50us tick.
	if (rawlen >= RAWBUF) {									// If the buffer is full to capacity
		rcvstate = STATE_STOP;								//  We had a transmission error. Stop recording.
	}
	switch(rcvstate) {
		case STATE_IDLE:									// We're idling, supposedly in the middle of a gap
			if (irdata == MARK) {							//   If it looks like that just ended
				if (timer < GAP_TICKS) {					//     Make sure it's big enough to be real.
					timer = 0;								//     If not ignore it.
				} else {									//     Else gap just ended
					rawlen = 0;								//       Record duration and start recording transmission
					rawbuf[rawlen++] = timer;
					timer = 0;
					rcvstate = STATE_MARK;
				}
			} else if (timer > GAP_TICKS) {					//   Else the gap continues
				timer = GAP_TICKS;							//     We just need to know it's a long one, not overflow
			}
			break;
		case STATE_MARK:									// We're timing a MARK
			if (irdata == SPACE) {  						//  If the MARK ended
				rawbuf[rawlen++] = timer;					//    Record the duration
				timer = 0;
				rcvstate = STATE_SPACE;						//    and start recording the SPACE that follows
			}
			break;
		case STATE_SPACE:									// We're timing a SPACE
			if (irdata == MARK) {							// If the SPACE just ended
				rawbuf[rawlen++] = timer;					//   Record the duration
				timer = 0;
				rcvstate = STATE_MARK;						//   and start recording the MARK that follows
			} else {										// Else the SPACE continues
				if (timer >= GAP_TICKS) {					//   If it's a long space
					rcvstate = STATE_STOP;					//   We're done recording the sequence. No recording
				}											//     until someone processes it.
			}
			break;
		case STATE_STOP:									// We're waiting for someone to process what we recorded
			timer = 0;										//   Nothing interesting going on; no need to count
			break;
	}
}

/*
 *
 * Resume recording transmissions.
 *
 * Restart the ISR state machine: STAT_STOP -> STATE_IDLE
 *
 */

void LRremote::resume() {
	rawlen = 0;												// Restart recording from beginning of buffer
	rcvstate = STATE_IDLE;									// ISR state machine starts in idle state
}

/*
 *
 * Here to decode the received IR message.
 *
 * Returns false if no data ready, true if data ready. When true is returned, the results of decoding are 
 * stored in value and related private instance variables.
 *
 */
bool LRremote::decode() {
	if (rcvstate != STATE_STOP) {
		return false;
	}
#ifdef DEBUG
	Serial.println("Attempting NEC decode");
#endif
	if (decodeNEC()) {
		return true;
	}
#ifdef DEBUG
	Serial.println("Attempting Sony decode");
#endif
	if (decodeSony()) {
		return true;
	}
#ifdef DEBUG
	Serial.println("Attempting Sanyo decode");
#endif
	if (decodeSanyo()) {
		return true;
	}
#ifdef DEBUG
	Serial.println("Attempting Mitsubishi decode");
#endif
	if (decodeMitsubishi()) {
		return true;
	}
#ifdef DEBUG
	Serial.println("Attempting RC5 decode");
#endif  
	if (decodeRC5()) {
		return true;
	}
#ifdef DEBUG
	Serial.println("Attempting RC6 decode");
#endif 
	if (decodeRC6()) {
		return true;
	}
#ifdef DEBUG
	Serial.println("Attempting Panasonic decode");
#endif 
	if (decodePanasonic()) {
		return true;
	}
#ifdef DEBUG
	Serial.println("Attempting LG decode");
#endif 
	if (decodeLG()) {
		return true;
	}
#ifdef DEBUG
	Serial.println("Attempting JVC decode");
#endif 
	if (decodeJVC()) {
		return true;
	}
#ifdef DEBUG
	Serial.println("Attempting SAMSUNG decode");
#endif
	if (decodeSAMSUNG()) {
		return true;
	}
	// decodeHash returns a hash on any input.
	// Thus, it needs to be last in the list.
	// If you add any decodes, add them before this.
	if (decodeHash()) {
		return true;
	}
	// Unrecognized; throw away and start over
	resume();
	return false;
}

// Two versions of MATCH, MATCH_MARK, and MATCH_SPACE the first for debugging the second -- macros -- for production
#ifdef DEBUG
int MATCH(int measured, int desired) {
	Serial.print("Testing: ");
	Serial.print(TICKS_LOW(desired), DEC);
	Serial.print(" <= ");
	Serial.print(measured, DEC);
	Serial.print(" <= ");
	Serial.println(TICKS_HIGH(desired), DEC);
	return measured >= TICKS_LOW(desired) && measured <= TICKS_HIGH(desired);
}

int MATCH_MARK(int measured_ticks, int desired_us) {
	Serial.print("Testing mark ");
	Serial.print(measured_ticks * USECPERTICK, DEC);
	Serial.print(" vs ");
	Serial.print(desired_us, DEC);
	Serial.print(": ");
	Serial.print(TICKS_LOW(desired_us + MARK_EXCESS), DEC);
	Serial.print(" <= ");
	Serial.print(measured_ticks, DEC);
	Serial.print(" <= ");
	Serial.println(TICKS_HIGH(desired_us + MARK_EXCESS), DEC);
	return measured_ticks >= TICKS_LOW(desired_us + MARK_EXCESS) && measured_ticks <= TICKS_HIGH(desired_us + MARK_EXCESS);
}

int MATCH_SPACE(int measured_ticks, int desired_us) {
	Serial.print("Testing space ");
	Serial.print(measured_ticks * USECPERTICK, DEC);
	Serial.print(" vs ");
	Serial.print(desired_us, DEC);
	Serial.print(": ");
	Serial.print(TICKS_LOW(desired_us - MARK_EXCESS), DEC);
	Serial.print(" <= ");
	Serial.print(measured_ticks, DEC);
	Serial.print(" <= ");
	Serial.println(TICKS_HIGH(desired_us - MARK_EXCESS), DEC);
	return measured_ticks >= TICKS_LOW(desired_us - MARK_EXCESS) && measured_ticks <= TICKS_HIGH(desired_us - MARK_EXCESS);
}
#else
int MATCH(int measured, int desired) {return measured >= TICKS_LOW(desired) && measured <= TICKS_HIGH(desired);}
int MATCH_MARK(int measured_ticks, int desired_us) {return MATCH(measured_ticks, (desired_us + MARK_EXCESS));}
int MATCH_SPACE(int measured_ticks, int desired_us) {return MATCH(measured_ticks, (desired_us - MARK_EXCESS));}
// Debugging versions are in IRremote.cpp
#endif

/*
 *
 * Decode functions for the various kinds of remotes we know about
 *
 * They returns true if the decode was successful, false if not. When the decode is successful, 
 * the results of the decode are in "value" and the related private instance variables.
 *
 */

// NEC.
bool LRremote::decodeNEC() {
	long data = 0;
	int offset = 1; // Skip first space
	// Initial mark
	if (!MATCH_MARK(rawbuf[offset], NEC_HDR_MARK)) {
		return false;
	}
	offset++;
	// Check for repeat
	if (rawlen == 4 &&									// NECs have a repeat only 4 items long
		MATCH_SPACE(rawbuf[offset], NEC_RPT_SPACE) &&
		MATCH_MARK(rawbuf[offset+1], NEC_BIT_MARK)) {
		bits = 0;
		value = REPEAT;
		decode_type = NEC;
		return true;
	}
	if (rawlen < 2 * NEC_BITS + 4) {
		return false;
	}
	// Initial space
	if (!MATCH_SPACE(rawbuf[offset], NEC_HDR_SPACE)) {
		return false;
	}
	offset++;
	for (int i = 0; i < NEC_BITS; i++) {
		if (!MATCH_MARK(rawbuf[offset], NEC_BIT_MARK)) {
			return false;
		}
		offset++;
		if (MATCH_SPACE(rawbuf[offset], NEC_ONE_SPACE)) {
			data = (data << 1) | 1;
		} 
		else if (MATCH_SPACE(rawbuf[offset], NEC_ZERO_SPACE)) {
			data <<= 1;
		} 
		else {
			return false;
		}
		offset++;
	}
	// Success
	bits = NEC_BITS;
	value = data;
	decode_type = NEC;
	return true;
}

// Sony.
bool LRremote::decodeSony() {
	long data = 0;
	if (rawlen < 2 * SONY_BITS + 2) {
		return false;
	}
	int offset = 0; // Dont skip first space, check its size

	// Some Sony's deliver repeats fast after first
	// unfortunately can't spot difference from of repeat from two fast clicks
	if (rawbuf[offset] < SONY_DOUBLE_SPACE_USECS) {
		// Serial.print("IR Gap found: ");
		bits = 0;
		value = REPEAT;
		decode_type = SANYO;
		return true;
	}
	offset++;

	// Initial mark
	if (!MATCH_MARK(rawbuf[offset], SONY_HDR_MARK)) {
		return false;
	}
	offset++;

	while (offset + 1 < rawlen) {
		if (!MATCH_SPACE(rawbuf[offset], SONY_HDR_SPACE)) {
			break;
		}
		offset++;
		if (MATCH_MARK(rawbuf[offset], SONY_ONE_MARK)) {
			data = (data << 1) | 1;
		} 
		else if (MATCH_MARK(rawbuf[offset], SONY_ZERO_MARK)) {
			data <<= 1;
		} 
		else {
			return false;
		}
		offset++;
	}

	// Success
	bits = (offset - 1) / 2;
	if (bits < 12) {
		bits = 0;
		return false;
	}
	value = data;
	decode_type = SONY;
	return true;
}

// Sanyo. Looks like Sony except for timings, 48 chars of data and time/space different
bool LRremote::decodeSanyo() {
	long data = 0;
	if (rawlen < 2 * SANYO_BITS + 2) {
		return false;
	}
	int offset = 0; // Skip first space
	// Initial space
/* Put this back in for debugging - note can't use #DEBUG as if Debug on we don't see the repeat cos of the delay
	Serial.print("IR Gap: ");
	Serial.println( rawbuf[offset]);
	Serial.println( "test against:");
	Serial.println(rawbuf[offset]);
*/
	if (rawbuf[offset] < SANYO_DOUBLE_SPACE_USECS) {
		// Serial.print("IR Gap found: ");
		bits = 0;
		value = REPEAT;
		decode_type = SANYO;
		return true;
	}
	offset++;

	// Initial mark
	if (!MATCH_MARK(rawbuf[offset], SANYO_HDR_MARK)) {
		return false;
	}
	offset++;

	// Skip Second Mark
	if (!MATCH_MARK(rawbuf[offset], SANYO_HDR_MARK)) {
		return false;
	}
	offset++;

	while (offset + 1 < rawlen) {
		if (!MATCH_SPACE(rawbuf[offset], SANYO_HDR_SPACE)) {
			break;
		}
		offset++;
		if (MATCH_MARK(rawbuf[offset], SANYO_ONE_MARK)) {
			data = (data << 1) | 1;
		} 
		else if (MATCH_MARK(rawbuf[offset], SANYO_ZERO_MARK)) {
			data <<= 1;
		} 
		else {
			return false;
		}
		offset++;
	}

	// Success
	bits = (offset - 1) / 2;
	if (bits < 12) {
		bits = 0;
		return false;
	}
	value = data;
	decode_type = SANYO;
	return true;
}

// Mitsubishi. Looks like Sony except for timings, 48 chars of data and time/space different
bool LRremote::decodeMitsubishi() {
/*
	Serial.print("?!? decoding Mitsubishi: ");
	Serial.print(rawlen);
	Serial.print(" want ");
	Serial.println( 2 * MITSUBISHI_BITS + 2);
*/
	long data = 0;
	if (rawlen < 2 * MITSUBISHI_BITS + 2) {
		return false;
	}
	int offset = 0; // Skip first space
	// Initial space
/* Put this back in for debugging - note can't use #DEBUG as if Debug on we don't see the repeat cos of the delay
	Serial.print("IR Gap: ");
	Serial.println( rawbuf[offset]);
	Serial.println( "test against:");
	Serial.println(rawbuf[offset]);
*/
/* Not seeing double keys from Mitsubishi
	if (rawbuf[offset] < MITSUBISHI_DOUBLE_SPACE_USECS) {
//		Serial.print("IR Gap found: ");
		bits = 0;
		value = REPEAT;
		decode_type = MITSUBISHI;
		return true;
	}
*/
	offset++;

	// Typical
	// 14200 7 41 7 42 7 42 7 17 7 17 7 18 7 41 7 18 7 17 7 17 7 18 7 41 8 17 7 17 7 18 7 17 7 

	// Initial Space
	if (!MATCH_MARK(rawbuf[offset], MITSUBISHI_HDR_SPACE)) {
		return false;
	}
	offset++;
	while (offset + 1 < rawlen) {
		if (MATCH_MARK(rawbuf[offset], MITSUBISHI_ONE_MARK)) {
			data = (data << 1) | 1;
		} 
		else if (MATCH_MARK(rawbuf[offset], MITSUBISHI_ZERO_MARK)) {
			data <<= 1;
		} 
		else {
//			Serial.println("A"); Serial.println(offset); Serial.println(rawbuf[offset]);
			return false;
		}
		offset++;
		if (!MATCH_SPACE(rawbuf[offset], MITSUBISHI_HDR_SPACE)) {
//			Serial.println("B"); Serial.println(offset); Serial.println(rawbuf[offset]);
			break;
		}
		offset++;
	}

	// Success
	bits = (offset - 1) / 2;
	if (bits < MITSUBISHI_BITS) {
		bits = 0;
		return false;
	}
	value = data;
	decode_type = MITSUBISHI;
	return true;
}

/*
 * getRClevel() helper for RC5/6 decoding
 *
 * Gets one undecoded level at a time from the raw buffer. The RC5/6 decoding is easier if 
 * the data is broken into time intervals. E.g. if the buffer has MARK for 2 time intervals 
 * and SPACE for 1, successive calls to getRClevel will return MARK, MARK, SPACE.
 *
 * The variables "offset" and "used" are updated to keep track of the current position.
 * The variable "t1" is the time interval for a single bit in microseconds. 
 *
 * Returns -1 for error (measured time interval is not a multiple of t1).
 *
 */
int LRremote::getRClevel(int *offset, int *used, int t1) {
	if (*offset >= rawlen) {
		// After end of recorded buffer, assume SPACE.
		return SPACE;
	}
	int width = rawbuf[*offset];
	int val = ((*offset) % 2) ? MARK : SPACE;
	int correction = (val == MARK) ? MARK_EXCESS : - MARK_EXCESS;

	int avail;
	if (MATCH(width, t1 + correction)) {
		avail = 1;
	} 
	else if (MATCH(width, 2*t1 + correction)) {
		avail = 2;
	} 
	else if (MATCH(width, 3*t1 + correction)) {
		avail = 3;
	}
	else {
		return -1;
	}

	(*used)++;
	if (*used >= avail) {
		*used = 0;
		(*offset)++;
	}
#ifdef DEBUG
	if (val == MARK) {
		Serial.println("MARK");
	} 
	else {
		Serial.println("SPACE");
	}
#endif
	return val;
}

// RC5.
bool LRremote::decodeRC5() {
	if (rawlen < MIN_RC5_SAMPLES + 2) {
		return false;
	}
	int offset = 1; // Skip gap space
	long data = 0;
	int used = 0;
	// Get start bits
	if (getRClevel(&offset, &used, RC5_T1) != MARK) return false;
	if (getRClevel(&offset, &used, RC5_T1) != SPACE) return false;
	if (getRClevel(&offset, &used, RC5_T1) != MARK) return false;
	int nbits;
	for (nbits = 0; offset < rawlen; nbits++) {
		int levelA = getRClevel(&offset, &used, RC5_T1); 
		int levelB = getRClevel(&offset, &used, RC5_T1);
		if (levelA == SPACE && levelB == MARK) {
			// 1 bit
			data = (data << 1) | 1;
		} 
		else if (levelA == MARK && levelB == SPACE) {
			// zero bit
			data <<= 1;
		} 
		else {
			return false;
		} 
	}

	// Success
	bits = nbits;
	value = data;
	decode_type = RC5;
	return true;
}

// RC6.
bool LRremote::decodeRC6() {
	if (rawlen < MIN_RC6_SAMPLES) {
		return false;
	}
	int offset = 1; // Skip first space
	// Initial mark
	if (!MATCH_MARK(rawbuf[offset], RC6_HDR_MARK)) {
		return false;
	}
	offset++;
	if (!MATCH_SPACE(rawbuf[offset], RC6_HDR_SPACE)) {
		return false;
	}
	offset++;
	long data = 0;
	int used = 0;
	// Get start bit (1)
	if (getRClevel(&offset, &used, RC6_T1) != MARK) return false;
	if (getRClevel(&offset, &used, RC6_T1) != SPACE) return false;
	int nbits;
	for (nbits = 0; offset < rawlen; nbits++) {
		int levelA, levelB; // Next two levels
		levelA = getRClevel(&offset, &used, RC6_T1); 
		if (nbits == 3) {
			// T bit is double wide; make sure second half matches
			if (levelA != getRClevel(&offset, &used, RC6_T1)) return false;
		} 
		levelB = getRClevel(&offset, &used, RC6_T1);
		if (nbits == 3) {
			// T bit is double wide; make sure second half matches
			if (levelB != getRClevel(&offset, &used, RC6_T1)) return false;
		} 
		if (levelA == MARK && levelB == SPACE) { // reversed compared to RC5
			// 1 bit
			data = (data << 1) | 1;
		} 
		else if (levelA == SPACE && levelB == MARK) {
			// zero bit
			data <<= 1;
		} 
		else {
			return false; // Error
		} 
	}
	// Success
	bits = nbits;
	value = data;
	decode_type = RC6;
	return true;
}

// Panasonic
bool LRremote::decodePanasonic() {
	unsigned long long data = 0;
	int offset = 1;

	if (!MATCH_MARK(rawbuf[offset], PANASONIC_HDR_MARK)) {
		return false;
	}
	offset++;
	if (!MATCH_MARK(rawbuf[offset], PANASONIC_HDR_SPACE)) {
		return false;
	}
	offset++;

	// decode address
	for (int i = 0; i < PANASONIC_BITS; i++) {
		if (!MATCH_MARK(rawbuf[offset++], PANASONIC_BIT_MARK)) {
			return false;
		}
		if (MATCH_SPACE(rawbuf[offset],PANASONIC_ONE_SPACE)) {
			data = (data << 1) | 1;
		} else if (MATCH_SPACE(rawbuf[offset],PANASONIC_ZERO_SPACE)) {
			data <<= 1;
		} else {
			return false;
		}
		offset++;
	}
	value = (unsigned long)data;
	panasonicAddress = (unsigned int)(data >> 32);
	decode_type = PANASONIC;
	bits = PANASONIC_BITS;
	return true;
}

// LG.
bool LRremote::decodeLG() {
	long data = 0;
	int offset = 1; // Skip first space
  
	// Initial mark
	if (!MATCH_MARK(rawbuf[offset], LG_HDR_MARK)) {
		return false;
	}
	offset++; 
	if (rawlen < 2 * LG_BITS + 1 ) {
		return false;
	}
	// Initial space 
	if (!MATCH_SPACE(rawbuf[offset], LG_HDR_SPACE)) {
		return false;
	}
	offset++;
	for (int i = 0; i < LG_BITS; i++) {
		if (!MATCH_MARK(rawbuf[offset], LG_BIT_MARK)) {
			return false;
		}
		offset++;
		if (MATCH_SPACE(rawbuf[offset], LG_ONE_SPACE)) {
			data = (data << 1) | 1;
		} 
		else if (MATCH_SPACE(rawbuf[offset], LG_ZERO_SPACE)) {
			data <<= 1;
		} 
		else {
			return false;
		}
		offset++;
	}
	//Stop bit
	if (!MATCH_MARK(rawbuf[offset], LG_BIT_MARK)){
		return false;
	}
	// Success
	bits = LG_BITS;
	value = data;
	decode_type = LG;
	return true;
}

// JVC.
bool LRremote::decodeJVC() {
	long data = 0;
	int offset = 1; // Skip first space
	// Check for repeat
	if (rawlen - 1 == 33 &&
		MATCH_MARK(rawbuf[offset], JVC_BIT_MARK) &&
		MATCH_MARK(rawbuf[rawlen-1], JVC_BIT_MARK)) {
		bits = 0;
		value = REPEAT;
		decode_type = JVC;
		return true;
	} 
	// Initial mark
	if (!MATCH_MARK(rawbuf[offset], JVC_HDR_MARK)) {
		return false;
	}
	offset++; 
	if (rawlen < 2 * JVC_BITS + 1 ) {
		return false;
	}
	// Initial space 
	if (!MATCH_SPACE(rawbuf[offset], JVC_HDR_SPACE)) {
		return false;
	}
	offset++;
	for (int i = 0; i < JVC_BITS; i++) {
		if (!MATCH_MARK(rawbuf[offset], JVC_BIT_MARK)) {
			return false;
		}
		offset++;
		if (MATCH_SPACE(rawbuf[offset], JVC_ONE_SPACE)) {
			data = (data << 1) | 1;
		} 
		else if (MATCH_SPACE(rawbuf[offset], JVC_ZERO_SPACE)) {
			data <<= 1;
		} 
		else {
			return false;
		}
		offset++;
	}
	//Stop bit
	if (!MATCH_MARK(rawbuf[offset], JVC_BIT_MARK)){
		return false;
	}
	// Success
	bits = JVC_BITS;
	value = data;
	decode_type = JVC;
	return true;
}

// SAMSUNG.
bool LRremote::decodeSAMSUNG() {
	long data = 0;
	int offset = 1; // Skip first space
	// Initial mark
	if (!MATCH_MARK(rawbuf[offset], SAMSUNG_HDR_MARK)) {
		return false;
	}
	offset++;
	// Check for repeat
	if (rawlen == 4 &&									// SAMSUNGs have a repeat only 4 items long
		MATCH_SPACE(rawbuf[offset], SAMSUNG_RPT_SPACE) &&
		MATCH_MARK(rawbuf[offset+1], SAMSUNG_BIT_MARK)) {
		bits = 0;
		value = REPEAT;
		decode_type = SAMSUNG;
		return true;
	}
	if (rawlen < 2 * SAMSUNG_BITS + 4) {
		return false;
	}
	// Initial space
	if (!MATCH_SPACE(rawbuf[offset], SAMSUNG_HDR_SPACE)) {
		return false;
	}
	offset++;
	for (int i = 0; i < SAMSUNG_BITS; i++) {
		if (!MATCH_MARK(rawbuf[offset], SAMSUNG_BIT_MARK)) {
			return false;
		}
		offset++;
		if (MATCH_SPACE(rawbuf[offset], SAMSUNG_ONE_SPACE)) {
			data = (data << 1) | 1;
		} 
		else if (MATCH_SPACE(rawbuf[offset], SAMSUNG_ZERO_SPACE)) {
			data <<= 1;
		} 
		else {
			return false;
		}
		offset++;
	}
	// Success
	bits = SAMSUNG_BITS;
	value = data;
	decode_type = SAMSUNG;
	return true;
}

/* -----------------------------------------------------------------------
 * hashdecode - decode an arbitrary IR code.
 * Instead of decoding using a standard encoding scheme
 * (e.g. Sony, NEC, RC5), the code is hashed to a 32-bit value.
 *
 * The algorithm: look at the sequence of MARK signals, and see if each one
 * is shorter (0), the same length (1), or longer (2) than the previous.
 * Do the same with the SPACE signals. Hash the resulting sequence of 0's,
 * 1's, and 2's to a 32-bit value. This will give a unique value for each
 * different code (probably), for most code systems.
 *
 * http://arcfn.com/2010/01/using-arbitrary-remotes-with-arduino.html
 */

// Compare two tick values, returning 0 if newval is shorter,
// 1 if newval is equal, and 2 if newval is longer
// Use a tolerance of 20%
int LRremote::compare(unsigned int oldval, unsigned int newval) {
	if (newval < oldval * .8) {
		return 0;
	} 
	else if (oldval < newval * .8) {
		return 2;
	} 
	else {
		return 1;
	}
}

// Use FNV hash algorithm: http://isthe.com/chongo/tech/comp/fnv/#FNV-param
#define FNV_PRIME_32 16777619
#define FNV_BASIS_32 2166136261

/* Converts the raw code values into a 32-bit hash code.
 * Hopefully this code is unique for each button.
 * This isn't a "real" decoding, just an arbitrary value.
 */
bool LRremote::decodeHash() {
#ifdef DEBUG
	Serial.print("decodeHash - rawbuf: ");
	for (int i = 0; i <= rawlen; i++) {
		Serial.print(rawbuf[i]);
		if (i < rawlen) Serial.print(", ");
	}
	Serial.println(".");
#endif
	// Require at least 6 samples to prevent triggering on noise
	if (rawlen < 6) {
		return false;
	}
	long hash = FNV_BASIS_32;
	for (int i = 1; i+2 < rawlen; i++) {
		int value =	compare(rawbuf[i], rawbuf[i+2]);
		// Add value into the hash
		hash = (hash * FNV_PRIME_32) ^ value;
	}
	value = hash;
	bits = 32;
	decode_type = UNKNOWN;
	return true;
}

/*
 *
 *   The onButton method.
 *
 *   Invoke this method to execute the function associated with whatever IR remote button was pressed. If no 
 *   button was pressed since the last invocation, false is returned. If a button has been pressed on the remote, 
 *   this method tries to match the received code with the list of "interesting" codes it is passed. If there's a
 *   match it calls corresponding function from the passed array of functions and returns true. It handles the 
 *   case of receiving an IR code that wasn't matched by returning false.
 *
 *   On NEC remotes, holding down a button generates one instance of the button code followed by the code for 
 *   "repeat" over and over. This is handled by behaving as though the button had been pressed repeatedly, but 
 *   only after the button has been held down for a bit. The delay is to avoid unintended repeats. This behavior
 *   may be overridden by including a button function for the repeat code.
 *
 *   Parameters:
 *     int code[]			An array listing the IR codes that are of interest.
 *     void (*fButton[])()	An array of function pointers corresponding to the code[] array. When IR code[x] is
 *							received, button function fButton[x] is invoked.
 *     long codeCount		The number of entries present in the code[] and fButton[] arrays
 */

bool LRremote::onButton(long code[], void (*fButton[])(), int codeCount) {
	int keyIx;												// Index for code[] and fButton[]
	if (decode()) {											// If an IR code was received
		for (keyIx = 0; keyIx < codeCount; keyIx++) {
			if (value == code[keyIx]) {						//   If it's a code of interest
				repeat = 0;									//     Reset repeat count
				lastValue = value;							//     Remember value for possible future REPEAT processing
				break;
			}
		}													//   Here keyIX = index of received code; codeCount if no match
		if (keyIx >= codeCount && value == REPEAT) {		//   If code is a REPEAT not matched by a button function
			if (++repeat < REPEAT_PAUSE) {					//     Force user to hold down the same key for more than a
				resume();									//     fraction of a second by ignoring the first few repeat
				return false;								//     codes
			}
			for (keyIx = 0; keyIx < codeCount; keyIx++) {	//     redo keyIx for lastValue
				if (lastValue == code[keyIx]) {
					break;
				}
			}
		}
		if (keyIx < codeCount) {							//   If we found a code of interest
			fButton[keyIx]();								//     Do whatever it is we're s'posed to do
			resume();										//     Start looking for codes again
			return true;									//     Say we processed a code
		}
															//   If we get here we received a code but aren't going to
															//   handle it.
		#ifdef DEBUG
		Serial.println("onButton: IR Code not recognized.");
		Serial.print("decode_type: 0x");
		Serial.print(decode_type, HEX);
		Serial.print(", value: 0x");
		Serial.print(value, HEX);
		Serial.print(", bits: ");
		Serial.println(bits);
#endif
		resume();											//   Resume looking for new IR codes
	}
	return false;											// Say we didn't do anything.
}