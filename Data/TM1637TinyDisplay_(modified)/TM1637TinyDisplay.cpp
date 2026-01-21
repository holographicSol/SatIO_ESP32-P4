//  TM1637 Tiny Display
//  Arduino tiny library for TM1637 LED Display
//
//  Author: Jason A. Cox - @jasonacox - https://github.com/jasonacox
//  Date: 27 June 2020
//
//  Based on TM1637Display library at https://github.com/avishorp/TM1637
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

extern "C" {
  #include <stdlib.h>
  #include <string.h>
  #include <inttypes.h>
}

#include <TM1637TinyDisplay.h>
#include <Arduino.h>

#define TM1637_I2C_COMM1    0x40
#define TM1637_I2C_COMM2    0xC0
#define TM1637_I2C_COMM3    0x80

//
//      A
//     ---
//  F |   | B
//     -G-
//  E |   | C
//     ---
//      D
const uint8_t digitToSegment[] = {
 // XGFEDCBA
  0b00111111,    // 0
  0b00000110,    // 1
  0b01011011,    // 2
  0b01001111,    // 3
  0b01100110,    // 4
  0b01101101,    // 5
  0b01111101,    // 6
  0b00000111,    // 7
  0b01111111,    // 8
  0b01101111,    // 9
  0b01110111,    // A
  0b01111100,    // b
  0b00111001,    // C
  0b01011110,    // d
  0b01111001,    // E
  0b01110001     // F
  };

// ASCII Map - Index 0 starts at ASCII 32
const uint8_t asciiToSegment[] = {
   0b00000000, // 032 (Space)
   0b00110000, // 033 !
   0b00100010, // 034 "
   0b01000001, // 035 #
   0b01101101, // 036 $
   0b01010010, // 037 %
   0b01111100, // 038 &
   0b00100000, // 039 '
   0b00111001, // 040 (
   0b00001111, // 041 )
   0b00100001, // 042 *
   0b01110000, // 043 +
   0b00001000, // 044 ,
   0b01000000, // 045 -
   0b00001000, // 046 .
   0b01010010, // 047 /
   0b00111111, // 048 0
   0b00000110, // 049 1
   0b01011011, // 050 2
   0b01001111, // 051 3
   0b01100110, // 052 4
   0b01101101, // 053 5
   0b01111101, // 054 6
   0b00000111, // 055 7
   0b01111111, // 056 8
   0b01101111, // 057 9
   0b01001000, // 058 :
   0b01001000, // 059 ;
   0b00111001, // 060 <
   0b01001000, // 061 =
   0b00001111, // 062 >
   0b01010011, // 063 ?
   0b01011111, // 064 @
   0b01110111, // 065 A
   0b01111100, // 066 B
   0b00111001, // 067 C
   0b01011110, // 068 D
   0b01111001, // 069 E
   0b01110001, // 070 F
   0b00111101, // 071 G
   0b01110110, // 072 H
   0b00000110, // 073 I
   0b00011110, // 074 J
   0b01110110, // 075 K
   0b00111000, // 076 L
   0b00010101, // 077 M
   0b00110111, // 078 N
   0b00111111, // 079 O
   0b01110011, // 080 P
   0b01100111, // 081 Q
   0b00110001, // 082 R
   0b01101101, // 083 S
   0b01111000, // 084 T
   0b00111110, // 085 U
   0b00011100, // 086 V
   0b00101010, // 087 W
   0b01110110, // 088 X
   0b01101110, // 089 Y
   0b01011011, // 090 Z
   0b00111001, // 091 [
   0b01100100, // 092 (backslash)
   0b00001111, // 093 ]
   0b00100011, // 094 ^
   0b00001000, // 095 _
   0b00100000, // 096 `
   0b01110111, // 097 a
   0b01111100, // 098 b
   0b01011000, // 099 c
   0b01011110, // 100 d
   0b01111001, // 101 e
   0b01110001, // 102 f
   0b01101111, // 103 g
   0b01110100, // 104 h
   0b00000100, // 105 i
   0b00011110, // 106 j
   0b01110110, // 107 k
   0b00011000, // 108 l
   0b00010101, // 109 m
   0b01010100, // 110 n
   0b01011100, // 111 o
   0b01110011, // 112 p
   0b01100111, // 113 q
   0b01010000, // 114 r
   0b01101101, // 115 s
   0b01111000, // 116 t
   0b00111110, // 117 u
   0b00011100, // 118 v
   0b00101010, // 119 w
   0b01110110, // 120 x
   0b01101110, // 121 y
   0b01011011, // 122 z
   0b00111001, // 123 {
   0b00110000, // 124 |
   0b00001111, // 125 }
   0b01000000, // 126 ~
   0b00000000  // 127 
};

static const uint8_t minusSegments = 0b01000000;
static const uint8_t degreeSegments = 0b01100011;

TM1637TinyDisplay::TM1637TinyDisplay(uint8_t pinClk, uint8_t pinDIO, unsigned int bitDelay, unsigned int scrollDelay)
{
	// Copy the pin numbers
	m_pinClk = pinClk;
	m_pinDIO = pinDIO;
	m_bitDelay = bitDelay;
  m_scrollDelay = scrollDelay;

	// Set the pin direction and default value.
	// Both pins are set as inputs, allowing the pull-up resistors to pull them up
  pinMode(m_pinClk, INPUT);
  pinMode(m_pinDIO,INPUT);
	digitalWrite(m_pinClk, LOW);
	digitalWrite(m_pinDIO, LOW);
}

void TM1637TinyDisplay::setBrightness(uint8_t brightness, bool on)
{
	m_brightness = (brightness & 0x07) | (on? 0x08 : 0x00);
}

void TM1637TinyDisplay::setScrolldelay(unsigned int scrollDelay)
{
	m_scrollDelay = scrollDelay;
}

void TM1637TinyDisplay::setSegments(const uint8_t segments[], uint8_t length, uint8_t pos)
{
  // Write COMM1
	start();
	writeByte(TM1637_I2C_COMM1);
	stop();

	// Write COMM2 + first digit address
	start();
	writeByte(TM1637_I2C_COMM2 + (pos & 0x03));

	// Write the data bytes
	for (uint8_t k=0; k < length; k++)
	  writeByte(segments[k]);

	stop();

	// Write COMM3 + brightness
	start();
	writeByte(TM1637_I2C_COMM3 + (m_brightness & 0x0f));
	stop();
}

void TM1637TinyDisplay::clear()
{
  uint8_t data[] = { 0, 0, 0, 0 };
	setSegments(data);
}

void TM1637TinyDisplay::showNumber(int num, bool leading_zero, uint8_t length, uint8_t pos)
{
  showNumberDec(num, 0, leading_zero, length, pos);
}

void TM1637TinyDisplay::showNumberDec(int num, uint8_t dots, bool leading_zero,
                                    uint8_t length, uint8_t pos)
{
  showNumberBaseEx(num < 0? -10 : 10, num < 0? -num : num, dots, leading_zero, length, pos);
}

void TM1637TinyDisplay::showNumberHex(uint16_t num, uint8_t dots, bool leading_zero,
                                    uint8_t length, uint8_t pos)
{
  showNumberBaseEx(16, num, dots, leading_zero, length, pos);
}

void TM1637TinyDisplay::showNumberBaseEx(int8_t base, uint16_t num, uint8_t dots, bool leading_zero,
                                    uint8_t length, uint8_t pos)
{
  bool negative = false;
	if (base < 0) {
	  base = -base;
		negative = true;
	}

  uint8_t digits[4];

	if (num == 0 && !leading_zero) {
		// Singular case - take care separately
		for(uint8_t i = 0; i < (length-1); i++) {
      digits[i] = 0;
    }
		digits[length-1] = encodeDigit(0);
	}
	else {		
		for(int i = length-1; i >= 0; --i) {
		    uint8_t digit = num % base;
			
			if (digit == 0 && num == 0 && leading_zero == false)
			    // Leading zero is blank
				digits[i] = 0;
			else
			  digits[i] = encodeDigit(digit);
				
			if (digit == 0 && num == 0 && negative) {
			  digits[i] = minusSegments;
				negative = false;
			}

			num /= base;
		}

		if(dots != 0)
		{
			showDots(dots, digits);
		}
  }
  setSegments(digits, length, pos);
}

/**
 * Display string on TM1637 4-digit display
 * - Short strings (≤4 chars ignoring dots/colons) → right aligned
 * - Long strings → scroll left with animation
 * - Supports '.' (decimal point after character) and ':' (middle colon)
 */
void TM1637TinyDisplay::showString(const char* s, uint8_t length, uint8_t pos)
{
    if (!s || !*s) {
        clear();
        return;
    }

    // Step 1: Extract visible characters + mark dots and colons
    char chars[128];              // cleaned characters (without . and :)
    uint8_t dotAfter[128] = {0};  // 1 = decimal point after this char
    uint8_t charCount = 0;
    bool pendingColon = false;

    for (const char* p = s; *p && charCount < sizeof(chars)-1; ++p)
    {
        if (*p == '.') {
            if (charCount > 0) {
                dotAfter[charCount-1] = 1;
            }
        }
        else if (*p == ':') {
            pendingColon = true;
        }
        else {
            chars[charCount] = *p;
            if (pendingColon) {
                dotAfter[charCount] |= 0x02;  // mark as needing colon (middle)
                pendingColon = false;
            }
            charCount++;
        }
    }
    chars[charCount] = '\0';

    // ───────────────────────────────────────────────
    // Case 1: Short enough → right aligned display
    // ───────────────────────────────────────────────
    if (charCount <= 4)
    {
        uint8_t display[4] = {0, 0, 0, 0};

        // Right align - fill from the right
        int startPos = 4 - charCount;

        for (int i = 0; i < charCount; i++)
        {
            uint8_t seg = encodeASCII((uint8_t)chars[i]);

            // Add decimal point if requested
            if (dotAfter[i] & 1) {
                seg |= 0x80;
            }

            display[startPos + i] = seg;
        }

        // Middle colon handling (most common case: time format xx:xx)
        bool hasColon = (strchr(s, ':') != NULL);
        if (pendingColon || hasColon) {
            // Put colon on second digit (index 1)
            if (charCount >= 3) {  // at least something before and after colon
                display[1] |= 0x80;
            }
        }

        setSegments(display, length, pos);
        return;
    }

    // ───────────────────────────────────────────────
    // Case 2: Long text → scrolling
    // ───────────────────────────────────────────────
    uint8_t window[4];

    // Scroll in (text appears from right)
    for (int step = 0; step < 4; step++)
    {
        window[0] = (step >= 1 && (step-1) < charCount) ? encodeASCII(chars[step-1]) : 0;
        window[1] = (step >= 2 && (step-2) < charCount) ? encodeASCII(chars[step-2]) : 0;
        window[2] = (step >= 3 && (step-3) < charCount) ? encodeASCII(chars[step-3]) : 0;
        window[3] = (step     < charCount) ? encodeASCII(chars[step])     : 0;

        // Add dots (we simplify: only dots in visible window)
        for (int i = 0; i < 4; i++) {
            int realIdx = step - (3 - i);
            if (realIdx >= 0 && realIdx < charCount) {
                if (dotAfter[realIdx] & 1) {
                    window[i] |= 0x80;
                }
            }
        }

        setSegments(window, length, pos);
        delay(m_scrollDelay);
    }

    // Main scrolling phase
    for (int start = 4; start <= charCount; start++)
    {
        for (int i = 0; i < 4; i++)
        {
            int idx = start - 3 + i;
            if (idx >= charCount) {
                window[i] = 0;
            } else {
                window[i] = encodeASCII(chars[idx]);
                if (dotAfter[idx] & 1) {
                    window[i] |= 0x80;
                }
            }
        }
        setSegments(window, length, pos);
        delay(m_scrollDelay);
    }

    // Scroll out
    for (int step = 0; step < 4; step++)
    {
        int base = charCount - 3 + step;

        window[0] = (base   < charCount) ? encodeASCII(chars[base])   : 0;
        window[1] = (base+1 < charCount) ? encodeASCII(chars[base+1]) : 0;
        window[2] = (base+2 < charCount) ? encodeASCII(chars[base+2]) : 0;
        window[3] = (base+3 < charCount) ? encodeASCII(chars[base+3]) : 0;

        // Dots during scroll-out
        for (int i = 0; i < 4; i++) {
            int realIdx = base + i;
            if (realIdx >= 0 && realIdx < charCount && (dotAfter[realIdx] & 1)) {
                window[i] |= 0x80;
            }
        }

        setSegments(window, length, pos);
        delay(m_scrollDelay);
    }

    clear();
}

void TM1637TinyDisplay::showLevel(unsigned int level, bool horizontal) 
{
  uint8_t digits[4] = {0,0,0,0};
  uint8_t digit = 0b00000000;

  if(level>100) level=100;

  if(horizontal) {
    // Must fit within 3 bars
    int bars = (int)((level*3)/100.0);
    if(bars == 0 && level > 0) bars = 1; // Only level=0 turns off display
    switch(bars) {
      case 1:
        digit = 0b00001000;
        break;
      case 2:
        digit = 0b01001000;
        break;
      case 3:
        digit = 0b01001001;
        break;
      default: // Keep at zero
        break;
    }
    for(int x = 0; x < 4; x++) {
      digits[x] = digit;
    }
  }
  else {
    // Must fit within 8 bars
    int bars = (int)((level*8)/100.0);
    if(bars == 0 && level > 0) bars = 1;
    for(int x = 0; x<4; x++) { // for each digit
      int left = bars-(x*2);
      if(left > 1) digits[x] = 0b00110110;
      if(left == 1) digits[x] = 0b00110000;
    }
  }
  setSegments(digits);
}

void TM1637TinyDisplay::showAnimation(const uint8_t data[][4], unsigned int frames, unsigned int ms)
{
  // Animation sequence
  for (int x = 0; x < frames; x++) {
    setSegments(data[x]);
    delay(ms);
  }
}

void TM1637TinyDisplay::bitDelay()
{
	delayMicroseconds(m_bitDelay);
}

void TM1637TinyDisplay::start()
{
  pinMode(m_pinDIO, OUTPUT);
  bitDelay();
}

void TM1637TinyDisplay::stop()
{
	pinMode(m_pinDIO, OUTPUT);
	bitDelay();
	pinMode(m_pinClk, INPUT);
	bitDelay();
	pinMode(m_pinDIO, INPUT);
	bitDelay();
}

bool TM1637TinyDisplay::writeByte(uint8_t b)
{
  uint8_t data = b;

  // 8 Data Bits
  for(uint8_t i = 0; i < 8; i++) {
    // CLK low
    pinMode(m_pinClk, OUTPUT);
    bitDelay();

	// Set data bit
    if (data & 0x01)
      pinMode(m_pinDIO, INPUT);
    else
      pinMode(m_pinDIO, OUTPUT);

    bitDelay();

	// CLK high
    pinMode(m_pinClk, INPUT);
    bitDelay();
    data = data >> 1;
  }

  // Wait for acknowledge
  // CLK to zero
  pinMode(m_pinClk, OUTPUT);
  pinMode(m_pinDIO, INPUT);
  bitDelay();
 
  // CLK to high
  pinMode(m_pinClk, INPUT);
  bitDelay();
  uint8_t ack = digitalRead(m_pinDIO);
  if (ack == 0)
    pinMode(m_pinDIO, OUTPUT);

  bitDelay();
  pinMode(m_pinClk, OUTPUT);
  bitDelay();

  return ack;
}

void TM1637TinyDisplay::showDots(uint8_t dots, uint8_t* digits)
{
  for(int i = 0; i < 4; ++i)
  {
      digits[i] |= (dots & 0x80);
      dots <<= 1;
  }
}

uint8_t TM1637TinyDisplay::encodeDigit(uint8_t digit)
{
	return digitToSegment[digit & 0x0f];
}

uint8_t TM1637TinyDisplay::encodeASCII(uint8_t chr)
{
  if(chr == 176) return degreeSegments;   // Degree mark
  if(chr > 127 || chr < 32) return 0;     // Blank
	return asciiToSegment[chr - 32];
}
