/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#pragma once

/**
 * Custom Status Screen bitmap
 *
 * Place this file in the root with your configuration files
 * and enable CUSTOM_STATUS_SCREEN_IMAGE in Configuration.h.
 *
 * Use the Marlin Bitmap Converter to make your own:
 * http://marlinfw.org/tools/u8glib/converter.html
 */

//
// Status Screen Logo bitmap
//



#ifdef MIX_STATUS_SCREEN_IMAGE
#define	STATUS_LOGO_X			5
#define STATUS_LOGO_Y           8 
#define STATUS_LOGO_WIDTH       40
#define	STATUS_LOGO_HEIGHT		13
const unsigned char status_logo_bmp[] PROGMEM = {
	B00110011,B00000000,B11001100,B00000000,B11001100,
	B00010010,B00000000,B01001000,B00000000,B01001000,
	B00010010,B00000000,B01001000,B00000000,B01001000,
	B00010010,B00000000,B01001000,B00000000,B01001000,
	B00010011,B11111111,B11001111,B11111111,B11001000,
	B00010000,B00000000,B00000000,B00000000,B00001000,
	B00011111,B11111111,B10000011,B11111111,B11111000,
	B00000000,B00001111,B11000111,B11100000,B00000000,
	B00000000,B00001111,B11101111,B11100000,B00000000,
	B00000000,B00000011,B11101111,B10000000,B00000000,
	B00000000,B00000001,B11101111,B00000000,B00000000,
	B00000000,B00000000,B01101100,B00000000,B00000000,
	B00000000,B00000000,B00000000,B00000000,B00000000
};
#elif 0//Z5X
#define STATUS_LOGO_Y           5 
#define STATUS_LOGO_WIDTH       40
const unsigned char status_logo_bmp[] PROGMEM = {
  B00000000,B00000000,B00000000,B00000000,B00000000,
  B00000111,B01110010,B01110111,B01110010,B01110000,
  B00000001,B01010101,B01000100,B00100101,B01010000,
  B00000010,B01010101,B01100111,B00100111,B01010000,
  B00000100,B01010101,B01000001,B00100101,B01100000,
  B00000111,B01110101,B01110111,B00100101,B01010000,
  B00000000,B00000000,B00000000,B00000000,B00000000,
  B00000000,B00000000,B00000000,B00000000,B00000000,
  B00000000,B00000000,B00000000,B00000000,B00000000,
  B00000001,B11111101,B11111100,B10000000,B10000000,
  B00000000,B00000101,B00000000,B01000001,B00000000,
  B00000100,B00001001,B00000000,B00100010,B00000010,
  B00001000,B00010001,B11110000,B00010100,B00000100,
  B00010000,B00100000,B00000100,B00001000,B00001000,
  B00100000,B01000000,B00000100,B00010100,B00010000,
  B01000000,B10000000,B00000100,B00100010,B00100000,
  B00000001,B00000001,B00001000,B01000001,B00000000,
  B00000001,B11111100,B11110000,B10000000,B10000000,
  B00000000,B00000000,B00000000,B00000000,B00000000,
  B00000000,B00000000,B00000000,B00000000,B00000000
};
#else //Z6
#define STATUS_LOGO_Y           5 
#define STATUS_LOGO_WIDTH       40
const unsigned char status_logo_bmp[] PROGMEM = {
  B00000000,B00000000,B00000000,B00000000,B00000000,
  B00000111,B01110010,B01110111,B01110010,B01110000,
  B00000001,B01010101,B01000100,B00100101,B01010000,
  B00000010,B01010101,B01100111,B00100111,B01010000,
  B00000100,B01010101,B01000001,B00100101,B01100000,
  B00000111,B01110101,B01110111,B00100101,B01010000,
  B00000000,B00000000,B00000000,B00000000,B00000000,
  B00000000,B00000000,B00000000,B00000000,B00000000,
  B00000000,B00000000,B00000000,B00000000,B00000000,
  B00000000,B00000111,B11110011,B11110000,B00000000,
  B00000000,B00000000,B00010100,B00010000,B00000000,
  B00000000,B01000000,B00100100,B00000000,B00010000,
  B00000000,B10000000,B01000100,B00000000,B00100000,
  B00000001,B00000000,B10000111,B11100000,B01000000,
  B00000010,B00000001,B00000100,B00010000,B10000000,
  B00000100,B00000010,B00000100,B00010001,B00000000,
  B00000000,B00000100,B00000100,B00010000,B00000000,
  B00000000,B00000111,B11110011,B11100000,B00000000,
  B00000000,B00000000,B00000000,B00000000,B00000000,
  B00000000,B00000000,B00000000,B00000000,B00000000
};
#endif

//
// Use default bitmaps
//
#define STATUS_HOTEND_ANIM
#define STATUS_BED_ANIM
#define STATUS_HEATERS_XSPACE   20

#ifdef MIX_STATUS_SCREEN_IMAGE
	#if HOTENDS < 2
	  #define STATUS_HEATERS_X      52
	  #define STATUS_BED_X          74
	#else
	  #define STATUS_HEATERS_X      40
	  #define STATUS_BED_X          80
	#endif
#else
	#if HOTENDS < 2
	  #define STATUS_HEATERS_X      48
	  #define STATUS_BED_X          72
	#else
	  #define STATUS_HEATERS_X      40
	  #define STATUS_BED_X          80
	#endif
#endif
