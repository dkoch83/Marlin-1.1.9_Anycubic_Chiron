/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
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

#ifndef CHIRONLCD_H
#define CHIRONLCD_H

#include "MarlinConfig.h"

#if ENABLED(CHIRON_LCD)
  void lcd_init();
  bool lcd_detected();
  void lcd_update();
  void lcd_setalertstatusPGM(const char* message);
#else
  inline void lcd_init() {}
  inline bool lcd_detected() { return true; }
  inline void lcd_update() {}
  inline void lcd_setalertstatusPGM(const char* message) { UNUSED(message); }
#endif

#define LCD_MESSAGEPGM(x)      lcd_setstatusPGM(PSTR(x))
#define LCD_ALERTMESSAGEPGM(x) lcd_setalertstatusPGM(PSTR(x))

constexpr bool lcd_wait_for_move = false;

inline void lcd_refresh() {}
inline bool lcd_hasstatus() { return false; }
inline void lcd_setstatus(const char* const message, const bool persist=false) { UNUSED(message); UNUSED(persist); }
inline void lcd_setstatusPGM(const char* const message, const int8_t level=0) { UNUSED(message); UNUSED(level); }
inline void lcd_status_printf_P(const uint8_t level, const char * const fmt, ...) { UNUSED(level); UNUSED(fmt); }
inline void lcd_reset_alert_level() {}
inline void lcd_reset_status() {}
inline void lcd_buttons_update() {}

#if ENABLED(CHIRON_LCD)
	void write_to_lcd_P(const char * const message);
	void write_to_lcd(const char * const message);
	void write_to_lcd_f(float value);
	void write_to_lcd_i(int value);

	// eeprom_index
	extern int z_values_index;
	extern int z_values_size;

	// USB Connection
	extern bool USBConnectFlag;
	extern bool UsbOnLineFlag;	
	
	//
	extern const unsigned int Max_ModelCooling;
	
#endif

#endif

