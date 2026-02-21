/*

  eeprom.h - plugin for I2C EEPROM or FRAM

  Part of grblHAL

  Copyright (c) 2017-2025 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.

*/

// The data model of this sever will support coils addresses 0 to 100 and registers addresses from 0 to 32
// Our RTU address
//#define RTU_SERVER_ADDRESS 1


#ifndef _NMBS_PLUGIN_H_
#define _NMBS_PLUGIN_H_

#include <stdint.h>
#include <stdbool.h>

/* ============================================================
   MODBUS COIL MEMORY MAP
   ============================================================

   0–9     : System status coils (read-only)
   10–19   : AUX inputs (read-only)
   20–29   : AUX outputs (write)
   30–33   : RGB outputs (write)
   
   Special coils:
   99 - execute a system reset

   ============================================================ */


/* -----------------------------
   System Coils
   ----------------------------- */

#define COIL_SYS_BASE          0
#define COIL_SYS_COUNT         8

typedef enum {
    SYS_COIL_ALARM = 0,
    SYS_COIL_IDLE,
    SYS_COIL_CYCLE,
    SYS_COIL_HOLD,
    SYS_COIL_JOG,
    SYS_COIL_HOMING,
    SYS_COIL_DOOR,
    SYS_COIL_CHECK_MODE
} sys_coil_index_t;


/* -----------------------------
   AUX Input Coils
   ----------------------------- */

#define COIL_AUXIN_BASE        10
#define COIL_AUXIN_COUNT       9

typedef enum {
    AUXIN0 = 0,
    AUXIN1,
    AUXIN2,
    AUXIN3,
    AUXIN4,
    AUXIN5,
    AUXIN6,
    AUXIN7,
    AUXIN8
} aux_input_index_t;


/* -----------------------------
   AUX Output Coils
   ----------------------------- */

#define COIL_AUXOUT_BASE       20
#define COIL_AUXOUT_COUNT      8

typedef enum {
    AUXOUT0 = 0,
    AUXOUT1,
    AUXOUT2,
    AUXOUT3,
    AUXOUT4,
    AUXOUT5,
    AUXOUT6,
    AUXOUT7
} aux_output_index_t;


/* -----------------------------
   RGB Coils
   ----------------------------- */

#define COIL_RGB_BASE          30
#define COIL_RGB_COUNT         3

typedef enum {
    RGB_RED = 0,
    RGB_GREEN,
    RGB_BLUE
} rgb_coil_index_t;

/* -----------------------------
   Coolant Coils
   ----------------------------- */

#define COIL_COOLANT_BASE          40
#define COIL_COOLANT_COUNT         2

typedef enum {
    FLOOD = 0,
    MIST,
} coolant_coil_index_t;

#define COIL_SYSTEM_UNLOCK 98
#define COIL_SYSTEM_RESET 99

#define COILS_ADDR_MAX 100

/* ============================================================
   Register Layout (optional expansion)
   ============================================================ */

#define REG_MACRO_TRIGGER      1
#define REG_STATE_WORD         10
#define REG_FEED_RATE          11 //currently unhandled
#define REG_COOLANT_RATE       12 //currently unhandleed

#define REGS_ADDR_MAX 12

/* ============================================================
   Function Prototypes
   ============================================================ */

/* Read-only domains */
static bool read_system_coil(uint8_t index);
static bool read_aux_input(uint8_t index);

/* writable domains */
static bool write_aux_output(uint8_t index, bool value);
static bool write_coolant_output(uint8_t index, bool value); //currently unimplemented

static void execute_system_reset(void);
static void execute_system_unlock (void);


#endif