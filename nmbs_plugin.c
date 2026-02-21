/*
  nmbs_plugin.c - driver code nanomodbus server

  Part of grblHAL

  Copyright (c) 2020-2024 Terje Io

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

#include "driver.h"
#include "nanomodbus.h"

#include <stdio.h>
#include <string.h>

//#include "serial.h"
#include "grbl/state_machine.h"
#include "grbl/modbus.h"
#include "grbl/settings.h"
#include "grbl/protocol.h"
#include "sdcard/fs_stream.h"
#include "grbl/nvs_buffer.h"

#include "nmbs_plugin.h"

PROGMEM static const setting_group_detail_t modbus_groups [] = {
    { Group_Root, Group_ModBus, "ModBus"}
};

typedef struct {
    uint8_t modbus_address;
    int modbus_baud_rate; //added by empyrean 2025-11-24
    //uint16_t rx_timeout;
} nmbs_settings_t;

// A single nmbs_bitfield variable can keep 2000 coils
nmbs_bitfield server_coils = {0};
uint16_t server_registers[REGS_ADDR_MAX + 1] = {0};

nmbs_platform_conf platform_conf;
nmbs_callbacks callbacks;

nmbs_t nmbs;

nmbs_settings_t nmbs_config;
static nvs_address_t nvs_address;

static on_report_options_ptr on_report_options;

static on_execute_realtime_ptr on_execute_realtime = NULL, on_execute_delay;

static const uint32_t baud[] = { 2400, 4800, 9600, 19200, 38400, 115200 };
static const modbus_silence_timeout_t dflt_timeout =
{
    .b2400   = 16,
    .b4800   = 8,
    .b9600   = 4,
    .b19200  = 2,
    .b38400  = 2,
    .b115200 = 2
};

static const io_stream_t *stream = NULL;
static io_stream_t nanomodbus_stream;
static uint8_t dir_port = IOPORT_UNASSIGNED;

static void execute_system_reset (void){
    grbl.enqueue_realtime_command(CMD_RESET);
    nmbs_bitfield_write(server_coils, COIL_SYSTEM_RESET, 0);
}

static void execute_system_unlock (void){
    grbl.enqueue_realtime_command(CMD_STOP);
    nmbs_bitfield_write(server_coils, COIL_SYSTEM_UNLOCK, 0);
}

static bool read_aux_input(uint8_t index)
{
    if(index >= COIL_AUXIN_COUNT)
        return false;

    return ioport_wait_on_input(true, index, WaitMode_Immediate, 0.0f);
}

static bool write_aux_output(uint8_t index, bool value){
    if(index >= COIL_AUXIN_COUNT)
        return false;
    
    return ioport_digital_out(index, value);
}

static void update_rgb_output(void *data){

    bool success = false;
#ifdef NEOPIXELS_PIN
/*
    rgb_ptr_t *strip = &hal.rgb0;

    bool r = nmbs_bitfield_read(server_coils, COIL_RGB_BASE + RGB_RED);
    bool g = nmbs_bitfield_read(server_coils, COIL_RGB_BASE + RGB_GREEN);
    bool b = nmbs_bitfield_read(server_coils, COIL_RGB_BASE + RGB_BLUE);

    rgb_color_t color = {
        .R = r ? 255 : 0,
        .G = g ? 255 : 0,
        .B = b ? 255 : 0,
        .W = 255
    };

    for(uint16_t device = 0; device < strip->num_devices; device++)
        strip->out(device, color);

    if(strip->num_devices > 1 && strip->write)
        strip->write();
*/
#endif
    return;
}

static bool read_system_coil(uint8_t index)
{
    switch(index) {
        case SYS_COIL_ALARM:      return state_get() == STATE_ALARM;
        case SYS_COIL_IDLE:       return state_get() == STATE_IDLE;
        case SYS_COIL_CYCLE:      return state_get() == STATE_CYCLE;
        case SYS_COIL_HOLD:       return state_get() == STATE_HOLD;
        case SYS_COIL_JOG:        return state_get() == STATE_JOG;
        default:                  return false;
    }
}

static volatile bool macro_pending = false;
static volatile bool rgb_pending = false;
static volatile uint16_t macro_number = 0;

static const setting_detail_t nmbs_settings[] = {
     { Setting_UserDefined_0, Group_ModBus, "NanoModbus Device Address", "", Format_Int8, "###0", "1", "250", Setting_NonCore, &nmbs_config.modbus_address, NULL, NULL },
     { Setting_UserDefined_1, Group_ModBus, "ModBus baud rate", NULL, Format_RadioButtons, "2400,4800,9600,19200,38400,115200", NULL, NULL, Setting_NonCore, &nmbs_config.modbus_baud_rate, NULL, NULL },
};

static const setting_descr_t nmbs_settings_descr[] = {
    { Setting_UserDefined_0, "NanoModbus Device Address.  Hard reset required." },
    { Setting_UserDefined_1, "NanoModbus Baud Rate. Hard reset required." },
};

void onError() {
    //report_message("Nanomodbus Error", Message_Error);

    //raise alarm state?

}


nmbs_error handle_read_coils(uint16_t address,
                             uint16_t quantity,
                             nmbs_bitfield coils_out,
                             uint8_t unit_id,
                             void* arg)
{
    /* Validate address range */
    if (address + quantity > COILS_ADDR_MAX + 1)
        return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;

    for (uint16_t i = 0; i < quantity; i++) {

        uint16_t coil = address + i;
        bool value = false;

           //System Coils (Read-Only)
        if (coil >= COIL_SYS_BASE &&
            coil < (COIL_SYS_BASE + COIL_SYS_COUNT)) {

            uint8_t index = coil - COIL_SYS_BASE;
            value = read_system_coil(index);
        }

           //AUX Input Coils (Read-Only)
        else if (coil >= COIL_AUXIN_BASE &&
                 coil < (COIL_AUXIN_BASE + COIL_AUXIN_COUNT)) {

            uint8_t index = coil - COIL_AUXIN_BASE;
            value = read_aux_input(index);
        } 

           //Everything Else(Writable / Stored Coils)
        else {
            value = nmbs_bitfield_read(server_coils, coil);
        }

        /* Write result into outgoing bitfield */
        nmbs_bitfield_write(coils_out, i, value);
    }

    return NMBS_ERROR_NONE;
}

nmbs_error handle_write_multiple_coils(uint16_t address, uint16_t quantity, const nmbs_bitfield coils, uint8_t unit_id,
                                       void* arg)
{
    /* Validate address range safely */
    if (quantity == 0 ||
        address > COILS_ADDR_MAX ||
        (address + quantity - 1) > COILS_ADDR_MAX)
    {
        return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
    }

    for (uint16_t i = 0; i < quantity; i++) {

        uint16_t coil = address + i;
        bool value = nmbs_bitfield_read(coils, i);

           //System Coils (Read-Only)
        if (coil >= COIL_SYS_BASE &&
            coil < (COIL_SYS_BASE + COIL_SYS_COUNT))
        {
            return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
        }

           //AUX Input Coils (Read-Only)
        else if (coil >= COIL_AUXIN_BASE &&
                 coil < (COIL_AUXIN_BASE + COIL_AUXIN_COUNT))
        {
            return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
        }

           //AUX Output Coils (Writable)
        else if (coil >= COIL_AUXOUT_BASE &&
                 coil < (COIL_AUXOUT_BASE + COIL_AUXOUT_COUNT))
        {
            uint8_t index = coil - COIL_AUXOUT_BASE;
            write_aux_output(index, value);

            /* Mirror into bitfield for readback */
            nmbs_bitfield_write(server_coils, coil, value);
        }
           //RGB Coils (Writable)
        else if (coil >= COIL_RGB_BASE &&
                 coil < (COIL_RGB_BASE + COIL_RGB_COUNT))
        {
            uint8_t index = coil - COIL_RGB_BASE;
            nmbs_bitfield_write(server_coils, coil, value);
            rgb_pending = true;
        }
            //system unlock coil
        else if (coil == COIL_SYSTEM_UNLOCK)
        {
            execute_system_unlock();
            nmbs_bitfield_write(server_coils, coil, value);
        }        
        //system reset coil
        else if (coil == COIL_SYSTEM_RESET)
        {
            execute_system_reset();
            nmbs_bitfield_write(server_coils, coil, value);
        }
           //Generic Stored Coils
        else
        {
            nmbs_bitfield_write(server_coils, coil, value);
        }
    }

    return NMBS_ERROR_NONE;
}

nmbs_error handle_write_single_coil(uint16_t address, bool value, uint8_t unit_id, void* arg)
{
    nmbs_bitfield bf = {0};
    if (value)
        bf[0] = 0x01;  // set bit 0
    return handle_write_multiple_coils(address, 1, bf, unit_id, arg);
}


nmbs_error handle_read_holding_registers(uint16_t address, uint16_t quantity, uint16_t* registers_out, uint8_t unit_id,
                                          void* arg) {
    if (address + quantity > REGS_ADDR_MAX + 1)
        return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;

    // Read our registers values into registers_out
    for (int i = 0; i < quantity; i++){
        uint16_t reg = address + i;  
        
        if (reg == REG_STATE_WORD) { //read machine state
            registers_out[i] = state_get();
        } else{
            registers_out[i] = server_registers[address + i];
        }
    }

    return NMBS_ERROR_NONE;
}

/*
nmbs_error handle_write_multiple_registers(uint16_t address, uint16_t quantity, const uint16_t* registers,
                                           uint8_t unit_id, void* arg) {
    if (address + quantity > REGS_ADDR_MAX + 1)
        return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;

for (int i = 0; i < quantity; i++) {
    uint16_t reg = address + i;
    server_registers[reg] = registers[i];

    if (reg == MACRO_TRIGGER_REGISTER) {
        macro_number = registers[i];
        macro_pending = true;
    }
}

    return NMBS_ERROR_NONE;
}
    */

nmbs_error handle_write_multiple_registers(uint16_t address, uint16_t quantity, const uint16_t* registers, uint8_t unit_id, void* arg)
{
    /* Validate address range safely */
    if (quantity == 0 ||
        address > REGS_ADDR_MAX ||
        (address + quantity - 1) > REGS_ADDR_MAX)
    {
        return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
    }

    for (uint16_t i = 0; i < quantity; i++) {

        uint16_t reg = address + i;
        uint16_t value = registers[i];

        /* -----------------------------
           Macro Trigger Register
           ----------------------------- */
        if (reg == REG_MACRO_TRIGGER) {

            macro_number = value;
            macro_pending = true;

            /* Optional: mirror into register table */
            server_registers[reg] = value;
        }

        /* -----------------------------
           Read-Only Registers
           ----------------------------- */
        else if (reg == REG_STATE_WORD) {

            /* Status word is read-only */
            return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
        }

        /* -----------------------------
           Generic Writable Registers
           ----------------------------- */
        else {

            server_registers[reg] = value;
        }
    }

    return NMBS_ERROR_NONE;
}    

nmbs_error handle_write_single_register(uint16_t address, uint16_t value, uint8_t unit_id, void* arg) {

    return handle_write_multiple_registers(address, 1, &value, unit_id, arg);
}

size_t stream_read_bytes(uint8_t *buf, size_t count, uint32_t timeout_ms)
{
    if (!stream || !nanomodbus_stream.read || !buf || count == 0)
        return 0;

    size_t read_count = 0;
    uint32_t start = hal.get_elapsed_ticks();

    while (read_count < count) {

        int16_t c = nanomodbus_stream.read();

        // Stop immediately if no byte is available
        if (c < 0)
            break;

        buf[read_count++] = (uint8_t)c;

        // Optional timeout guard (usually not needed for non-blocking mode)
        if (timeout_ms && (hal.get_elapsed_ticks() - start) >= timeout_ms)
            break;
    }

    return read_count;
}

int32_t read_serial(uint8_t* buf, uint16_t count, int32_t byte_timeout_ms, void* arg) {

    return stream_read_bytes(buf, count, byte_timeout_ms);
}


int32_t write_serial(const uint8_t* buf, uint16_t count, int32_t byte_timeout_ms, void* arg) {
    ioport_digital_out(dir_port, 1);
    hal.delay_ms(1,NULL);
    nanomodbus_stream.write_n(buf, count);
    while(nanomodbus_stream.get_tx_buffer_count());
    //hal.delay_ms(10,NULL);
    ioport_digital_out(dir_port, 0);

    return count;
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt){
    	report_plugin("NanoModbus", "0.01");
    }

}

static void check_macro_execute(uint_fast16_t state)
{
    if (!macro_pending)
        return;

    macro_pending = false;

    char fname[32];
    snprintf(fname, sizeof(fname), "%d.gcode", macro_number);

    report_message("Executing Macro", Message_Plain);
    report_message(fname, Message_Plain);

    stream_file(state, fname);
}

static void check_update_rgb(uint_fast16_t state)
{
    if (!rgb_pending)
        return;

    rgb_pending = false;

    report_message("Update RGB lights", Message_Plain);
    
    task_add_immediate(update_rgb_output, NULL);
}

static void onExecuteRealtime (uint_fast16_t state)
{

    nmbs_server_poll(&nmbs);
    check_macro_execute(state);
    on_execute_realtime(state);
}

static void onExecuteDelay (uint_fast16_t state)
{

    nmbs_server_poll(&nmbs);
    check_macro_execute(state);
    on_execute_delay(state);
}

static void nmbs_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&nmbs_config, sizeof(nmbs_settings_t), true);
}

static void nmbs_settings_restore (void)
{
    nmbs_config.modbus_address = 1;
    nmbs_config.modbus_baud_rate = 5;
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&nmbs_config, sizeof(nmbs_settings_t), true);
}

static void nmbs_settings_load (void)
{
    if((hal.nvs.memcpy_from_nvs((uint8_t *)&nmbs_config, nvs_address, sizeof(nmbs_settings_t), true) != NVS_TransferResult_OK))
        nmbs_settings_restore();

    //nmbs_settings_load();

    nmbs_platform_conf_create(&platform_conf);
    platform_conf.transport = NMBS_TRANSPORT_RTU;
    platform_conf.read = read_serial;
    platform_conf.write = write_serial;
    platform_conf.arg = NULL;

    nmbs_callbacks_create(&callbacks);
    callbacks.read_coils = handle_read_coils;
    callbacks.write_single_coil = handle_write_single_coil;
    callbacks.write_multiple_coils = handle_write_multiple_coils;
    callbacks.read_holding_registers = handle_read_holding_registers;
    callbacks.write_single_register = handle_write_single_register;
    callbacks.write_multiple_registers = handle_write_multiple_registers;

    uint32_t actual_baud = baud[nmbs_config.modbus_baud_rate];
  
    nmbs_server_create(&nmbs, nmbs_config.modbus_address, &platform_conf, &callbacks);

    nmbs_set_read_timeout(&nmbs, 1000);
    nmbs_set_byte_timeout(&nmbs, 100);
  
    bool ok;

    stream = stream_open_instance(NANOMODBUS_STREAM, actual_baud, NULL, "nanoModbus");

    if(stream)
        report_message("NanoModbus stream opened", Message_Plain);
    else
        report_message("NanoModbus stream FAILED", Message_Warning);    
        
    if((ok = stream != NULL)) {
        memcpy(&nanomodbus_stream, stream, sizeof(io_stream_t));

        stream = stream_null_init(actual_baud);

        nanomodbus_stream.set_enqueue_rt_handler(stream_buffer_all);
    }        
}

void my_plugin_init (void)
{
    xbar_t *dir_pin; // TODO: move to top and use for direct access
    io_port_cfg_t d_out;

    ioports_cfg(&d_out, Port_Digital, Port_Output);

    int8_t dir_aux = MODBUS_DIR_AUX;

    dir_port = dir_aux != -1 ? dir_aux : (d_out.n_ports ? d_out.n_ports - 1 : IOPORT_UNASSIGNED);

    if((dir_pin = d_out.claim(&d_out, &dir_port, NULL, (pin_cap_t){}))) {
        ioport_set_function(dir_pin, Output_RS485_Direction, NULL);
        ioport_digital_out(dir_port, 0);
    }

    static setting_details_t nmbs_setting_details = {
        .groups = modbus_groups,
        .n_groups = sizeof(modbus_groups) / sizeof(setting_group_detail_t),        
        .settings = nmbs_settings,
        .n_settings = sizeof(nmbs_settings) / sizeof(setting_detail_t),
    #ifndef NO_SETTINGS_DESCRIPTIONS
        .descriptions = nmbs_settings_descr,
        .n_descriptions = sizeof(nmbs_settings_descr) / sizeof(setting_descr_t),
    #endif
        .load = nmbs_settings_load,
        .restore = nmbs_settings_restore,
        .save = nmbs_settings_save
    };

    if((nvs_address = nvs_alloc(sizeof(nmbs_settings_t)))) 
        settings_register(&nmbs_setting_details);       

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;
    
    on_execute_realtime = grbl.on_execute_realtime;
    grbl.on_execute_realtime = onExecuteRealtime;

    on_execute_delay = grbl.on_execute_delay;
    grbl.on_execute_delay = onExecuteDelay;

}


