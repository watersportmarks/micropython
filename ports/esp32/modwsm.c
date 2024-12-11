/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2017 "Eric Poulsen" <eric@zyxod.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <stdio.h>
#include <string.h>

#include <time.h>
#include <sys/time.h>
#include "soc/rtc_cntl_reg.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_heap_caps.h"

#include "py/nlr.h"
#include "py/obj.h"
#include "py/runtime.h"
#include "py/mphal.h"
#include "shared/timeutils/timeutils.h"
#include "modmachine.h"
#include "modwsm.h"
#include "../../../../../main/esp_http_client_example.h"

char api_version[6] = "XX.XX\0";
char temp_buff[512];

/// \method wsm_bt_updated()
/// Return true if BT connected and an update from phone is received.
mp_obj_t wsm_bt_updated(void) {
    return mp_obj_new_bool(bt_updated());
}
static MP_DEFINE_CONST_FUN_OBJ_0(wsm_bt_updated_obj, wsm_bt_updated);

/// \method wsm_get_bt_update()
/// Get values updated from BT.
mp_obj_t wsm_get_bt_update(void) {
    mp_obj_list_t *data = MP_OBJ_TO_PTR(mp_obj_new_list(14, NULL));
    data->items[0] = mp_obj_new_float(get_desFW());
    data->items[1] = mp_obj_new_int(get_yawStart());
    data->items[2] = mp_obj_new_int(get_controlType());
    data->items[3] = mp_obj_new_float(get_start_lat());
    data->items[4] = mp_obj_new_float(get_start_lon());
    data->items[5] = mp_obj_new_bool(get_goalChanged());
    data->items[6] = mp_obj_new_float(get_xx());
    data->items[7] = mp_obj_new_float(get_yy());
    data->items[8] = mp_obj_new_float(get_delta_lat());
    data->items[9] = mp_obj_new_float(get_delta_lon());
    data->items[10] = mp_obj_new_bool(get_freshGPS());
    data->items[11] = mp_obj_new_float(get_x_des());
    data->items[12] = mp_obj_new_float(get_y_des());
    data->items[13] = mp_obj_new_float(get_k_headingDrift());
    return MP_OBJ_FROM_PTR(data);
}
static MP_DEFINE_CONST_FUN_OBJ_0(wsm_get_bt_update_obj, wsm_get_bt_update);

// Get current WSM API version.
static mp_obj_t get_api_version(void) {
    snprintf(api_version, 6, "%2d.%-2d", WSM_API_MAJOR_VERSION, WSM_API_MINOR_VERSION);
    return mp_obj_new_str(api_version, strlen(api_version));
}
static MP_DEFINE_CONST_FUN_OBJ_0(get_api_version_obj, get_api_version);

/// \method wsm_set_heading()
mp_obj_t wsm_set_heading(mp_obj_t value) {
    int val = mp_obj_get_int(value);
    set_heading(val);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(wsm_set_heading_obj, wsm_set_heading);

/// \method wsm_set_delta_lat()
mp_obj_t wsm_set_delta_lat(mp_obj_t value) {
    double val = mp_obj_get_float(value);
    set_delta_lat(val);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(wsm_set_delta_lat_obj, wsm_set_delta_lat);

/// \method wsm_set_delta_lon()
mp_obj_t wsm_set_delta_lon(mp_obj_t value) {
    double val = mp_obj_get_float(value);
    set_delta_lon(val);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(wsm_set_delta_lon_obj, wsm_set_delta_lon);

/// \method wsm_set_gps_precision()
mp_obj_t wsm_set_gps_precision(mp_obj_t value) {
    float val = mp_obj_get_float(value);
    set_GPSprecision(val);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(wsm_set_gps_precision_obj, wsm_set_gps_precision);

/// \method wsm_set_gps_heading()
mp_obj_t wsm_set_gps_heading(mp_obj_t value) {
    float val = mp_obj_get_float(value);
    set_GPSheading(val);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(wsm_set_gps_heading_obj, wsm_set_gps_heading);

/// \method wsm_set_gps_delta_dist()
mp_obj_t wsm_set_gps_delta_dist(mp_obj_t value) {
    float val = mp_obj_get_float(value);
    set_GPSdeltaDist(val);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(wsm_set_gps_delta_dist_obj, wsm_set_gps_delta_dist);

/// \method wsm_set_control_type()
mp_obj_t wsm_set_control_type(mp_obj_t value) {
    int val = mp_obj_get_int(value);
    set_controlType(val);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(wsm_set_control_type_obj, wsm_set_control_type);

/// \method wsm_set_mag_cal()
mp_obj_t wsm_set_mag_cal(mp_obj_t value) {
    int val = mp_obj_get_int(value);
    set_magCal(val);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(wsm_set_mag_cal_obj, wsm_set_mag_cal);

/// \method wsm_set_mr_duty()
mp_obj_t wsm_set_mr_duty(mp_obj_t value) {
    int val = mp_obj_get_int(value);
    set_mR_duty(val);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(wsm_set_mr_duty_obj, wsm_set_mr_duty);

/// \method wsm_set_ml_duty()
mp_obj_t wsm_set_ml_duty(mp_obj_t value) {
    int val = mp_obj_get_int(value);
    set_mL_duty(val);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(wsm_set_ml_duty_obj, wsm_set_ml_duty);

/// \method wsm_set_volt()
mp_obj_t wsm_set_volt(mp_obj_t value) {
    float val = mp_obj_get_float(value);
    set_volt(val);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(wsm_set_volt_obj, wsm_set_volt);

/// \method wsm_set_amp()
mp_obj_t wsm_set_amp(mp_obj_t value) {
    float val = mp_obj_get_float(value);
    set_amp(val);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(wsm_set_amp_obj, wsm_set_amp);

/// \method wsm_set_mah()
mp_obj_t wsm_set_mah(mp_obj_t value) {
    float val = mp_obj_get_float(value);
    set_mAh(val);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(wsm_set_mah_obj, wsm_set_mah);

/// \method wsm_set_ang_wind()
mp_obj_t wsm_set_ang_wind(mp_obj_t value) {
    int val = mp_obj_get_int(value);
    set_angWind(val);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(wsm_set_ang_wind_obj, wsm_set_ang_wind);

/// \method wsm_set_vwind()
mp_obj_t wsm_set_vwind(mp_obj_t value) {
    int val = mp_obj_get_int(value);
    set_vWind(val);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(wsm_set_vwind_obj, wsm_set_vwind);

/// \method wsm_set_lat()
mp_obj_t wsm_set_lat(mp_obj_t value) {
    float val = mp_obj_get_float(value);
    set_lat(val);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(wsm_set_lat_obj, wsm_set_lat);

/// \method wsm_set_lon()
mp_obj_t wsm_set_lon(mp_obj_t value) {
    float val = mp_obj_get_float(value);
    set_lon(val);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(wsm_set_lon_obj, wsm_set_lon);

/// \method wsm_print_log()
mp_obj_t wsm_print_log(mp_obj_t value) {
    //printf("print log: %s (%d)\r\n", mp_obj_str_get_str(value), strlen(mp_obj_str_get_str(value)));
    //memset(temp_buff, 0x00, 512);
    //memcpy(temp_buff, mp_obj_str_get_str(value), strlen(mp_obj_str_get_str(value)));
    //printf("print log2: %s (%d)\r\n", temp_buff, strlen(temp_buff));
    //const char* val = mp_obj_str_get_str(value);
    print_log(mp_obj_str_get_str(value));
    //uint16_t fsize = strlen(mp_obj_str_get_str(value));
    //char *fdata = mp_obj_str_get_data(value, (size_t*)&fsize);
    //my_print_log(fdata);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(wsm_print_log_obj, wsm_print_log);

/// \method wsm_print_log()
mp_obj_t wsm_set_wifi_credentials(mp_obj_t ssid, mp_obj_t pw, mp_obj_t index) {
    set_wifi_credentials(mp_obj_str_get_str(ssid), mp_obj_str_get_str(pw), mp_obj_get_int(index));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_3(wsm_set_wifi_credentials_obj, wsm_set_wifi_credentials);

/// \method wsm_get_log()
static mp_obj_t wsm_get_log(void) {
    byte *buf;

    int32_t fsize = get_log_size();
    printf("read fsize = %ld\n", fsize);

    buf = m_new(byte, fsize);
    //printf("buffer allocated\n");
    get_log(buf, fsize);

    //return mp_obj_new_bytes(buf, fsize);  // Using this function a new buffer is allocated with same size as "buf", with large files
                                            // it leads to memory exhaustion.
    return mp_obj_new_bytearray_by_ref(fsize, buf); // This function could be used instead...  

    //return MP_OBJ_FROM_PTR(o);   // Second version...
}
static MP_DEFINE_CONST_FUN_OBJ_0(wsm_get_log_obj, wsm_get_log);


static const mp_rom_map_elem_t wsm_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_wsm) },

    { MP_ROM_QSTR(MP_QSTR_bt_updated), MP_ROM_PTR(&wsm_bt_updated_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_bt_update), MP_ROM_PTR(&wsm_get_bt_update_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_api_version), MP_ROM_PTR(&get_api_version_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_heading), MP_ROM_PTR(&wsm_set_heading_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_delta_lat), MP_ROM_PTR(&wsm_set_delta_lat_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_delta_lon), MP_ROM_PTR(&wsm_set_delta_lon_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_gps_precision), MP_ROM_PTR(&wsm_set_gps_precision_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_gps_heading), MP_ROM_PTR(&wsm_set_gps_heading_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_gps_delta_dist), MP_ROM_PTR(&wsm_set_gps_delta_dist_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_control_type), MP_ROM_PTR(&wsm_set_control_type_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_mag_cal), MP_ROM_PTR(&wsm_set_mag_cal_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_mr_duty), MP_ROM_PTR(&wsm_set_mr_duty_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_ml_duty), MP_ROM_PTR(&wsm_set_ml_duty_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_volt), MP_ROM_PTR(&wsm_set_volt_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_amp), MP_ROM_PTR(&wsm_set_amp_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_mah), MP_ROM_PTR(&wsm_set_mah_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_ang_wind), MP_ROM_PTR(&wsm_set_ang_wind_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_vwind), MP_ROM_PTR(&wsm_set_vwind_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_lat), MP_ROM_PTR(&wsm_set_lat_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_lon), MP_ROM_PTR(&wsm_set_lon_obj) },
    { MP_ROM_QSTR(MP_QSTR_print_log), MP_ROM_PTR(&wsm_print_log_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_wifi_credentials), MP_ROM_PTR(&wsm_set_wifi_credentials_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_log), MP_ROM_PTR(&wsm_get_log_obj) },
};

static MP_DEFINE_CONST_DICT(wsm_module_globals, wsm_module_globals_table);

const mp_obj_module_t wsm_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&wsm_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_wsm, wsm_module);
