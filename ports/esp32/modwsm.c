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
#include <math.h>
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
#include "../../../../../main/main.h"
#include "../../../../../main/bluetooth.h"
#include "../../../../../main/logging.h"
#include "../../../../../main/alerts.h"

#define degrees(rad) ((rad) * (180.0 / M_PI))

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

/// \method wsm_set_heading_filt()
mp_obj_t wsm_set_heading_filt(mp_obj_t value) {
    int val = mp_obj_get_int(value);
    set_headingFilt(val);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(wsm_set_heading_filt_obj, wsm_set_heading_filt);

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

/// \method wsm_set_gps_fix_quality()
mp_obj_t wsm_set_gps_fix_quality(mp_obj_t value) {
    float val = mp_obj_get_float(value);
    set_GPSFixQuality(val);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(wsm_set_gps_fix_quality_obj, wsm_set_gps_fix_quality);

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

/// \method wsm_set_update_goal_on_db()
mp_obj_t wsm_set_update_goal_on_db(mp_obj_t value) {
    bool val = mp_obj_get_int(value);
    set_updateGoalOnDB(val);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(wsm_set_update_goal_on_db_obj, wsm_set_update_goal_on_db);

/// \method wsm_set_start_lat()
mp_obj_t wsm_set_start_lat(mp_obj_t value) {
    double val = mp_obj_get_float(value);
    set_start_lat(val);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(wsm_set_start_lat_obj, wsm_set_start_lat);

/// \method wsm_set_start_lon()
mp_obj_t wsm_set_start_lon(mp_obj_t value) {
    double val = mp_obj_get_float(value);
    set_start_lon(val);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(wsm_set_start_lon_obj, wsm_set_start_lon);

/// \method wsm_print_log()
mp_obj_t wsm_print_log(mp_obj_t value) {
    //printf("print log: %s (%d)\r\n", mp_obj_str_get_str(value), strlen(mp_obj_str_get_str(value)));
    //memset(temp_buff, 0x00, 512);
    //memcpy(temp_buff, mp_obj_str_get_str(value), strlen(mp_obj_str_get_str(value)));
    //printf("print log2: %s (%d)\r\n", temp_buff, strlen(temp_buff));
    //const char* val = mp_obj_str_get_str(value);
    print_log((char*)mp_obj_str_get_str(value));
    //uint16_t fsize = strlen(mp_obj_str_get_str(value));
    //char *fdata = mp_obj_str_get_data(value, (size_t*)&fsize);
    //my_print_log(fdata);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(wsm_print_log_obj, wsm_print_log);

/// \method wsm_set_wifi_credentials()
mp_obj_t wsm_set_wifi_credentials(mp_obj_t ssid, mp_obj_t pw, mp_obj_t index) {
    set_wifi_credentials((char*)mp_obj_str_get_str(ssid), (char*)mp_obj_str_get_str(pw), mp_obj_get_int(index));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_3(wsm_set_wifi_credentials_obj, wsm_set_wifi_credentials);

/// \method wsm_get_control_type()
mp_obj_t wsm_get_control_type(void) {
    return mp_obj_new_int(get_controlType());
}
static MP_DEFINE_CONST_FUN_OBJ_0(wsm_get_control_type_obj, wsm_get_control_type);

/// \method wsm_get_delta_lat()
mp_obj_t wsm_get_delta_lat(void) {
    return mp_obj_new_float(get_delta_lat());
}
static MP_DEFINE_CONST_FUN_OBJ_0(wsm_get_delta_lat_obj, wsm_get_delta_lat);

/// \method wsm_get_delta_lat_sim7600()
mp_obj_t wsm_get_delta_lat_sim7600(void) {
    return mp_obj_new_float(get_delta_lat_sim7600());
}
static MP_DEFINE_CONST_FUN_OBJ_0(wsm_get_delta_lat_sim7600_obj, wsm_get_delta_lat_sim7600);

/// \method wsm_get_delta_lon()
mp_obj_t wsm_get_delta_lon(void) {
    return mp_obj_new_float(get_delta_lon());
}
static MP_DEFINE_CONST_FUN_OBJ_0(wsm_get_delta_lon_obj, wsm_get_delta_lon);

/// \method wsm_get_delta_lon_sim7600()
mp_obj_t wsm_get_delta_lon_sim7600(void) {
    return mp_obj_new_float(get_delta_lon_sim7600());
}
static MP_DEFINE_CONST_FUN_OBJ_0(wsm_get_delta_lon_sim7600_obj, wsm_get_delta_lon_sim7600);

/// \method wsm_get_start_lat()
mp_obj_t wsm_get_start_lat(void) {
    return mp_obj_new_float(get_start_lat());
}
static MP_DEFINE_CONST_FUN_OBJ_0(wsm_get_start_lat_obj, wsm_get_start_lat);

/// \method wsm_get_start_lon()
mp_obj_t wsm_get_start_lon(void) {
    return mp_obj_new_float(get_start_lon());
}
static MP_DEFINE_CONST_FUN_OBJ_0(wsm_get_start_lon_obj, wsm_get_start_lon);

/// \method wsm_set_gpstime()
mp_obj_t wsm_set_gpstime(mp_obj_t value) {
    set_GPStime((char*)mp_obj_str_get_str(value));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(wsm_set_gpstime_obj, wsm_set_gpstime);

/// \method wsm_get_speed_limit()
mp_obj_t wsm_get_speed_limit(void) {
    return mp_obj_new_float(get_SPEEDlimit());
}
static MP_DEFINE_CONST_FUN_OBJ_0(wsm_get_speed_limit_obj, wsm_get_speed_limit);

/// \method wsm_set_speed_limit()
mp_obj_t wsm_set_speed_limit(mp_obj_t value) {
    float val = mp_obj_get_float(value);
    set_SPEEDlimit(val);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(wsm_set_speed_limit_obj, wsm_set_speed_limit);

/// \method wsm_set_alert()
mp_obj_t wsm_set_alert(mp_obj_t value) {
    int val = mp_obj_get_int(value);
    set_AlertToSend(val);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(wsm_set_alert_obj, wsm_set_alert);

/// \method wsm_get_control_type()
mp_obj_t wsm_get_mark_id(void) {
    return mp_obj_new_int_from_ull(get_boaID());  // Safe for 64-bit
}
static MP_DEFINE_CONST_FUN_OBJ_0(wsm_get_mark_id_obj, wsm_get_mark_id);

/// \method wsm_set_force_forward()
mp_obj_t wsm_set_force_forward(mp_obj_t value) {
    bool val = mp_obj_get_int(value);
    set_forceForward(val);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(wsm_set_force_forward_obj, wsm_set_force_forward);

/// \method wsm_get_fresh_gps()
mp_obj_t wsm_get_fresh_gps(void) {
    return mp_obj_new_bool(get_freshGPS());
}
static MP_DEFINE_CONST_FUN_OBJ_0(wsm_get_fresh_gps_obj, wsm_get_fresh_gps);

/// \method wsm_get_fresh_gps_sim7600()
mp_obj_t wsm_get_fresh_gps_sim7600(void) {
    return mp_obj_new_bool(get_freshGPS_sim7600());
}
static MP_DEFINE_CONST_FUN_OBJ_0(wsm_get_fresh_gps_sim7600_obj, wsm_get_fresh_gps_sim7600);

/// \method wsm_set_freshGPS()
mp_obj_t wsm_set_fresh_gps(mp_obj_t value) {
    bool val = mp_obj_is_true(value);
    set_freshGPS(val);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(wsm_set_fresh_gps_obj, wsm_set_fresh_gps);

/// \method wsm_set_freshGPS_sim7600()
mp_obj_t wsm_set_fresh_gps_sim7600(mp_obj_t value) {
    bool val = mp_obj_is_true(value);
    set_freshGPS_sim7600(val);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(wsm_set_fresh_gps_sim7600_obj, wsm_set_fresh_gps_sim7600);

/// \method wsm_get_gps_delta_dist()
mp_obj_t wsm_get_gps_delta_dist(void) {
    return mp_obj_new_float(get_GPSdeltaDist());
}
static MP_DEFINE_CONST_FUN_OBJ_0(wsm_get_gps_delta_dist_obj, wsm_get_gps_delta_dist);

/// \method wsm_get_gps_delta_dist_sim7600()
mp_obj_t wsm_get_gps_delta_dist_sim7600(void) {
    return mp_obj_new_float(get_GPSdeltaDist_sim7600());
}
static MP_DEFINE_CONST_FUN_OBJ_0(wsm_get_gps_delta_dist_sim7600_obj, wsm_get_gps_delta_dist_sim7600);

/// \method wsm_get_gps_heading()
mp_obj_t wsm_get_gps_heading(void) {
    return mp_obj_new_float(get_GPSheading());
}
static MP_DEFINE_CONST_FUN_OBJ_0(wsm_get_gps_heading_obj, wsm_get_gps_heading);

/// \method wsm_get_gps_heading_sim7600()
mp_obj_t wsm_get_gps_heading_sim7600(void) {
    return mp_obj_new_float(get_GPSheading_sim7600());
}
static MP_DEFINE_CONST_FUN_OBJ_0(wsm_get_gps_heading_sim7600_obj, wsm_get_gps_heading_sim7600);

/// \method wsm_get_gps_precision()
mp_obj_t wsm_get_gps_precision(void) {
    return mp_obj_new_float(get_GPSprecision());
}
static MP_DEFINE_CONST_FUN_OBJ_0(wsm_get_gps_precision_obj, wsm_get_gps_precision);

/// \method wsm_get_gps_precision_sim7600()
mp_obj_t wsm_get_gps_precision_sim7600(void) {
    return mp_obj_new_float(get_GPSprecision_sim7600());
}
static MP_DEFINE_CONST_FUN_OBJ_0(wsm_get_gps_precision_sim7600_obj, wsm_get_gps_precision_sim7600);

/// \method wsm_get_lat()
mp_obj_t wsm_get_lat(void) {
    return mp_obj_new_float(get_lat());
}
static MP_DEFINE_CONST_FUN_OBJ_0(wsm_get_lat_obj, wsm_get_lat);

/// \method wsm_get_lat_sim7600()
mp_obj_t wsm_get_lat_sim7600(void) {
    return mp_obj_new_float(get_lat_sim7600());
}
static MP_DEFINE_CONST_FUN_OBJ_0(wsm_get_lat_sim7600_obj, wsm_get_lat_sim7600);

/// \method wsm_get_lon()
mp_obj_t wsm_get_lon(void) {
    return mp_obj_new_float(get_lon());
}
static MP_DEFINE_CONST_FUN_OBJ_0(wsm_get_lon_obj, wsm_get_lon);

/// \method wsm_get_lon_sim7600()
mp_obj_t wsm_get_lon_sim7600(void) {
    return mp_obj_new_float(get_lon_sim7600());
}
static MP_DEFINE_CONST_FUN_OBJ_0(wsm_get_lon_sim7600_obj, wsm_get_lon_sim7600);

/// \method wsm_set_mag_degrees()
mp_obj_t wsm_set_mag_degrees(mp_obj_t value) {
    float val = mp_obj_get_float(value);
    set_mag_degrees(val);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(wsm_set_mag_degrees_obj, wsm_set_mag_degrees);

/// \method wsm_get_mag_degrees()
mp_obj_t wsm_get_mag_degrees(void) {
    return mp_obj_new_float(get_mag_degrees());
}
static MP_DEFINE_CONST_FUN_OBJ_0(wsm_get_mag_degrees_obj, wsm_get_mag_degrees);

/// \method wsm_set_pitch()
mp_obj_t wsm_set_pitch(mp_obj_t value) {
    float val = mp_obj_get_float(value);
    set_pitch(val);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(wsm_set_pitch_obj, wsm_set_pitch);

/// \method wsm_use_gps_sim7600()
mp_obj_t wsm_use_gps_sim7600(mp_obj_t value) {
    bool val = mp_obj_get_int(value);
    set_useGpsSim7600(val);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(wsm_use_gps_sim7600_obj, wsm_use_gps_sim7600);

/// \method wsm_get_goal_changed()
mp_obj_t wsm_get_goal_changed(void) {
    return mp_obj_new_bool(get_goalChanged());
}
static MP_DEFINE_CONST_FUN_OBJ_0(wsm_get_goal_changed_obj, wsm_get_goal_changed);

/// \method wsm_set_goal_changed()
mp_obj_t wsm_set_goal_changed(mp_obj_t value) {
    bool val = mp_obj_is_true(value);
    set_goalChanged(val);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(wsm_set_goal_changed_obj, wsm_set_goal_changed);

/// \method wsm_set_delta_lat_sim7600()
mp_obj_t wsm_set_delta_lat_sim7600(mp_obj_t value) {
    double val = mp_obj_get_float(value);
    set_delta_lat_sim7600(val);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(wsm_set_delta_lat_sim7600_obj, wsm_set_delta_lat_sim7600);

/// \method wsm_set_delta_lon_sim7600()
mp_obj_t wsm_set_delta_lon_sim7600(mp_obj_t value) {
    double val = mp_obj_get_float(value);
    set_delta_lon_sim7600(val);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(wsm_set_delta_lon_sim7600_obj, wsm_set_delta_lon_sim7600);

/// \method wsm_get_alert()
mp_obj_t wsm_get_alert(void) {
    return mp_obj_new_int(get_AlertToSend());
}
static MP_DEFINE_CONST_FUN_OBJ_0(wsm_get_alert_obj, wsm_get_alert);

/// \method wsm_get_desfw()
mp_obj_t wsm_get_desfw(void) {
    return mp_obj_new_float(get_desFW());
}
static MP_DEFINE_CONST_FUN_OBJ_0(wsm_get_desfw_obj, wsm_get_desfw);

/// \method wsm_get_yaw_start()
mp_obj_t wsm_get_yaw_start(void) {
    return mp_obj_new_int(get_yawStart());
}
static MP_DEFINE_CONST_FUN_OBJ_0(wsm_get_yaw_start_obj, wsm_get_yaw_start);

/// \method wsm_power_down()
mp_obj_t wsm_turn_off(void) {
    power_down();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(wsm_turn_off_obj, wsm_turn_off);

/// \method wsm_compute_euler()
/// Compute Euler angles.
mp_obj_t wsm_compute_euler(size_t n_args, const mp_obj_t *args) {
    mp_obj_list_t *data = MP_OBJ_TO_PTR(mp_obj_new_list(3, NULL));
    double x = mp_obj_get_float(args[0]);
    double y = mp_obj_get_float(args[1]);
    double z = mp_obj_get_float(args[2]);
    double w = mp_obj_get_float(args[3]);
    double ysqr = y * y;

    // Roll (x-axis rotation)
    double t0 = 2.0 * (w * x + y * z);
    double t1 = 1.0 - 2.0 * (x * x + ysqr);
    double roll = degrees(atan2(t0, t1));

    // Pitch (y-axis rotation)
    double t2 = 2.0 * (w * y - z * x);
    // Manual clamping to [-1.0, 1.0] to avoid nan in asin
    t2 = (t2 > 1.0) ? 1.0 : t2;
    t2 = (t2 < -1.0) ? -1.0 : t2;
    double pitch = degrees(asin(t2));

    // Yaw (z-axis rotation)
    double t3 = 2.0 * (w * z + x * y);
    double t4 = 1.0 - 2.0 * (ysqr + z * z);
    double yaw = degrees(atan2(t3, t4));

    data->items[0] = mp_obj_new_float(roll);
    data->items[1] = mp_obj_new_float(pitch);
    data->items[2] = mp_obj_new_float(yaw);
    return MP_OBJ_FROM_PTR(data);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(wsm_compute_euler_obj, 4, 4, wsm_compute_euler);

static const mp_rom_map_elem_t wsm_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_wsm) },

    { MP_ROM_QSTR(MP_QSTR_bt_updated), MP_ROM_PTR(&wsm_bt_updated_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_bt_update), MP_ROM_PTR(&wsm_get_bt_update_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_api_version), MP_ROM_PTR(&get_api_version_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_heading), MP_ROM_PTR(&wsm_set_heading_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_heading_filt), MP_ROM_PTR(&wsm_set_heading_filt_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_delta_lat), MP_ROM_PTR(&wsm_set_delta_lat_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_delta_lon), MP_ROM_PTR(&wsm_set_delta_lon_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_gps_precision), MP_ROM_PTR(&wsm_set_gps_precision_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_gps_heading), MP_ROM_PTR(&wsm_set_gps_heading_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_gps_delta_dist), MP_ROM_PTR(&wsm_set_gps_delta_dist_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_gps_fix_quality), MP_ROM_PTR(&wsm_set_gps_fix_quality_obj) },
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
    { MP_ROM_QSTR(MP_QSTR_set_update_goal_on_db), MP_ROM_PTR(&wsm_set_update_goal_on_db_obj) },
    { MP_ROM_QSTR(MP_QSTR_print_log), MP_ROM_PTR(&wsm_print_log_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_wifi_credentials), MP_ROM_PTR(&wsm_set_wifi_credentials_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_control_type), MP_ROM_PTR(&wsm_get_control_type_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_start_lat), MP_ROM_PTR(&wsm_set_start_lat_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_start_lon), MP_ROM_PTR(&wsm_set_start_lon_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_delta_lat), MP_ROM_PTR(&wsm_get_delta_lat_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_delta_lat_sim7600), MP_ROM_PTR(&wsm_get_delta_lat_sim7600_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_delta_lon), MP_ROM_PTR(&wsm_get_delta_lon_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_delta_lon_sim7600), MP_ROM_PTR(&wsm_get_delta_lon_sim7600_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_start_lat), MP_ROM_PTR(&wsm_get_start_lat_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_start_lon), MP_ROM_PTR(&wsm_get_start_lon_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_gpstime), MP_ROM_PTR(&wsm_set_gpstime_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_speed_limit), MP_ROM_PTR(&wsm_get_speed_limit_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_speed_limit), MP_ROM_PTR(&wsm_set_speed_limit_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_alert), MP_ROM_PTR(&wsm_set_alert_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_mark_id), MP_ROM_PTR(&wsm_get_mark_id_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_force_forward), MP_ROM_PTR(&wsm_set_force_forward_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_fresh_gps), MP_ROM_PTR(&wsm_set_fresh_gps_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_fresh_gps_sim7600), MP_ROM_PTR(&wsm_set_fresh_gps_sim7600_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_fresh_gps), MP_ROM_PTR(&wsm_get_fresh_gps_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_fresh_gps_sim7600), MP_ROM_PTR(&wsm_get_fresh_gps_sim7600_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_gps_delta_dist), MP_ROM_PTR(&wsm_get_gps_delta_dist_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_gps_delta_dist_sim7600), MP_ROM_PTR(&wsm_get_gps_delta_dist_sim7600_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_gps_heading), MP_ROM_PTR(&wsm_get_gps_heading_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_gps_heading_sim7600), MP_ROM_PTR(&wsm_get_gps_heading_sim7600_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_gps_precision), MP_ROM_PTR(&wsm_get_gps_precision_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_gps_precision_sim7600), MP_ROM_PTR(&wsm_get_gps_precision_sim7600_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_lat), MP_ROM_PTR(&wsm_get_lat_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_lat_sim7600), MP_ROM_PTR(&wsm_get_lat_sim7600_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_lon), MP_ROM_PTR(&wsm_get_lon_obj) },   
    { MP_ROM_QSTR(MP_QSTR_get_lon_sim7600), MP_ROM_PTR(&wsm_get_lon_sim7600_obj) },  
    { MP_ROM_QSTR(MP_QSTR_set_mag_degrees), MP_ROM_PTR(&wsm_set_mag_degrees_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_mag_degrees), MP_ROM_PTR(&wsm_get_mag_degrees_obj) },    
    { MP_ROM_QSTR(MP_QSTR_set_pitch), MP_ROM_PTR(&wsm_set_pitch_obj) },
    { MP_ROM_QSTR(MP_QSTR_use_gps_sim7600), MP_ROM_PTR(&wsm_use_gps_sim7600_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_goal_changed), MP_ROM_PTR(&wsm_get_goal_changed_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_goal_changed), MP_ROM_PTR(&wsm_set_goal_changed_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_delta_lat_sim7600), MP_ROM_PTR(&wsm_set_delta_lat_sim7600_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_delta_lon_sim7600), MP_ROM_PTR(&wsm_set_delta_lon_sim7600_obj) },  
    { MP_ROM_QSTR(MP_QSTR_get_alert), MP_ROM_PTR(&wsm_get_alert_obj) },  
    { MP_ROM_QSTR(MP_QSTR_get_desfw), MP_ROM_PTR(&wsm_get_desfw_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_yaw_start), MP_ROM_PTR(&wsm_get_yaw_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_turn_off), MP_ROM_PTR(&wsm_turn_off_obj) },
    { MP_ROM_QSTR(MP_QSTR_compute_euler), MP_ROM_PTR(&wsm_compute_euler_obj) },
};

static MP_DEFINE_CONST_DICT(wsm_module_globals, wsm_module_globals_table);

const mp_obj_module_t wsm_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&wsm_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_wsm, wsm_module);
