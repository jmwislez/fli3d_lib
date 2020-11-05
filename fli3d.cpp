/*
 * Fli3d - Library (file system, wifi, TM/TC, comms functionality)
 * version: 2020-11-05 (native-ccsds branch)
 */

#ifndef PLATFORM_ESP8266_RADIO
 
#include "fli3d.h"
#include "secrets.h"
#ifdef PLATFORM_ESP32CAM
#include "FS.h"
#include "SD_MMC.h"
#include "SPI.h"
#endif

WiFiUDP wifiUDP;
WiFiUDP wifiUDP_NTP;
#ifdef ASYNCUDP
AsyncUDP asyncUDP_yamcs_tc;
#else
WiFiUDP wifiUDP_yamcs_tc;
#endif
FtpServer wifiTCP_FTP;
NTPClient timeClient(wifiUDP_NTP, config_network.ntp_server, 0);

char buffer[JSON_MAX_SIZE];
char serial_buffer[JSON_MAX_SIZE];
char path_buffer[32];

extern void ota_setup ();
#ifdef PLATFORM_ESP32
extern void gps_set_samplerate (uint8_t rate);
extern void motion_set_samplerate (uint8_t rate);
extern void mpu6050_set_accel_range (uint8_t accel_range);
extern void mpu6050_set_gyro_range (uint8_t gyro_range);
extern void publish_radio ();
#endif 

ccsds_hdr_t        ccsds_hdr;
ccsds_t            ccsds_tc_buffer;

sts_esp32_t        sts_esp32;
sts_esp32cam_t     sts_esp32cam;
tm_esp32_t         esp32;
tm_esp32cam_t      esp32cam;
tm_camera_t        ov2640;
tm_gps_t           neo6mv2;
tm_motion_t        mpu6050;
tm_pressure_t      bmp280;
tm_radio_t         radio;
tm_timer_t         timer;
tc_esp32_t         tc_esp32;
tc_esp32cam_t      tc_esp32cam;

var_timer_t        var_timer;
var_esp32_t        var_esp32;
var_esp32cam_t     var_esp32cam;

config_network_t   config_network;
config_esp32_t     config_esp32;
config_esp32cam_t  config_esp32cam;

#ifdef PLATFORM_ESP32
tm_esp32_t         *tm_this = &esp32;
tm_esp32cam_t      *tm_other = &esp32cam;
tc_esp32_t         *tc_this = &tc_esp32;
tc_esp32cam_t      *tc_other = &tc_esp32cam;
sts_esp32_t        *sts_this = &sts_esp32;
sts_esp32cam_t     *sts_other = &sts_esp32cam;
config_esp32_t     *config_this = &config_esp32;
var_esp32_t        *var_this = &var_esp32;
#endif
#ifdef PLATFORM_ESP32CAM
tm_esp32cam_t      *tm_this = &esp32cam;
tm_esp32_t         *tm_other = &esp32;
tc_esp32cam_t      *tc_this = &tc_esp32cam;
tc_esp32_t         *tc_other = &tc_esp32;
sts_esp32cam_t     *sts_this = &sts_esp32cam;
sts_esp32_t        *sts_other = &sts_esp32;
config_esp32cam_t  *config_this = &config_esp32cam;
var_esp32cam_t     *var_this = &var_esp32cam;
#endif

const char pidName[NUMBER_OF_PID][13] =   { "sts_esp32", "sts_esp32cam", "tm_esp32", "tm_esp32cam", "tm_camera", "tm_gps", "tm_motion", "tm_pressure", "tm_radio", "tm_timer", "tc_esp32", "tc_esp32cam" };
const char eventName[8][9] =              { "init", "info", "warning", "error", "cmd", "cmd_ack", "cmd_resp", "cmd_fail" };
const char subsystemName[13][14] =        { "esp32", "esp32cam", "ov2640", "neo6mv2", "mpu6050", "bmp280", "radio", "sd", "separation", "timer", "fli3d", "ground", "any" };
const char modeName[7][10] =              { "init", "checkout", "ready", "thrust", "freefall", "parachute", "static" };
const char cameraModeName[4][7] =         { "init", "idle", "single", "stream" };
const char cameraResolutionName[11][10] = { "160x120", "invalid1", "invalid2", "240x176", "320x240", "400x300", "640x480", "800x600", "1024x768", "1280x1024", "1600x1200" };
const char dataEncodingName[3][8] =       { "CCSDS", "JSON", "ASCII" };
const char commLineName[9][13] =          { "serial", "wifi_udp", "wifi_yamcs", "wifi_cam", "sd_ccsds", "sd_json", "sd_cam", "fs", "radio" };
const char tcName[5][20] =                { "reboot", "set_opsmode", "load_config", "load_routing", "set_parameter" };
const char gpsStatusName[9][11] =         { "none", "est", "time_only", "std", "dgps", "rtk_float", "rtk_fixed", "status_pps", "waiting" }; 

char routing_serial[NUMBER_OF_PID];
char routing_udp[NUMBER_OF_PID];
char routing_yamcs[NUMBER_OF_PID];
char routing_fs[NUMBER_OF_PID];
#ifdef PLATFORM_ESP32CAM
char routing_sd_json[NUMBER_OF_PID];
char routing_sd_ccsds[NUMBER_OF_PID];
#endif


// FS FUNCTIONALITY

bool fs_setup () {
  if (LITTLEFS.begin(false)) {
    if (LITTLEFS.totalBytes()-LITTLEFS.usedBytes() <= 4096) {
      // FS is full, delete old data 
      #ifdef PLATFORM_ESP32
      if (!esp32.separation_sts) { // old data will only be deleted from ESP32 module if SEP_STS_PIN is connected to GND (Fli3d mated to rocket)
      #endif
      #ifdef PLATFORM_ESP32CAM
      if (true) { // TODO: add viable inhibit for ESP32CAM
      #endif
        File dir = LITTLEFS.open ("/");
        File file = dir.openNextFile ();
        char local_path_buffer[32];
        while (file) {
        if (file.isDirectory()) {
          sprintf (path_buffer, "%s", file.name());
          file.close ();
          File local_dir = LITTLEFS.open (path_buffer);
          File local_file = local_dir.openNextFile ();
          while (local_file) {
          sprintf (local_path_buffer, "%s", local_file.name());
          local_file.close ();
          LITTLEFS.remove (local_path_buffer);
          local_file = local_dir.openNextFile ();
          }  
          LITTLEFS.rmdir (path_buffer);
        }
        file = dir.openNextFile ();
        }
        publish_event (STS_THIS, SS_THIS, EVENT_WARNING, "Deleted all data files since FS is full");
      }
      else {
        publish_event (STS_THIS, SS_THIS, EVENT_WARNING, "FS is full, but data not deleted because this is enhibited");
      }
    }
    // ensure there are no stale log files
    File dir = LITTLEFS.open ("/");
    File file = dir.openNextFile ();
    while (file) {
      if (!strcmp(file.name(), "/json.log") or strstr(file.name(), ".raw")) {
        sprintf (path_buffer, "%s", file.name());
        file.close ();
        LITTLEFS.remove (path_buffer);
      }
      file = dir.openNextFile ();
    }
    tm_this->fs_enabled = true;          
    tm_this->fs_active = true;
    sprintf (buffer, "Initialized FS (size: %d kB; free: %d kB)", LITTLEFS.totalBytes()/2024, fs_free());
    publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);
    return true;
  }
  else {
    if (LITTLEFS.begin(true)) {
      sprintf (buffer, "Formatted and initialized FS (size: %d kB; free: %d kB)", LITTLEFS.totalBytes()/1024, fs_free());
      publish_event (STS_THIS, SS_THIS, EVENT_WARNING, buffer);
      tm_this->fs_enabled = true;
      tm_this->fs_active = true;
      return true;
    }
    else {
      publish_event (STS_THIS, SS_THIS, EVENT_ERROR, "Failed to initialize or format FS");
      config_this->fs_enable = false;
      tm_this->fs_enabled = false;
      tm_this->err_fs_dataloss = true;
      return false;
    }
  }
}

bool ftp_setup () {
  if (tm_this->fs_enabled) {
    wifiTCP_FTP.begin(config_network.ftp_user, config_network.ftp_password);
    sprintf (buffer, "Starting passive FTP server on TCP port %s:%u", WiFi.localIP().toString().c_str(), 21);
    publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);
  }
  else {
    publish_event (STS_THIS, SS_THIS, EVENT_ERROR, "Failed to initialize FTP server as no file system available");
  }
}

bool ftp_check () {
  wifiTCP_FTP.handleFTP (LITTLEFS);
  tm_this->fs_ftp_enabled = true;
  tm_this->ftp_active = true;
}

void fs_create_today_dir () {
  char sequencer = 'A';
  char local_path_buffer[32];
  while (LITTLEFS.exists(var_this->today_dir)) {
    sprintf (var_this->today_dir, "/%s%s%s%c", timeClient.getFormattedDate().substring(0,4), timeClient.getFormattedDate().substring(5,7), timeClient.getFormattedDate().substring(8,10), sequencer++);
  }
  LITTLEFS.mkdir(var_this->today_dir);
  File dir = LITTLEFS.open ("/");
  File file = dir.openNextFile ();
  while (file) {
    if (!strcmp(file.name(), "/json.log") or strstr(file.name(), ".raw")) {
      sprintf (local_path_buffer, "%s", file.name());
      sprintf (path_buffer, "%s/%s", var_this->today_dir, file.name());
      file.close ();
      LITTLEFS.rename (local_path_buffer, path_buffer);
    }
    file = dir.openNextFile ();
  }
  sprintf (buffer, "Created storage directory %s for current session, and moved current log files to it", var_this->today_dir);
  publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);
  tm_this->fs_active = true;
}

uint16_t fs_free () {
  if (config_this->fs_enable) {
    if (LITTLEFS.totalBytes()-LITTLEFS.usedBytes() <= 4096) {
      config_this->fs_enable = false;
      tm_this->fs_enabled = false;
      tm_this->err_fs_dataloss = true;
      sprintf (buffer, "Disabling further access to FS because it is full");
      publish_event (STS_THIS, SS_THIS, EVENT_ERROR, buffer);
      return (0);
    }
    else {
      return ((LITTLEFS.totalBytes()-LITTLEFS.usedBytes())/1024);
    }
  }
  else { 
    return (0);
  }
}

#ifdef PLATFORM_ESP32CAM
uint16_t sd_free () {
  return ((SD_MMC.totalBytes() - SD_MMC.usedBytes())/1024);
}
#endif

// CONFIGURATION FUNCTIONALITY

void load_default_config () {
  strcpy (config_network.wifi_ssid, default_wifi_ssid); 
  strcpy (config_network.wifi_password, default_wifi_password);
  strcpy (config_network.ap_ssid, default_ap_ssid); 
  strcpy (config_network.ap_password, default_ap_password); 
  strcpy (config_network.ftp_user, default_ftp_user); 
  strcpy (config_network.ftp_password, default_ftp_password); 
  strcpy (config_network.udp_server, default_udp_server);
  strcpy (config_network.yamcs_server, default_yamcs_server); 
  strcpy (config_network.ntp_server, default_ntp_server);
  config_network.udp_port = default_udp_port; 
  config_network.yamcs_tm_port = default_yamcs_tm_port; 
  config_network.yamcs_tc_port = default_yamcs_tc_port; 
  config_esp32.radio_rate = 1;
  config_esp32.pressure_rate = 1;
  config_esp32.motion_rate = 1;
  config_esp32.gps_rate = 1;
  strcpy (config_esp32.config_file, "/default.cfg");
  strcpy (config_esp32.routing_file, "/default.rt");
  config_esp32.mpu6050_accel_sensitivity = 595;
  config_esp32.mpu6050_accel_offset_x = 0;
  config_esp32.mpu6050_accel_offset_y = 0;
  config_esp32.mpu6050_accel_offset_z = 0;
  config_esp32.radio_enable = false;
  config_esp32.pressure_enable = false;
  config_esp32.motion_enable = false;
  config_esp32.gps_enable = false;
  config_esp32.camera_enable = false;
  config_esp32.wifi_enable = true;
  config_esp32.wifi_sta_enable = true;
  config_esp32.wifi_ap_enable = true;
  config_esp32.wifi_udp_enable = true;
  config_esp32.wifi_yamcs_enable = true;
  config_esp32.fs_enable = true;
  config_esp32.serial_format = ENC_JSON; // TODO: put to ENC_CCSDS after debug phase
  config_esp32.debug_over_serial = true;
  config_esp32.ota_enable = false;
  config_esp32.motion_udp_raw_enable = false;
  config_esp32.gps_udp_raw_enable = false;
  mpu6050.accel_range = 3;
  mpu6050.gyro_range = 3;
  strcpy (config_esp32cam.config_file, "/default.cfg");
  strcpy (config_esp32cam.routing_file, "/default.rt");
  config_esp32cam.wifi_enable = true;
  config_esp32cam.wifi_sta_enable = true;
  config_esp32cam.wifi_ap_enable = true;
  config_esp32cam.wifi_udp_enable = true;
  config_esp32cam.wifi_yamcs_enable = true;
  config_esp32cam.wifi_image_enable = true; 
  config_esp32cam.fs_enable = true;
  config_esp32cam.sd_enable = true;
  config_esp32cam.sd_json_enable = true;
  config_esp32cam.sd_ccsds_enable = true;
  config_esp32cam.sd_image_enable = true;
  config_esp32cam.serial_format = ENC_JSON; // TODO: put to ENC_CCSDS after debug phase
  config_esp32cam.debug_over_serial = false;
  //                       0: STS_ESP32 
  //                       |  1: STS_ESP32CAM 
  //                       |  |  2: TM_ESP32 
  //                       |  |  |  3: TM_ESP32CAM 
  //                       |  |  |  |  4: TM_CAMERA 
  //                       |  |  |  |  |  5: TM_GPS 
  //                       |  |  |  |  |  |  6: TM_MOTION 
  //                       |  |  |  |  |  |  |  7: TM_PRESSURE 
  //                       |  |  |  |  |  |  |  |  8: TM_RADIO 
  //                       |  |  |  |  |  |  |  |  |  9: TM_TIMER 
  //                       |  |  |  |  |  |  |  |  |  |  A: TC_ESP32
  //                       |  |  |  |  |  |  |  |  |  |  |  B: TC_ESP32CAM
  //                       0  1  2  3  4  5  6  7  8  9  A  B
  // ESP32                 *     *        *  *  *  *  *     *
  // ESP32CAM                 *     *  *                 *   
  //                       ----------------------------------  
  #ifdef PLATFORM_ESP32
  char rt_serial[40] =   " 1, 0, 1, 0, 0, 1, 1, 1, 1, 1, 0, 1 ";
  char rt_yamcs[40] =    " 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0 ";
  char rt_udp[40] =      " 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ";
  char rt_fs[40] =       " 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 ";
  set_routing (routing_serial, (const char*)rt_serial);
  set_routing (routing_yamcs, (const char*)rt_yamcs);
  set_routing (routing_udp, (const char*)rt_udp);
  set_routing (routing_fs, (const char*)rt_fs);
  #endif
  #ifdef PLATFORM_ESP32CAM
  char rt_serial[40] =   " 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0 ";
  char rt_yamcs[40] =    " 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0 ";
  char rt_udp[40] =      " 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ";
  char rt_fs[40] =       " 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1 ";
  char rt_sd_json[40] =  " 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1 ";
  char rt_sd_ccsds[40] = " 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1 ";
  set_routing (routing_serial, (const char*)rt_serial);
  set_routing (routing_yamcs, (const char*)rt_yamcs);
  set_routing (routing_udp, (const char*)rt_udp);
  set_routing (routing_fs, (const char*)rt_fs);
  set_routing (routing_sd_json, (const char*)rt_sd_json);
  set_routing (routing_sd_ccsds, (const char*)rt_sd_ccsds);
  #endif
}

bool fs_load_settings () {
  char linebuffer[80];
  uint8_t value_start;
  File file = LITTLEFS.open ("/current.cfg");
  if (!file) {
    publish_event (STS_THIS, SS_THIS, EVENT_ERROR, "Failed to open configuration file '/current.cfg' from FS");
    return false;
  }
  uint8_t i = 0; 
  while (file.available ()) {
    linebuffer[i] = file.read();
    if (linebuffer[i] == '=') {
      value_start = i + 1;
    }
    if (linebuffer[i] != '\r') { // ignore \r
      if (linebuffer[i] == '\n' or !file.available()) { // full line read
        linebuffer[i] = 0; // mark end of c string
        if (String(linebuffer).startsWith("config")) {
          strcpy (config_this->config_file, String(linebuffer).substring(value_start).c_str());
          Serial.println ("Set config file to " + String (config_this->config_file));
        }
        if (String(linebuffer).startsWith("routing")) {
          strcpy (config_this->routing_file, String(linebuffer).substring(value_start).c_str());
          Serial.println ("Set routing file to " + String (config_this->routing_file));
        }
        i = 0;
      }
      else {
        i++;
      }
    }
  }
  file.close();
  publish_event (STS_THIS, SS_THIS, EVENT_INIT, "Determined settings from FS current.config file");
  tm_this->fs_active = true;
  return true;
}

bool fs_load_config (const char* filename) { 
  char linebuffer[80];
  uint8_t value_start;
  File file = LITTLEFS.open (filename);
  if (!file) {
    sprintf (buffer, "Failed to open configuration file '%s' from FS", filename);
    publish_event (STS_THIS, SS_THIS, EVENT_ERROR, buffer);
    return false;
  }
  uint8_t i = 0; 
  while (file.available ()) {
    linebuffer[i] = file.read();
    if (linebuffer[i] == '=') {
      value_start = i + 1;
    }
    if (linebuffer[i] != '\r') { // ignore \r
      if (linebuffer[i] == '\n' or !file.available()) { // full line read
        linebuffer[i] = 0; // mark end of c string
        if (set_parameter (String(linebuffer).substring(0, value_start-1).c_str(), String(linebuffer).substring(value_start).c_str())) {
          Serial.println (buffer);
        }
        i = 0;
      }
      else {
        i++;
      }
    }
  }
  file.close();
  sprintf (buffer, "Read settings from '%s' configuration file on FS", config_this->config_file);
  publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);
  tm_this->fs_active = true;
  return true;
}

bool fs_load_routing (const char* filename) {
  char linebuffer[80], parameter[80];
  uint8_t value_start;
  File file = LITTLEFS.open (filename);
  if (!file) {
    sprintf (buffer, "Failed to open routing file '%s' from FS", filename);
    publish_event (STS_THIS, SS_THIS, EVENT_ERROR, buffer);
    return false;
  }
  uint8_t i = 0; 
  while (file.available ()) {
    linebuffer[i] = file.read();    
    if (linebuffer[i] == '=') {
      value_start = i + 1;
    }
    if (linebuffer[i] != '\r') { // ignore \r
      if (linebuffer[i] == '\n' or !file.available()) { // full line read
        linebuffer[i] = 0; // mark end of c string
        strcpy (parameter, String(linebuffer).substring(value_start).c_str());
        if (String(linebuffer).startsWith("rt_serial")) {
          Serial.println ("Set routing_serial to " + set_routing (routing_serial, (const char*)parameter));
        }
        if (String(linebuffer).startsWith("rt_yamcs")) {
          Serial.println ("Set routing_yamcs to " + set_routing (routing_yamcs, (const char*)parameter));
        }
        if (String(linebuffer).startsWith("rt_udp")) {
          Serial.println ("Set routing_udp to " + set_routing (routing_udp, (const char*)parameter));
        }
        if (String(linebuffer).startsWith("rt_fs")) {
          Serial.println ("Set routing_fs to " + set_routing (routing_fs, (const char*)parameter));
        }
        #ifdef PLATFORM_ESP32CAM
        if (String(linebuffer).startsWith("rt_sd_json")) {
          Serial.println ("Set routing_sd_json to " + set_routing (routing_sd_json, (const char*)parameter));
        }
        if (String(linebuffer).startsWith("rt_sd_ccsds")) {
          Serial.println ("Set routing_sd_ccsds to " + set_routing (routing_sd_ccsds, (const char*)parameter));
        }
        #endif
        i = 0;
      }
      else {
        i++;
      }
    }
  }
  file.close();
  sprintf (buffer, "Read routing from '%s' configuration file on FS", config_this->routing_file);
  publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);
  tm_this->fs_active = true;
  return true;
}

String set_routing (char* routing_table, const char* routing_string) {
  uint16_t PID;
  String return_string;
  for (uint8_t i = 0; i < strlen (routing_string); i++) {
  	if (routing_string[i] == '0') {
  	  *routing_table++ = false;
  	  return_string += String(PID++) + ":0 ";
  	}
  	else if (routing_string[i] == '1') {
  	  *routing_table++ = true;
  	  return_string += String(PID++) + ":1 ";
  	}
  }
  return (return_string);
}

bool set_parameter (const char* parameter, const char* value) {
  bool success = false;
  if (!strcmp(parameter, "wifi_ssid")) { 
    strcpy (config_network.wifi_ssid, value);
    sprintf (buffer, "Set wifi_ssid to %s", config_network.wifi_ssid);
    success = true;
  }
  else if (!strcmp(parameter, "wifi_password")) { 
    strcpy (config_network.wifi_password, value);
    sprintf (buffer, "Set wifi_password to %s", config_network.wifi_password);
    success = true;
  }
  else if (!strcmp(parameter, "ap_ssid")) { 
    strcpy (config_network.ap_ssid, value);
    sprintf (buffer, "Set ap_ssid to %s", config_network.ap_ssid);
    success = true;
  }
  else if (!strcmp(parameter, "ap_password")) { 
    strcpy (config_network.ap_password, value);
    sprintf (buffer, "Set ap_password to %s", config_network.ap_password);
    success = true;
  }
  else if (!strcmp(parameter, "ftp_user")) { 
    strcpy (config_network.ftp_user, value);
    sprintf (buffer, "Set ftp_user to %s", config_network.ftp_user);
    success = true;
  }
  else if (!strcmp(parameter, "ftp_password")) { 
    strcpy (config_network.ftp_password, value);
    sprintf (buffer, "Set ftp_password to %s", config_network.ftp_password);
    success = true;
  }
  else if (!strcmp(parameter, "udp_server")) { 
    strcpy (config_network.udp_server, value);
    sprintf (buffer, "Set udp_server to %s", config_network.udp_server);
    success = true;
  }
  else if (!strcmp(parameter, "yamcs_server")) { 
    strcpy (config_network.yamcs_server, value);
    sprintf (buffer, "Set yamcs_server to %s", config_network.yamcs_server);
    success = true;
  }
  else if (!strcmp(parameter, "ntp_server")) { 
    strcpy (config_network.ntp_server, value);
    sprintf (buffer, "Set ntp_server to %s", config_network.ntp_server);
    success = true;
  }
  else if (!strcmp(parameter, "udp_port")) { 
    config_network.udp_port = atoi(value);
    sprintf (buffer, "Set udp_port to %u", config_network.udp_port);
    success = true;
  }
  else if (!strcmp(parameter, "yamcs_tm_port")) { 
    config_network.yamcs_tm_port = atoi(value);
    sprintf (buffer, "Set yamcs_tm_port to %u", config_network.yamcs_tm_port);
    success = true;
  }
  else if (!strcmp(parameter, "yamcs_tc_port")) { 
    config_network.yamcs_tc_port = atoi(value);
    sprintf (buffer, "Set yamcs_tc_port to %u", config_network.yamcs_tc_port);
    success = true;
  }  
  else if (!strcmp(parameter, "wifi_enable")) { 
    config_this->wifi_enable = atoi(value);
    sprintf (buffer, "Set wifi_enable to %s", config_this->wifi_enable?"true":"false");
    success = true;
  }
  else if (!strcmp(parameter, "wifi_sta_enable")) { 
    config_this->wifi_sta_enable = atoi(value);
    sprintf (buffer, "Set wifi_sta_enable to %s", config_this->wifi_sta_enable?"true":"false");
    success = true;
  }  
  else if (!strcmp(parameter, "wifi_ap_enable")) { 
    config_this->wifi_ap_enable = atoi(value);
    sprintf (buffer, "Set wifi_ap_enable to %s", config_this->wifi_ap_enable?"true":"false");
    success = true;
  }  
  else if (!strcmp(parameter, "wifi_udp_enable")) { 
    config_this->wifi_udp_enable = atoi(value);
    sprintf (buffer, "Set wifi_udp_enable to %s", config_this->wifi_udp_enable?"true":"false");
    success = true;
  }  
  else if (!strcmp(parameter, "wifi_yamcs_enable")) { 
    config_this->wifi_yamcs_enable = atoi(value);
    sprintf (buffer, "Set wifi_yamcs_enable to %s", config_this->wifi_yamcs_enable?"true":"false");
    success = true;
  }  
  else if (!strcmp(parameter, "fs_enable")) { 
    config_this->fs_enable = atoi(value);
    sprintf (buffer, "Set fs_enable to %s", config_this->fs_enable?"true":"false");
    success = true;
  }  
  else if (!strcmp(parameter, "serial_format")) { 
    config_this->serial_format = atoi(value);
    sprintf (buffer, "Set serial_format to %s", dataEncodingName[config_this->serial_format]);
    success = true;
  }  
  else if (!strcmp(parameter, "debug_over_serial")) { 
    config_this->debug_over_serial = atoi(value);
    sprintf (buffer, "Set debug_over_serial to %s", config_this->debug_over_serial?"true":"false");
    success = true;
  }  
  else if (!strcmp(parameter, "ota_enable")) { 
    config_this->ota_enable = atoi(value);
    sprintf (buffer, "Set ota_enable to %s", config_this->ota_enable?"true":"false");
    success = true;
  }  
  #ifdef PLATFORM_ESP32
  else if (!strcmp(parameter, "radio_rate")) { 
    config_this->radio_rate = atoi(value);
    if (config_this->radio_rate) {
      var_timer.radio_interval = (1000 / config_this->radio_rate);
      sprintf (buffer, "Set radio_rate to %u Hz", config_this->radio_rate);
    }
    else {
      tm_this->radio_enabled = false;
      sprintf (buffer, "Set radio_enabled to false");
    }
    success = true;
  }
  else if (!strcmp(parameter, "pressure_rate")) { 
    config_this->pressure_rate = atoi(value);
    if (config_this->pressure_rate) {
      var_timer.pressure_interval = (1000 / config_this->pressure_rate);
      sprintf (buffer, "Set pressure_rate to %u Hz", config_this->pressure_rate);
    }
    else {
      tm_this->pressure_enabled = false;
      sprintf (buffer, "Set pressure_enabled to false");
    }  
    success = true;
  } 
  else if (!strcmp(parameter, "motion_rate")) { 
    config_this->motion_rate = atoi(value);
     if (config_this->motion_rate) {
      var_timer.motion_interval = (1000 / config_this->motion_rate);
      motion_set_samplerate (config_this->motion_rate);
      sprintf (buffer, "Set motion_rate to %u Hz", config_this->motion_rate);
    }
    else {
      tm_this->motion_enabled = false;
      sprintf (buffer, "Set motion_enabled to false");
    }
    success = true;
  } 
  else if (!strcmp(parameter, "gps_rate")) { 
    config_this->gps_rate = atoi(value);
     if (config_this->gps_rate) {
      var_timer.gps_interval = (1000 / config_this->gps_rate);
      if (esp32.gps_enabled) {
        gps_set_samplerate (config_this->gps_rate);
      }
      sprintf (buffer, "Set gps_rate to %u Hz", config_this->gps_rate); 
    }
    else {
      tm_this->gps_enabled = false;
      sprintf (buffer, "Set gps_enabled to false");
    }
    success = true;
  } 
  else if (!strcmp(parameter, "radio_enable")) { 
    config_this->radio_enable = atoi(value);
    sprintf (buffer, "Set radio_enable to %s", config_this->radio_enable?"true":"false");
    success = true;
  }  
  else if (!strcmp(parameter, "pressure_enable")) { 
    config_this->pressure_enable = atoi(value);
    sprintf (buffer, "Set pressure_enable to %s", config_this->pressure_enable?"true":"false");
    success = true;
  }  
  else if (!strcmp(parameter, "motion_enable")) { 
    config_this->motion_enable = atoi(value);
    sprintf (buffer, "Set motion_enable to %s", config_this->motion_enable?"true":"false");
    success = true;
  }  
  else if (!strcmp(parameter, "gps_enable")) { 
    config_this->gps_enable = atoi(value);
    sprintf (buffer, "Set gps_enable to %s", config_this->gps_enable?"true":"false");
    success = true;
  }  
  else if (!strcmp(parameter, "camera_enable")) { 
    config_this->camera_enable = atoi(value);
    sprintf (buffer, "Set camera_enable to %s", config_this->camera_enable?"true":"false");
    success = true;
  }  
  else if (!strcmp(parameter, "motion_udp_raw_enable")) { 
    config_this->motion_udp_raw_enable = atoi(value);
    sprintf (buffer, "Set motion_udp_raw_enable to %s", config_this->motion_udp_raw_enable?"true":"false");
    success = true;
  }
  else if (!strcmp(parameter, "gps_udp_raw_enable")) { 
    config_this->gps_udp_raw_enable = atoi(value);
    sprintf (buffer, "Set gps_udp_raw_enable to %s", config_this->gps_udp_raw_enable?"true":"false");
    success = true;
  }
  else if (!strcmp(parameter, "mpu6050_accel_offset_x")) { 
    config_this->mpu6050_accel_offset_x = atoi(value);
    sprintf (buffer, "Set mpu6050_accel_offset_x to %d", config_this->mpu6050_accel_offset_x);
    success = true;
  }  
  else if (!strcmp(parameter, "mpu6050_accel_offset_y")) { 
    config_this->mpu6050_accel_offset_y = atoi(value);
    sprintf (buffer, "Set mpu6050_accel_offset_y to %d", config_this->mpu6050_accel_offset_y);
    success = true;
  }  
  else if (!strcmp(parameter, "mpu6050_accel_offset_z")) { 
    config_this->mpu6050_accel_offset_z = atoi(value);
    sprintf (buffer, "Set mpu6050_accel_offset_z to %d", config_this->mpu6050_accel_offset_z);
    success = true;
  }
  else if (!strcmp(parameter, "mpu6050_accel_sensitivity")) { 
    config_this->mpu6050_accel_sensitivity = atoi(value);
    sprintf (buffer, "Set mpu6050_accel_sensitivity to %d", config_this->mpu6050_accel_sensitivity);
    success = true;
  }
  else if (!strcmp(parameter, "mpu6050_accel_range")) { 
    mpu6050.accel_range = atoi(value);
    mpu6050_set_accel_range (mpu6050.accel_range);
    sprintf (buffer, "Set mpu6050_accel_range to %d", mpu6050.accel_range);
    success = true;
  }
  else if (!strcmp(parameter, "mpu6050_gyro_range")) { 
    mpu6050.gyro_range = atoi(value);
    mpu6050_set_gyro_range (mpu6050.gyro_range);
    sprintf (buffer, "Set mpu6050_gyro_range to %d", mpu6050.gyro_range);
    success = true;
  }
  else if (!strcmp(parameter, "radio_enabled")) { 
    tm_this->radio_enabled = atoi(value);
    sprintf (buffer, "Set radio_enabled to %s", tm_this->radio_enabled?"true":"false");
    success = true;
  }  
  else if (!strcmp(parameter, "pressure_enabled")) { 
    tm_this->pressure_enabled = atoi(value);
    sprintf (buffer, "Set pressure_enabled to %s", tm_this->pressure_enabled?"true":"false");
    success = true;
  }  
  else if (!strcmp(parameter, "motion_enabled")) { 
    tm_this->motion_enabled = atoi(value);
    sprintf (buffer, "Set motion_enabled to %s", tm_this->motion_enabled?"true":"false");
    success = true;
  }  
  else if (!strcmp(parameter, "gps_enabled")) { 
    tm_this->gps_enabled = atoi(value);
    sprintf (buffer, "Set gps_enabled to %s", tm_this->gps_enabled?"true":"false");
    success = true;
  }  
  else if (!strcmp(parameter, "camera_enabled")) { 
    tm_this->camera_enabled = atoi(value);
    sprintf (buffer, "Set camera_enabled to %s", tm_this->camera_enabled?"true":"false");
    success = true;
  }
  else if (!strcmp(parameter, "wifi_udp_enabled")) { 
    tm_this->wifi_udp_enabled = atoi(value);
    sprintf (buffer, "Set wifi_udp_enabled to %s", tm_this->wifi_udp_enabled?"true":"false");
    success = true;
  }
  else if (!strcmp(parameter, "wifi_yamcs_enabled")) { 
    tm_this->wifi_yamcs_enabled = atoi(value);
    sprintf (buffer, "Set wifi_yamcs_enabled to %s", tm_this->wifi_yamcs_enabled?"true":"false");
    success = true;
  }
  else if (!strcmp(parameter, "fs_enabled")) { 
    tm_this->fs_enabled = atoi(value);
    sprintf (buffer, "Set fs_enabled to %s", tm_this->fs_enabled?"true":"false");
    success = true;
  }
  else if (!strcmp(parameter, "fs_ftp_enabled")) { 
    tm_this->fs_ftp_enabled = atoi(value);
    sprintf (buffer, "Set fs_ftp_enabled to %s", tm_this->fs_ftp_enabled?"true":"false");
    success = true;
  }
  #endif
  #ifdef PLATFORM_ESP32CAM
  else if (!strcmp(parameter, "sd_enable")) { 
    config_this->sd_enable = atoi(value);
    sprintf (buffer, "Set sd_enable to %s", config_this->sd_enable?"true":"false");
    success = true;
  }  
  else if (!strcmp(parameter, "sd_json_enable")) { 
    config_this->sd_json_enable = atoi(value);
    sprintf (buffer, "Set sd_json_enable to %s", config_this->sd_json_enable?"true":"false");
    success = true;
  }  
  else if (!strcmp(parameter, "sd_ccsds_enable")) { 
    config_this->sd_ccsds_enable = atoi(value);
    sprintf (buffer, "Set sd_ccsds_enable to %s", config_this->sd_ccsds_enable?"true":"false");
    success = true;
  }  
  else if (!strcmp(parameter, "sd_image_enable")) { 
    config_this->sd_image_enable = atoi(value);
    sprintf (buffer, "Set sd_image_enable to %s", config_this->sd_image_enable?"true":"false");
    success = true;
  }  
  #endif
  return (success);
}

// WIFI FUNCTIONALITY

bool wifi_setup () {
  bool return_wifi_ap = true;
  bool return_wifi_sta = true;
  bool return_wifi_yamcs = true;
  WiFi.mode(WIFI_AP_STA); 
  if (config_this->wifi_ap_enable) {
    return_wifi_ap = wifi_ap_setup ();
    tm_this->wifi_enabled = true;
  }
  if (config_this->wifi_sta_enable) {
    return_wifi_sta = wifi_sta_setup ();
    tm_this->wifi_enabled = true;
  }
  if (config_this->wifi_yamcs_enable) {
    return_wifi_yamcs = yamcs_tc_setup ();
  }
  return (return_wifi_ap and return_wifi_sta and return_wifi_yamcs);
}

bool wifi_ap_setup () {
  WiFi.softAP(config_network.ap_ssid, config_network.ap_password);
  sprintf (buffer, "Started WiFi access point %s with IP %s", config_network.ap_ssid, WiFi.softAPIP().toString().c_str());
  publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);
  return true;
}

bool wifi_sta_setup () {
  WiFi.begin(config_network.wifi_ssid, config_network.wifi_password);  
  timeClient.begin();
  uint8_t i = 0;
  while (i++ < WIFI_TIMEOUT and WiFi.status() != WL_CONNECTED) {
    delay (1000);
  } 
  if (i >= WIFI_TIMEOUT) {
    sprintf (buffer, "Tired of waiting for connection of %s to %s WiFi, continuing in background", subsystemName[SS_THIS], config_network.wifi_ssid);
    publish_event (STS_THIS, SS_THIS, EVENT_WARNING, buffer);
  }
  else {
    sprintf (buffer, "%s WiFi connected to %s with IP %s", subsystemName[SS_THIS], config_network.wifi_ssid, WiFi.localIP().toString().c_str());
    publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);
    if (config_this->wifi_udp_enable) {
      tm_this->wifi_udp_enabled = true;
    }
    if (config_this->wifi_yamcs_enable) {
      tm_this->wifi_yamcs_enabled = true;
    }
    tm_this->wifi_connected = true;
  } 
  i = 0;
  while (i++ < NTP_TIMEOUT and !time_check()) {
    delay (1000);
  }
  if (i >= NTP_TIMEOUT) {
    sprintf (buffer, "Tired of waiting for NTP time from %s, continuing in background", config_network.ntp_server);
    publish_event (STS_THIS, SS_THIS, EVENT_WARNING, buffer);
  }
  return tm_this->wifi_connected;
}

bool wifi_check () {
  if (WiFi.status() != WL_CONNECTED) {
    if (tm_this->wifi_connected) {
      tm_this->wifi_connected = false;
      tm_this->warn_wifi_connloss = true;
    }
    return false;
  }
  else {
    if (!tm_this->wifi_connected) {
      tm_this->wifi_connected = true;
    }
    return true;
  }
}

bool time_check () {
  if (!tm_this->time_set and timeClient.update()) {
    sprintf (buffer, "Time on %s set via NTP server %s: %s", subsystemName[SS_THIS], config_network.ntp_server, timeClient.getFormattedDate().c_str());
    publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);
    tm_this->time_set = true;
    fs_create_today_dir ();
    return true;
  }
  return false;
}

// TM/TC FUNCTIONALITY (serial/udp/yamcs/fs/sd)

void publish_packet (ccsds_t* ccsds_ptr) { 
  static uint16_t payload_ctr;
  static uint32_t start_millis;
  static uint16_t PID;

  payload_ctr = update_packet (ccsds_ptr); // TODO: payload_ctr is not used, return PID instead?
  PID = get_ccsds_apid (ccsds_ptr) - 42;
  // save CCSDS packet to FS (which will also act as a buffer for replay if active from the start)
  if (routing_fs[PID] and config_this->fs_enable) {
    start_millis = millis();
    publish_fs (ccsds_ptr);
    timer.publish_fs_duration += millis() - start_millis;
  }
  // serial
  if (routing_serial[PID]) {
    start_millis = millis();
    publish_serial (ccsds_ptr);
    timer.publish_serial_duration += millis() - start_millis;
  }
  // Yamcs
  if (routing_yamcs[PID] and config_this->wifi_enable and config_this->wifi_yamcs_enable) {
    start_millis = millis();
    publish_yamcs (ccsds_ptr);
    timer.publish_yamcs_duration += millis() - start_millis;
  }
  // UDP
  if (routing_udp[PID] and config_this->wifi_enable and config_this->wifi_udp_enable) {
    start_millis = millis();
    publish_udp (ccsds_ptr);
    timer.publish_udp_duration += millis() - start_millis;
  }
  // radio
  #ifdef PLATFORM_ESP32
  if (tm_this->radio_enabled and PID == TM_RADIO) {
    publish_radio ();
  }
  #endif
  // SD-card
  #ifdef PLATFORM_ESP32CAM
  if (routing_sd_json[PID] and config_this->sd_enable and config_this->sd_json_enable) {
    publish_sd_json (ccsds_ptr);
  }
  if (routing_sd_ccsds[PID] and config_this->sd_enable and config_this->sd_ccsds_enable) {
    publish_sd_ccsds (ccsds_ptr);
  }
  #endif
  reset_packet (ccsds_ptr);
}

void publish_event (uint16_t PID, uint8_t subsystem, uint8_t event_type, const char* event_message) {
  switch (event_type) {
    case EVENT_ERROR:    tm_this->error_ctr++; break;  
    case EVENT_WARNING:  tm_this->warning_ctr++; break;  
    case EVENT_CMD_RESP: tm_this->tc_exec_ctr++; break; 
    case EVENT_CMD_FAIL: tm_this->tc_fail_ctr++; break; 
  }  
  switch (PID) {
    case STS_ESP32:      sts_esp32.subsystem = subsystem;
                         sts_esp32.type = event_type;
                         strcpy (sts_esp32.message, event_message);
                         set_ccsds_len ((ccsds_t*)&sts_esp32, strlen (event_message) + 7);
                         publish_packet ((ccsds_t*)&sts_esp32);
                         break; 
    case STS_ESP32CAM:   sts_esp32cam.subsystem = subsystem;
                         sts_esp32cam.type = event_type;
                         strcpy (sts_esp32cam.message, event_message);
                         set_ccsds_len ((ccsds_t*)&sts_esp32cam, strlen (event_message) + 7);
                         publish_packet ((ccsds_t*)&sts_esp32cam);
                         break;
  }
}

bool publish_fs (ccsds_t* ccsds_ptr) {
  uint16_t PID = get_ccsds_apid (ccsds_ptr) - 42;
  uint16_t payload_ctr = get_ccsds_ctr (ccsds_ptr); 
  uint16_t ccsds_len = get_ccsds_len (ccsds_ptr);
  
  sprintf (path_buffer, "%s/PID_%u.raw", var_this->today_dir, PID);
  File fs_ccsds = LITTLEFS.open(path_buffer, FILE_APPEND);
  if (!fs_ccsds) {
    sprintf (buffer, "Failed to open '%s' on FS in append mode; disabling FS", path_buffer);
    publish_event (STS_THIS, SS_THIS, EVENT_ERROR, buffer);
    tm_this->fs_enabled = false;
    tm_this->err_fs_dataloss = true;
    return false;
  }
  else {
    tm_this->fs_enabled = true;
    tm_this->fs_active = true;
    fs_ccsds.write ((const uint8_t*)ccsds_ptr, ccsds_len);
    if (PID == STS_ESP32 or PID == STS_ESP32CAM or PID == TC_ESP32 or PID == TC_ESP32CAM) {
      // variable length packet, need to zero-fill to fixed even length when stored
      for (uint16_t i=0; i < JSON_MAX_SIZE + (JSON_MAX_SIZE%2) + 6 + sizeof(ccsds_hdr_t) - ccsds_len; i++) {
        fs_ccsds.write ('\0');
      }
    }
    else if (ccsds_len % 2) {
      // uneven packet length, add one \0 to reach even packet length on file
      fs_ccsds.write ('\0');
    }      
    tm_this->fs_rate++;
    var_this->fs_last_packet[PID] = payload_ctr;  
    fs_ccsds.close();
    return true;
  }
}

bool publish_serial (ccsds_t* ccsds_ptr) {
  uint16_t PID = get_ccsds_apid (ccsds_ptr) - 42;
  uint16_t payload_ctr = get_ccsds_ctr (ccsds_ptr); 
  uint16_t ccsds_len = get_ccsds_len (ccsds_ptr);
  if (tm_this->serial_connected) {
    if (payload_ctr > var_this->serial_last_packet[PID] + 1 and tm_this->fs_enabled and routing_fs[PID]) {
      // replay batch of unsent messages (non-realtime)
      sprintf (path_buffer, "%s/PID_%u.raw", var_this->today_dir, PID);
      File fs_ccsds = LITTLEFS.open(path_buffer, FILE_READ);
      if (!fs_ccsds) {
        tm_this->serial_buffer = 0;
        var_this->serial_last_packet[PID] = -1;
        tm_this->err_serial_dataloss = true;
        sprintf (buffer, "Failed to open '%s' on FS for serial replay; disabling replay for PID %u", path_buffer, PID);
        publish_event (STS_THIS, SS_THIS, EVENT_ERROR, buffer);
        return false;
      }
      else {
        tm_this->serial_buffer++; // account for the triggering packet that was added to the queue
        if (PID == STS_ESP32 or PID == STS_ESP32CAM or PID == TC_ESP32 or PID == TC_ESP32CAM) {
          ccsds_len = JSON_MAX_SIZE + (JSON_MAX_SIZE%2) + 6 + sizeof(ccsds_hdr_t);
        }
        else if (ccsds_len % 2) {
          ccsds_len++;
        }
        uint8_t batch_count = 0;
        tm_this->fs_active = true;
        fs_ccsds.seek(ccsds_len * var_this->serial_last_packet[PID]);
        while (payload_ctr > var_this->serial_last_packet[PID] and batch_count++ < BUFFER_RELEASE_BATCH_SIZE) {
          fs_ccsds.read((uint8_t*)ccsds_ptr, (size_t)ccsds_len);
          if (get_ccsds_packet_ctr(ccsds_ptr) == ++var_this->serial_last_packet[PID]) {
            switch ((uint8_t)config_this->serial_format) {
              case ENC_JSON:  build_json_str ((char*)&buffer, PID, ccsds_ptr);
                              Serial.println (String("[") + get_ccsds_millis (ccsds_ptr) + "] " + pidName[PID] + " " + buffer);
                              break;
              case ENC_CCSDS: Serial.write ((const uint8_t*)ccsds_ptr, get_ccsds_len (ccsds_ptr) + sizeof(ccsds_hdr_t));
                              Serial.println ();
                              break;
            }    
            tm_this->serial_out_rate++;
            tm_this->serial_buffer--;
          }
          else {
            tm_this->serial_buffer = 0;
            uint16_t expected_packet = var_this->serial_last_packet[PID];
            var_this->serial_last_packet[PID] = -1;
            fs_ccsds.close();
          	sprintf (buffer, "Stopped replay after getting packet %u instead of %u when replaying %s from FS for serial replay", get_ccsds_packet_ctr(ccsds_ptr), expected_packet, pidName[PID]);
            publish_event (STS_THIS, SS_THIS, EVENT_ERROR, buffer);
            return false;
          }
        }
        fs_ccsds.close();
        return true;
      }
    }
    else {
      // write the latest message (real-time)
      switch ((uint8_t)config_this->serial_format) {
        case ENC_JSON:  build_json_str ((char*)&buffer, PID, ccsds_ptr);
                        Serial.println (String("[") + get_ccsds_millis (ccsds_ptr) + "] " + pidName[PID] + " " + buffer);
                        break;
        case ENC_CCSDS: Serial.write ((const uint8_t*)ccsds_ptr, ccsds_len);
                        Serial.println ();
                        break;
      }
      tm_this->serial_out_rate++;
      var_this->serial_last_packet[PID] = payload_ctr;
      return true;
    }
  }
  else {
    tm_this->serial_buffer++;
    return false;
  }  
}
  
bool publish_yamcs (ccsds_t* ccsds_ptr) { 
  uint16_t PID = get_ccsds_apid (ccsds_ptr) - 42;
  uint16_t payload_ctr = get_ccsds_ctr (ccsds_ptr); 
  uint16_t ccsds_len = get_ccsds_len (ccsds_ptr);
  if (tm_this->wifi_connected) {
    if (payload_ctr > var_this->yamcs_last_packet[PID] + 1 and tm_this->wifi_yamcs_enabled and routing_fs[PID]) {
      // replay batch of unsent messages (non-realtime)
      sprintf (path_buffer, "%s/PID_%u.raw", var_this->today_dir, PID);
      File fs_ccsds = LITTLEFS.open(path_buffer, FILE_READ);
      if (!fs_ccsds) {
        tm_this->yamcs_buffer = 0;
        var_this->yamcs_last_packet[PID] = -1;
        tm_this->err_yamcs_dataloss = true;
        sprintf (buffer, "Failed to open '%s' on FS for Yamcs replay; disabling replay for PID %u", path_buffer, PID);
        publish_event (STS_THIS, SS_THIS, EVENT_ERROR, buffer);        
        return false;
      }
      else {
        tm_this->yamcs_buffer++; // account for the triggering packet that was added to the queue
        if (PID == STS_ESP32 or PID == STS_ESP32CAM or PID == TC_ESP32 or PID == TC_ESP32CAM) {
          ccsds_len = JSON_MAX_SIZE + (JSON_MAX_SIZE%2) + 6 + sizeof(ccsds_hdr_t);
        }
        else if (ccsds_len % 2) {
          ccsds_len++;
        }
        uint8_t batch_count = 0;
        tm_this->fs_active = true;
        fs_ccsds.seek(ccsds_len * var_this->yamcs_last_packet[PID]);         
        while (payload_ctr > var_this->yamcs_last_packet[PID] and batch_count++ < BUFFER_RELEASE_BATCH_SIZE) {
          fs_ccsds.read((uint8_t*)ccsds_ptr, (size_t)ccsds_len); 
          if (get_ccsds_packet_ctr(ccsds_ptr) == ++var_this->yamcs_last_packet[PID]) {
            wifiUDP.beginPacket(config_network.yamcs_server, config_network.yamcs_tm_port);
            wifiUDP.write ((const uint8_t*)ccsds_ptr, get_ccsds_len (ccsds_ptr) + sizeof(ccsds_hdr_t));
            wifiUDP.endPacket();
            tm_this->yamcs_rate++;
            tm_this->yamcs_buffer--; 
          }
          else {
            tm_this->yamcs_buffer = 0;
            uint16_t expected_packet = var_this->yamcs_last_packet[PID];
            var_this->yamcs_last_packet[PID] = -1;
            fs_ccsds.close();
            sprintf (buffer, "Stopped replay after getting packet %u instead of %u when replaying %s from FS for yamcs replay", get_ccsds_packet_ctr(ccsds_ptr), expected_packet, pidName[PID]);
            publish_event (STS_THIS, SS_THIS, EVENT_ERROR, buffer);
            return false;
          }
        }
        fs_ccsds.close();
        return true;
      }
    }
    else {
      // write the latest message (real-time)
      wifiUDP.beginPacket(config_network.yamcs_server, config_network.yamcs_tm_port);
      wifiUDP.write ((const uint8_t*)ccsds_ptr, ccsds_len);
      wifiUDP.endPacket();            
      tm_this->yamcs_rate++;
      var_this->yamcs_last_packet[PID] = payload_ctr;
      return true;
    }
  }
  else {
    tm_this->yamcs_buffer++;
    return false;
  }
}

bool publish_udp (ccsds_t* ccsds_ptr) { 
  uint16_t PID = get_ccsds_apid (ccsds_ptr) - 42;
  if (tm_this->wifi_connected) {     
    build_json_str ((char*)&buffer, PID, ccsds_ptr);
    wifiUDP.beginPacket(config_network.udp_server, config_network.udp_port);
    wifiUDP.println (String("[") + get_ccsds_millis (ccsds_ptr) + "] " + pidName[PID] + " " + buffer);
    wifiUDP.endPacket();   
    tm_this->udp_rate++;
    return true;
  }
  else {
    return false;
  }
}

bool publish_udp_text (const char* message) { 
  if (tm_this->wifi_connected) {
    wifiUDP.beginPacket(config_network.udp_server, config_network.udp_port);
    wifiUDP.println (message);
    wifiUDP.endPacket();   
    tm_this->udp_rate++;
    return true;
  }
  else {
    return false;
  }
}

#ifdef PLATFORM_ESP32CAM
bool publish_sd_ccsds (ccsds_t* ccsds_ptr) {
  sprintf (path_buffer, "%s/PID_%u.raw", var_this->today_dir, PID);
  File sd_ccsds = SD_MMC.open(path_buffer, FILE_APPEND);
  if (!sd_ccsds) {
    sprintf (buffer, "Failed to open '%s' on SD in append mode; disabling SD", path_buffer);
    publish_event (STS_THIS, SS_THIS, EVENT_ERROR, buffer);
    tm_this->sd_enabled = false;
    tm_this->err_sd_dataloss = true;
    return false;
  }
  else {
    tm_this->sd_ccsds_enabled = true;
    tm_this->sd_active = true;
    sd_ccsds.write ((const uint8_t*)ccsds_ptr, ccsds_len);
    if (PID == STS_ESP32 or PID == STS_ESP32CAM or PID == TC_ESP32 or PID == TC_ESP32CAM) {
      // variable length packet, need to zero-fill to fixed even length when stored
      for (uint16_t i=0; i < JSON_MAX_SIZE + (JSON_MAX_SIZE%2) + 6 + sizeof(ccsds_hdr_t) - ccsds_len; i++) {
        sd_ccsds.write ('\0');
      }
    }
    else if (ccsds_len % 2) {
      // uneven packet length, add one \0 to reach even packet length on file
      sd_ccsds.write ('\0');
    }      
    tm_this->sd_rate++;
    var_this->sd_ccsds_last_packet[PID] = payload_ctr;  
    sd_ccsds.close();
    return true;
  }
}

bool publish_sd_json (ccsds_t* ccsds_ptr) {
  sprintf (path_buffer, "%s/json.raw", var_this->today_dir);
  File sd_json = SD_MMC.open(path_buffer, FILE_APPEND);
  if (!sd_json) {
    sprintf (buffer, "Failed to open '%s' on SD in append mode; disabling SD", path_buffer);
    publish_event (STS_THIS, SS_THIS, EVENT_ERROR, buffer);
    tm_this->sd_enabled = false;
    tm_this->err_sd_dataloss = true;
    return false;
  }
  else {
    tm_this->sd_json_enabled = true;
    tm_this->sd_active = true;
    build_json_str (buffer, PID, ccsds_ptr);
    sd_json.println (buffer);
    tm_this->sd_rate++;
    sd_json.close();
    return true;
  }
}
#endif

#ifdef ASYNCUDP
bool yamcs_tc_setup () {
  if (asyncUDP_yamcs_tc.listen(config_network.yamcs_tc_port)) {
    sprintf (buffer, "Listening for CCSDS commands on UDP port %s:%u (async)", WiFi.localIP().toString().c_str(), config_network.yamcs_tc_port);
    publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);
    asyncUDP_yamcs_tc.onPacket([](AsyncUDPPacket packet) {  
      memcpy (&ccsds_tc_buffer, packet.data(), packet.length());
      parse_ccsds ((ccsds_t*)&ccsds_tc_buffer);
    });
  }
  else {
    sprintf (buffer, "Fail to listen for commands on UDP port %s:%u (async)", WiFi.localIP().toString().c_str(), config_network.yamcs_tc_port);
    publish_event (STS_THIS, SS_THIS, EVENT_ERROR, buffer);  
  }
}
#endif

#ifndef ASYNCUDP
bool yamcs_tc_setup () {
  if (wifiUDP_yamcs_tc.begin(WiFi.localIP(), config_network.yamcs_tc_port)) {
    sprintf (buffer, "Listening for CCSDS commands on UDP port %s:%u", WiFi.localIP().toString().c_str(), config_network.yamcs_tc_port);
    publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);
  }
  else {
    sprintf (buffer, "Fail to listen for commands on UDP port %s:%u", WiFi.localIP().toString().c_str(), config_network.yamcs_tc_port);
    publish_event (STS_THIS, SS_THIS, EVENT_ERROR, buffer);  
  }
}

bool yamcs_tc_check () {
  if (wifiUDP_yamcs_tc.parsePacket()) {
    wifiUDP_yamcs_tc.read((char*)&ccsds_tc_buffer, JSON_MAX_SIZE);
    parse_ccsds ((ccsds_t*)&ccsds_tc_buffer);
  }
}
#endif

uint16_t update_packet (ccsds_t* ccsds_ptr) {
  static uint16_t packet_ctr;
  ((ccsds_hdr_t*)ccsds_ptr)->seq_ctr_L++;
  if (((ccsds_hdr_t*)ccsds_ptr)->seq_ctr_L == 0) {
    ((ccsds_hdr_t*)ccsds_ptr)->seq_ctr_H++;
  }
  switch (get_ccsds_apid (ccsds_ptr) - 42) {
    #ifdef PLATFORM_ESP32
    case STS_ESP32:      sts_esp32.millis = millis();
                         packet_ctr = ++sts_esp32.packet_ctr;
                         break;                
    case TM_ESP32:       esp32.millis = millis();
                         packet_ctr = ++esp32.packet_ctr; 
                         esp32.mem_free = ESP.getFreeHeap()/1024;
                         esp32.fs_free = fs_free ();
                         // compensate for this packet being prepared before it is actually sent (anticipating a successful send)
                         if (routing_fs[TM_ESP32] and config_this->fs_enable) {
                           esp32.fs_rate++;
                         }
                         if (routing_serial[TM_ESP32]) {
                           esp32.serial_out_rate++;
                         }
                         if (routing_yamcs[TM_ESP32] and config_this->wifi_enable and config_this->wifi_yamcs_enable) {
                           esp32.yamcs_rate++;
                         }
                         if (routing_udp[TM_ESP32] and config_this->wifi_enable and config_this->wifi_udp_enable) {
                           esp32.udp_rate++;
                         }
                         break;             
    case TM_GPS:         packet_ctr = ++neo6mv2.packet_ctr;
                         break;
    case TM_MOTION:      packet_ctr = ++mpu6050.packet_ctr;
                         break;
    case TM_PRESSURE:    packet_ctr = ++bmp280.packet_ctr;
                         break;
    case TM_RADIO:       radio.millis = millis(); 
                         packet_ctr = ++radio.packet_ctr;
                         radio.opsmode = esp32.opsmode;
                         radio.error_ctr = min(255, esp32.error_ctr + esp32cam.error_ctr);
                         radio.warning_ctr = min(255, esp32.warning_ctr + esp32cam.warning_ctr);
                         radio.pressure_height = max(0, min(255, (bmp280.height+50)/100));
                         radio.pressure_velocity_v = int8_t((bmp280.velocity_v+((bmp280.velocity_v > 0) - (bmp280.velocity_v < 0))*50)/100);
                         radio.temperature = int8_t((bmp280.temperature+((bmp280.temperature > 0) - (bmp280.temperature < 0))*50)/100);
                         radio.motion_tilt = uint8_t((mpu6050.tilt+50)/100);
                         radio.motion_g = uint8_t((mpu6050.g+50)/100);  
                         radio.motion_a = int8_t((mpu6050.a+((mpu6050.a > 0) - (mpu6050.a < 0))*50)/100);
                         radio.motion_rpm = int8_t((mpu6050.rpm+((mpu6050.rpm > 0) - (mpu6050.rpm < 0))*50)/100); 
                         radio.gps_satellites = neo6mv2.satellites;
                         radio.gps_velocity_v = int8_t(-(neo6mv2.v_down+((neo6mv2.v_down > 0) - (neo6mv2.v_down < 0))*50)/100);
                         radio.gps_velocity = uint8_t(sqrt (neo6mv2.v_north*neo6mv2.v_north + neo6mv2.v_east*neo6mv2.v_east + neo6mv2.v_down*neo6mv2.v_down) / 1000000);   
                         radio.gps_height = max(0, min(255, (neo6mv2.z+50)/100));
                         radio.camera_image_ctr = ov2640.packet_ctr;
                         radio.esp32_serial_connected = esp32.serial_connected;
                         radio.esp32_wifi_connected = esp32.wifi_connected;
                         radio.esp32_warn_serial_connloss = esp32.warn_serial_connloss;
                         radio.esp32_warn_wifi_connloss = esp32.warn_wifi_connloss;
                         radio.esp32_err_serial_dataloss = esp32.err_serial_dataloss;
                         radio.esp32_err_yamcs_dataloss = esp32.err_yamcs_dataloss;
                         radio.esp32_err_fs_dataloss = esp32.err_fs_dataloss;
                         radio.separation_sts = esp32.separation_sts;
                         radio.esp32cam_serial_connected = esp32cam.serial_connected;
                         radio.esp32cam_wifi_connected = esp32cam.wifi_connected;
                         radio.esp32cam_warn_serial_connloss = esp32cam.warn_serial_connloss;
                         radio.esp32cam_warn_wifi_connloss = esp32cam.warn_wifi_connloss;
                         radio.esp32cam_err_serial_dataloss = esp32cam.err_serial_dataloss;
                         radio.esp32cam_err_yamcs_dataloss = esp32cam.err_yamcs_dataloss;
                         radio.esp32cam_err_fs_dataloss = esp32cam.err_fs_dataloss;
                         radio.esp32cam_err_sd_dataloss = esp32cam.err_sd_dataloss;
                         break;
    case TM_TIMER:       packet_ctr = ++timer.packet_ctr;
                         timer.idle_duration = max(0, 1000 - timer.radio_duration - timer.pressure_duration - timer.motion_duration - timer.gps_duration - timer.esp32cam_duration - timer.serial_duration - timer.ota_duration - timer.ftp_duration - timer.wifi_duration - timer.tc_duration);
                         break;
    case TC_ESP32CAM:    // do nothing
                         break;
    #endif
    #ifdef PLATFORM_ESP32CAM
    case STS_ESP32CAM:   sts_esp32cam.millis = millis();
                         packet_ctr = ++sts_esp32cam.packet_ctr; 
                         break;
    case TM_ESP32CAM:    esp32cam.millis = millis();
                         packet_ctr = ++esp32cam.packet_ctr; 
                         esp32cam.mem_free = ESP.getFreeHeap()/1024;
                         esp32cam.fs_free = fs_free();
                         esp32cam.sd_free = sd_free();
                         // compensate for this packet being prepared before it is actually sent (anticipating a successful send)
                         if (routing_fs[TM_ESP32CAM] and config_this->fs_enable) {
                           esp32cam.fs_rate++;
                         }
                         if (routing_serial[TM_ESP32CAM]) {
                           esp32cam.serial_out_rate++;
                         }
                         if (routing_yamcs[TM_ESP32CAM] and config_this->wifi_enable and config_this->wifi_yamcs_enable) {
                           esp32cam.yamcs_rate++;
                         }
                         if (routing_udp[TM_ESP32CAM] and config_this->wifi_enable and config_this->wifi_udp_enable) {
                           esp32cam.udp_rate++;
                         }
                         if (routing_sd_json[TM_ESP32CAM] and config_this->sd_enable and config_this->sd_json_enable) {
                           esp32cam.sd_json_rate++;
                         }
                         if (routing_sd_ccsds[TM_ESP32CAM] and config_this->sd_enable and config_this->sd_ccsds_enable) {
                           esp32cam.sd_ccsds_rate++;
                         }
                         break;
    case TM_CAMERA:      packet_ctr = ++ov2640.packet_ctr; 
                         break;        
    case TC_ESP32:       // do nothing
                         break;
    #endif
  }
  return (packet_ctr);
}

void reset_packet (ccsds_t* ccsds_ptr) { 
  switch (get_ccsds_apid (ccsds_ptr) - 42) {
    #ifdef PLATFORM_ESP32
    case STS_ESP32:      sts_esp32.message[0] = 0;
                         break;
    case TM_ESP32:       esp32.radio_rate = 0;
                         esp32.pressure_rate = 0;
                         esp32.motion_rate = 0;
                         esp32.gps_rate = 0;
                         esp32.camera_rate = 0;
                         esp32.udp_rate = 0;
                         esp32.yamcs_rate = 0;
                         esp32.serial_out_rate = 0;    
                         esp32.serial_in_rate = 0;  
                         esp32.fs_rate = 0;
                         esp32.warn_serial_connloss = false;
                         esp32.warn_wifi_connloss = false;
                         esp32.err_serial_dataloss = false;
                         esp32.err_yamcs_dataloss = false;    
                         esp32.err_fs_dataloss = false;    
                         esp32.radio_active = false;
                         esp32.pressure_active = false;
                         esp32.motion_active = false;
                         esp32.gps_active = false; 
                         esp32.camera_active = false;
                         esp32.fs_active = false;
                         esp32.ftp_active = false;
                         esp32.ota_enabled = false;
                         break;
    case TM_GPS:         esp32.gps_rate++;
    	                 neo6mv2.status = 8;  // set default to "none"
                         break;
    case TM_MOTION:      esp32.motion_rate++;
                         break;
    case TM_PRESSURE:    esp32.pressure_rate++;
                         break;
    case TM_RADIO:       radio.pressure_active = false;
                         radio.motion_active = false;
                         radio.gps_active = false; 
                         radio.camera_active = false; 
                         esp32.radio_rate++;
                         esp32.radio_active = true;
                         break;
    case TM_TIMER:       timer.radio_duration = 0;
                         timer.pressure_duration = 0;
                         timer.motion_duration = 0;
                         timer.gps_duration = 0;
                         timer.esp32cam_duration = 0;
                         timer.serial_duration = 0;
                         timer.ota_duration = 0;
                         timer.ftp_duration = 0;
                         timer.wifi_duration = 0;
                         timer.tc_duration = 0;
                         timer.idle_duration = 0;
                         timer.publish_fs_duration = 0;
                         timer.publish_serial_duration = 0;
                         timer.publish_yamcs_duration = 0;
                         timer.publish_udp_duration = 0;
                         break;
    case TC_ESP32CAM:    tc_esp32cam.str_parameter[0] = 0;
                         break;
    #endif
    #ifdef PLATFORM_ESP32CAM
    case STS_ESP32CAM:   sts_esp32cam.message[0] = 0;
                         break;
    case TM_ESP32CAM:    esp32cam.udp_rate = 0;
                         esp32cam.yamcs_rate = 0;
                         esp32cam.serial_in_rate = 0;
                         esp32cam.serial_out_rate = 0;
                         esp32cam.fs_rate = 0;
                         esp32cam.sd_json_rate = 0;
                         esp32cam.sd_ccsds_rate = 0;
                         esp32cam.sd_image_rate = 0;
                         esp32cam.camera_image_rate = 0;
                         esp32cam.warn_serial_connloss = false;
                         esp32cam.warn_wifi_connloss = false;
                         esp32cam.err_serial_dataloss = false;
                         esp32cam.err_yamcs_dataloss = false;    
                         esp32cam.err_fs_dataloss = false;    
                         esp32cam.err_sd_dataloss = false;    
                         esp32cam.camera_active = false;
                         esp32cam.fs_active = false;
                         esp32cam.sd_active = false;
                         esp32cam.ftp_active = false;
                         esp32cam.ota_enabled = false;
                         break;
    case TM_CAMERA:      strcpy (ov2640.filename, "");
                         ov2640.filesize = 0;
                         ov2640.wifi_ms = 0;
                         ov2640.sd_ms = 0;
                         ov2640.exposure_ms = 0;
                         esp32.camera_rate++;
                         esp32.camera_active = true;
                         radio.camera_active = true;                            
    case TC_ESP32:       tc_esp32.str_parameter[0] = 0;
                         break;
    #endif
  }
}

// CCSDS FUNCTIONALITY

void ccsds_init () {
  ccsds_init_hdr ((ccsds_t*)&sts_esp32, STS_ESP32, PKT_TM, sizeof (sts_esp32_t));
  ccsds_init_hdr ((ccsds_t*)&sts_esp32cam, STS_ESP32CAM, PKT_TM, sizeof (sts_esp32cam_t));
  ccsds_init_hdr ((ccsds_t*)&esp32, TM_ESP32, PKT_TM, sizeof (tm_esp32_t));
  ccsds_init_hdr ((ccsds_t*)&esp32cam, TM_ESP32CAM, PKT_TM, sizeof (tm_esp32cam_t));
  ccsds_init_hdr ((ccsds_t*)&ov2640, TM_CAMERA, PKT_TM, sizeof (tm_camera_t));
  ccsds_init_hdr ((ccsds_t*)&neo6mv2, TM_GPS, PKT_TM, sizeof (tm_gps_t));
  ccsds_init_hdr ((ccsds_t*)&mpu6050, TM_MOTION, PKT_TM, sizeof (tm_motion_t));
  ccsds_init_hdr ((ccsds_t*)&bmp280, TM_PRESSURE, PKT_TM, sizeof (tm_pressure_t));
  ccsds_init_hdr ((ccsds_t*)&radio, TM_RADIO, PKT_TM, sizeof (tm_radio_t));
  ccsds_init_hdr ((ccsds_t*)&timer, TM_TIMER, PKT_TM, sizeof (tm_timer_t));
  ccsds_init_hdr ((ccsds_t*)&tc_esp32, TC_ESP32, PKT_TC, sizeof (tc_esp32_t));
  ccsds_init_hdr ((ccsds_t*)&tc_esp32cam, TC_ESP32CAM, PKT_TC, sizeof (tc_esp32cam_t));
}

void ccsds_init_hdr (ccsds_t* ccsds_ptr, uint16_t PID, uint8_t pkt_type, uint16_t pkt_len) {
  uint16_t APID = PID + 42;
  uint16_t len = pkt_len - sizeof(ccsds_hdr_t) - 1;
  (ccsds_ptr->ccsds_hdr).version = 0;
  (ccsds_ptr->ccsds_hdr).type = pkt_type;
  (ccsds_ptr->ccsds_hdr).sec_hdr = false;
  (ccsds_ptr->ccsds_hdr).apid_H = (uint8_t)(APID >> 8);
  (ccsds_ptr->ccsds_hdr).apid_L = (uint8_t)APID;
  (ccsds_ptr->ccsds_hdr).seq_flag = 3;
  (ccsds_ptr->ccsds_hdr).pkt_len_H = (uint8_t)(len >> 8);  
  (ccsds_ptr->ccsds_hdr).pkt_len_L = (uint8_t)len;
}

bool valid_ccsds_hdr (ccsds_t* ccsds_ptr, bool pkt_type) {
  if (((ccsds_hdr_t*)ccsds_ptr)->version == 0 and
      ((ccsds_hdr_t*)ccsds_ptr)->type == pkt_type and
      ((ccsds_hdr_t*)ccsds_ptr)->sec_hdr == 0 and
      ((ccsds_hdr_t*)ccsds_ptr)->seq_flag == 3) {
    return true;
  }
  else {
    return false;
  }
}

uint16_t get_ccsds_apid (ccsds_t* ccsds_ptr) {
  return (256*((ccsds_hdr_t*)ccsds_ptr)->apid_H + ((ccsds_hdr_t*)ccsds_ptr)->apid_L);
}

uint16_t get_ccsds_packet_ctr (ccsds_t* ccsds_ptr) { // TODO: needed?
  return (256*((ccsds_hdr_t*)ccsds_ptr)->seq_ctr_H + ((ccsds_hdr_t*)ccsds_ptr)->seq_ctr_L);
}

uint16_t get_ccsds_len (ccsds_t* ccsds_ptr) { // TODO: rename get_ccsds_packet_len
  return (sizeof (ccsds_hdr_t) + 256*((ccsds_hdr_t*)ccsds_ptr)->pkt_len_H + ((ccsds_hdr_t*)ccsds_ptr)->pkt_len_L + 1);
}

void set_ccsds_len (ccsds_t* ccsds_ptr, uint16_t len) { // TODO: does not take hdr into account while get_ccsds_len does ... rename set_ccsds_payload_len
  (ccsds_ptr->ccsds_hdr).pkt_len_H = (uint8_t)((len - 1) >> 8);  
  (ccsds_ptr->ccsds_hdr).pkt_len_L = (uint8_t)(len - 1);
}
  
uint32_t get_ccsds_millis (ccsds_t* ccsds_ptr) {
  return (65536*(uint8_t)*((byte*)ccsds_ptr+8)+256*(uint8_t)*((byte*)ccsds_ptr+7)+(uint8_t)*((byte*)ccsds_ptr+6));
}

uint32_t get_ccsds_ctr (ccsds_t* ccsds_ptr) {
  return (256*(uint8_t)*((byte*)ccsds_ptr+10)+(uint8_t)*((byte*)ccsds_ptr+9));
}

void parse_ccsds (ccsds_t* ccsds_ptr) {     
  if (valid_ccsds_hdr (ccsds_ptr, PKT_TM)) {
    switch (get_ccsds_apid (ccsds_ptr) - 42) {
      case STS_OTHER:      memcpy (sts_other, ccsds_ptr, get_ccsds_len(ccsds_ptr));
                           publish_packet ((ccsds_t*)sts_other);
                           break;
      case TM_OTHER:       memcpy (tm_this, ccsds_ptr, get_ccsds_len(ccsds_ptr));
                           publish_packet ((ccsds_t*)tm_other);
                           break;
      #ifdef PLATFORM_ESP32
      case TM_CAMERA:      memcpy (&ov2640, ccsds_ptr, sizeof(tm_camera_t));
                           publish_packet ((ccsds_t*)&ov2640);
                           break;
      #endif
      #ifdef PLATFORM_ESP32CAM
      case TM_GPS:         memcpy (&neo6mv2, ccsds_ptr, sizeof(tm_gps_t));
                           publish_packet ((ccsds_t*)&neo6mv2);
                           break;
      case TM_MOTION:      memcpy (&mpu6050, ccsds_ptr, sizeof(tm_motion_t));
                           publish_packet ((ccsds_t*)&mpu6050);
                           break;
      case TM_PRESSURE:    memcpy (&bmp280, ccsds_ptr, sizeof(tm_pressure_t));
                           publish_packet ((ccsds_t*)&bmp280);
                           break;
      case TM_RADIO:       memcpy (&radio, ccsds_ptr, sizeof(tm_radio_t));
                           publish_packet ((ccsds_t*)&radio);
                           break;
      case TM_TIMER:       memcpy (&timer, ccsds_ptr, sizeof(tm_timer_t));
                           publish_packet ((ccsds_t*)&timer);
                           break;                           
      #endif
      default:             sprintf (buffer, "Received TM packet with unexpected APID %d", get_ccsds_apid (ccsds_ptr));
                           publish_event (STS_THIS, SS_THIS, EVENT_ERROR, buffer);
                           break;
    }
  }
  else if (valid_ccsds_hdr (ccsds_ptr, PKT_TC)) {
    switch (get_ccsds_apid (ccsds_ptr) - 42) {
      case TC_THIS:        memcpy (tc_this, ccsds_ptr, get_ccsds_len(ccsds_ptr));
                           sprintf (buffer, "Received TC for %s with cmd_id %u", subsystemName[SS_THIS], tc_this->cmd_id);
                           publish_event (STS_THIS, SS_THIS, EVENT_CMD_ACK, buffer);
                           switch (tc_this->cmd_id) {
                             case TC_REBOOT:             cmd_reboot (tc_this->int_parameter);
                                                         break;
                             #ifdef PLATFORM_ESP32
                             case TC_SET_OPSMODE:        cmd_set_opsmode (tc_this->int_parameter);
                                                         break;
                             #endif
                             case TC_LOAD_CONFIG:        cmd_load_config ((const char*)tc_this->str_parameter);
                                                         break;                                                       
                             case TC_LOAD_ROUTING:       cmd_load_routing ((const char*)tc_this->str_parameter); 
                                                         break;
                             case TC_SET_PARAMETER:      cmd_set_parameter (tc_this->str_parameter, (char*)(tc_this->str_parameter + strlen(tc_this->str_parameter) + 1)); 
                                                         break;
                             default:                    sprintf (buffer,  "CCSDS command to %s not understood", subsystemName[SS_THIS]);
                                                         publish_event (STS_THIS, SS_THIS, EVENT_CMD_FAIL, buffer);
                                                         break;
                           }
                           break;
      case TC_OTHER   :    memcpy (tc_other, ccsds_ptr, get_ccsds_len(ccsds_ptr));
                           publish_packet ((ccsds_t*)tc_other);
                           break;
      default:             sprintf (buffer, "Received TC packet with unexpected APID %d", get_ccsds_apid (ccsds_ptr));
                           publish_event (STS_THIS, SS_THIS, EVENT_CMD_FAIL, buffer);
                           break;
    }
  }
  else {
    sprintf (buffer, "Received packet with invalid CCSDS header (%s...)", get_hex_str ((char*)ccsds_ptr, 6).c_str());
    publish_event (STS_THIS, SS_THIS, EVENT_CMD_FAIL, buffer);  
  }
}

// JSON FUNCTIONALITY

void build_json_str (char* json_buffer, uint16_t PID, ccsds_t* ccsds_ptr) { // TODO: remove PID as parameter
  switch (PID) {
    case STS_ESP32:      { 
                           sts_esp32_t* sts_esp32_ptr = (sts_esp32_t*)((byte*)ccsds_ptr + sizeof(ccsds_hdr_t));
                           sprintf (json_buffer, "{\"ctr\":%u,\"type\":\"%s\",\"ss\":\"%s\",\"msg\":\"%s\"}", 
                             sts_esp32_ptr->packet_ctr, eventName[sts_esp32_ptr->type], subsystemName[sts_esp32_ptr->subsystem], sts_esp32_ptr->message);
                         }
                         break;
    case STS_ESP32CAM:   { 
                           sts_esp32cam_t* sts_esp32cam_ptr = (sts_esp32cam_t*)((byte*)ccsds_ptr + sizeof(ccsds_hdr_t));
                           sprintf (json_buffer, "{\"ctr\":%u,\"type\":\"%s\",\"ss\":\"%s\",\"msg\":\"%s\"}", 
                             sts_esp32cam_ptr->packet_ctr, eventName[sts_esp32cam_ptr->type], subsystemName[sts_esp32cam_ptr->subsystem], sts_esp32cam_ptr->message);
                         }
                         break;                  
    case TM_ESP32:       {
                           tm_esp32_t* esp32_ptr = (tm_esp32_t*)((byte*)ccsds_ptr + sizeof(ccsds_hdr_t));
                           sprintf (json_buffer, "{\"ctr\":%u,\"mode\":\"%s\",\"err\":%u,\"warn\":%u,\"tc\":{\"exec\":%u,\"fail\":%u},\"mem\":[%u,%u],\"buf\":[%u,%u],\"inst_rate\":[%u,%u,%u,%u,%u],\"comm_rate\":[%u,%u,%u,%u,%u],\"sep\":%d,\"ena\":\"%d%d%d%d%d%d%d%d%d%d%d\",\"act\":\"%d%d%d%d%d%d%d%d\",\"conn\":{\"up\":\"%d%d\",\"warn\":\"%d%d\",\"err\":\"%d%d%d\"}}", 
                                    esp32_ptr->packet_ctr, modeName[esp32_ptr->opsmode], esp32_ptr->error_ctr, esp32_ptr->warning_ctr, 
                                    esp32_ptr->tc_exec_ctr, esp32_ptr->tc_fail_ctr,
                                    esp32_ptr->mem_free, esp32_ptr->fs_free, 
                                    esp32_ptr->yamcs_buffer, esp32_ptr->serial_buffer, 
                                    esp32_ptr->radio_rate, esp32_ptr->pressure_rate, esp32_ptr->motion_rate, esp32_ptr->gps_rate, esp32_ptr->camera_rate, esp32_ptr->udp_rate, esp32_ptr->yamcs_rate, esp32_ptr->serial_in_rate, esp32_ptr->serial_out_rate, esp32_ptr->fs_rate, 
                                    esp32_ptr->separation_sts,
                                    esp32_ptr->radio_enabled, esp32_ptr->pressure_enabled, esp32_ptr->motion_enabled, esp32_ptr->gps_enabled, esp32_ptr->camera_enabled, esp32_ptr->wifi_enabled, esp32_ptr->wifi_udp_enabled, esp32_ptr->wifi_yamcs_enabled, esp32_ptr->fs_enabled, esp32_ptr->fs_ftp_enabled, esp32_ptr->time_set,
                                    esp32_ptr->radio_active, esp32_ptr->pressure_active, esp32_ptr->motion_active, esp32_ptr->gps_active, esp32_ptr->camera_active, esp32_ptr->fs_active, esp32_ptr->ftp_active, esp32_ptr->ota_enabled,
                                    esp32_ptr->serial_connected, esp32_ptr->wifi_connected, esp32_ptr->warn_serial_connloss, esp32_ptr->warn_wifi_connloss, esp32_ptr->err_serial_dataloss, esp32_ptr->err_yamcs_dataloss, esp32_ptr->err_fs_dataloss); 
                         }
                         break;
    case TM_ESP32CAM:    {
                           tm_esp32cam_t* esp32cam_ptr = (tm_esp32cam_t*)((byte*)ccsds_ptr + sizeof(ccsds_hdr_t));
                           sprintf (json_buffer, "{\"ctr\":%u,\"err\":%u,\"warn\":%u,\"tc\":{\"exec\":%u,\"fail\":%u},\"mem\":[%u,%u,%u],\"buf\":[%u,%u],\"rate\":[%u,%u,%u,%u,%u,%u,%u,%u,%u],\"ena\":\"%d%d%d%d%d%d%d%d%d%d%d%d\",\"act\":\"%d%d%d%d%d\",\"conn\":{\"up\":\"%d%d\",\"warn\":\"%d%d\",\"err\":\"%d%d%d\"}}", 
                                    esp32cam_ptr->packet_ctr, esp32cam_ptr->error_ctr, esp32cam_ptr->warning_ctr, 
                                    esp32cam_ptr->tc_exec_ctr, esp32cam_ptr->tc_fail_ctr,
                                    esp32cam_ptr->mem_free, esp32cam_ptr->fs_free, esp32cam_ptr->sd_free, 
                                    esp32cam_ptr->yamcs_buffer, esp32cam_ptr->serial_buffer, 
                                    esp32cam_ptr->camera_image_rate, esp32cam_ptr->udp_rate, esp32cam_ptr->yamcs_rate, esp32cam_ptr->serial_in_rate, esp32cam_ptr->serial_out_rate, esp32cam_ptr->fs_rate, esp32cam_ptr->sd_json_rate, esp32cam_ptr->sd_ccsds_rate, esp32cam_ptr->sd_image_rate,  
                                    esp32cam_ptr->camera_enabled, esp32cam_ptr->wifi_enabled, esp32cam_ptr->wifi_udp_enabled, esp32cam_ptr->wifi_yamcs_enabled, esp32cam_ptr->wifi_image_enabled, esp32cam_ptr->fs_enabled, esp32cam_ptr->fs_ftp_enabled, esp32cam_ptr->time_set, esp32cam_ptr->sd_enabled, esp32cam_ptr->sd_image_enabled, esp32cam_ptr->sd_json_enabled, esp32cam_ptr->sd_ccsds_enabled,
                                    esp32cam_ptr->camera_active, esp32cam_ptr->fs_active, esp32cam_ptr->sd_active, esp32cam_ptr->ftp_active, esp32cam_ptr->ota_enabled,
                                    esp32cam_ptr->serial_connected, esp32cam_ptr->wifi_connected, esp32cam_ptr->warn_serial_connloss, esp32cam_ptr->warn_wifi_connloss, esp32cam_ptr->err_serial_dataloss, esp32cam_ptr->err_yamcs_dataloss, esp32cam_ptr->err_fs_dataloss); 
                         }
                         break;
    case TM_CAMERA:      {
                           tm_camera_t* ov2640_ptr = (tm_camera_t*)((byte*)ccsds_ptr + sizeof(ccsds_hdr_t));
                           sprintf (json_buffer, "{\"ctr\":%u,\"mode\":\"%s\",\"res\":\"%s\",\"auto_res\":%d,\"file\":\"%s\",\"size\":%u,\"ms\":{\"exp\":%u,\"sd\":%u,\"wifi\":%u}}", 
                                    ov2640_ptr->packet_ctr, cameraModeName[ov2640_ptr->camera_mode], cameraResolutionName[ov2640_ptr->resolution], ov2640_ptr->auto_res, ov2640_ptr->filename, ov2640_ptr->filesize, ov2640_ptr->exposure_ms, ov2640_ptr->sd_ms, ov2640_ptr->wifi_ms);
                         }
                         break;
    case TM_GPS:         {
                           tm_gps_t* neo6mv2_ptr = (tm_gps_t*)((byte*)ccsds_ptr + sizeof(ccsds_hdr_t));
                           *json_buffer++ = '{';
                           json_buffer += sprintf (json_buffer, "\"ctr\":%u,\"sts\":\"%s\",\"sats\":%d", neo6mv2_ptr->packet_ctr, gpsStatusName[neo6mv2_ptr->status], neo6mv2_ptr->satellites);
                           if (neo6mv2_ptr->time_valid) {
                             json_buffer += sprintf (json_buffer, ",\"time\":\"%02d:%02d:%02d.%02d\"", neo6mv2_ptr->hours, neo6mv2_ptr->minutes, neo6mv2_ptr->seconds, neo6mv2_ptr->centiseconds);
                           }
                           if (neo6mv2_ptr->location_valid) {
                             json_buffer += sprintf (json_buffer, ",\"loc\":[%d,%d]", neo6mv2_ptr->latitude, neo6mv2_ptr->longitude);
                           }
                           if (neo6mv2_ptr->altitude_valid) {
                             json_buffer += sprintf (json_buffer, ",\"alt\":%d", neo6mv2_ptr->altitude);
                           }  
                           if (neo6mv2_ptr->location_valid and neo6mv2_ptr->altitude_valid) {
                             json_buffer += sprintf (json_buffer, ",\"zero\":[%d,%d,%d]", neo6mv2_ptr->latitude_zero, neo6mv2_ptr->longitude_zero, neo6mv2_ptr->altitude_zero);
                           }
                           if (neo6mv2_ptr->offset_valid) {
                             json_buffer += sprintf (json_buffer, ",\"xyz\":[%d,%d,%d]", neo6mv2_ptr->x, neo6mv2_ptr->y, neo6mv2_ptr->z);
                           }
                           if (neo6mv2_ptr->speed_valid) {
                             json_buffer += sprintf (json_buffer, ",\"v\":[%d,%d,%d]", neo6mv2_ptr->v_north, neo6mv2_ptr->v_east, neo6mv2_ptr->v_down);
                           }
                           if (neo6mv2_ptr->hdop_valid and neo6mv2_ptr->vdop_valid and neo6mv2_ptr->pdop_valid) {
                             json_buffer += sprintf (json_buffer, ",\"dop\":[%u,%u,%u]", neo6mv2_ptr->milli_hdop, neo6mv2_ptr->milli_vdop, neo6mv2_ptr->milli_pdop);
                           }
                           if (neo6mv2_ptr->error_valid) {
                             json_buffer += sprintf (json_buffer, ",\"err\":[%d,%d,%d]", neo6mv2_ptr->x_err, neo6mv2_ptr->y_err, neo6mv2_ptr->z_err);
                           }
                           json_buffer += sprintf (json_buffer, ",\"valid\":\"%d%d%d%d%d%d%d%d%d\"", neo6mv2_ptr->time_valid, neo6mv2_ptr->location_valid, neo6mv2_ptr->altitude_valid, neo6mv2_ptr->speed_valid, neo6mv2_ptr->hdop_valid, neo6mv2_ptr->vdop_valid, neo6mv2_ptr->pdop_valid, neo6mv2_ptr->error_valid, neo6mv2_ptr->offset_valid);
                           *json_buffer++ = '}';
                           *json_buffer++ = 0;
                         }
                         break;
    case TM_MOTION:      {
                           tm_motion_t* mpu6050_ptr = (tm_motion_t*)((byte*)ccsds_ptr + sizeof(ccsds_hdr_t));
                           sprintf (json_buffer, "{\"ctr\":%u,\"accel\":[%d,%d,%d],\"gyro\":[%d,%d,%d],\"tilt\":%d,\"g\":%d,\"a\":%d,\"rpm\":%d,\"range\":[%u,%u],\"valid\":\"%d%d\"}", 
                                    mpu6050_ptr->packet_ctr, 
                                    mpu6050_ptr->accel_x, mpu6050_ptr->accel_y, mpu6050_ptr->accel_z, 
                                    mpu6050_ptr->gyro_x, mpu6050_ptr->gyro_y, mpu6050_ptr->gyro_z, 
                                    mpu6050_ptr->tilt, mpu6050_ptr->g, mpu6050_ptr->a, mpu6050_ptr->rpm,
                                    mpu6050_ptr->accel_range, mpu6050_ptr->gyro_range, mpu6050_ptr->accel_valid, mpu6050_ptr->gyro_valid); 
                         }
                         break;
    case TM_PRESSURE:    {
                           tm_pressure_t* bmp280_ptr = (tm_pressure_t*)((byte*)ccsds_ptr + sizeof(ccsds_hdr_t));
                           sprintf (json_buffer, "{\"ctr\":%u,\"p\":%.2f,\"p0\":%.2f,\"T\":%.2f,\"h\":%.2f,\"v_v\":%.2f,\"valid\":%u}", 
                                    bmp280_ptr->packet_ctr, bmp280_ptr->pressure, bmp280_ptr->zero_level_pressure, bmp280_ptr->temperature, bmp280_ptr->height, bmp280_ptr->velocity_v, bmp280_ptr->height_valid);
                         }
                         break;
    case TM_RADIO:       {
                           tm_radio_t* radio_ptr = (tm_radio_t*)((byte*)ccsds_ptr + sizeof(ccsds_hdr_t));
                           sprintf (json_buffer, "{\"ctr\":%u,\"data\":\"%s\"}", radio_ptr->packet_ctr, get_hex_str ((char*)&radio, sizeof(tm_radio_t)));
                         }
                         break;
    case TM_TIMER:       {
                           tm_timer_t* timer_ptr = (tm_timer_t*)((byte*)ccsds_ptr + sizeof(ccsds_hdr_t));
                           sprintf (json_buffer, "{\"ctr\":%u,\"idle\":%u,\"instr\":[%u,%u,%u,%u,%u],\"fun\":[%u,%u,%u,%u,%u],\"pub\":[%u,%u,%u,%u]}", 
                                    timer_ptr->packet_ctr, 
                                    timer_ptr->idle_duration,
                                    timer_ptr->radio_duration, timer_ptr->pressure_duration, timer_ptr->motion_duration, timer_ptr->gps_duration, timer_ptr->esp32cam_duration,
                                    timer_ptr->serial_duration, timer_ptr->ota_duration, timer_ptr->ftp_duration, timer_ptr->wifi_duration, timer_ptr->tc_duration,
                                    timer_ptr->publish_fs_duration, timer_ptr->publish_serial_duration, timer_ptr->publish_yamcs_duration, timer_ptr->publish_udp_duration);
                         }
                         break;
    case TC_ESP32:       {
                           tc_esp32_t* tc_esp32_ptr = (tc_esp32_t*)((byte*)ccsds_ptr + sizeof(ccsds_hdr_t));
                           *json_buffer++ = '{';
                           json_buffer += sprintf (json_buffer, "\"cmd\":\"%s\"", tcName[tc_esp32_ptr->cmd_id-42]);
                           if (tc_esp32_ptr->cmd_id-42 == TC_REBOOT or tc_esp32_ptr->cmd_id-42 == TC_SET_OPSMODE) {
                             json_buffer += sprintf (json_buffer, ",\"int_val\":\"%u\"", tc_esp32_ptr->int_parameter);
                           }
                           if (tc_esp32_ptr->cmd_id-42 == TC_LOAD_CONFIG or tc_esp32_ptr->cmd_id-42 == TC_LOAD_ROUTING) {
                             json_buffer += sprintf (json_buffer, ",\"str_val\":\"%s\"", tc_esp32_ptr->str_parameter);
                           }
                           if (tc_esp32_ptr->cmd_id-42 == TC_SET_PARAMETER) {
                             json_buffer += sprintf (json_buffer, ",\"param\":\"%s\",\"value\":\"%s\"", tc_esp32_ptr->str_parameter, (char*)(&tc_esp32_ptr->str_parameter+strlen(tc_esp32_ptr->str_parameter)+1)); // TODO: value
                           }
                           *json_buffer++ = '}';
                           *json_buffer++ = 0;
                         }
                         break;
    case TC_ESP32CAM:    {
                           tc_esp32cam_t* tc_esp32cam_ptr = (tc_esp32cam_t*)((byte*)ccsds_ptr + sizeof(ccsds_hdr_t));
                           *json_buffer++ = '{';
                           json_buffer += sprintf (json_buffer, "\"cmd\":\"%s\"", tcName[tc_esp32cam_ptr->cmd_id-42]);
                           if (tc_esp32cam_ptr->cmd_id-42 == TC_REBOOT or tc_esp32cam_ptr->cmd_id-42 == TC_SET_OPSMODE) {
                             json_buffer += sprintf (json_buffer, ",\"int_val\":\"%u\"", tc_esp32cam_ptr->int_parameter);
                           }
                           if (tc_esp32cam_ptr->cmd_id-42 == TC_LOAD_CONFIG or tc_esp32cam_ptr->cmd_id-42 == TC_LOAD_ROUTING) {
                             json_buffer += sprintf (json_buffer, ",\"str_val\":\"%s\"", tc_esp32cam_ptr->str_parameter);
                           }
                           if (tc_esp32cam_ptr->cmd_id-42 == TC_SET_PARAMETER) {
                             json_buffer += sprintf (json_buffer, ",\"param\":\"%s\",\"value\":\"%s\"", tc_esp32cam_ptr->str_parameter, tc_esp32cam_ptr->str_parameter); // TODO: value
                           }
                           *json_buffer++ = '}';
                           *json_buffer++ = 0;
                         } 
                         break;  
  }
}

bool parse_json (const char* json_string) { // TODO: maybe add millis as parameter somehow, take from str or take from current clock?
  static StaticJsonDocument<JSON_MAX_SIZE> obj;
  char * json_str = strchr (json_string, '{');
  char * tag_str = strchr (json_string, ' ') + 1;
  char * separator = strchr (tag_str, ' ');
  separator[0] = 0x00;
  deserializeJson(obj, (const char*)json_str);
  switch (id_of (tag_str, sizeof(pidName[0]), (char*)pidName, sizeof(pidName))) {
    #ifdef PLATFORM_ESP32
    case STS_ESP32CAM:  // {\"ctr\":%u,\"type\":\"%s\",\"ss\":\"%s\",\"msg\":\"%s\"}
                        sts_esp32cam.packet_ctr = obj["ctr"];
                        sts_esp32cam.type = id_of ((const char*)obj["type"], sizeof(eventName[0]), (char*)&eventName, sizeof(eventName));
                        sts_esp32cam.subsystem = id_of ((const char*)obj["ss"], sizeof(subsystemName[0]), (char*)&subsystemName, sizeof(subsystemName));
                        strcpy (sts_esp32cam.message, obj["msg"]);
                        set_ccsds_len ((ccsds_t*)&sts_esp32cam, strlen (sts_esp32cam.message) + 7);
                        publish_packet ((ccsds_t*)&sts_esp32cam);
                        break;                        
    case TM_ESP32CAM:   // {\"ctr\":%u,\"err\":%u,\"warn\":%u,\"tc\":{\"exec\":%u,\"fail\":%u},\"mem\":[%u,%u,%u],\"buf\":[%u,%u],\"rate\":[%u,%u,%u,%u,%u,%u,%u,%u,%u],\"ena\":\"%d%d%d%d%d%d%d%d%d%d%d%d\",\"act\":\"%d%d%d%d%d\",\"conn\":{\"up\":\"%d%d\",\"warn\":\"%d%d\",\"err\":\"%d%d%d\"}}
                        esp32cam.packet_ctr = obj["ctr"];
                        esp32cam.error_ctr = obj["err"];
                        esp32cam.warning_ctr = obj["warn"];
                        esp32cam.tc_exec_ctr = obj["tc"]["exec"];
                        esp32cam.tc_fail_ctr = obj["tc"]["fail"];
                        esp32cam.camera_image_rate = obj["rate"][0];
                        esp32cam.udp_rate = obj["rate"][1];
                        esp32cam.yamcs_rate = obj["rate"][2];
                        esp32cam.serial_in_rate = obj["rate"][3];
                        esp32cam.serial_out_rate = obj["rate"][4];
                        esp32cam.fs_rate = obj["rate"][5];
                        esp32cam.sd_json_rate = obj["rate"][6];
                        esp32cam.sd_ccsds_rate = obj["rate"][7];
                        esp32cam.sd_image_rate = obj["rate"][8];
                        esp32cam.yamcs_buffer = obj["buf"][0];
                        esp32cam.serial_buffer = obj["buf"][1];
                        esp32cam.mem_free = obj["mem"][0];
                        esp32cam.fs_free = obj["mem"][1];
                        esp32cam.sd_free = obj["mem"][2];
                        esp32cam.camera_enabled = (obj["ena"][0] == '1')?1:0;  
                        esp32cam.wifi_enabled = (obj["ena"][1] == '1')?1:0;     
                        esp32cam.wifi_udp_enabled = (obj["ena"][2] == '1')?1:0;      
                        esp32cam.wifi_yamcs_enabled = (obj["ena"][3] == '1')?1:0;    
                        esp32cam.wifi_image_enabled = (obj["ena"][4] == '1')?1:0;    
                        esp32cam.fs_enabled = (obj["ena"][5] == '1')?1:0; 
                        esp32cam.fs_ftp_enabled = (obj["ena"][6] == '1')?1:0; 
                        esp32cam.time_set = (obj["ena"][7] == '1')?1:0;  
                        esp32cam.sd_enabled = (obj["ena"][8] == '1')?1:0; 
                        esp32cam.sd_image_enabled = (obj["ena"][9] == '1')?1:0; 
                        esp32cam.sd_json_enabled = (obj["ena"][10] == '1')?1:0;  
                        esp32cam.sd_ccsds_enabled = (obj["ena"][11] == '1')?1:0; 
                        esp32cam.serial_connected = (obj["conn"]["up"][0] == '1')?1:0;
                        esp32cam.wifi_connected = (obj["conn"]["up"][1] == '1')?1:0; 
                        esp32cam.warn_serial_connloss = (obj["conn"]["warn"][0] == '1')?1:0; 
                        esp32cam.warn_wifi_connloss = (obj["conn"]["warn"][1] == '1')?1:0; 
                        esp32cam.err_serial_dataloss = (obj["conn"]["err"][0] == '1')?1:0; 
                        esp32cam.err_yamcs_dataloss = (obj["conn"]["err"][1] == '1')?1:0; 
                        esp32cam.err_fs_dataloss = (obj["conn"]["err"][2] == '1')?1:0; 
                        esp32cam.err_sd_dataloss = (obj["conn"]["err"][3] == '1')?1:0; 
                        esp32cam.camera_active = (obj["act"][0] == '1')?1:0; 
                        esp32cam.fs_active = (obj["act"][1] == '1')?1:0;           
                        esp32cam.sd_active = (obj["act"][2] == '1')?1:0;           
                        esp32cam.ftp_active = (obj["act"][3] == '1')?1:0;           
                        esp32cam.ota_enabled = (obj["act"][4] == '1')?1:0;           
                        publish_packet ((ccsds_t*)&esp32cam);
                        break;                                                                                                                          
    case TM_CAMERA:     // {\"ctr\":%u,\"mode\":\"%s\",\"res\":\"%s\",\"auto_res\":%d,\"file\":\"%s\",\"size\":%u,\"ms\":{\"exp\":%u,\"sd\":%u,\"wifi\":%u}}
                        ov2640.packet_ctr = obj["ctr"];
                        ov2640.camera_mode = id_of(obj["mode"], sizeof(cameraModeName[0]), (char*)&cameraModeName, sizeof(cameraModeName));
                        ov2640.resolution = id_of(obj["res"], sizeof(cameraResolutionName[0]), (char*)cameraResolutionName, sizeof(cameraResolutionName));
                        ov2640.auto_res = obj["auto_res"];
                        strcpy (ov2640.filename, obj["filename"]);
                        ov2640.filesize = obj["size"];
                        ov2640.exposure_ms = obj["ms"]["exp"];
                        ov2640.sd_ms = obj["ms"]["sd"];
                        ov2640.wifi_ms = obj["ms"]["wifi"];
                        publish_packet ((ccsds_t*)&ov2640);
                        break;
    case TC_ESP32:      // execute command
                        if (!strcmp(obj["cmd"], "reboot")) {
                          // {"cmd":"reboot","subsystem":0|1|2}
                          cmd_reboot (atoi(obj["subsystem"]));
                        }
                        else if (!strcmp(obj["cmd"], "set_opsmode")) {
                          // {"cmd":"set_opsmode","opsmode":"checkout|ready|static"}
                          cmd_set_opsmode (id_of (obj["opsmode"], sizeof(modeName[0]), (char*)modeName, sizeof(modeName)));
                        }
                        else if (!strcmp(obj["cmd"], "load_config")) {
                          // {"cmd":"load_config","filename":"xxxxxx.cfg"}
                          cmd_load_config (obj["filename"]);
                        }
                        else if (!strcmp(obj["cmd"], "load_routing")) {
                          // {"cmd":"load_routing","filename":"xxxxxx.cfg"}
                          cmd_load_routing (obj["filename"]);
                        }
                        else if (!strcmp(obj["cmd"], "set_parameter")) {
                          // {"cmd":"set_parameter","parameter":"xxxxxx","value":"xxxxxx"}
                          cmd_set_parameter (obj["parameter"], obj["value"]);
                        }
                        else {
                          publish_event (STS_THIS, SS_THIS, EVENT_ERROR, "JSON command to ESP32 not understood");
                          return false;
                        }
                        break;
    case TC_ESP32CAM:   // forward command
                        tc_esp32cam.cmd_id = id_of (obj["cmd"], sizeof(tcName[0]), (char*)tcName, sizeof(tcName));
                        switch (tc_esp32cam.cmd_id) {
                          case TC_REBOOT:        // {"cmd":"reboot","subsystem":0|1|2}
                                                 tc_esp32cam.int_parameter = atoi (obj["subsystem"]);
                                                 tc_esp32cam.str_parameter[0] = 0;
                                                 set_ccsds_len ((ccsds_t*)&tc_esp32cam, 7);
                                                 break;
                          case TC_LOAD_CONFIG:   // {"cmd":"load_config","filename":"xxxxxx.cfg"}
                                                 tc_esp32cam.int_parameter = 0;
                                                 strcpy (tc_esp32cam.str_parameter, obj["filename"]);
                                                 set_ccsds_len ((ccsds_t*)&tc_esp32cam, strlen (tc_esp32cam.str_parameter) + 7);
                                                 break;
                          case TC_LOAD_ROUTING:  // {"cmd":"load_routing","filename":"xxxxxx.cfg"}
                                                 tc_esp32cam.int_parameter = 0;
                                                 strcpy (tc_esp32cam.str_parameter, obj["filename"]);
                                                 set_ccsds_len ((ccsds_t*)&tc_esp32cam, strlen (tc_esp32cam.str_parameter) + 7);
                                                 break;
                          case TC_SET_PARAMETER: // {"cmd":"set_parameter","parameter":"xxxxxx","value":"xxxxxx"}
                                                 tc_esp32cam.int_parameter = 0;
                                                 strcpy ((char*)&tc_esp32cam.str_parameter, obj["parameter"]);
                                                 tc_esp32cam.str_parameter[strlen (obj["parameter"])] = 0;
                                                 strcpy ((char*)(&tc_esp32cam.str_parameter + strlen (obj["parameter"]) + 1), obj["parameter"]);
                                                 set_ccsds_len ((ccsds_t*)&tc_esp32cam, strlen (obj["parameter"]) + strlen (obj["value"]) + 8);
                                                 break;
                          default:               publish_event (STS_THIS, SS_THIS, EVENT_ERROR, "JSON command to ESP32CAM not understood");
                                                 return false;
                            
                        }
                        publish_packet ((ccsds_t*)&tc_esp32cam);
                        break;
    #endif
    #ifdef PLATFORM_ESP32CAM
    case STS_ESP32:     // {\"ctr\":%u,\"type\":\"%s\",\"ss\":\"%s\",\"msg\":\"%s\"}
                        sts_esp32.packet_ctr = obj["ctr"];
                        sts_esp32.type = id_of ((const char*)obj["type"], sizeof(eventName[0]), (char*)&eventName, sizeof(eventName));
                        sts_esp32.subsystem = id_of ((const char*)obj["ss"], sizeof(subsystemName[0]), (char*)&subsystemName, sizeof(subsystemName));
                        strcpy (sts_esp32.message, obj["msg"]);
                        sts_esp32.message_size = strlen (sts_esp32.message);
                        publish_packet ((ccsds_t*)&sts_esp32);
                        break;
    case TM_ESP32:      // {\"ctr\":%u,\"mode\":\"%s\",\"err\":%u,\"warn\":%u,\"tc\":{\"exec\":%u,\"fail\":%u},\"mem\":[%u,%u],\"buf\":[%u,%u],\"inst_rate\":[%u,%u,%u,%u,%u],\"comm_rate\":[%u,%u,%u,%u,%u],\"sep\":%d,\"ena\":\"%d%d%d%d%d%d%d%d%d%d%d\",\"act\":\"%d%d%d%d%d%d%d%d\",\"conn\":{\"up\":\"%d%d\",\"warn\":\"%d%d\",\"err\":\"%d%d%d\"}}
                        esp32.packet_ctr = obj["ctr"];
                        esp32.opsmode = id_of(obj["opsmode"], sizeof(opsmodeName[0]), (char*)&opsmodeName, sizeof(opsmodeName));
                        esp32.error_ctr = obj["err"];
                        esp32.warning_ctr = obj["warn"];
                        esp32.tc_exec_ctr = obj["tc"]["exec"];
                        esp32.tc_fail_ctr = obj["tc"]["fail"];
                        esp32.radio_rate = obj["inst_rate"][0];
                        esp32.pressure_rate = obj["inst_rate"][1];
                        esp32.motion_rate = obj["inst_rate"][2];
                        esp32.gps_rate = obj["inst_rate"][3];
                        esp32.camera_rate = obj["inst_rate"][4];
                        esp32cam.udp_rate = obj["comm_rate"][1];
                        esp32cam.yamcs_rate = obj["comm_rate"][2];
                        esp32cam.serial_in_rate = obj["comm_rate"][3];
                        esp32cam.serial_out_rate = obj["comm_rate"][4];
                        esp32cam.fs_rate = obj["comm_rate"][5];
                        esp32.yamcs_buffer = obj["buf"][0];
                        esp32.serial_buffer = obj["buf"][1];
                        esp32.mem_free = obj["mem"][0];
                        esp32.fs_free = obj["mem"][1];
                        esp32.separation_sts = obj["sep"];
                        esp32.radio_enabled = (obj["ena"][0] == '1')?1:0;     
                        esp32.pressure_enabled = (obj["ena"][1] == '1')?1:0;     
                        esp32.motion_enabled = (obj["ena"][2] == '1')?1:0;     
                        esp32.gps_enabled = (obj["ena"][3] == '1')?1:0;     
                        esp32.camera_enabled = (obj["ena"][4] == '1')?1:0;     
                        esp32.wifi_enabled = (obj["ena"][5] == '1')?1:0;     
                        esp32.wifi_udp_enabled = (obj["ena"][6] == '1')?1:0;      
                        esp32.wifi_yamcs_enabled = (obj["ena"][7] == '1')?1:0;
                        esp32.fs_enabled = (obj["ena"][8] == '1')?1:0; 
                        esp32.fs_ftp_enabled = (obj["ena"][9] == '1')?1:0; 
                        esp32.time_set = (obj["ena"][10] == '1')?1:0;  
                        esp32.serial_connected = (obj["conn"]["up"][0] == '1')?1:0;
                        esp32.wifi_connected = (obj["conn"]["up"][1] == '1')?1:0; 
                        esp32.warn_serial_connloss = (obj["conn"]["warn"][0] == '1')?1:0; 
                        esp32.warn_yamcs_connloss = (obj["conn"]["warn"][1] == '1')?1:0; 
                        esp32.err_serial_dataloss = (obj["conn"]["err"][0] == '1')?1:0; 
                        esp32.err_yamcs_dataloss = (obj["conn"]["err"][1] == '1')?1:0; 
                        esp32.err_fs_dataloss = (obj["conn"]["err"][2] == '1')?1:0; 
                        esp32.err_sd_dataloss = (obj["conn"]["err"][3] == '1')?1:0; 
                        esp32.radio_active = (obj["act"][0] == '1')?1:0;     
                        esp32.pressure_active = (obj["act"][1] == '1')?1:0;     
                        esp32.motion_active = (obj["act"][2] == '1')?1:0;     
                        esp32.gps_active = (obj["act"][3] == '1')?1:0;     
                        esp32.camera_active = (obj["act"][4] == '1')?1:0; 
                        esp32.fs_active = (obj["act"][5] == '1')?1:0;           
                        esp32.sd_active = (obj["act"][6] == '1')?1:0;           
                        esp32.ftp_active = (obj["act"][7] == '1')?1:0;           
                        esp32.ota_enabled = (obj["act"][8] == '1')?1:0;  
                        publish_packet ((ccsds_t*)&esp32);
                        break;
    case TM_GPS:        // {\"ctr\":%u,\"sts\":\"%s\",\"sats\":%d,\"time\":\"%02d:%02d:%02d\",\"loc\":[%d,%d,%d],\"zero\":[%d,%d,%d],\"xyz\":[%d,%d,%d],\"v\":[%d,%d,%d],\"dop\":[%d,%d,%d],\"valid\":\"%d%d%d%d%d%d\"}
                        neo6mv2.packet_ctr = obj["ctr"];
                        neo6mv2.status = id_of(obj["sts"], sizeof(gpsName[0]), (char*)&gpsName, sizeof(gpsName));
                        neo6mv2.satellites = obj["sats"];
                        if (obj.containsKey("time")) {
                          neo6mv2.time_valid = true;
                          neo6mv2.hours = obj["time"].substring(0,1);
                          neo6mv2.minutes = obj["time"].substring(3,4);
                          neo6mv2.seconds = obj["time"].substring(6,7);
                          neo6mv2.centiseconds = obj["time"].substring(9,11);
                        }
                        if (obj.containsKey("loc")) {
                          neo6mv2.latitude = obj["loc"][0];
                          neo6mv2.longitude = obj["loc"][1];
                        }
                        if (obj.containsKey("alt")) {
                          neo6mv2.altitude = obj["alt"];
                        }
                        if (obj.containsKey("zero")) {
                          neo6mv2.latitude = obj["zero"][0];
                          neo6mv2.longitude = obj["zero"][1];
                          neo6mv2.altitude = obj["zero"][2];
                        }
                        if (obj.containsKey("xyz")) {
                          neo6mv2.x = obj["xyz"][0];
                          neo6mv2.y = obj["xyz"][1];
                          neo6mv2.z = obj["xyz"][2];
                        }
                        if (obj.containsKey("v")) {
                          neo6mv2.v_north = obj["v"][0];
                          neo6mv2.v_east = obj["v"][1];
                          neo6mv2.v_down = obj["v"][2];
                        }
                        if (obj.containsKey("dop")) {
                          neo6mv2.milli_hdop = obj["dop"][0];
                          neo6mv2.milli_vdop = obj["dop"][1];
                          neo6mv2.milli_pdop = obj["dop"][2];
                        }         
                        if (obj.containsKey("err")) {
                          neo6mv2.x_err = obj["err"][0];
                          neo6mv2.y_err = obj["err"][1];
                          neo6mv2.z_err = obj["err"][2];
                        }
                        neo6mv2.time_valid = (obj["valid"][0] == '1')?1:0;
                        neo6mv2.location_valid = (obj["valid"][1] == '1')?1:0;
                        neo6mv2.altitude_valid = (obj["valid"][2] == '1')?1:0;
                        neo6mv2.speed_valid = (obj["valid"][3] == '1')?1:0;
                        neo6mv2.hdop_valid = (obj["valid"][4] == '1')?1:0;
                        neo6mv2.vdop_valid = (obj["valid"][5] == '1')?1:0;
                        neo6mv2.pdop_valid = (obj["valid"][6] == '1')?1:0;
                        neo6mv2.error_valid = (obj["valid"][7] == '1')?1:0;
                        neo6mv2.offset_valid = (obj["valid"][8] == '1')?1:0;
                        publish_packet ((ccsds_t*)&neo6mv2);
                        break;
    case TM_MOTION:     // {\"ctr\":%u,\"accel\":[%d,%d,%d],\"gyro\":[%d,%d,%d],\"tilt\":%d,\"g\":%d,\"a\":%d,\"rpm\":%d,\"range\":[%u,%u],\"valid\":\"%d%d\"}
                        mpu6050.packet_ctr = obj["ctr"];
                        mpu6050.accel_x = obj["accel"][0];
                        mpu6050.accel_y = obj["accel"][1];
                        mpu6050.accel_z = obj["accel"][2];
                        mpu6050.gyro_x = obj["gyro"][0];
                        mpu6050.gyro_y = obj["gyro"][1];
                        mpu6050.gyro_z = obj["gyro"][2];
                        mpu6050.tilt = obj["tilt"];
                        mpu6050.g = obj["g"];
                        mpu6050.a = obj["a"];
                        mpu6050.rpm = obj["rpm"];
                        mpu6050.accel_range = obj["range"][0];
                        mpu6050.gyro_range = obj["range"][1];
                        mpu6050.accel_valid = obj["valid"][0];
                        mpu6050.gyro_valid = obj["valid"][1];
                        publish_packet ((ccsds_t*)&mpu6050);
                        break;
    case TM_PRESSURE:   // {\"ctr\":%u,\"p\":%.2f,\"p0\":%.2f,\"T\":%.2f,\"h\":%.2f,\"v_v\":%.2f}
                        bmp280.packet_ctr = obj["ctr"];
                        bmp280.pressure = obj["p"];
                        bmp280.zero_level_pressure = obj["p0"];
                        bmp280.height = obj["h"];
                        bmp280.velocity_v = obj["v_v"];
                        bmp280.temperature = obj["T"];
                        bmp280.height_valid = obj["valid"];
                        publish_packet ((ccsds_t*)&bmp280);
                        break;
    case TM_RADIO:      // {\"ctr\":%u,\"data\":\"%s\"} 
                        radio.packet_ctr = obj["ctr"];
                        hex_to_bin (&radio, obj["data"]);
                        publish_packet ((ccsds_t*)&radio);
                        break;
    case TM_TIMER:      // {\"ctr\":%u,\"idle\":%u,\"instr\":[%u,%u,%u,%u,%u],\"fun\":[%u,%u,%u,%u,%u],\"pub\":[%u,%u,%u,%u]}
                        timer.packet_ctr = obj["ctr"];
                        timer.radio_duration = obj["instr"][0];
                        timer.pressure_duration = obj["instr"][1];
                        timer.motion_duration = obj["instr"][2];
                        timer.gps_duration = obj["instr"][3];
                        timer.esp32cam_duration = obj["instr"][4];
                        timer.serial_duration = obj["fun"][0];
                        timer.ota_duration = obj["fun"][1];
                        timer.ftp_duration = obj["fun"][2];
                        timer.wifi_duration = obj["fun"][3];
                        timer.tc_duration = obj["fun"][4];
                        timer.idle_duration = obj["idle"];
                        timer.publish_fs_duration = obj["pub"][0];
                        timer.publish_serial_duration = obj["pub"][1];
                        timer.publish_yamcs_duration = obj["pub"][2];
                        timer.publish_udp_duration = obj["pub"][3];
                        publish_packet ((ccsds_t*)&timer);
                        break;
    case TC_ESP32:      // forward command
                        tc_esp32.cmd_id = id_of (obj["cmd"], sizeof(tcName[0]), (char*)tcName, sizeof(tcName));
                        switch (tc_esp32.cmd_id) {
                          case TC_REBOOT:        // {"cmd":"reboot","subsystem":0|1|2}
                                                 tc_esp32.int_parameter = obj["subsystem"];
                                                 tc_esp32.str_parameter[0] = 0;
                                                 set_ccsds_len ((ccsds_t*)&tc_esp32, 7);
                                                 break;
                          case TC_SET_OPSMODE:   // {"cmd":"set_opsmode","opsmode":"checkout|ready|static"}
                                                 tc_esp32.int_parameter = id_of (obj["opsmode"], sizeof(modeName[0]), (char*)modeName, sizeof(modeName))
                                                 tc_esp32.str_parameter[0] = 0;
                                                 set_ccsds_len ((ccsds_t*)&tc_esp32, 7);
                                                 break;
                          case TC_LOAD_CONFIG:   // {"cmd":"load_config","filename":"xxxxxx.cfg"}
                                                 tc_esp32.int_parameter = 0;
                                                 strcpy (tc_esp32.str_parameter, obj["filename"]);
                                                 set_ccsds_len ((ccsds_t*)&tc_esp32, strlen (sts_esp32cam.str_parameter) + 7);
                                                 tc_esp32.str_parameter_size = strlen (tc_esp32.str_parameter);
                                                 break;
                          case TC_LOAD_ROUTING:  // {"cmd":"load_routing","filename":"xxxxxx.cfg"}
                                                 tc_esp32.int_parameter = 0;
                                                 strcpy (tc_esp32.str_parameter, obj["filename"]);
                                                 set_ccsds_len ((ccsds_t*)&tc_esp32, strlen (sts_esp32cam.str_parameter) + 7);
                                                 tc_esp32.str_parameter_size = strlen (esp32.str_parameter);
                                                 break;
                          case TC_SET_PARAMETER: // {"cmd":"set_parameter","parameter":"xxxxxx","value":"xxxxxx"}
                                                 tc_esp32.int_parameter = 0;
                                                 strcpy ((char*)&tc_esp32.str_parameter, obj["parameter"]);
                                                 tc_esp32.str_parameter[strlen (obj["parameter"])] = 0;
                                                 strcpy ((char*)(&tc_esp32.str_parameter + strlen (obj["parameter"]) + 1), obj["parameter"]);
                                                 set_ccsds_len ((ccsds_t*)&tc_esp32, strlen (obj["parameter"]) + strlen (obj["value"]) + 8);
                                                 break;
                          default:               publish_event (STS_THIS, SS_THIS, EVENT_ERROR, "JSON command to ESP32 not understood");
                                                 return false;
                            
                        }
                        publish_packet ((ccsds_t*)&tc_esp32);
                        break;
    case TC_ESP32CAM:   // execute command
                        if (!strcmp(obj["cmd"], "reboot")) {
                          // {"cmd":"reboot","subsystem":0|1|2}
                          cmd_reboot (atoi(obj["subsystem"]));
                        }
                        else if (!strcmp(obj["cmd"], "set_opsmode")) {
                          // {"cmd":"set_opsmode","opsmode":"checkout|ready|static"}
                          cmd_set_opsmode (id_of (obj["opsmode"], sizeof(modeName[0]), (char*)modeName, sizeof(modeName)));
                        }
                        else if (!strcmp(obj["cmd"], "load_config")) {
                          // {"cmd":"load_config","filename":"xxxxxx.cfg"}
                          cmd_load_config (obj["filename"]);
                        }
                        else if (!strcmp(obj["cmd"], "load_routing")) {
                          // {"cmd":"load_routing","filename":"xxxxxx.cfg"}
                          cmd_load_routing (obj["filename"]);
                        }
                        else if (!strcmp(obj["cmd"], "set_parameter")) {
                          // {"cmd":"set_parameter","parameter":"xxxxxx","value":"xxxxxx"}
                          cmd_set_parameter (obj["parameter"], obj["value"]);
                        }
                        else {
                          publish_event (STS_THIS, SS_THIS, EVENT_ERROR, "JSON command to ESP32CAM not understood");
                        }
                        break;
    #endif
    default:            sprintf (buffer, "Ignored JSON packet with PID %d", id_of (tag_str, sizeof(pidName[0]), (char*)pidName, sizeof(pidName)));
                        publish_event (STS_THIS, SS_THIS, EVENT_WARNING, buffer); 
                        return false;
                        break;
  }
  return true;
}


// SERIAL FUNCTIONALITY

uint16_t serial_check () {
  // gets data from Serial, put it in serial_buffer, and return length of string that is ready for processing or false if string is not complete
  static uint8_t serial_status;
  static uint16_t data_len;
  static uint8_t pkt_apid;
  static uint16_t serial_buffer_pos;
  while(Serial.available()) {
    var_timer.last_serial_in_millis = millis();
    char c = Serial.read();
    serial_buffer[serial_buffer_pos++] = c;
    if (serial_status == SERIAL_COMPLETE) {
      // new transmission started: find out what we're getting ...
      switch (c) {
        case 0x00:
        case 0x01: // start of CCSDS transmission
                   Serial.println ("DEBUG: Start of CCSDS transmission");
                   serial_status = SERIAL_CCSDS;
                   data_len = JSON_MAX_SIZE;
                   break;
        case '[':  // start of JSON-encoded data string (with a timestamp preamble [])
                   Serial.println ("DEBUG: Start of JSON or ASCII transmission");
                   serial_status = SERIAL_JSON;
                   break;
        case 'O':  
        case 'o':  // keepalive or ASCII
                   Serial.println ("DEBUG: Start of KEEPALIVE or ASCII transmission");
                   serial_status = SERIAL_KEEPALIVE;
                   break;
        default:   // ASCII
                   Serial.println ("Start of ASCII transmission");
                   serial_status = SERIAL_ASCII;
                   break;
      }
    }
    if (serial_status == SERIAL_CCSDS) {
      if (serial_buffer_pos == 5) { // CCSDS header should be in, we can check whether it's valid
        if (valid_ccsds_hdr ((ccsds_t*)&serial_buffer, PKT_TM) or valid_ccsds_hdr ((ccsds_t*)&serial_buffer, PKT_TC)) {
          data_len = get_ccsds_len ((ccsds_t*)&serial_buffer) + sizeof(ccsds_hdr_t);
        }
        else {
          sprintf (buffer, "Received packet over serial with invalid CCSDS header (%s...)", get_hex_str ((char*)serial_buffer, 6).c_str());
          publish_event (STS_THIS, SS_THIS, EVENT_WARNING, buffer); 
          serial_buffer_pos = 0;
          serial_status = SERIAL_UNKNOWN;
          return false;
        }
      }
      if (serial_buffer_pos == data_len) { // CCSDS packet is complete               
        Serial.println ("DEBUG: CCSDS packet complete");
        if (!tm_this->serial_connected) { 
          tm_this->serial_connected = true;
          tm_this->warn_serial_connloss = false; 
        }
        tm_this->serial_in_rate++;
        serial_status = SERIAL_COMPLETE;
        serial_buffer_pos = 0;
        return (data_len);              
      }
    }
    else { // normal ASCII encoded transmission (JSON, SERIAL, KEEPALIVE)
      switch (c) {
        case 0x0A:  // received carriage return, EOL reached
                    if (serial_status == SERIAL_KEEPALIVE) {
                      switch (serial_buffer[0]) {
                        case 'O': // keep-alive confirming good connection by counterpart
                                  if (!tm_this->serial_connected) {
                                    tm_this->serial_connected = true;
                                    tm_this->warn_serial_connloss = false; 
                                  }
                                  break;
                        case 'o': // keep-alive indicating counterpart not receiving over serial
                                  if (!config_this->debug_over_serial and tm_this->serial_connected) {
                                    tm_this->serial_connected = false;
                                    tm_this->warn_serial_connloss = true;
                                  }
                                  break;
                      }
                      Serial.println ("DEBUG: KEEPALIVE transmission completed");
                      serial_status = SERIAL_COMPLETE;
                      serial_buffer_pos = 0;
                      return false;
                    }
                    else { // ASCII/JSON transmission complete
                      serial_buffer[serial_buffer_pos-1] = 0x00;
                      if (!tm_this->serial_connected) { 
                        tm_this->serial_connected = true;
                        tm_this->warn_serial_connloss = false; 
                      }
                      tm_this->serial_in_rate++;
                      Serial.print (String("DEBUG: ASCII/JSON transmission completed: [") + strlen(serial_buffer) + "] " + serial_buffer);
                      serial_status = SERIAL_COMPLETE;
                      serial_buffer_pos = 0;
                      return (strlen(serial_buffer));
                    }
                    break;
        case 0x0D:  if (serial_status == SERIAL_COMPLETE) {    
                      // received line feed after carriage return, all smooth, ignore
                      serial_buffer_pos = 0;
                    }
                    else {
                      // received line feed but not after carriage return, don't know what this is ...
                      serial_buffer_pos = 0;
                      serial_status = SERIAL_UNKNOWN;
                      sprintf (buffer, "Received CR without LF from %s", subsystemName[SS_OTHER]);
                      publish_event (STS_THIS, SS_OTHER, EVENT_WARNING, buffer);                    
                    }
                    break;
        default:    // happily build up ASCII-encoded string
                    if (serial_buffer_pos > 1 and serial_status == SERIAL_KEEPALIVE) {
                      // we're getting characters after 'o' or 'O', this is a normal ASCII transmission
                      serial_status = SERIAL_ASCII;
                    }
                    break;  
      }
    }
    if (serial_buffer_pos == JSON_MAX_SIZE) { // something is wrong: line too long
      serial_buffer[JSON_MAX_SIZE-1] = '\0';
      serial_buffer_pos = 0;
      Serial.println (get_hex_str ((char*)&serial_buffer, JSON_MAX_SIZE));
      if (!config_this->debug_over_serial) {
        tm_this->serial_connected = false;
      }
      sprintf (buffer, "Received too long message from %s", subsystemName[SS_OTHER]);
      publish_event (STS_THIS, SS_OTHER, EVENT_ERROR, buffer);
      Serial.println (String("DEBUG: too long message on Serial: ") + serial_buffer);
      serial_status = SERIAL_UNKNOWN;
      return false; // invalid serial_buffer content, do not use
    }
  } // while Serial.available
  return false; // no serial data ready
}

void serial_parse () {
  if (serial_buffer[0] == '[') {
    // JSON formatted message
    parse_json ((char*)&serial_buffer);
  }
  else if (serial_buffer[0] == 0x00 or serial_buffer[0] == 0x01) {
    // CCSDS formatted message
    parse_ccsds ((ccsds_t*)&serial_buffer);
  }
  else {
    // generic ASCII string: likely debug information
    publish_udp_text (serial_buffer);
  }
}

// COMMAND FUNCTIONALITY

bool cmd_reboot (uint8_t subsystem) {
  switch (subsystem) {
    case 0: // this
            sprintf (buffer, "Rebooting %s subsystem", subsystemName[SS_THIS]);
            publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
            delay (1000);
            ESP.restart();
            break;
    case 1: // other
            sprintf (buffer, "Sending reboot command to %s subsystem", subsystemName[SS_OTHER]);
            publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
            tc_other->cmd_id = TC_REBOOT;
            tc_other->int_parameter = 0;
            tc_other->str_parameter[0] = 0;
            set_ccsds_len ((ccsds_t*)tc_other, 7); 
            publish_packet ((ccsds_t*)tc_other);
            break;
    case 2: // both
            tc_other->cmd_id = TC_REBOOT;
            tc_other->int_parameter = 0;
            tc_other->str_parameter[0] = 0;
            set_ccsds_len ((ccsds_t*)tc_other, 7); 
            publish_packet ((ccsds_t*)tc_other);
            sprintf (buffer, "Sending reboot command to %s and rebooting %s subsystem", subsystemName[SS_OTHER], subsystemName[SS_THIS]);
            publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
            delay (1000);
            ESP.restart();
  }     
}

bool cmd_set_opsmode (uint8_t opsmode) {
  if (opsmode == MODE_CHECKOUT or opsmode == MODE_READY or opsmode == MODE_STATIC) {
      sprintf (buffer, "Setting opsmode for ESP32 subsystem to '%s'", modeName[opsmode]);
      publish_event (STS_ESP32, SS_ESP32, EVENT_CMD_RESP, buffer);
      esp32.opsmode = opsmode;
  }
  else {
      sprintf (buffer, "Opsmode '%s' not known or not allowed", modeName[opsmode]);
      publish_event (STS_THIS, SS_THIS, EVENT_CMD_FAIL, buffer);
  }
}

bool cmd_set_parameter (const char* parameter, const char* value) {
  if (set_parameter (parameter, value)) {
    publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
    return true;
  }
  else {
    publish_event (STS_THIS, SS_THIS, EVENT_CMD_FAIL, "Command needs valid parameter/value");
    return false; 
  }
}

bool cmd_load_config (const char* filename) {
  fs_load_config (filename);
}

bool cmd_load_routing (const char* filename) {
  fs_load_routing (filename);
}

bool cmd_toggle_routing (uint16_t PID, const char interface) {
  // TODO: TBW + integrate
}

// SUPPORT FUNCTIONS

uint8_t id_of (const char* string2, uint8_t string_len, const char* array_of_strings, uint16_t array_len) { 
  char string[string_len]; 
  strcpy (string, string2);
  for (uint8_t id = 0; id < array_len / string_len; id++) {
    if (!strcmp (string, (char*)(array_of_strings + string_len*id))) {
      return id;
    }
  }
  return -1;
}

String get_hex_str (char* blob, uint16_t length) {
  char* pblob = blob;
  const char * hex = "0123456789ABCDEF";
  String hex_str;
  for(uint8_t i=0; i < length; i++, pblob++){
    hex_str += hex[(*pblob>>4) & 0xF];
    hex_str += hex[ *pblob     & 0xF];
  }
  return (hex_str);
}

void hex_to_bin (byte* destination, char* hex_input) {
  // TODO: TBW
}

#endif