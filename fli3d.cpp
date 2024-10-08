/*
 * Fli3d - Library (file system, wifi, TM/TC, comms functionality)
 */

#ifndef ARDUINO_ESP8266_NODEMCU

#include <fli3d.h>
#include <fli3d_secrets.h>
#include <Arduino.h>
#ifdef PLATFORM_ESP32CAM
#include <SD_MMC.h>
#include <SPI.h>
#endif

//SerialTransfer serialTransfer;
WiFiUDP wifiUDP;
WiFiUDP wifiUDP_NTP;
#ifdef ASYNCUDP
AsyncUDP asyncUDP_yamcs_tc;
#else
WiFiUDP wifiUDP_yamcs_tc;
#endif
FtpServer wifiTCP_FTP;
NTPClient timeClient(wifiUDP_NTP, config_network.ntp_server, 0);
File file_ccsds;
File file_json;
buffer_t ccsds_archive;
ccsds_t replayed_ccsds;
UnixTime datetime(0);
char buffer[BUFFER_MAX_SIZE];
char serial_in_buffer[BUFFER_MAX_SIZE];
char ccsds_path_buffer[38] = "/nodate.ccsds";
char json_path_buffer[38] = "/nodate.json";
char lock_filename[32] = "/opsmode.lock";
char today_dir[16] = "/";

#ifdef PLATFORM_ESP32
extern void ota_setup ();
extern void set_gps_samplerate (uint8_t rate);
extern void motion_set_samplerate (uint8_t rate);
extern void mpu_set_accel_range (uint8_t accel_range);
extern void mpu_set_gyro_range (uint8_t gyro_range);
extern void publish_radio ();
#endif 

ccsds_hdr_t         ccsds_hdr;
ccsds_t             ccsds_tc_buffer;

sts_esp32_t         sts_esp32;
sts_esp32cam_t      sts_esp32cam;
tm_esp32_t          esp32;
tm_esp32cam_t       esp32cam;
tm_camera_t         ov2640;
tm_gps_t            neo6mv2;
tm_motion_t         motion;
tm_pressure_t       bmp280;
tm_radio_t          radio;
timer_esp32_t       timer_esp32;
timer_esp32cam_t    timer_esp32cam;
tc_esp32_t          tc_esp32;
tc_esp32cam_t       tc_esp32cam;

var_timer_t         var_timer;
config_network_t    config_network;
config_esp32_t      config_esp32;
config_esp32cam_t   config_esp32cam;

#ifdef PLATFORM_ESP32
tm_esp32_t          *tm_this = &esp32;
tm_esp32cam_t       *tm_other = &esp32cam;
tc_esp32_t          *tc_this = &tc_esp32;
tc_esp32cam_t       *tc_other = &tc_esp32cam;
sts_esp32_t         *sts_this = &sts_esp32;
sts_esp32cam_t      *sts_other = &sts_esp32cam;
timer_esp32_t       *timer_this = &timer_esp32;
timer_esp32cam_t    *timer_other = &timer_esp32cam;
config_esp32_t      *config_this = &config_esp32;
#endif
#ifdef PLATFORM_ESP32CAM
tm_esp32cam_t       *tm_this = &esp32cam;
tm_esp32_t          *tm_other = &esp32;
tc_esp32cam_t       *tc_this = &tc_esp32cam;
tc_esp32_t          *tc_other = &tc_esp32;
sts_esp32cam_t      *sts_this = &sts_esp32cam;
sts_esp32_t         *sts_other = &sts_esp32;
timer_esp32cam_t    *timer_this = &timer_esp32cam;
timer_esp32_t       *timer_other = &timer_esp32;
config_esp32cam_t   *config_this = &config_esp32cam;
#endif

const char pidName[NUMBER_OF_PID][15] =   { "sts_esp32", "sts_esp32cam", "tm_esp32", "tm_esp32cam", "tm_camera", "tm_gps", "tm_motion", "tm_pressure", "tm_radio", "timer_esp32", "timer_esp32cam", "tc_esp32", "tc_esp32cam" };
const char eventName[8][9] =              { "init", "info", "warning", "error", "cmd", "cmd_ack", "cmd_resp", "cmd_fail" };
const char subsystemName[13][14] =        { "esp32", "esp32cam", "ov2640", "neo6mv2", "mpuXX50", "bmp280", "radio", "sd", "separation", "timer", "fli3d", "ground", "any" };
const char modeName[4][12] =              { "init", "checkout", "nominal", "maintenance" };
const char stateName[4][10] =             { "static", "thrust", "freefall", "parachute" };
const char cameraModeName[4][7] =         { "init", "idle", "single", "stream" };
const char cameraResolutionName[11][10] = { "160x120", "invalid1", "invalid2", "240x176", "320x240", "400x300", "640x480", "800x600", "1024x768", "1280x1024", "1600x1200" };
const char dataEncodingName[3][8] =       { "CCSDS", "JSON", "ASCII" };

const char commLineName[9][13] =          { "serial", "wifi_udp", "wifi_yamcs", "wifi_cam", "sd_ccsds", "sd_json", "sd_cam", "fs", "radio" };
const char tcName[6][20] =                { "reboot", "set_opsmode", "load_config", "load_routing", "set_parameter", "freeze_opsmode" };
const char gpsStatusName[9][11] =         { "none", "est", "time_only", "std", "dgps", "rtk_float", "rtk_fixed", "status_pps", "waiting" }; 
const char dhtName[5][7] =                { "AUTO", "DHT11", "DHT22", "AM2302", "RHT03" }; 
const char fsName[3][5] =                 { "none", "FS", "SD" };
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
  #ifndef ESP_ARDUINO_VERSION_MAJOR // ESP32 core v1.0.x
  if (LITTLEFS.begin(false)) {
    sprintf (buffer, "Initialized FS (size: %u kB; free: %u kB)", LITTLEFS.totalBytes()/1024, fs_free());
  #else
  if (LittleFS.begin(false)) {
    sprintf (buffer, "Initialized FS (size: %u kB; free: %u kB)", LittleFS.totalBytes()/1024, fs_free());
  #endif 
    tm_this->fs_enabled = true;
    tm_this->fs_active = true;
    publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);
    return true;
  }
  else {
    #ifndef ESP_ARDUINO_VERSION_MAJOR // ESP32 core v1.0.x  
    if (LITTLEFS.begin(true)) {
      sprintf (buffer, "Formatted and initialized FS (size: %d kB; free: %d kB)", LITTLEFS.totalBytes()/1024, fs_free());
    #else
    if (LittleFS.begin(true)) {
      sprintf (buffer, "Formatted and initialized FS (size: %d kB; free: %d kB)", LittleFS.totalBytes()/1024, fs_free());
    #endif
      tm_this->fs_enabled = true;
      tm_this->fs_active = true;
      publish_event (STS_THIS, SS_THIS, EVENT_WARNING, buffer);
      return true;
    }
    else {
      publish_event (STS_THIS, SS_THIS, EVENT_ERROR, "Failed to initialize or format FS");
      tm_this->err_fs_dataloss = true;
      return false;
    }
  }
}

bool fs_flush_data () {
  #ifdef PLATFORM_ESP32
  if (!esp32.separation_sts) { // old data will only be deleted from ESP32 module if SEP_STS_PIN is connected to GND (Fli3d mated to rocket) - protect data after flight
  #endif
  #ifdef PLATFORM_ESP32CAM
  if (true) { // TODO: add viable inhibit for ESP32CAM
  #endif
    #ifndef ESP_ARDUINO_VERSION_MAJOR // ESP32 core v1.0.x
    File dir = LITTLEFS.open ("/");
    #else
    File dir = LittleFS.open ("/");
    #endif  
    File file = dir.openNextFile ();
    char local_path_buffer[32];
    char path_buffer[32];
    while (file) {
      if (file.isDirectory()) {
        sprintf (path_buffer, "%s", file.name());
        file.close ();
        #ifndef ESP_ARDUINO_VERSION_MAJOR // ESP32 core v1.0.x
        File local_dir = LITTLEFS.open (path_buffer);
        #else
        File local_dir = LittleFS.open (path_buffer);
        #endif
        File local_file = local_dir.openNextFile ();
        while (local_file) {
          sprintf (local_path_buffer, "%s", local_file.name());
          local_file.close ();
          #ifndef ESP_ARDUINO_VERSION_MAJOR // ESP32 core v1.0.x
          LITTLEFS.remove (local_path_buffer);
          #else
          LittleFS.remove (local_path_buffer);
          #endif
          local_file = local_dir.openNextFile ();
        }  
        #ifndef ESP_ARDUINO_VERSION_MAJOR // ESP32 core v1.0.x  
        LITTLEFS.rmdir (path_buffer);
        #else
        LittleFS.rmdir (path_buffer);
        #endif
      }
      file = dir.openNextFile ();
    }
    tm_this->fs_enabled = true; // needed to have next message stored
    publish_event (STS_THIS, SS_THIS, EVENT_WARNING, "Deleted all data files since FS is full");
    return true;
  }
  else {
    publish_event (STS_THIS, SS_THIS, EVENT_WARNING, "FS is full, but data not deleted because this is inhibited");
    return false;
  }
}

void create_today_dir (uint8_t filesystem) {
  char today_tag[14];
  char sequencer1 = 'A';
  char sequencer2 = 'A';
  sync_file_ccsds ();
  sync_file_json ();
  #ifdef PLATFORM_ESP32  
  #ifndef ESP_ARDUINO_VERSION_MAJOR // ESP32 core v1.0.x
  while (!strcmp(today_dir, "/") or (filesystem == FS_LITTLEFS and LITTLEFS.exists(today_dir))) {
  #else
  while (!strcmp(today_dir, "/") or (filesystem == FS_LITTLEFS and LittleFS.exists(today_dir))) {
  #endif
  #endif
  #ifdef PLATFORM_ESP32CAM  
  #ifndef ESP_ARDUINO_VERSION_MAJOR // ESP32 core v1.0.x
  while (!strcmp(today_dir, "/") or ((filesystem == FS_LITTLEFS and LITTLEFS.exists(today_dir)) or (filesystem == FS_SD_MMC and SD_MMC.exists(today_dir)))) {
  #else
  while (!strcmp(today_dir, "/") or ((filesystem == FS_LITTLEFS and LittleFS.exists(today_dir)) or (filesystem == FS_SD_MMC and SD_MMC.exists(today_dir)))) {
  #endif
  #endif
    datetime.getDateTime(config_this->boot_epoch + millis()/1000);
    sprintf (today_tag, "%02u%02u%02u%c%c", datetime.year, datetime.month, datetime.day, sequencer1, sequencer2++);
    if (sequencer2 == '[') {
      sequencer1++;
      sequencer2 = 'A';
    }
    sprintf (today_dir, "/%s", today_tag);
  }
  if (filesystem == FS_LITTLEFS) {
  #ifndef ESP_ARDUINO_VERSION_MAJOR // ESP32 core v1.0.x
    LITTLEFS.mkdir(today_dir);
    if (LITTLEFS.exists("/nodate.ccsds")) {
      sprintf (ccsds_path_buffer, "%s/%s.ccsds", today_dir, today_tag);
      LITTLEFS.rename ("/nodate.ccsds", ccsds_path_buffer);
    }  
    if (LITTLEFS.exists("/nodate.json")) {
      sprintf (json_path_buffer, "%s/%s.json", today_dir, today_tag);
      LITTLEFS.rename ("/nodate.json", json_path_buffer);
    }
  }
  #else
    LittleFS.mkdir(today_dir);
    if (LittleFS.exists("/nodate.ccsds")) {
      sprintf (ccsds_path_buffer, "%s/%s.ccsds", today_dir, today_tag);
      LittleFS.rename ("/nodate.ccsds", ccsds_path_buffer);
    }  
    if (LittleFS.exists("/nodate.json")) {
      sprintf (json_path_buffer, "%s/%s.json", today_dir, today_tag);
      LittleFS.rename ("/nodate.json", json_path_buffer);
    }
  }
  #endif
  #ifdef PLATFORM_ESP32CAM 
  else if (filesystem == FS_SD_MMC) {
    SD_MMC.mkdir(today_dir);
    if (SD_MMC.exists("/nodate.ccsds")) {
      sprintf (ccsds_path_buffer, "%s/%s.ccsds", today_dir, today_tag);
      SD_MMC.rename ("/nodate.ccsds", ccsds_path_buffer);
    }  
    if (SD_MMC.exists("/nodate.json")) {
      sprintf (json_path_buffer, "%s/%s.json", today_dir, today_tag);
      SD_MMC.rename ("/nodate.json", json_path_buffer);
    }
  }
  #endif
  sprintf (buffer, "Created storage directory %s on %s for current session, and moved current log files to it", today_dir, fsName[filesystem]);
  publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);
  tm_this->fs_active = true;
}

uint16_t fs_free () {
  if (config_this->fs_enable) {
    sync_file_ccsds ();
    sync_file_json ();
    #ifndef ESP_ARDUINO_VERSION_MAJOR // ESP32 core v1.0.x
    if (tm_this->fs_enabled and LITTLEFS.totalBytes()-LITTLEFS.usedBytes() <= 8192) {
    #else
    if (tm_this->fs_enabled and LittleFS.totalBytes()-LittleFS.usedBytes() <= 8192) {
    #endif
      tm_this->fs_enabled = false;
      tm_this->err_fs_dataloss = true;
      sprintf (buffer, "Disabling further write access to FS because it is full");
      publish_event (STS_THIS, SS_THIS, EVENT_ERROR, buffer);
    }
    #ifndef ESP_ARDUINO_VERSION_MAJOR // ESP32 core v1.0.x
    return ((LITTLEFS.totalBytes()-LITTLEFS.usedBytes())/1024);
    #else
    return ((LittleFS.totalBytes()-LittleFS.usedBytes())/1024);
    #endif
  }
  else { 
    return (0);
  }
}

#ifdef PLATFORM_ESP32CAM
bool sd_setup () {
  if (!SD_MMC.begin()) {
    publish_event (STS_ESP32CAM, SS_SD, EVENT_ERROR, "Card reader initialisation failed");
    return false;
  }
  if (SD_MMC.cardType() == CARD_NONE) {
    publish_event (STS_ESP32CAM, SS_SD, EVENT_ERROR, "No SD card inserted");
    return false;
  }
  esp32cam.sd_enabled = true;
  sprintf (buffer, "SD card mounted: size: %llu MB; space: %llu MB; used: %llu MB", SD_MMC.cardSize() / (1024 * 1024), SD_MMC.totalBytes() / (1024 * 1024), SD_MMC.usedBytes() / (1024 * 1024));
  publish_event (STS_ESP32CAM, SS_SD, EVENT_INIT, buffer);
  return true;
}

uint16_t sd_free () {
  return ((SD_MMC.totalBytes() - SD_MMC.usedBytes())/1024/1024);
}
#endif

bool ftp_setup () {
  wifiTCP_FTP.begin(config_network.ftp_user, config_network.ftp_password);
  tm_this->ftp_fs = config_this->ftp_fs;
  sprintf (buffer, "Starting passive FTP server on TCP port %s:%u", WiFi.localIP().toString().c_str(), 21);
  publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);
  return true;
}

bool ftp_check (uint8_t filesystem) {
  switch (filesystem) {
  case FS_LITTLEFS: 
                    #ifndef ESP_ARDUINO_VERSION_MAJOR // ESP32 core v1.0.x  
                    wifiTCP_FTP.handleFTP (LITTLEFS);  
                    #else
                    wifiTCP_FTP.handleFTP (LittleFS);  
                    #endif
                    break;
  #ifdef PLATFORM_ESP32CAM
  case FS_SD_MMC:   wifiTCP_FTP.handleFTP (SD_MMC);
                    break;
  #endif
  }
  tm_this->ftp_fs = filesystem;
  tm_this->ftp_active = true;
  return true;
}

// CONFIGURATION FUNCTIONALITY

void load_default_config () {
  strcpy (config_network.wifi_ssid, default_wifi_ssid[0]); 
  strcpy (config_network.wifi_password, default_wifi_password[0]);
  strcpy (config_network.ap_ssid, default_ap_ssid); 
  strcpy (config_network.ap_password, default_ap_password); 
  strcpy (config_network.ftp_user, default_ftp_user); 
  strcpy (config_network.ftp_password, default_ftp_password); 
  strcpy (config_network.udp_server, default_udp_server[0]);
  strcpy (config_network.yamcs_server, default_yamcs_server[0]); 
  strcpy (config_network.ntp_server, default_ntp_server[0]);
  config_network.udp_port = default_udp_port; 
  config_network.yamcs_tm_port = default_yamcs_tm_port; 
  config_network.yamcs_tc_port = default_yamcs_tc_port; 
  config_esp32.radio_rate = 1;
  config_esp32.pressure_rate = 1;
  config_esp32.motion_rate = 1;
  config_esp32.gps_rate = 1;
  strcpy (config_esp32.config_file, "/default.cfg");
  strcpy (config_esp32.routing_file, "/default.rt");
  config_esp32.mpu_accel_sensitivity = 595;
  config_esp32.mpu_accel_offset_x = 0;
  config_esp32.mpu_accel_offset_y = 0;
  config_esp32.mpu_accel_offset_z = 0;
  config_esp32.radio_enable = true;
  config_esp32.pressure_enable = true;
  config_esp32.motion_enable = true;
  config_esp32.gps_enable = true;
  config_esp32.camera_enable = false;
  config_esp32.temperature_enable = true;
  config_esp32.wifi_enable = true;
  config_esp32.wifi_sta_enable = true;
  config_esp32.wifi_ap_enable = true;
  config_esp32.wifi_udp_enable = false;
  config_esp32.wifi_yamcs_enable = true;
  config_esp32.fs_enable = true;
  config_esp32.ftp_enable = true;
  config_esp32.ftp_fs = FS_LITTLEFS;
  config_esp32.buffer_fs = FS_LITTLEFS;
  config_esp32.serial_format = ENC_JSON;
  config_esp32.ota_enable = true;
  config_esp32.motion_udp_raw_enable = false;
  config_esp32.gps_udp_raw_enable = false;
  motion.accel_range = 3;
  motion.gyro_range = 3;
  strcpy (config_esp32cam.config_file, "/default.cfg");
  strcpy (config_esp32cam.routing_file, "/default.rt");
  config_esp32cam.camera_rate = 2;
  config_esp32cam.wifi_enable = true;
  config_esp32cam.wifi_sta_enable = true;
  config_esp32cam.wifi_ap_enable = true;
  config_esp32cam.wifi_udp_enable = false;
  config_esp32cam.wifi_yamcs_enable = true;
  config_esp32cam.wifi_image_enable = true; 
  config_esp32cam.camera_enable = true;
  config_esp32cam.fs_enable = false;
  config_esp32cam.ftp_enable = true;
  config_esp32cam.ota_enable = true;
  config_esp32cam.sd_enable = true;
  config_esp32cam.sd_json_enable = true;
  config_esp32cam.sd_ccsds_enable = true;
  config_esp32cam.sd_image_enable = true;
  config_esp32cam.buffer_fs = FS_SD_MMC;
  config_esp32cam.ftp_fs = FS_SD_MMC;
  config_esp32cam.serial_format = ENC_JSON;
  //                       0: STS_ESP32 
  //                       |  1: STS_ESP32CAM 
  //                       |  |  2: TM_ESP32 
  //                       |  |  |  3: TM_ESP32CAM 
  //                       |  |  |  |  4: TM_CAMERA 
  //                       |  |  |  |  |  5: TM_GPS 
  //                       |  |  |  |  |  |  6: TM_MOTION 
  //                       |  |  |  |  |  |  |  7: TM_PRESSURE 
  //                       |  |  |  |  |  |  |  |  8: TM_RADIO 
  //                       |  |  |  |  |  |  |  |  |  9: TIMER_ESP32
  //                       |  |  |  |  |  |  |  |  |  |  A: TIMER_ESP32CAM
  //                       |  |  |  |  |  |  |  |  |  |  |  B: TC_ESP32
  //                       |  |  |  |  |  |  |  |  |  |  |  |  C: TC_ESP32CAM
  //                       0  1  2  3  4  5  6  7  8  9  A  B  C
  // ESP32                 *     *        *  *  *  *  *        *
  //                       -------------------------------------  
  #ifdef PLATFORM_ESP32
  char rt_serial[40] =   " 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ";
  char rt_yamcs[40] =    " 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0 ";
  char rt_udp[40] =      " 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ";
  char rt_fs[40] =       " 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1 ";
  set_routing (routing_serial, (const char*)rt_serial);
  set_routing (routing_yamcs, (const char*)rt_yamcs);
  set_routing (routing_udp, (const char*)rt_udp);
  set_routing (routing_fs, (const char*)rt_fs);
  #endif
  #ifdef PLATFORM_ESP32CAM
  //                       0  1  2  3  4  5  6  7  8  9  A  B  C
  // ESP32CAM                 *     *  *                 *  *   
  //                       -------------------------------------  
  char rt_serial[40] =   " 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ";
  char rt_yamcs[40] =    " 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0 ";
  char rt_udp[40] =      " 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ";
  char rt_fs[40] =       " 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1 ";
  char rt_sd_json[40] =  " 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1 ";
  char rt_sd_ccsds[40] = " 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1 ";
  set_routing (routing_serial, (const char*)rt_serial);
  set_routing (routing_yamcs, (const char*)rt_yamcs);
  set_routing (routing_udp, (const char*)rt_udp);
  set_routing (routing_fs, (const char*)rt_fs);
  set_routing (routing_sd_json, (const char*)rt_sd_json);
  set_routing (routing_sd_ccsds, (const char*)rt_sd_ccsds);
  #endif
}

bool file_load_settings (uint8_t filesystem) {
  char linebuffer[80];
  uint8_t value_start = 0;
  File file;
  switch (filesystem) {
  case FS_LITTLEFS: 
                    #ifndef ESP_ARDUINO_VERSION_MAJOR // ESP32 core v1.0.x  
                    file = LITTLEFS.open ("/settings.ini");
                    #else
                    file = LittleFS.open ("/settings.ini");
                    #endif
                    break;
  #ifdef PLATFORM_ESP32CAM
  case FS_SD_MMC:   file = SD_MMC.open ("/settings.ini");
                    break;
  #endif
  }
  if (!file) {
    sprintf (buffer, "Failed to open configuration file '/settings.ini' from %s", fsName[filesystem]);
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
        if (String(linebuffer).startsWith("config")) {
          strcpy (config_this->config_file, String(linebuffer).substring(value_start).c_str());
        }
        if (String(linebuffer).startsWith("routing")) {
          strcpy (config_this->routing_file, String(linebuffer).substring(value_start).c_str());
        }
        i = 0;
      }
      else {
        i++;
      }
    }
  }
  file.close();
  tm_this->fs_active = true;
  return true;
}

bool file_load_config (uint8_t filesystem, const char* filename) { 
  char linebuffer[80];
  uint8_t value_start = 0;
  File file;
  switch (filesystem) {
  case FS_LITTLEFS: 
                    #ifndef ESP_ARDUINO_VERSION_MAJOR // ESP32 core v1.0.x  
                    file = LITTLEFS.open (filename);
                    #else
                    file = LittleFS.open (filename);
                    #endif
                break;
  #ifdef PLATFORM_ESP32CAM
  case FS_SD_MMC:   file = SD_MMC.open (filename);
                break;
  #endif
  }
  if (!file) {
    sprintf (buffer, "Failed to open configuration file '%s' from %s", filename, fsName[filesystem]);
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
          publish_udp_text (buffer);
        }  
        i = 0;
      }
      else {
        i++;
      }
    }
  }
  file.close();
  sprintf (buffer, "Read settings from '%s' configuration file on %s", config_this->config_file, fsName[filesystem]);
  publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);
  tm_this->fs_active = true;
  return true;
}

bool file_load_routing (uint8_t filesystem, const char* filename) {
  char linebuffer[80], parameter[80];
  uint8_t value_start = 0;
  File file;
  switch (filesystem) {
  case FS_LITTLEFS: 
                    #ifndef ESP_ARDUINO_VERSION_MAJOR // ESP32 core v1.0.x  
                    file = LITTLEFS.open (filename);
                    #else
                    file = LittleFS.open (filename);
                    #endif
                    break;
  #ifdef PLATFORM_ESP32CAM
  case FS_SD_MMC:   file = SD_MMC.open (filename);
                    break;
  #endif
  }
  if (!file) {
    sprintf (buffer, "Failed to open routing file '%s' from %s", filename, fsName[filesystem]);
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
          sprintf (buffer, "Set routing_serial to %s", set_routing (routing_serial, (const char*)parameter).c_str());
          publish_udp_text (buffer);
        }
        if (String(linebuffer).startsWith("rt_yamcs")) {
          sprintf (buffer, "Set routing_yamcs to %s", set_routing (routing_yamcs, (const char*)parameter).c_str());
          publish_udp_text (buffer);
        }
        if (String(linebuffer).startsWith("rt_udp")) {
          sprintf (buffer, "Set routing_udp to %s", set_routing (routing_udp, (const char*)parameter).c_str());
          publish_udp_text (buffer);
        }
        if (String(linebuffer).startsWith("rt_fs")) {
          sprintf (buffer, "Set routing_fs to %s", set_routing (routing_fs, (const char*)parameter).c_str());
          publish_udp_text (buffer);
        }
        #ifdef PLATFORM_ESP32CAM
        if (String(linebuffer).startsWith("rt_sd_json")) {
          sprintf (buffer, "Set routing_sd_json to %s", set_routing (routing_sd_json, (const char*)parameter).c_str());
          publish_udp_text (buffer);
        }
        if (String(linebuffer).startsWith("rt_sd_ccsds")) {
          sprintf (buffer, "Set routing_sd_ccsds to %s", set_routing (routing_sd_ccsds, (const char*)parameter).c_str());
          publish_udp_text (buffer);
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
  sprintf (buffer, "Read routing from '%s' configuration file on %s", config_this->routing_file, fsName[filesystem]);
  publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);
  tm_this->fs_active = true;
  return true;
}

String set_routing (char* routing_table, const char* routing_string) {
  uint16_t PID = 0;
  String return_string;
  for (uint8_t i = 0; i < strlen (routing_string); i++) {
    if (routing_string[i] == '0') {
      *routing_table++ = false;
      return_string += String(pidName[PID++]) + ":0 ";
    }
    else if (routing_string[i] == '1') {
      *routing_table++ = true;
      return_string += String(pidName[PID++]) + ":1 ";
    }
  }
  publish_udp_text (return_string.c_str());
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
  else if (!strcmp(parameter, "ftp_enable")) { 
    config_this->ftp_enable = atoi(value);
    sprintf (buffer, "Set ftp_enable to %s", config_this->ftp_enable?"true":"false");
    success = true;
  }  
  else if (!strcmp(parameter, "camera_enable")) { 
    config_this->camera_enable = atoi(value);
    sprintf (buffer, "Set camera_enable to %s", config_this->camera_enable?"true":"false");
    success = true;
  } 
  else if (!strcmp(parameter, "ftp_fs")) { 
    config_this->ftp_fs = atoi(value);
    sprintf (buffer, "Set ftp_fs to %s", fsName[config_this->ftp_fs]);
    success = true;
  } 
  else if (!strcmp(parameter, "buffer_fs")) { 
    config_this->buffer_fs = atoi(value);
    sprintf (buffer, "Set buffer_fs to %s", fsName[config_this->buffer_fs]);
    success = true;
  } 
  else if (!strcmp(parameter, "serial_format")) { 
    config_this->serial_format = atoi(value);
    sprintf (buffer, "Set serial_format to %s", dataEncodingName[config_this->serial_format]);
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
  #ifdef MOTION
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
        set_gps_samplerate (config_this->gps_rate);
      }
      sprintf (buffer, "Set gps_rate to %u Hz", config_this->gps_rate); 
    }
    else {
      tm_this->gps_enabled = false;
      sprintf (buffer, "Set gps_enabled to false");
    }
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
  else if (!strcmp(parameter, "mpu_accel_offset_x")) { 
    config_this->mpu_accel_offset_x = atoi(value);
    sprintf (buffer, "Set mpu_accel_offset_x to %d", config_this->mpu_accel_offset_x);
    success = true;
  }  
  else if (!strcmp(parameter, "mpu_accel_offset_y")) { 
    config_this->mpu_accel_offset_y = atoi(value);
    sprintf (buffer, "Set mpu_accel_offset_y to %d", config_this->mpu_accel_offset_y);
    success = true;
  }  
  else if (!strcmp(parameter, "mpu_accel_offset_z")) { 
    config_this->mpu_accel_offset_z = atoi(value);
    sprintf (buffer, "Set mpu_accel_offset_z to %d", config_this->mpu_accel_offset_z);
    success = true;
  }
  else if (!strcmp(parameter, "mpu_accel_sensitivity")) { 
    config_this->mpu_accel_sensitivity = atoi(value);
    sprintf (buffer, "Set mpu_accel_sensitivity to %d", config_this->mpu_accel_sensitivity);
    success = true;
  }
  else if (!strcmp(parameter, "mpu_accel_range")) { 
    motion.accel_range = atoi(value);
    mpu_set_accel_range (motion.accel_range);
    sprintf (buffer, "Set mpu_accel_range to %d", motion.accel_range);
    success = true;
  }
  else if (!strcmp(parameter, "mpu_gyro_range")) { 
    motion.gyro_range = atoi(value);
    mpu_set_gyro_range (motion.gyro_range);
    sprintf (buffer, "Set mpu_gyro_range to %d", motion.gyro_range);
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
  #endif
  #ifdef RADIO
  else if (!strcmp(parameter, "radio_enable")) { 
    config_this->radio_enable = atoi(value);
    sprintf (buffer, "Set radio_enable to %s", config_this->radio_enable?"true":"false");
    success = true;
  }  
  else if (!strcmp(parameter, "radio_enabled")) { 
    tm_this->radio_enabled = atoi(value);
    sprintf (buffer, "Set radio_enabled to %s", tm_this->radio_enabled?"true":"false");
    success = true;
  }  
  #endif
  #ifdef PRESSURE
  else if (!strcmp(parameter, "pressure_enable")) { 
    config_this->pressure_enable = atoi(value);
    sprintf (buffer, "Set pressure_enable to %s", config_this->pressure_enable?"true":"false");
    success = true;
  }  
  else if (!strcmp(parameter, "temperature_enable")) { 
    config_this->temperature_enable = atoi(value);
    sprintf (buffer, "Set temperature_enable to %s", config_this->temperature_enable?"true":"false");
    success = true;
  }  
  else if (!strcmp(parameter, "pressure_enabled")) { 
    tm_this->pressure_enabled = atoi(value);
    sprintf (buffer, "Set pressure_enabled to %s", tm_this->pressure_enabled?"true":"false");
    success = true;
  }  
  #endif
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
  else if (!strcmp(parameter, "camera_rate")) { 
    config_this->camera_rate = atoi(value);
    if (config_this->camera_rate) {
      var_timer.camera_interval = (1000 / config_this->camera_rate);
      sprintf (buffer, "Set camera_rate to %u Hz", config_this->camera_rate);
    }
    else {
      tm_this->camera_enabled = false;
      sprintf (buffer, "Set camera_enabled to false");
    }  
    success = true;
  } 
  #endif
  publish_udp_text (buffer);
  return (success);
}

void set_opsmode (uint8_t default_opsmode) {
  File lock_file;
  uint8_t opsmode  = default_opsmode;
  bool frozen = false;
  switch (tm_this->buffer_fs) {
  case FS_LITTLEFS: 
              #ifndef ESP_ARDUINO_VERSION_MAJOR // ESP32 core v1.0.x  
              if (LITTLEFS.exists(lock_filename)) {
                lock_file = LITTLEFS.open(lock_filename, "r");
              #else
              if (LittleFS.exists(lock_filename)) {
                lock_file = LittleFS.open(lock_filename, "r");
              #endif
                lock_file.read(&opsmode, 1);
                lock_file.close();
                frozen = true;
              }
              break;
  #ifdef PLATFORM_ESP32CAM
  case FS_SD_MMC:   
              #ifndef ESP_ARDUINO_VERSION_MAJOR // ESP32 core v1.0.x  
              if (LITTLEFS.exists(lock_filename)) {
              #else
              if (LittleFS.exists(lock_filename)) {
              #endif
                lock_file = SD_MMC.open(lock_filename, "r");
                lock_file.read(&opsmode, 1);
                lock_file.close();
                frozen = true;
              }
              break;
  #endif
  }
  tm_this->opsmode = opsmode;
  sprintf(buffer, "Set %s opsmode to '%s' (%s)", subsystemName[SS_THIS], modeName[opsmode], frozen?"frozen":"not frozen");
  publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);  
}

// WIFI FUNCTIONALITY

bool wifi_setup () {
  bool return_wifi_ap = true;
  bool return_wifi_sta = true;
  bool return_wifi_yamcs = true;
  WiFi.mode(WIFI_AP_STA); 
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  #ifdef PLATFORM_ESP32
  WiFi.setHostname("fli3d-esp32");
  #endif
  #ifdef PLATFORM_ESP32CAM
  WiFi.setHostname("fli3d-esp32cam");
  #endif
  if (config_this->wifi_ap_enable) {
    return_wifi_ap = wifi_ap_setup ();
    tm_this->wifi_enabled = true;
  }
  if (config_this->wifi_sta_enable) {
    return_wifi_sta = wifi_sta_setup ();
    tm_this->wifi_enabled = true;
  }
  if (config_this->wifi_yamcs_enable) {
    sprintf (buffer, "Sending CCSDS telemetry to UDP port %s:%u", config_network.yamcs_server, config_network.yamcs_tm_port);
    publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);
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
  uint8_t wifi_timeout_ctr = 0;
  uint8_t configured_wifi_id = 0;
  uint8_t scanned_wifi_id = 0;
  bool wifi_detected = false;
  int scanned_wifi_num = WiFi.scanNetworks();
  sprintf (buffer, "%s found %u WiFi networks", subsystemName[SS_THIS], scanned_wifi_num);
  publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);
  while (scanned_wifi_id < scanned_wifi_num) {
    if (!strcmp(config_network.wifi_ssid, WiFi.SSID(scanned_wifi_id).c_str())) {
      wifi_detected = true;
      wifi_timeout_ctr = 0;
      WiFi.begin(config_network.wifi_ssid, config_network.wifi_password);
      while (wifi_timeout_ctr++ < WIFI_TIMEOUT and WiFi.status() != WL_CONNECTED) {
        delay (1000);
      }
      if (WiFi.status() != WL_CONNECTED) {
        sprintf (buffer, "%s tired of trying to connect to default WiFi network %s after %u seconds", subsystemName[SS_THIS], config_network.wifi_ssid, WIFI_TIMEOUT);
        publish_event (STS_THIS, SS_THIS, EVENT_WARNING, buffer);
      }
    }
    scanned_wifi_id++;
  }
  if (!wifi_detected) {
    sprintf (buffer, "%s did not detect default WiFi network %s", subsystemName[SS_THIS], config_network.wifi_ssid);
    publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer); 
  }
  if (WiFi.status() != WL_CONNECTED) {
  	// try all configured networks
    while (configured_wifi_id < NUMBER_OF_WIFI and WiFi.status() != WL_CONNECTED) {
      scanned_wifi_id = 0;
      wifi_detected = false;
      while (scanned_wifi_id < scanned_wifi_num and WiFi.status() != WL_CONNECTED) {
        if (!strcmp(default_wifi_ssid[configured_wifi_id], WiFi.SSID(scanned_wifi_id).c_str())) {
          wifi_detected = true;
          WiFi.begin(default_wifi_ssid[configured_wifi_id], default_wifi_password[configured_wifi_id]); 
          wifi_timeout_ctr = 0;
          while (wifi_timeout_ctr++ < WIFI_TIMEOUT and WiFi.status() != WL_CONNECTED) {
            delay (1000);
          }
          if (WiFi.status() != WL_CONNECTED) {
            sprintf (buffer, "%s tired of trying to connect to %s WiFi network", subsystemName[SS_THIS], default_wifi_ssid[configured_wifi_id]);
            publish_event (STS_THIS, SS_THIS, EVENT_WARNING, buffer);
          }
        }
        scanned_wifi_id++;
      }
      if (!wifi_detected) {
        sprintf (buffer, "%s did not detect %s WiFi network", subsystemName[SS_THIS], default_wifi_ssid[configured_wifi_id]);
        publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);      	  
      }
      configured_wifi_id++;
    }
  }
  if (WiFi.status() == WL_CONNECTED) {
  	if (configured_wifi_id-- != 0) {
      strcpy (config_network.wifi_ssid, default_wifi_ssid[configured_wifi_id]);
      strcpy (config_network.udp_server, default_udp_server[configured_wifi_id]);
      strcpy (config_network.yamcs_server, default_yamcs_server[configured_wifi_id]);
      strcpy (config_network.ntp_server, default_ntp_server[configured_wifi_id]);
    }
    sprintf (buffer, "%s connected to WiFi network %s with IP %s", subsystemName[SS_THIS], config_network.wifi_ssid, WiFi.localIP().toString().c_str());
    publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);
    if (config_this->wifi_udp_enable) {
      tm_this->wifi_udp_enabled = true;
    }
    if (config_this->wifi_yamcs_enable) {
      tm_this->wifi_yamcs_enabled = true;
    }
    tm_this->wifi_connected = true;
  } 
  timeClient.begin();
  ntp_check();
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
      tm_this->warn_wifi_connloss = false;
    }
    return true;
  }
}

bool ntp_check () {
  if (timeClient.update() and !tm_this->time_set) {
    sprintf (buffer, "Time on %s set via NTP server %s: %s", subsystemName[SS_THIS], config_network.ntp_server, timeClient.getFormattedDate().c_str());
    publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);
    tm_this->time_set = true;
    config_this->boot_epoch = timeClient.getEpochTime() - millis()/1000;
    #ifdef PLATFORM_ESP32CAM
    if (tm_this->sd_enabled) {
      create_today_dir (FS_SD_MMC); 
    }
    #endif
    if (tm_this->fs_enabled) {
      create_today_dir (FS_LITTLEFS);
    }
    return true;
  }
  return false;
}

// TM/TC FUNCTIONALITY (serial/udp/yamcs/fs/sd)

void publish_packet (ccsds_t* ccsds_ptr) { 
  static uint32_t start_millis;
  static uint16_t PID;

  if (tm_this->opsmode != MODE_MAINTENANCE) {
    PID = update_packet (ccsds_ptr);
    ccsds_archive.packet_saved = false;
    // SD-card (potentially needed for packet recovery, so needs to be first in line)
    #ifdef PLATFORM_ESP32CAM
    start_millis = millis();
    if (routing_sd_json[PID] and config_this->sd_enable and tm_this->sd_json_enabled) {
      publish_file (FS_SD_MMC, ENC_JSON, ccsds_ptr);
    }
    if (routing_sd_ccsds[PID] and config_this->sd_enable and tm_this->sd_ccsds_enabled) {
      publish_file (FS_SD_MMC, ENC_CCSDS, ccsds_ptr);
    }
    timer_this->publish_sd_duration += millis() - start_millis;
    #endif
    // FS (potentially needed for packet recovery, so needs to be first in line)
    if (routing_fs[PID] and tm_this->fs_enabled) {
      start_millis = millis();
      publish_file (FS_LITTLEFS, ENC_CCSDS, ccsds_ptr);
      timer_this->publish_fs_duration += millis() - start_millis;
    }
    // serial
    #ifdef SERIAL_TCTM
    if (routing_serial[PID]) {
      start_millis = millis();
      publish_serial (ccsds_ptr);
      timer_this->publish_serial_duration += millis() - start_millis;
    }
    #endif
    // Yamcs
    if (routing_yamcs[PID] and config_this->wifi_enable and config_this->wifi_yamcs_enable) {
      start_millis = millis();
      publish_yamcs (ccsds_ptr);
      timer_this->publish_yamcs_duration += millis() - start_millis;
    }
    // UDP
    if (routing_udp[PID] and config_this->wifi_enable and config_this->wifi_udp_enable) {
      start_millis = millis();
      publish_udp (ccsds_ptr);
      timer_this->publish_udp_duration += millis() - start_millis;
    }
    // radio
    #ifdef PLATFORM_ESP32
    if (tm_this->radio_enabled and PID == TM_RADIO) {
      publish_radio ();
    }
    #endif
    reset_packet (ccsds_ptr);
  }
}

void publish_event (uint16_t PID, uint8_t subsystem, uint8_t event_type, const char* event_message) {
  Serial.println(event_message);
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
                         set_ccsds_payload_len ((ccsds_t*)&sts_esp32, strlen (event_message) + 7);
                         publish_packet ((ccsds_t*)&sts_esp32);
                         break; 
    case STS_ESP32CAM:   sts_esp32cam.subsystem = subsystem;
                         sts_esp32cam.type = event_type;
                         strcpy (sts_esp32cam.message, event_message);
                         set_ccsds_payload_len ((ccsds_t*)&sts_esp32cam, strlen (event_message) + 7);
                         publish_packet ((ccsds_t*)&sts_esp32cam);
                         break;
  }
}

bool open_file_ccsds (uint8_t filesystem) { 
  if (!file_ccsds) {
    switch (filesystem) {
    case FS_LITTLEFS: 
                        #ifndef ESP_ARDUINO_VERSION_MAJOR // ESP32 core v1.0.x  
                        file_ccsds = LITTLEFS.open(ccsds_path_buffer, "a+");
                        #else
                        file_ccsds = LittleFS.open(ccsds_path_buffer, "a+");
                        #endif
                        break;
    #ifdef PLATFORM_ESP32CAM
    case FS_SD_MMC:     file_ccsds = SD_MMC.open(ccsds_path_buffer, "a+");
                        break;
    #endif
    }
    if (!file_ccsds) {
      sprintf (buffer, "Failed to open '%s' on %s in append/read mode", ccsds_path_buffer, fsName[filesystem]);
      publish_event (STS_THIS, SS_THIS, EVENT_ERROR, buffer);
      switch (filesystem) {
      case FS_LITTLEFS: tm_this->err_fs_dataloss = true;
                        break;
      #ifdef PLATFORM_ESP32CAM
      case FS_SD_MMC:   tm_this->sd_ccsds_enabled = false;
                        tm_this->err_sd_dataloss = true;
                        break;
      #endif
      }
      if (tm_this->buffer_fs == filesystem) {
        tm_this->buffer_fs = FS_NONE;
      }
      return false;
    }
  }
  return true;
}

bool open_file_json (uint8_t filesystem) { 
  if (!file_json) {
    switch (filesystem) {
    case FS_LITTLEFS: 
                        #ifndef ESP_ARDUINO_VERSION_MAJOR // ESP32 core v1.0.x  
                        file_json = LITTLEFS.open(json_path_buffer, "a+");
                        #else
                        file_json = LittleFS.open(json_path_buffer, "a+");
                        #endif
                        break;
    #ifdef PLATFORM_ESP32CAM
    case FS_SD_MMC:   file_json = SD_MMC.open(json_path_buffer, "a+");
                          break;
    #endif
    }
    if (!file_json) {
      sprintf (buffer, "Failed to open '%s' on %s in append/read mode", json_path_buffer, fsName[filesystem]);
      publish_event (STS_THIS, SS_THIS, EVENT_ERROR, buffer);
      switch (filesystem) {
      case FS_LITTLEFS: tm_this->err_fs_dataloss = true;
                        break;
      #ifdef PLATFORM_ESP32CAM
      case FS_SD_MMC:   tm_this->sd_json_enabled = false;
                        tm_this->err_sd_dataloss = true;
                        break;
      #endif
      }
      return false;
    }
  }
  return true;
}

bool sync_file_ccsds () { // to be executed periodically to avoid data loss
  static uint32_t start_millis;
  if (file_ccsds) {
    start_millis = millis();
    file_ccsds.close();
    switch (config_this->buffer_fs) {
    case FS_LITTLEFS: tm_this->fs_active = true;
                      timer_this->publish_fs_duration += millis() - start_millis;
                      break;
    #ifdef PLATFORM_ESP32CAM
    case FS_SD_MMC:   tm_this->sd_active = true;
                      timer_this->publish_sd_duration += millis() - start_millis;
                      break;
    #endif
    }               
  }
  return true;
}

bool sync_file_json () { // to be executed periodically to avoid data loss
  static uint32_t start_millis;
  if (file_json) {
    start_millis = millis();
    file_json.close();
    switch (config_this->buffer_fs) {
    case FS_LITTLEFS: tm_this->fs_active = true;
                      timer_this->publish_fs_duration += millis() - start_millis;
                      break;
    #ifdef PLATFORM_ESP32CAM
    case FS_SD_MMC:   tm_this->sd_active = true;
                      timer_this->publish_sd_duration += millis() - start_millis;
                      break;
    #endif
    }    
  }
  return true;
}

bool publish_file (uint8_t filesystem, uint8_t encoding, ccsds_t* ccsds_ptr) {
  static uint16_t packet_len;
  packet_len = get_ccsds_packet_len(ccsds_ptr);
  if (filesystem == FS_LITTLEFS and tm_this->fs_enabled and open_file_ccsds (FS_LITTLEFS)) {
    if (config_this->buffer_fs == FS_LITTLEFS) {
      file_ccsds.write ((const uint8_t*)ccsds_ptr, packet_len);
      ccsds_archive.packet_len = packet_len;
      ccsds_archive.packet_offset = file_ccsds.position() - packet_len;
      ccsds_archive.packet_saved = true;
      tm_this->buffer_fs = filesystem;
      tm_this->buffer_active = true;
    }
    else {
      file_ccsds.write ((const uint8_t*)ccsds_ptr, packet_len);
    }
    tm_this->fs_active = true;
    tm_this->fs_rate++;
    return true;
  }
  #ifdef PLATFORM_ESP32CAM
  else if (filesystem == FS_SD_MMC and tm_this->sd_enabled and encoding == ENC_CCSDS and open_file_ccsds (FS_SD_MMC)) {
    if (config_this->buffer_fs == FS_SD_MMC) {
      file_ccsds.write ((const uint8_t*)ccsds_ptr, packet_len);
      ccsds_archive.packet_len = packet_len;
      ccsds_archive.packet_saved = true;
      ccsds_archive.packet_offset = file_ccsds.position() - packet_len;
      tm_this->buffer_fs = filesystem;
      tm_this->buffer_active = true;
    }
    else {
      file_ccsds.write ((const uint8_t*)ccsds_ptr, packet_len);
    }
    tm_this->sd_active = true;
    tm_this->sd_ccsds_rate++;
    return true;
  }
  else if (filesystem == FS_SD_MMC and tm_this->sd_enabled and encoding == ENC_JSON and open_file_json (FS_SD_MMC)) {
    build_json_str (buffer, ccsds_ptr);
    file_json.println (buffer);
    tm_this->sd_active = true;
    tm_this->sd_json_rate++;
    return true;
  }
  #endif
  else {
    return false;
  }
}

bool publish_serial (ccsds_t* ccsds_ptr) { 
  static LinkedList<buffer_t*> serial_out_buffer;
  static buffer_t* serial_out_buffer_entry;

  if (tm_this->serial_connected) {
    // we can publish now
    if (serial_out_buffer.size() == 0) {
      // publish real-time
      switch ((uint8_t)config_this->serial_format) {
        case ENC_JSON:  build_json_str ((char*)&buffer, ccsds_ptr);
                        /*if (serialTransfer.available()) {
                            serialTransfer.sendDatum(buffer, strlen(buffer));
                        }
                        else {
                            Serial.println(buffer);
                        } */
                        publish_udp_text(buffer);
                        break;
        case ENC_CCSDS: /* if (serialTransfer.available()) {
                            serialTransfer.sendDatum((const uint8_t*)ccsds_ptr, get_ccsds_packet_len(ccsds_ptr));
                        } */
                        break;
      }
      tm_this->serial_out_rate++;
      var_timer.last_serial_out_millis = millis();
      return true; 
    }
    else {
      // there's a buffer to empty first
      if (open_file_ccsds (config_this->buffer_fs)) {
        // first safely add the new packet to the buffer
        serial_out_buffer_entry = (buffer_t*)malloc(sizeof(buffer_t));
        serial_out_buffer_entry->packet_len = ccsds_archive.packet_len;
        serial_out_buffer_entry->packet_offset = ccsds_archive.packet_offset;
        serial_out_buffer.add(serial_out_buffer_entry);
        // then replay buffer
        uint8_t replay_count = 0;
        while (serial_out_buffer.size() and replay_count++ < BUFFER_RELEASE_BATCH_SIZE) {
          serial_out_buffer_entry = serial_out_buffer.remove(0);
          file_ccsds.seek(serial_out_buffer_entry->packet_offset);         
          file_ccsds.read((uint8_t*)&replayed_ccsds, serial_out_buffer_entry->packet_len);
          if (valid_ccsds_hdr (&replayed_ccsds, PKT_TM)) {
            // good packet recovered from buffer, publish
            switch ((uint8_t)config_this->serial_format) {
              case ENC_JSON:  build_json_str ((char*)&buffer, &replayed_ccsds);
                              /*if (serialTransfer.available()) {
                                  serialTransfer.sendDatum(buffer, strlen(buffer));
                              }
                              else {
                                  Serial.println(buffer);
                              }*/
                              publish_udp_text(buffer);
                              break;
              case ENC_CCSDS: /* if (serialTransfer.available()) {
                                  serialTransfer.sendDatum((const uint8_t*)&replayed_ccsds, get_ccsds_packet_len(&replayed_ccsds));
                              } */
                              break;
            }            
            tm_this->serial_out_rate++;          
          }
          else {
            // archive corruption
            tm_this->err_serial_dataloss = true;            
            sprintf (buffer, "Got invalid CCSDS packet when reading packet from buffer");
            publish_event (STS_THIS, SS_THIS, EVENT_WARNING, buffer);             
          }
          free (serial_out_buffer_entry);
        }         
        tm_this->fs_active = true;
        tm_this->serial_out_buffer = min (255, serial_out_buffer.size());
        return true;
      }
      else {
        // cannot open buffer file: dataloss!
        tm_this->serial_out_buffer = 0; // TODO: release LL and pointer memory
        tm_this->err_serial_dataloss = true;
        return false;
      }
    } 
  }
  else {
    // no serial, we cannot publish now
    if (ccsds_archive.packet_saved) {
      // packet was stored on fs so we can add it to buffer index
      serial_out_buffer_entry = (buffer_t*)malloc(sizeof(buffer_t));
      serial_out_buffer_entry->packet_len = ccsds_archive.packet_len;
      serial_out_buffer_entry->packet_offset = ccsds_archive.packet_offset;
      serial_out_buffer.add(serial_out_buffer_entry);
      tm_this->serial_out_buffer = min (255, serial_out_buffer.size());
      return true;
    }
    else {
      // packet not stored on fs for buffer: dataloss!
      tm_this->serial_out_buffer = 0; // TODO: release LL and pointer memory
      tm_this->err_serial_dataloss = true;
      return false;
    }
  } 
}

bool publish_yamcs (ccsds_t* ccsds_ptr) { 
  static LinkedList<buffer_t*> yamcs_buffer;
  static buffer_t* yamcs_buffer_entry;

  if (tm_this->wifi_connected) {
    // we can publish now
    if (tm_this->opsmode == MODE_NOMINAL or yamcs_buffer.size() == 0) {
      // publish real-time
      wifiUDP.beginPacket(config_network.yamcs_server, config_network.yamcs_tm_port);
      wifiUDP.write ((const uint8_t*)ccsds_ptr, get_ccsds_packet_len (ccsds_ptr));
      wifiUDP.endPacket();
      tm_this->yamcs_rate++;
      return true; 
    }
    else {
      // there's a buffer to empty first
      if (open_file_ccsds (config_this->buffer_fs)) {
        // first safely add the new packet to the buffer
        yamcs_buffer_entry = (buffer_t*)malloc(sizeof(buffer_t));
        yamcs_buffer_entry->packet_len = ccsds_archive.packet_len;
        yamcs_buffer_entry->packet_offset = ccsds_archive.packet_offset;
        yamcs_buffer.add(yamcs_buffer_entry);
        // then replay buffer
        uint8_t replay_count = 0;
        while (yamcs_buffer.size() and replay_count++ < BUFFER_RELEASE_BATCH_SIZE) {
          yamcs_buffer_entry = yamcs_buffer.remove(0);
          file_ccsds.seek(yamcs_buffer_entry->packet_offset);         
          file_ccsds.read((uint8_t*)&replayed_ccsds, yamcs_buffer_entry->packet_len);
          if (valid_ccsds_hdr (&replayed_ccsds, PKT_TM)) {
            // good packet recovered from buffer, publish
            wifiUDP.beginPacket(config_network.yamcs_server, config_network.yamcs_tm_port);
            wifiUDP.write ((const uint8_t*)&replayed_ccsds, yamcs_buffer_entry->packet_len);
            wifiUDP.endPacket();
            tm_this->yamcs_rate++;          
          }
          else {
            // archive corruption
            tm_this->err_yamcs_dataloss = true;            
            sprintf (buffer, "Got invalid CCSDS packet when reading packet from fs");
            publish_event (STS_THIS, SS_THIS, EVENT_WARNING, buffer);             
          }
          free (yamcs_buffer_entry);
        }         
        tm_this->fs_active = true;
        tm_this->yamcs_buffer = min (255, yamcs_buffer.size());
        return true;
      }
      else {
        // cannot open buffer file: dataloss!
        tm_this->yamcs_buffer = 0; // TODO: release LL and pointer memory
        tm_this->err_yamcs_dataloss = true;
        return false;
      }
    } 
  }
  else {
    // no wifi, we cannot publish now
    if (ccsds_archive.packet_saved) {
      // packet was stored on fs so we can add it to buffer index
      yamcs_buffer_entry = (buffer_t*)malloc(sizeof(buffer_t));
      yamcs_buffer_entry->packet_len = ccsds_archive.packet_len;
      yamcs_buffer_entry->packet_offset = ccsds_archive.packet_offset;
      yamcs_buffer.add(yamcs_buffer_entry);
      tm_this->yamcs_buffer = min (255, yamcs_buffer.size());
      return true;
    }
    else {
      // packet not stored on fs for buffer: dataloss!
      tm_this->yamcs_buffer = 0; // TODO: release LL and pointer memory
      tm_this->err_yamcs_dataloss = true;
      return false;
    }
  } 
}

bool publish_udp (ccsds_t* ccsds_ptr) { 
  if (tm_this->wifi_connected) { // TODO: and publish_udp_enabled???
    build_json_str ((char*)&buffer, ccsds_ptr);
    wifiUDP.beginPacket(config_network.udp_server, config_network.udp_port);
    wifiUDP.println (buffer);
    wifiUDP.endPacket();   
    tm_this->udp_rate++;
    return true;
  }
  else {
    return false;
  }
}

bool publish_udp_text (const char* message) { 
  if (tm_this->wifi_connected) { // TODO: and publish_udp_enabled???
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
    return true;
  }
  else {
    sprintf (buffer, "Fail to listen for commands on UDP port %s:%u", WiFi.localIP().toString().c_str(), config_network.yamcs_tc_port);
    publish_event (STS_THIS, SS_THIS, EVENT_ERROR, buffer);  
    return false;
  }
}

bool yamcs_tc_check () {
  if (wifiUDP_yamcs_tc.parsePacket()) {
    Serial.println ("Received command");
    wifiUDP_yamcs_tc.read((char*)&ccsds_tc_buffer, BUFFER_MAX_SIZE);
    parse_ccsds ((ccsds_t*)&ccsds_tc_buffer);
    return true;
  }
  else {
    return false;
  }
}
#endif

uint16_t update_packet (ccsds_t* ccsds_ptr) {
  static uint16_t PID;
  ((ccsds_hdr_t*)ccsds_ptr)->seq_ctr_L++;
  if (((ccsds_hdr_t*)ccsds_ptr)->seq_ctr_L == 0) {
    ((ccsds_hdr_t*)ccsds_ptr)->seq_ctr_H++;
  }
  PID = get_ccsds_apid (ccsds_ptr) - 42;
  switch (PID) {
    #ifdef PLATFORM_ESP32
    case STS_ESP32:      sts_esp32.millis = millis();
                         sts_esp32.packet_ctr++;
                         break;                
    case TM_ESP32:       esp32.millis = millis();
                         esp32.packet_ctr++; 
                         esp32.mem_free = ESP.getFreeHeap()/1024;
                         esp32.fs_free = fs_free ();
                         // compensate for this packet being prepared before it is actually sent (anticipating a successful send)
                         if (routing_fs[TM_ESP32] and tm_this->fs_enabled) {
                           //esp32.fs_rate++; // TODO: understand why
                         }
                         if (routing_serial[TM_ESP32]) {
                           //esp32.serial_out_rate++; // TODO: understand why
                         }
                         if (routing_yamcs[TM_ESP32] and config_this->wifi_enable and config_this->wifi_yamcs_enable) {
                           esp32.yamcs_rate++;
                         }
                         if (routing_udp[TM_ESP32] and tm_this->wifi_enabled and tm_this->wifi_udp_enabled) {
                           esp32.udp_rate++;
                         }
                         break;             
    case TM_GPS:         neo6mv2.packet_ctr++;
                         break;
    case TM_MOTION:      motion.packet_ctr++;
                         break;
    case TM_PRESSURE:    bmp280.packet_ctr++;
                         break;
    case TM_RADIO:       radio.millis = millis(); 
                         radio.packet_ctr++;
                         radio.opsmode = esp32.opsmode;
                         radio.error_ctr = min(255, esp32.error_ctr + esp32cam.error_ctr);
                         radio.warning_ctr = min(255, esp32.warning_ctr + esp32cam.warning_ctr);
                         radio.pressure_height = max(0, min(255, (bmp280.height+50)/100));
                         radio.pressure_velocity_v = int8_t((bmp280.velocity_v+((bmp280.velocity_v > 0) - (bmp280.velocity_v < 0))*50)/100);
                         radio.temperature = int8_t((bmp280.temperature+((bmp280.temperature > 0) - (bmp280.temperature < 0))*50)/100);
                         radio.motion_tilt = uint8_t((motion.tilt+50)/100);
                         radio.motion_g = uint8_t((motion.g+50)/100);  
                         radio.motion_a = int8_t((motion.a+((motion.a > 0) - (motion.a < 0))*50)/100);
                         radio.motion_rpm = int8_t((motion.rpm+((motion.rpm > 0) - (motion.rpm < 0))*50)/100); 
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
                         radio.esp32_buffer_active = esp32.buffer_active;
                         radio.separation_sts = esp32.separation_sts;
                         radio.esp32cam_serial_connected = esp32cam.serial_connected;
                         radio.esp32cam_wifi_connected = esp32cam.wifi_connected;
                         radio.esp32cam_warn_serial_connloss = esp32cam.warn_serial_connloss;
                         radio.esp32cam_warn_wifi_connloss = esp32cam.warn_wifi_connloss;
                         radio.esp32cam_err_serial_dataloss = esp32cam.err_serial_dataloss;
                         radio.esp32cam_err_yamcs_dataloss = esp32cam.err_yamcs_dataloss;
                         radio.esp32cam_err_fs_dataloss = esp32cam.err_fs_dataloss;
                         radio.esp32cam_err_sd_dataloss = esp32cam.err_sd_dataloss;
                         radio.esp32cam_buffer_active = esp32cam.buffer_active;
                         radio.esp32cam_sd_image_enabled = esp32cam.wifi_image_enabled;
                         break;
    case TIMER_ESP32:    timer_esp32.packet_ctr++;
                         timer_esp32.idle_duration = max(0, 1000 - timer_esp32.radio_duration - timer_esp32.pressure_duration - timer_esp32.motion_duration - timer_esp32.gps_duration - timer_esp32.esp32cam_duration - timer_esp32.serial_duration - timer_esp32.ota_duration - timer_esp32.ftp_duration - timer_esp32.wifi_duration - timer_esp32.tc_duration);
                         break;
    case TC_ESP32CAM:    // do nothing
                         break;
    #endif
    #ifdef PLATFORM_ESP32CAM
    case STS_ESP32CAM:   sts_esp32cam.millis = millis();
                         sts_esp32cam.packet_ctr++; 
                         break;
    case TM_ESP32CAM:    esp32cam.millis = millis();
                         esp32cam.packet_ctr++; 
                         esp32cam.mem_free = ESP.getFreeHeap()/1024;
                         esp32cam.fs_free = fs_free();
                         esp32cam.sd_free = sd_free();
                         // compensate for this packet being prepared before it is actually sent (anticipating a successful send)
                         if (routing_fs[TM_ESP32CAM] and tm_this->fs_enabled) {
                           esp32cam.fs_rate++;
                         }
                         if (routing_serial[TM_ESP32CAM]) {
                           //esp32cam.serial_out_rate++;
                         }
                         if (routing_yamcs[TM_ESP32CAM] and config_this->wifi_enable and config_this->wifi_yamcs_enable) {
                           esp32cam.yamcs_rate++;
                         }
                         if (routing_udp[TM_ESP32CAM] and tm_this->wifi_enabled and tm_this->wifi_udp_enabled) {
                           esp32cam.udp_rate++;
                         }
                         if (routing_sd_json[TM_ESP32CAM] and tm_this->sd_enabled and tm_this->sd_json_enabled) {
                           esp32cam.sd_json_rate++;
                         }
                         if (routing_sd_ccsds[TM_ESP32CAM] and tm_this->sd_enabled and tm_this->sd_ccsds_enabled) {
                           esp32cam.sd_ccsds_rate++;
                         }
                         break;     
    case TM_CAMERA:      ov2640.packet_ctr++; 
                         break;     
    case TIMER_ESP32CAM: timer_esp32cam.packet_ctr++;
                         timer_esp32cam.idle_duration = max(0, 1000 - timer_esp32cam.sd_duration - timer_esp32cam.camera_duration - timer_esp32cam.serial_duration - timer_esp32cam.ftp_duration - timer_esp32cam.wifi_duration - timer_esp32cam.tc_duration);
                         break;                     
    case TC_ESP32:       // do nothing
                         break;
    #endif
  }
  return (PID);
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
                         //esp32.warn_serial_connloss = false;
                         //esp32.warn_wifi_connloss = false;
                         //esp32.err_serial_dataloss = false;
                         //esp32.err_yamcs_dataloss = false;    
                         //esp32.err_fs_dataloss = false;    
                         esp32.radio_active = false;
                         esp32.pressure_active = false;
                         esp32.motion_active = false;
                         esp32.gps_active = false; 
                         esp32.camera_active = false;
                         esp32.fs_active = false;
                         esp32.ftp_active = false;
                         esp32.buffer_active = false;
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
    case TIMER_ESP32:    timer_esp32.radio_duration = 0;
                         timer_esp32.pressure_duration = 0;
                         timer_esp32.motion_duration = 0;
                         timer_esp32.gps_duration = 0;
                         timer_esp32.esp32cam_duration = 0;
                         timer_esp32.serial_duration = 0;
                         timer_esp32.ota_duration = 0;
                         timer_esp32.ftp_duration = 0;
                         timer_esp32.wifi_duration = 0;
                         timer_esp32.tc_duration = 0;
                         timer_esp32.idle_duration = 0;
                         timer_esp32.publish_fs_duration = 0;
                         timer_esp32.publish_serial_duration = 0;
                         timer_esp32.publish_yamcs_duration = 0;
                         timer_esp32.publish_udp_duration = 0;
                         break;    
    case TC_ESP32CAM:    tc_esp32cam.parameter[0] = 0;
                         break;
    #endif
    #ifdef PLATFORM_ESP32CAM
    case STS_ESP32CAM:   sts_esp32cam.message[0] = 0;
                         break;
    case TM_ESP32CAM:    esp32cam.camera_rate = 0;
                         esp32cam.udp_rate = 0;
                         esp32cam.yamcs_rate = 0;
                         esp32cam.serial_in_rate = 0;
                         esp32cam.serial_out_rate = 0;
                         esp32cam.fs_rate = 0;
                         esp32cam.sd_json_rate = 0;
                         esp32cam.sd_ccsds_rate = 0;
                         esp32cam.sd_image_rate = 0;
                         //esp32cam.warn_serial_connloss = false;
                         //esp32cam.warn_wifi_connloss = false;
                         //esp32cam.err_serial_dataloss = false;
                         //esp32cam.err_yamcs_dataloss = false;    
                         //esp32cam.err_fs_dataloss = false;    
                         //esp32cam.err_sd_dataloss = false;    
                         esp32cam.camera_active = false;
                         esp32cam.fs_active = false;
                         esp32cam.sd_active = false;
                         esp32cam.ftp_active = false;
                         esp32cam.http_active = false;
                         esp32cam.buffer_active = false; 
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
                         break;
    case TIMER_ESP32CAM: timer_esp32cam.camera_duration = 0;
                         timer_esp32cam.serial_duration = 0;
                         timer_esp32cam.tc_duration = 0;
                         timer_esp32cam.sd_duration = 0;
                         timer_esp32cam.ftp_duration = 0;
                         timer_esp32cam.wifi_duration = 0;
                         timer_esp32cam.idle_duration = 0;
                         timer_esp32cam.publish_sd_duration = 0;
                         timer_esp32cam.publish_fs_duration = 0;
                         timer_esp32cam.publish_serial_duration = 0;
                         timer_esp32cam.publish_yamcs_duration = 0;
                         timer_esp32cam.publish_udp_duration = 0;
                         break;                     
    case TC_ESP32:       tc_esp32.parameter[0] = 0;
                         break;
    #endif
  }
}

// CCSDS FUNCTIONALITY

void ccsds_init () {
  ccsds_hdr_init ((ccsds_t*)&sts_esp32, STS_ESP32, PKT_TM, sizeof (sts_esp32_t));
  ccsds_hdr_init ((ccsds_t*)&sts_esp32cam, STS_ESP32CAM, PKT_TM, sizeof (sts_esp32cam_t));
  ccsds_hdr_init ((ccsds_t*)&esp32, TM_ESP32, PKT_TM, sizeof (tm_esp32_t));
  ccsds_hdr_init ((ccsds_t*)&esp32cam, TM_ESP32CAM, PKT_TM, sizeof (tm_esp32cam_t));
  ccsds_hdr_init ((ccsds_t*)&ov2640, TM_CAMERA, PKT_TM, sizeof (tm_camera_t));
  ccsds_hdr_init ((ccsds_t*)&neo6mv2, TM_GPS, PKT_TM, sizeof (tm_gps_t));
  ccsds_hdr_init ((ccsds_t*)&motion, TM_MOTION, PKT_TM, sizeof (tm_motion_t));
  ccsds_hdr_init ((ccsds_t*)&bmp280, TM_PRESSURE, PKT_TM, sizeof (tm_pressure_t));
  ccsds_hdr_init ((ccsds_t*)&radio, TM_RADIO, PKT_TM, sizeof (tm_radio_t));
  ccsds_hdr_init ((ccsds_t*)&timer_esp32, TIMER_ESP32, PKT_TM, sizeof (timer_esp32_t));
  ccsds_hdr_init ((ccsds_t*)&timer_esp32cam, TIMER_ESP32CAM, PKT_TM, sizeof (timer_esp32cam_t));
  ccsds_hdr_init ((ccsds_t*)&tc_esp32, TC_ESP32, PKT_TC, sizeof (tc_esp32_t));
  ccsds_hdr_init ((ccsds_t*)&tc_esp32cam, TC_ESP32CAM, PKT_TC, sizeof (tc_esp32cam_t));
}

void ccsds_hdr_init (ccsds_t* ccsds_ptr, uint16_t PID, uint8_t pkt_type, uint16_t pkt_len) {
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

uint16_t get_ccsds_packet_ctr (ccsds_t* ccsds_ptr) {
  return (256*((ccsds_hdr_t*)ccsds_ptr)->seq_ctr_H + ((ccsds_hdr_t*)ccsds_ptr)->seq_ctr_L);
}

uint16_t get_ccsds_packet_len (ccsds_t* ccsds_ptr) {
  return (sizeof (ccsds_hdr_t) + 256*((ccsds_hdr_t*)ccsds_ptr)->pkt_len_H + ((ccsds_hdr_t*)ccsds_ptr)->pkt_len_L + 1);
}

void set_ccsds_payload_len (ccsds_t* ccsds_ptr, uint16_t len) {
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
      case STS_OTHER:      memcpy (sts_other, ccsds_ptr, get_ccsds_packet_len(ccsds_ptr));
                           publish_packet ((ccsds_t*)sts_other);
                           break;
      case TM_OTHER:       memcpy (tm_this, ccsds_ptr, get_ccsds_packet_len(ccsds_ptr));
                           publish_packet ((ccsds_t*)tm_other);
                           break;
      #ifdef PLATFORM_ESP32
      case TM_CAMERA:      memcpy (&ov2640, ccsds_ptr, sizeof(tm_camera_t));
                           publish_packet ((ccsds_t*)&ov2640);
                           break;
      case TIMER_ESP32CAM: memcpy (&timer_esp32cam, ccsds_ptr, sizeof(timer_esp32cam_t));
                           publish_packet ((ccsds_t*)&timer_esp32cam);
                           break;         
      #endif
      #ifdef PLATFORM_ESP32CAM
      case TM_GPS:         memcpy (&neo6mv2, ccsds_ptr, sizeof(tm_gps_t));
                           publish_packet ((ccsds_t*)&neo6mv2);
                           break;
      case TM_MOTION:      memcpy (&motion, ccsds_ptr, sizeof(tm_motion_t));
                           publish_packet ((ccsds_t*)&motion);
                           break;
      case TM_PRESSURE:    memcpy (&bmp280, ccsds_ptr, sizeof(tm_pressure_t));
                           publish_packet ((ccsds_t*)&bmp280);
                           break;
      case TM_RADIO:       memcpy (&radio, ccsds_ptr, sizeof(tm_radio_t));
                           publish_packet ((ccsds_t*)&radio);
                           break;
      case TIMER_ESP32:    memcpy (&timer_esp32, ccsds_ptr, sizeof(timer_esp32_t));
                           publish_packet ((ccsds_t*)&timer_esp32);
                           break;                           
      #endif
      default:             sprintf (buffer, "Received TM packet with unexpected APID %d", get_ccsds_apid (ccsds_ptr));
                           publish_event (STS_THIS, SS_THIS, EVENT_ERROR, buffer);
                           break;
    }
  }
  else if (valid_ccsds_hdr (ccsds_ptr, PKT_TC)) {
    switch (get_ccsds_apid (ccsds_ptr) - 42) {
      case TC_THIS:        memcpy (tc_this, ccsds_ptr, get_ccsds_packet_len(ccsds_ptr)+6);
                           sprintf (buffer, "Received TC for %s with cmd_id %u (%s), parameter [%s]", subsystemName[SS_THIS], tc_this->cmd_id, tcName[tc_this->cmd_id-42], (const char*)tc_this->parameter);
                           publish_event (STS_THIS, SS_THIS, EVENT_CMD_ACK, buffer);
                           switch (tc_this->cmd_id) {
                             case TC_REBOOT:             cmd_reboot ((uint8_t)tc_this->parameter[0]);
                                                         break;
                             case TC_SET_OPSMODE:        cmd_set_opsmode ((uint8_t)tc_this->parameter[0]);
                                                         break;
                             case TC_LOAD_CONFIG:        cmd_load_config ((const char*)tc_this->parameter);
                                                         break;                                                       
                             case TC_LOAD_ROUTING:       cmd_load_routing ((const char*)tc_this->parameter); 
                                                         break;
                             case TC_SET_PARAMETER:      cmd_set_parameter (tc_this->parameter, (char*)(tc_this->parameter + strlen(tc_this->parameter) + 1)); 
                                                         break;
                             case TC_FREEZE_OPSMODE:     cmd_freeze_opsmode ((uint8_t)tc_this->parameter[0]);
                                                         break;
                             default:                    sprintf (buffer,  "CCSDS command to %s not understood", subsystemName[SS_THIS]);
                                                         publish_event (STS_THIS, SS_THIS, EVENT_CMD_FAIL, buffer);
                                                         break;
                           }
                           break;
      case TC_OTHER:       memcpy (tc_other, ccsds_ptr, get_ccsds_packet_len(ccsds_ptr)+6);
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

void build_json_str (char* json_buffer, ccsds_t* ccsds_ptr) {
  uint16_t PID = get_ccsds_apid (ccsds_ptr) - 42;
  switch (PID) {
    case STS_ESP32:      { 
                           sts_esp32_t* sts_esp32_ptr = (sts_esp32_t*)ccsds_ptr;
                           sprintf (json_buffer, "{\"id\":\"%s\",\"ctr\":%u,\"millis\":%u,\"type\":\"%s\",\"ss\":\"%s\",\"msg\":\"%s\"}", 
                                    pidName[PID], sts_esp32_ptr->packet_ctr, sts_esp32_ptr->millis, 
                                    eventName[sts_esp32_ptr->type], subsystemName[sts_esp32_ptr->subsystem], sts_esp32_ptr->message);
                         }
                         break;
    case STS_ESP32CAM:   { 
                           sts_esp32cam_t* sts_esp32cam_ptr = (sts_esp32cam_t*)ccsds_ptr;
                           sprintf (json_buffer, "{\"id\":\"%s\",\"ctr\":%u,\"millis\":%u,\"type\":\"%s\",\"ss\":\"%s\",\"msg\":\"%s\"}", 
                                    pidName[PID], sts_esp32cam_ptr->packet_ctr, sts_esp32cam_ptr->millis, 
                                    eventName[sts_esp32cam_ptr->type], subsystemName[sts_esp32cam_ptr->subsystem], sts_esp32cam_ptr->message);
                         }
                         break;                  
    case TM_ESP32:       {
                           tm_esp32_t* esp32_ptr = (tm_esp32_t*)ccsds_ptr;
                           sprintf (json_buffer, "{\"id\":\"%s\",\"ctr\":%u,\"millis\":%u,\"mode\":\"%s\",\"state\":\"%s\",\"err\":%u,\"warn\":%u,\"tc\":{\"exec\":%u,\"fail\":%u},\"mem\":[%u,%u],\"buf\":[%u,%u],\"fs\":[%u,%u],\"ntp\":%u,\"inst_rate\":[%u,%u,%u,%u,%u],\"comm_rate\":[%u,%u,%u,%u,%u],\"sep\":%d,\"ena\":\"%d%d%d%d%d%d%d%d%d%d%d\",\"act\":\"%d%d%d%d%d%d%d%d\",\"conn\":{\"up\":\"%d%d\",\"warn\":\"%d%d\",\"err\":\"%d%d%d\"}}", 
                                    pidName[PID], esp32_ptr->packet_ctr, esp32_ptr->millis,  
                                    modeName[esp32_ptr->opsmode], stateName[esp32_ptr->state], esp32_ptr->error_ctr, esp32_ptr->warning_ctr, 
                                    esp32_ptr->tc_exec_ctr, esp32_ptr->tc_fail_ctr,
                                    esp32_ptr->mem_free, esp32_ptr->fs_free, 
                                    esp32_ptr->yamcs_buffer, esp32_ptr->serial_out_buffer, 
                                    esp32_ptr->ftp_fs, esp32_ptr->buffer_fs,
                                    esp32_ptr->time_set, 
                                    esp32_ptr->radio_rate, esp32_ptr->pressure_rate, esp32_ptr->motion_rate, esp32_ptr->gps_rate, esp32_ptr->camera_rate, esp32_ptr->udp_rate, esp32_ptr->yamcs_rate, esp32_ptr->serial_in_rate, esp32_ptr->serial_out_rate, esp32_ptr->fs_rate, 
                                    esp32_ptr->separation_sts,
                                    esp32_ptr->radio_enabled, esp32_ptr->pressure_enabled, esp32_ptr->motion_enabled, esp32_ptr->gps_enabled, esp32_ptr->camera_enabled, esp32_ptr->wifi_enabled, esp32_ptr->wifi_udp_enabled, esp32_ptr->wifi_yamcs_enabled, esp32_ptr->fs_enabled, esp32_ptr->ftp_enabled, esp32_ptr->time_set,
                                    esp32_ptr->radio_active, esp32_ptr->pressure_active, esp32_ptr->motion_active, esp32_ptr->gps_active, esp32_ptr->camera_active, esp32_ptr->fs_active, esp32_ptr->ftp_active, esp32_ptr->ota_enabled,
                                    esp32_ptr->serial_connected, esp32_ptr->wifi_connected, esp32_ptr->warn_serial_connloss, esp32_ptr->warn_wifi_connloss, esp32_ptr->err_serial_dataloss, esp32_ptr->err_yamcs_dataloss, esp32_ptr->err_fs_dataloss); 
                         }
                         break;
    case TM_ESP32CAM:    { // TODO: ota_enabled missing
                           tm_esp32cam_t* esp32cam_ptr = (tm_esp32cam_t*)ccsds_ptr;
                           sprintf (json_buffer, "{\"id\":\"%s\",\"ctr\":%u,\"millis\":%u,\"mode\":\"%s\",\"err\":%u,\"warn\":%u,\"tc\":{\"exec\":%u,\"fail\":%u},\"mem\":[%u,%u,%u],\"buf\":[%u,%u],\"fs\":[%u,%u],\"ntp\":%u,\"rate\":[%u,%u,%u,%u,%u,%u,%u,%u,%u],\"ena\":\"%d%d%d%d%d%d%d%d%d%d%d\",\"act\":\"%d%d%d%d\",\"conn\":{\"up\":\"%d%d\",\"warn\":\"%d%d\",\"err\":\"%d%d%d%d\"}}",
                                    pidName[PID], esp32cam_ptr->packet_ctr, esp32cam_ptr->millis,  
                                    cameraModeName[esp32cam_ptr->opsmode], esp32cam_ptr->error_ctr, esp32cam_ptr->warning_ctr, 
                                    esp32cam_ptr->tc_exec_ctr, esp32cam_ptr->tc_fail_ctr,
                                    esp32cam_ptr->mem_free, esp32cam_ptr->fs_free, esp32cam_ptr->sd_free, 
                                    esp32cam_ptr->yamcs_buffer, esp32cam_ptr->serial_out_buffer, 
                                    esp32cam_ptr->ftp_fs, esp32cam_ptr->buffer_fs, 
                                    esp32cam_ptr->time_set, 
                                    esp32cam_ptr->camera_rate, esp32cam_ptr->udp_rate, esp32cam_ptr->yamcs_rate, esp32cam_ptr->serial_in_rate, esp32cam_ptr->serial_out_rate, esp32cam_ptr->fs_rate, esp32cam_ptr->sd_json_rate, esp32cam_ptr->sd_ccsds_rate, esp32cam_ptr->sd_image_rate,  
                                    esp32cam_ptr->camera_enabled, esp32cam_ptr->wifi_enabled, esp32cam_ptr->wifi_udp_enabled, esp32cam_ptr->wifi_yamcs_enabled, esp32cam_ptr->wifi_image_enabled, esp32cam_ptr->fs_enabled, esp32cam_ptr->sd_enabled, esp32cam_ptr->ftp_enabled, esp32cam_ptr->sd_json_enabled, esp32cam_ptr->sd_ccsds_enabled, esp32cam_ptr->sd_image_enabled, 
                                    esp32cam_ptr->camera_active, esp32cam_ptr->fs_active, esp32cam_ptr->sd_active, esp32cam_ptr->ftp_active, 
                                    esp32cam_ptr->serial_connected, esp32cam_ptr->wifi_connected, esp32cam_ptr->warn_serial_connloss, esp32cam_ptr->warn_wifi_connloss, esp32cam_ptr->err_serial_dataloss, esp32cam_ptr->err_yamcs_dataloss, esp32cam_ptr->err_fs_dataloss, esp32cam_ptr->err_sd_dataloss); 
                         }
                         break;
    case TM_CAMERA:      {
                           tm_camera_t* ov2640_ptr = (tm_camera_t*)ccsds_ptr;
                           sprintf (json_buffer, "{\"id\":\"%s\",\"ctr\":%u,\"millis\":%u,\"mode\":\"%s\",\"res\":\"%s\",\"auto_res\":%d,\"file\":\"%s\",\"size\":%u,\"ms\":{\"exp\":%u,\"sd\":%u,\"wifi\":%u}}", 
                                    pidName[PID], ov2640_ptr->packet_ctr, ov2640_ptr->millis, 
                                    cameraModeName[ov2640_ptr->camera_mode], cameraResolutionName[ov2640_ptr->resolution], ov2640_ptr->auto_res, ov2640_ptr->filename, ov2640_ptr->filesize, ov2640_ptr->exposure_ms, ov2640_ptr->sd_ms, ov2640_ptr->wifi_ms);
                         }
                         break;
    case TM_GPS:         { 
                           tm_gps_t* neo6mv2_ptr = (tm_gps_t*)ccsds_ptr;
                           *json_buffer++ = '{';
                           json_buffer += sprintf (json_buffer, "\"id\":\"%s\",\"ctr\":%u,\"millis\":%u,\"sts\":\"%s\",\"sats\":%d", pidName[PID], neo6mv2_ptr->packet_ctr, neo6mv2_ptr->millis, gpsStatusName[neo6mv2_ptr->status], neo6mv2_ptr->satellites);
                           if (neo6mv2_ptr->time_valid) {
                             json_buffer += sprintf (json_buffer, ",\"time\":\"%02d:%02d:%02d.%02d\"", neo6mv2_ptr->hours, neo6mv2_ptr->minutes, neo6mv2_ptr->seconds, neo6mv2_ptr->centiseconds);
                           }
                           if (neo6mv2_ptr->location_valid) {
                           	 #ifndef ESP_ARDUINO_VERSION_MAJOR // ESP32 core v1.0.x  
                             json_buffer += sprintf (json_buffer, ",\"loc\":[%d,%d]", neo6mv2_ptr->latitude, neo6mv2_ptr->longitude);
                             #else
                             json_buffer += sprintf (json_buffer, ",\"loc\":[%ld,%ld]", neo6mv2_ptr->latitude, neo6mv2_ptr->longitude);
                             #endif
                           }
                           if (neo6mv2_ptr->altitude_valid) {
                           	 #ifndef ESP_ARDUINO_VERSION_MAJOR // ESP32 core v1.0.x  
                             json_buffer += sprintf (json_buffer, ",\"alt\":%d", neo6mv2_ptr->altitude);
                             #else
                             json_buffer += sprintf (json_buffer, ",\"alt\":%ld", neo6mv2_ptr->altitude);
                             #endif
                           }  
                           if (neo6mv2_ptr->location_valid and neo6mv2_ptr->altitude_valid) {
                           	 #ifndef ESP_ARDUINO_VERSION_MAJOR // ESP32 core v1.0.x  
                             json_buffer += sprintf (json_buffer, ",\"zero\":[%d,%d,%d]", neo6mv2_ptr->latitude_zero, neo6mv2_ptr->longitude_zero, neo6mv2_ptr->altitude_zero);
                             #else
                             json_buffer += sprintf (json_buffer, ",\"zero\":[%ld,%ld,%ld]", neo6mv2_ptr->latitude_zero, neo6mv2_ptr->longitude_zero, neo6mv2_ptr->altitude_zero);
                             #endif
                           }
                           if (neo6mv2_ptr->offset_valid) {
                             json_buffer += sprintf (json_buffer, ",\"xyz\":[%d,%d,%d]", neo6mv2_ptr->x, neo6mv2_ptr->y, neo6mv2_ptr->z);
                           }
                           if (neo6mv2_ptr->speed_valid) {
                           	 #ifndef ESP_ARDUINO_VERSION_MAJOR // ESP32 core v1.0.x
                             json_buffer += sprintf (json_buffer, ",\"v\":[%d,%d,%d]", neo6mv2_ptr->v_north, neo6mv2_ptr->v_east, neo6mv2_ptr->v_down);
                             #else
                             json_buffer += sprintf (json_buffer, ",\"v\":[%ld,%ld,%ld]", neo6mv2_ptr->v_north, neo6mv2_ptr->v_east, neo6mv2_ptr->v_down);
                             #endif
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
                           tm_motion_t* motion_ptr = (tm_motion_t*)ccsds_ptr;
                           sprintf (json_buffer, "{\"id\":\"%s\",\"ctr\":%u,\"millis\":%u,\"accel\":[%d,%d,%d],\"gyro\":[%d,%d,%d],\"tilt\":%d,\"g\":%d,\"a\":%d,\"rpm\":%d,\"range\":[%u,%u],\"valid\":\"%d%d\"}", 
                                    pidName[PID], motion_ptr->packet_ctr, motion_ptr->millis, 
                                    motion_ptr->accel_x, motion_ptr->accel_y, motion_ptr->accel_z, 
                                    motion_ptr->gyro_x, motion_ptr->gyro_y, motion_ptr->gyro_z, 
                                    motion_ptr->tilt, motion_ptr->g, motion_ptr->a, motion_ptr->rpm,
                                    motion_ptr->accel_range, motion_ptr->gyro_range, motion_ptr->accel_valid, motion_ptr->gyro_valid); 
                         }
                         break;
    case TM_PRESSURE:    {
                           tm_pressure_t* bmp280_ptr = (tm_pressure_t*)ccsds_ptr;
                           #ifndef ESP_ARDUINO_VERSION_MAJOR // ESP32 core v1.0.x
                           sprintf (json_buffer, "{\"id\":\"%s\",\"ctr\":%u,\"millis\":%u,\"p\":%u,\"p0\":%u,\"T\":%d,\"h\":%d,\"v_v\":%d,\"valid\":%u}", 
                                    pidName[PID], bmp280_ptr->packet_ctr, bmp280_ptr->millis, 
                                    bmp280_ptr->pressure, bmp280_ptr->zero_level_pressure, bmp280_ptr->temperature, bmp280_ptr->height, bmp280_ptr->velocity_v, bmp280_ptr->height_valid);
                           #else
                           sprintf (json_buffer, "{\"id\":\"%s\",\"ctr\":%u,\"millis\":%u,\"p\":%lu,\"p0\":%lu,\"T\":%d,\"h\":%d,\"v_v\":%d,\"valid\":%u}", 
                                    pidName[PID], bmp280_ptr->packet_ctr, bmp280_ptr->millis, 
                                    bmp280_ptr->pressure, bmp280_ptr->zero_level_pressure, bmp280_ptr->temperature, bmp280_ptr->height, bmp280_ptr->velocity_v, bmp280_ptr->height_valid);
                           #endif
                         }
                         break;
    case TM_RADIO:       {
                           tm_radio_t* radio_ptr = (tm_radio_t*)ccsds_ptr;
                           sprintf (json_buffer, "{\"id\":\"%s\",\"ctr\":%u,\"millis\":%u,\"data\":\"%s\"}", 
                                    pidName[PID], radio_ptr->packet_ctr, radio_ptr->millis, 
                                    get_hex_str((char*)&radio, sizeof(tm_radio_t)).c_str());
                         }
                         break;
    case TIMER_ESP32:    {
                           timer_esp32_t* timer_esp32_ptr = (timer_esp32_t*)ccsds_ptr;
                           sprintf (json_buffer, "{\"id\":\"%s\",\"ctr\":%u,\"millis\":%u,\"idle\":%u,\"instr\":[%u,%u,%u,%u,%u],\"fun\":[%u,%u,%u,%u,%u],\"pub\":[%u,%u,%u,%u]}", 
                                    pidName[PID], timer_esp32_ptr->packet_ctr, timer_esp32_ptr->millis,  
                                    timer_esp32_ptr->idle_duration,
                                    timer_esp32_ptr->radio_duration, timer_esp32_ptr->pressure_duration, timer_esp32_ptr->motion_duration, timer_esp32_ptr->gps_duration, timer_esp32_ptr->esp32cam_duration,
                                    timer_esp32_ptr->serial_duration, timer_esp32_ptr->ota_duration, timer_esp32_ptr->ftp_duration, timer_esp32_ptr->wifi_duration, timer_esp32_ptr->tc_duration,
                                    timer_esp32_ptr->publish_fs_duration, timer_esp32_ptr->publish_serial_duration, timer_esp32_ptr->publish_yamcs_duration, timer_esp32_ptr->publish_udp_duration);
                         }
                         break;
    case TIMER_ESP32CAM: { // TODO: fine-tune packet
                           timer_esp32cam_t* timer_esp32cam_ptr = (timer_esp32cam_t*)ccsds_ptr;
                           sprintf (json_buffer, "{\"id\":\"%s\",\"ctr\":%u,\"millis\":%u,\"idle\":%u,\"cam\":%u,\"fun\":[%u,%u,%u,%u,%u],\"pub\":[%u,%u,%u,%u,%u]}", 
                                    pidName[PID], timer_esp32cam_ptr->packet_ctr, timer_esp32cam_ptr->millis, 
                                    timer_esp32cam_ptr->idle_duration,
                                    timer_esp32cam_ptr->camera_duration,
                                    timer_esp32cam_ptr->serial_duration, timer_esp32cam_ptr->tc_duration, timer_esp32cam_ptr->sd_duration, timer_esp32cam_ptr->ftp_duration, timer_esp32cam_ptr->wifi_duration, 
                                    timer_esp32cam_ptr->publish_sd_duration, timer_esp32cam_ptr->publish_fs_duration, timer_esp32cam_ptr->publish_serial_duration, timer_esp32cam_ptr->publish_yamcs_duration, timer_esp32cam_ptr->publish_udp_duration);
                         }
                         break;
    case TC_ESP32:       { 
                           tc_esp32_t* tc_esp32_ptr = (tc_esp32_t*)ccsds_ptr;
                           *json_buffer++ = '{';
                           json_buffer += sprintf (json_buffer, "\"id\":\"%s\",\"millis\":%lu,\"cmd\":\"%s\"", pidName[PID], millis(), tcName[tc_esp32_ptr->cmd_id-42]);
                           if (tc_esp32_ptr->cmd_id-42 == TC_REBOOT or tc_esp32_ptr->cmd_id-42 == TC_SET_OPSMODE) {
                             json_buffer += sprintf (json_buffer, ",\"int_val\":\"%u\"", (uint8_t)tc_esp32_ptr->parameter[0]);
                           }
                           if (tc_esp32_ptr->cmd_id-42 == TC_LOAD_CONFIG or tc_esp32_ptr->cmd_id-42 == TC_LOAD_ROUTING) {
                             json_buffer += sprintf (json_buffer, ",\"str_val\":\"%s\"", tc_esp32_ptr->parameter);
                           }
                           if (tc_esp32_ptr->cmd_id-42 == TC_SET_PARAMETER) {
                             json_buffer += sprintf (json_buffer, ",\"param\":\"%s\",\"value\":\"%s\"", tc_esp32_ptr->parameter, (char*)(&tc_esp32_ptr->parameter+strlen(tc_esp32_ptr->parameter)+1)); // TODO: value
                           }
                           *json_buffer++ = '}';
                           *json_buffer++ = 0;
                         }
                         break;
    case TC_ESP32CAM:    { 
                           tc_esp32cam_t* tc_esp32cam_ptr = (tc_esp32cam_t*)ccsds_ptr;
                           *json_buffer++ = '{';
                           json_buffer += sprintf (json_buffer, "\"id\":\"%s\",\"millis\":%lu,\"cmd\":\"%s\"", pidName[PID], millis(), tcName[tc_esp32cam_ptr->cmd_id-42]);
                           if (tc_esp32cam_ptr->cmd_id-42 == TC_REBOOT or tc_esp32cam_ptr->cmd_id-42 == TC_SET_OPSMODE) {
                             json_buffer += sprintf (json_buffer, ",\"int_val\":\"%u\"", (uint8_t)tc_esp32cam_ptr->parameter[0]);
                           }
                           if (tc_esp32cam_ptr->cmd_id-42 == TC_LOAD_CONFIG or tc_esp32cam_ptr->cmd_id-42 == TC_LOAD_ROUTING) {
                             json_buffer += sprintf (json_buffer, ",\"str_val\":\"%s\"", tc_esp32cam_ptr->parameter);
                           }
                           if (tc_esp32cam_ptr->cmd_id-42 == TC_SET_PARAMETER) {
                             json_buffer += sprintf (json_buffer, ",\"param\":\"%s\",\"value\":\"%s\"", tc_esp32cam_ptr->parameter, tc_esp32cam_ptr->parameter); // TODO: value
                           }
                           *json_buffer++ = '}';
                           *json_buffer++ = 0;
                         } 
                         break;   
  } 
}

bool parse_json (const char* json_string) {
//  static Document<BUFFER_MAX_SIZE> obj;
  static JsonDocument obj;
  deserializeJson(obj, (const char*)json_string);
  uint16_t PID = id_of (obj["id"], sizeof(pidName[0]), (char*)pidName, sizeof(pidName));
  switch (PID) {
    #ifdef PLATFORM_ESP32
    case STS_ESP32CAM:  // {\"id\":\"%s\",\"ctr\":%u,\"millis\":%u,\"type\":\"%s\",\"ss\":\"%s\",\"msg\":\"%s\"}
                        sts_esp32cam.packet_ctr = obj["ctr"];
                        sts_esp32cam.millis = obj["millis"],
                        sts_esp32cam.type = id_of ((const char*)obj["type"], sizeof(eventName[0]), (char*)&eventName, sizeof(eventName));
                        sts_esp32cam.subsystem = id_of ((const char*)obj["ss"], sizeof(subsystemName[0]), (char*)&subsystemName, sizeof(subsystemName));
                        strcpy (sts_esp32cam.message, obj["msg"]);
                        set_ccsds_payload_len ((ccsds_t*)&sts_esp32cam, strlen (sts_esp32cam.message) + 7);
                        publish_packet ((ccsds_t*)&sts_esp32cam);
                        break;                        
    case TM_ESP32CAM:   // {\"ctr\":%u,\"mode\":\"%s\",\"err\":%u,\"warn\":%u,\"tc\":{\"exec\":%u,\"fail\":%u},\"mem\":[%u,%u,%u],\"buf\":[%u,%u],\"fs\":[%u,%u],\"ntp\":%u,\"rate\":[%u,%u,%u,%u,%u,%u,%u,%u,%u],\"ena\":\"%d%d%d%d%d%d%d%d%d%d%d\",\"act\":\"%d%d%d%d\",\"conn\":{\"up\":\"%d%d\",\"warn\":\"%d%d\",\"err\":\"%d%d%d%d\"}}
                        // TODO: ota_enabled missing
                        esp32cam.packet_ctr = obj["ctr"];
                        esp32cam.millis = obj["millis"],
                        esp32cam.opsmode = obj["mode"];
                        esp32cam.error_ctr = obj["err"];
                        esp32cam.warning_ctr = obj["warn"];
                        esp32cam.tc_exec_ctr = obj["tc"]["exec"];
                        esp32cam.tc_fail_ctr = obj["tc"]["fail"];
                        esp32cam.mem_free = obj["mem"][0];
                        esp32cam.fs_free = obj["mem"][1];
                        esp32cam.sd_free = obj["mem"][2];
                        esp32cam.yamcs_buffer = obj["buf"][0];
                        esp32cam.serial_out_buffer = obj["buf"][1];
                        esp32cam.ftp_fs = obj["fs"][0];
                        esp32cam.buffer_fs = obj["fs"][1];
                        esp32cam.time_set = (obj["ntp"] == '1')?1:0;       
                        esp32cam.camera_rate = obj["rate"][0];
                        esp32cam.udp_rate = obj["rate"][1];
                        esp32cam.yamcs_rate = obj["rate"][2];
                        esp32cam.serial_in_rate = obj["rate"][3];
                        esp32cam.serial_out_rate = obj["rate"][4];
                        esp32cam.fs_rate = obj["rate"][5];
                        esp32cam.sd_json_rate = obj["rate"][6];
                        esp32cam.sd_ccsds_rate = obj["rate"][7];
                        esp32cam.sd_image_rate = obj["rate"][8];
                        esp32cam.camera_enabled = (obj["ena"][0] == '1')?1:0;  
                        esp32cam.wifi_enabled = (obj["ena"][1] == '1')?1:0;     
                        esp32cam.wifi_udp_enabled = (obj["ena"][2] == '1')?1:0;      
                        esp32cam.wifi_yamcs_enabled = (obj["ena"][3] == '1')?1:0;    
                        esp32cam.wifi_image_enabled = (obj["ena"][4] == '1')?1:0;    
                        esp32cam.fs_enabled = (obj["ena"][5] == '1')?1:0; 
                        esp32cam.sd_enabled = (obj["ena"][6] == '1')?1:0; 
                        esp32cam.ftp_enabled = (obj["ena"][7] == '1')?1:0; 
                        esp32cam.sd_json_enabled = (obj["ena"][8] == '1')?1:0;  
                        esp32cam.sd_ccsds_enabled = (obj["ena"][9] == '1')?1:0; 
                        esp32cam.sd_image_enabled = (obj["ena"][10] == '1')?1:0; 
                        esp32cam.camera_active = (obj["act"][0] == '1')?1:0; 
                        esp32cam.fs_active = (obj["act"][1] == '1')?1:0;           
                        esp32cam.sd_active = (obj["act"][2] == '1')?1:0;           
                        esp32cam.ftp_active = (obj["act"][3] == '1')?1:0;   
                        esp32cam.serial_connected = (obj["conn"]["up"][0] == '1')?1:0;
                        esp32cam.wifi_connected = (obj["conn"]["up"][1] == '1')?1:0; 
                        esp32cam.warn_serial_connloss = (obj["conn"]["warn"][0] == '1')?1:0; 
                        esp32cam.warn_wifi_connloss = (obj["conn"]["warn"][1] == '1')?1:0; 
                        esp32cam.err_serial_dataloss = (obj["conn"]["err"][0] == '1')?1:0; 
                        esp32cam.err_yamcs_dataloss = (obj["conn"]["err"][1] == '1')?1:0; 
                        esp32cam.err_fs_dataloss = (obj["conn"]["err"][2] == '1')?1:0; 
                        esp32cam.err_sd_dataloss = (obj["conn"]["err"][3] == '1')?1:0; 
                        publish_packet ((ccsds_t*)&esp32cam);
                        break;                                                                                                                          
    case TM_CAMERA:     // {\"ctr\":%u,\"mode\":\"%s\",\"res\":\"%s\",\"auto_res\":%d,\"file\":\"%s\",\"size\":%u,\"ms\":{\"exp\":%u,\"sd\":%u,\"wifi\":%u}}
                        ov2640.packet_ctr = obj["ctr"];
                        ov2640.millis = obj["millis"],
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
    case TIMER_ESP32CAM: // TODO: fine-tune packet
                    // {\"ctr\":%u,\"idle\":%u,\"cam\":%u,\"fun\":[%u,%u,%u,%u,%u],\"pub\":[%u,%u,%u,%u,%u]}
                        timer_esp32cam.packet_ctr = obj["ctr"];
                        timer_esp32cam.millis = obj["millis"],
                        timer_esp32cam.camera_duration = obj["cam"];
                        timer_esp32cam.serial_duration = obj["fun"][0];
                        timer_esp32cam.tc_duration = obj["fun"][1];
                        timer_esp32cam.sd_duration = obj["fun"][2];
                        timer_esp32cam.ftp_duration = obj["fun"][3];
                        timer_esp32cam.wifi_duration = obj["fun"][4];
                        timer_esp32cam.idle_duration = obj["idle"];
                        timer_esp32cam.publish_sd_duration = obj["pub"][0];
                        timer_esp32cam.publish_fs_duration = obj["pub"][1];
                        timer_esp32cam.publish_serial_duration = obj["pub"][2];
                        timer_esp32cam.publish_yamcs_duration = obj["pub"][3];
                        timer_esp32cam.publish_udp_duration = obj["pub"][4];
                        publish_packet ((ccsds_t*)&timer_esp32cam);
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
                        else if (!strcmp(obj["cmd"], "freeze_opsmode")) {
                          // {"cmd":"freeze_opsmode", "frozen":0|1}
                          cmd_freeze_opsmode (obj["frozen"]);
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
                                                 tc_esp32cam.parameter[0] = (char) atoi (obj["subsystem"]);
                                                 tc_esp32cam.parameter[1] = 0;
                                                 set_ccsds_payload_len ((ccsds_t*)&tc_esp32cam, 7);
                                                 break;
                          case TC_LOAD_CONFIG:   // {"cmd":"load_config","filename":"xxxxxx.cfg"}
                                                 strcpy (tc_esp32cam.parameter, obj["filename"]);
                                                 set_ccsds_payload_len ((ccsds_t*)&tc_esp32cam, strlen (tc_esp32cam.parameter) + 7);
                                                 break;
                          case TC_LOAD_ROUTING:  // {"cmd":"load_routing","filename":"xxxxxx.cfg"}
                                                 strcpy (tc_esp32cam.parameter, obj["filename"]);
                                                 set_ccsds_payload_len ((ccsds_t*)&tc_esp32cam, strlen (tc_esp32cam.parameter) + 7);
                                                 break;
                          case TC_SET_PARAMETER: // {"cmd":"set_parameter","parameter":"xxxxxx","value":"xxxxxx"}
                                                 strcpy ((char*)&tc_esp32cam.parameter, obj["parameter"]);
                                                 tc_esp32cam.parameter[strlen (obj["parameter"])] = 0;
                                                 strcpy ((char*)(&tc_esp32cam.parameter + strlen (obj["parameter"]) + 1), obj["parameter"]);
                                                 set_ccsds_payload_len ((ccsds_t*)&tc_esp32cam, strlen (obj["parameter"]) + strlen (obj["value"]) + 8);
                                                 break;
                          case TC_FREEZE_OPSMODE:// {"cmd":"freeze_opsmode","frozen":0|1}
                                             tc_esp32cam.parameter[0] = (char) atoi (obj["frozen"]);
                                             tc_esp32cam.parameter[1] = 0;
                                                 set_ccsds_payload_len ((ccsds_t*)&tc_esp32cam, 7);
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
                        sts_esp32.millis = obj["millis"],
                        sts_esp32.type = id_of ((const char*)obj["type"], sizeof(eventName[0]), (char*)&eventName, sizeof(eventName));
                        sts_esp32.subsystem = id_of ((const char*)obj["ss"], sizeof(subsystemName[0]), (char*)&subsystemName, sizeof(subsystemName));
                        strcpy (sts_esp32.message, obj["msg"]);
                        set_ccsds_payload_len ((ccsds_t*)&sts_esp32, strlen (sts_esp32.message) + 7);
                        publish_packet ((ccsds_t*)&sts_esp32);
                        break;      
    case TM_ESP32:      // {\"ctr\":%u,\"mode\":\"%s\",\"state\":\"%s\",\"err\":%u,\"warn\":%u,\"tc\":{\"exec\":%u,\"fail\":%u},\"mem\":[%u,%u],\"buf\":[%u,%u],\"fs\":[%u,%u],\"ntp\":%u,\"inst_rate\":[%u,%u,%u,%u,%u],\"comm_rate\":[%u,%u,%u,%u,%u],\"sep\":%d,\"ena\":\"%d%d%d%d%d%d%d%d%d%d%d\",\"act\":\"%d%d%d%d%d%d%d%d\",\"conn\":{\"up\":\"%d%d\",\"warn\":\"%d%d\",\"err\":\"%d%d%d\"}}
                        esp32.packet_ctr = obj["ctr"];
                        esp32.millis = obj["millis"],
                        esp32.opsmode = id_of(obj["mode"], sizeof(modeName[0]), (char*)&modeName, sizeof(modeName));
                        esp32.state = id_of(obj["sts"], sizeof(stateName[0]), (char*)&stateName, sizeof(stateName));
                        esp32.error_ctr = obj["err"];
                        esp32.warning_ctr = obj["warn"];
                        esp32.tc_exec_ctr = obj["tc"]["exec"];
                        esp32.tc_fail_ctr = obj["tc"]["fail"];
                        esp32.mem_free = obj["mem"][0];
                        esp32.fs_free = obj["mem"][1];
                        esp32.yamcs_buffer = obj["buf"][0];
                        esp32.serial_out_buffer = obj["buf"][1];
                        esp32.ftp_fs = obj["fs"][0];
                        esp32.buffer_fs = obj["fs"][1];
                        esp32cam.time_set = (obj["ntp"] == '1')?1:0;  
                        esp32.radio_rate = obj["inst_rate"][0];
                        esp32.pressure_rate = obj["inst_rate"][1];
                        esp32.motion_rate = obj["inst_rate"][2];
                        esp32.gps_rate = obj["inst_rate"][3];
                        esp32.camera_rate = obj["inst_rate"][4];
                        esp32.udp_rate = obj["comm_rate"][0];
                        esp32.yamcs_rate = obj["comm_rate"][1];
                        esp32.serial_in_rate = obj["comm_rate"][2];
                        esp32.serial_out_rate = obj["comm_rate"][3];
                        esp32.fs_rate = obj["comm_rate"][4];
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
                        esp32.ftp_enabled = (obj["ena"][9] == '1')?1:0; 
                        esp32.time_set = (obj["ena"][10] == '1')?1:0;  
                        esp32.radio_active = (obj["act"][0] == '1')?1:0;     
                        esp32.pressure_active = (obj["act"][1] == '1')?1:0;     
                        esp32.motion_active = (obj["act"][2] == '1')?1:0;     
                        esp32.gps_active = (obj["act"][3] == '1')?1:0;     
                        esp32.camera_active = (obj["act"][4] == '1')?1:0; 
                        esp32.fs_active = (obj["act"][5] == '1')?1:0;           
                        esp32.ftp_active = (obj["act"][7] == '1')?1:0;           
                        esp32.ota_enabled = (obj["act"][8] == '1')?1:0;  
                        esp32.serial_connected = (obj["conn"]["up"][0] == '1')?1:0;
                        esp32.wifi_connected = (obj["conn"]["up"][1] == '1')?1:0; 
                        esp32.warn_serial_connloss = (obj["conn"]["warn"][0] == '1')?1:0; 
                        esp32.warn_wifi_connloss = (obj["conn"]["warn"][1] == '1')?1:0; 
                        esp32.err_serial_dataloss = (obj["conn"]["err"][0] == '1')?1:0; 
                        esp32.err_yamcs_dataloss = (obj["conn"]["err"][1] == '1')?1:0; 
                        esp32.err_fs_dataloss = (obj["conn"]["err"][2] == '1')?1:0; 
                        publish_packet ((ccsds_t*)&esp32);
                        break;
    case TM_GPS:        // {\"ctr\":%u,\"sts\":\"%s\",\"sats\":%d,\"time\":\"%02d:%02d:%02d\",\"loc\":[%d,%d,%d],\"zero\":[%d,%d,%d],\"xyz\":[%d,%d,%d],\"v\":[%d,%d,%d],\"dop\":[%d,%d,%d],\"valid\":\"%d%d%d%d%d%d\"}
                        neo6mv2.packet_ctr = obj["ctr"];
                        neo6mv2.millis = obj["millis"],
                        neo6mv2.status = id_of(obj["sts"], sizeof(gpsStatusName[0]), (char*)&gpsStatusName, sizeof(gpsStatusName));
                        neo6mv2.satellites = obj["sats"];
                        if (obj.containsKey("time")) {
                          neo6mv2.time_valid = true;
                          //neo6mv2.hours = obj["time"].substring(0,1); // TODO: assess why ArduinoJson throws error
                          //neo6mv2.minutes = obj["time"].substring(3,4);
                          //neo6mv2.seconds = obj["time"].substring(6,7);
                          //neo6mv2.centiseconds = obj["time"].substring(9,11);
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
                        motion.packet_ctr = obj["ctr"];
                        motion.millis = obj["millis"],
                        motion.accel_x = obj["accel"][0];
                        motion.accel_y = obj["accel"][1];
                        motion.accel_z = obj["accel"][2];
                        motion.gyro_x = obj["gyro"][0];
                        motion.gyro_y = obj["gyro"][1];
                        motion.gyro_z = obj["gyro"][2];
                        motion.tilt = obj["tilt"];
                        motion.g = obj["g"];
                        motion.a = obj["a"];
                        motion.rpm = obj["rpm"];
                        motion.accel_range = obj["range"][0];
                        motion.gyro_range = obj["range"][1];
                        motion.accel_valid = obj["valid"][0];
                        motion.gyro_valid = obj["valid"][1];
                        publish_packet ((ccsds_t*)&motion);
                        break;
    case TM_PRESSURE:   // {\"ctr\":%u,\"p\":%.2f,\"p0\":%.2f,\"T\":%.2f,\"h\":%.2f,\"v_v\":%.2f}
                        bmp280.packet_ctr = obj["ctr"];
                        bmp280.millis = obj["millis"],
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
                        radio.millis = obj["millis"],
                        // hex_to_bin ((byte*)&radio, obj["data"]); // TODO: find out with ArduinoJson throuws error
                        publish_packet ((ccsds_t*)&radio);
                        break;
    case TIMER_ESP32:   // {\"ctr\":%u,\"idle\":%u,\"instr\":[%u,%u,%u,%u,%u],\"fun\":[%u,%u,%u,%u,%u],\"pub\":[%u,%u,%u,%u]}
                        timer_esp32.packet_ctr = obj["ctr"];
                        timer_esp32.millis = obj["millis"],
                        timer_esp32.radio_duration = obj["instr"][0];
                        timer_esp32.pressure_duration = obj["instr"][1];
                        timer_esp32.motion_duration = obj["instr"][2];
                        timer_esp32.gps_duration = obj["instr"][3];
                        timer_esp32.esp32cam_duration = obj["instr"][4];
                        timer_esp32.serial_duration = obj["fun"][0];
                        timer_esp32.ota_duration = obj["fun"][1];
                        timer_esp32.ftp_duration = obj["fun"][2];
                        timer_esp32.wifi_duration = obj["fun"][3];
                        timer_esp32.tc_duration = obj["fun"][4];
                        timer_esp32.idle_duration = obj["idle"];
                        timer_esp32.publish_fs_duration = obj["pub"][0];
                        timer_esp32.publish_serial_duration = obj["pub"][1];
                        timer_esp32.publish_yamcs_duration = obj["pub"][2];
                        timer_esp32.publish_udp_duration = obj["pub"][3];
                        publish_packet ((ccsds_t*)&timer_esp32);
                        break;    
    case TC_ESP32:      // forward command
                        tc_esp32.cmd_id = id_of (obj["cmd"], sizeof(tcName[0]), (char*)tcName, sizeof(tcName));
                        switch (tc_esp32.cmd_id) {
                          case TC_REBOOT:        // {"cmd":"reboot","subsystem":0|1|2}
                                                 tc_esp32.parameter[0] = (unsigned char)obj["subsystem"];
                                                 tc_esp32.parameter[1] = 0;
                                                 set_ccsds_payload_len ((ccsds_t*)&tc_esp32, 7);
                                                 break;
                          case TC_SET_OPSMODE:   // {"cmd":"set_opsmode","opsmode":"checkout|ready|static"}
                                                 tc_esp32.parameter[0] = (unsigned char)id_of (obj["opsmode"], sizeof(modeName[0]), (char*)modeName, sizeof(modeName));
                                                 tc_esp32.parameter[1] = 0;
                                                 set_ccsds_payload_len ((ccsds_t*)&tc_esp32, 7);
                                                 break;
                          case TC_LOAD_CONFIG:   // {"cmd":"load_config","filename":"xxxxxx.cfg"}
                                                 strcpy (tc_esp32.parameter, obj["filename"]);
                                                 set_ccsds_payload_len ((ccsds_t*)&tc_esp32, strlen (tc_esp32.parameter) + 7);
                                                 break;
                          case TC_LOAD_ROUTING:  // {"cmd":"load_routing","filename":"xxxxxx.cfg"}
                                                 strcpy (tc_esp32.parameter, obj["filename"]);
                                                 set_ccsds_payload_len ((ccsds_t*)&tc_esp32, strlen (tc_esp32.parameter) + 7);
                                                 break;
                          case TC_SET_PARAMETER: // {"cmd":"set_parameter","parameter":"xxxxxx","value":"xxxxxx"}
                                                 strcpy ((char*)&tc_esp32.parameter, obj["parameter"]);
                                                 tc_esp32.parameter[strlen (obj["parameter"])] = 0;
                                                 strcpy ((char*)(&tc_esp32.parameter + strlen (obj["parameter"]) + 1), obj["parameter"]);
                                                 set_ccsds_payload_len ((ccsds_t*)&tc_esp32, strlen (obj["parameter"]) + strlen (obj["value"]) + 8);
                                                 break;
                          case TC_FREEZE_OPSMODE:// {"cmd":"freeze_opsmode","frozen"0:1}
                                             tc_esp32.parameter[0] = (unsigned char)obj["frozen"];
                                                 tc_esp32.parameter[1] = 0;
                                                 set_ccsds_payload_len ((ccsds_t*)&tc_esp32, 7);
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
                        else if (!strcmp(obj["cmd"], "freeze_opsmode")) {
                          // {"cmd":"freeze_opsmode", "frozen":0|1}
                          cmd_freeze_opsmode (obj["frozen"]);
                        }
                        else {
                          publish_event (STS_THIS, SS_THIS, EVENT_ERROR, "JSON command to ESP32CAM not understood");
                        }
                        break;
    #endif
    default:            publish_event (STS_THIS, SS_THIS, EVENT_WARNING, "Ignored JSON packet"); 
                        return false;
                        break;
  }
  return true;
}


// SERIAL FUNCTIONALITY

bool serial_setup () {
  Serial.begin (SERIAL_BAUD);
  //Serial.setRxBufferSize(128);
  #ifdef SERIAL_TCTM
  Serial.setDebugOutput (false);
  //serialTransfer.begin(Serial);
  #endif
  return true;
}

void serial_keepalive () {
  #ifdef SERIAL_TCTM
  if (millis() - var_timer.last_serial_out_millis > KEEPALIVE_INTERVAL) {
    if (tm_this->serial_connected) {
      if (serialTransfer.available()) { 
        serialTransfer.sendDatum("O", 1);
      }
      //publish_udp_text("DEBUG: Sent over serial: [O]");
    }
    else {
      if (serialTransfer.available()) {     
        serialTransfer.sendDatum("o", 1);
      }
      //publish_udp_text("DEBUG: Sent over serial: [o]");
    }
    var_timer.last_serial_out_millis = millis();
  }
  if (tm_this->serial_connected and millis()-var_timer.last_serial_in_millis > 2*KEEPALIVE_INTERVAL) {
    tm_this->serial_connected = false;
    tm_this->warn_serial_connloss = true;
  }
  #endif
}

bool serial_check () {
  #ifdef SERIAL_TCTM
  static uint8_t serial_len;
  if (serialTransfer.available()) {
    serial_len = serialTransfer.rxObj(serial_in_buffer);
    serial_in_buffer[serial_len] = '\0';
    tm_this->serial_in_rate++;
    tm_this->serial_connected = true;
    var_timer.last_serial_in_millis = millis();
    publish_udp_text(buffer);
    return true;
  }
  else {
    sprintf (buffer, "DEBUG: Received over serial: [%s]\0", serial_in_buffer); 
  #endif
  return false;
}

void serial_parse () {
  if (serial_in_buffer[0] == '{') {
    // JSON formatted message
    //publish_udp_text("DEBUG: Parsing JSON message");
    //publish_udp_text(serial_in_buffer);
    parse_json ((char*)&serial_in_buffer);
  }
  else if (serial_in_buffer[0] == 0x00 or serial_in_buffer[0] == 0x01) {
    // CCSDS formatted message
    //publish_udp_text("DEBUG: Parsing CCSDS message");
    parse_ccsds ((ccsds_t*)&serial_in_buffer);
  }
  else {
    // generic ASCII string: likely debug information
    //publish_udp_text("DEBUG: Message identified as ASCII");
  }
}

// COMMAND FUNCTIONALITY

bool cmd_reboot (uint8_t subsystem) {
  switch (subsystem) {
    case 0: // this
            sprintf (buffer, "Rebooting %s subsystem", subsystemName[SS_THIS]);
            publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
            sync_file_ccsds ();
            sync_file_json ();
            delay (1000);
            ESP.restart();
            break;
    case 1: // other
            sprintf (buffer, "Sending reboot command to %s subsystem", subsystemName[SS_OTHER]);
            publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
            tc_other->cmd_id = TC_REBOOT;
            tc_other->parameter[0] = 0;
            set_ccsds_payload_len ((ccsds_t*)tc_other, 7); 
            publish_packet ((ccsds_t*)tc_other);
            break;
    case 2: // both
            tc_other->cmd_id = TC_REBOOT;
            tc_other->parameter[0] = 0;
            set_ccsds_payload_len ((ccsds_t*)tc_other, 7); 
            publish_packet ((ccsds_t*)tc_other);
            sprintf (buffer, "Sending reboot command to %s and rebooting %s subsystem", subsystemName[SS_OTHER], subsystemName[SS_THIS]);
            publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
            sync_file_ccsds ();
            sync_file_json ();
            delay (1000);
            ESP.restart();
  }     
  return true;
}

bool cmd_set_opsmode (uint8_t opsmode) {
  if (opsmode == MODE_CHECKOUT or opsmode == MODE_NOMINAL or opsmode == MODE_MAINTENANCE) {
    sprintf (buffer, "Setting opsmode to '%s'", modeName[opsmode]);
    publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
    tm_this->opsmode = opsmode;
  }
  else {
    sprintf (buffer, "Opsmode '%s' not known or not allowed", modeName[opsmode]);
    publish_event (STS_THIS, SS_THIS, EVENT_CMD_FAIL, buffer);
  }
  return true;
}

bool cmd_set_parameter (const char* parameter, const char* value) {
  if (tm_this->opsmode != MODE_NOMINAL) {
    if (set_parameter (parameter, value)) {
      publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
      return true;
    }
    else {
      publish_event (STS_THIS, SS_THIS, EVENT_CMD_FAIL, "Command needs valid parameter/value");
      return false; 
    }
  }
  else {
    publish_event (STS_THIS, SS_THIS, EVENT_CMD_FAIL, "Command not allowed in NOMINAL mode");
    return false;   
  }
}

bool cmd_load_config (const char* filename) {
  if (tm_this->opsmode != MODE_NOMINAL) {
    file_load_config (config_this->ftp_fs, filename); // TODO: needs to become config_fs
    return true;
  }
  else {
    publish_event (STS_THIS, SS_THIS, EVENT_CMD_FAIL, "Command not allowed in NOMINAL mode");
    return false;   
  }  
}

bool cmd_load_routing (const char* filename) {
  if (esp32.opsmode == MODE_CHECKOUT) {
    file_load_routing (config_this->ftp_fs, filename);
    return true;
  }
  else {
    publish_event (STS_THIS, SS_THIS, EVENT_CMD_FAIL, "Command only allowed in CHECKOUT mode");
    return false;   
  }   
}

bool cmd_toggle_routing (uint16_t PID, const char interface) {
  // TODO: TBW + integrate
  return false;
}

bool cmd_freeze_opsmode (bool frozen) {
  File file_lock;
  switch (tm_this->buffer_fs) {
  case FS_LITTLEFS: if (frozen) {
                      #ifndef ESP_ARDUINO_VERSION_MAJOR // ESP32 core v1.0.x  
                      file_lock = LITTLEFS.open(lock_filename, "w");
                      #else
                      file_lock = LittleFS.open(lock_filename, "w");
                      #endif
                      file_lock.write ((char)tm_this->opsmode);
                      file_lock.close ();
                      sprintf (buffer, "Frozen opsmode to '%s'", modeName[tm_this->opsmode]);
                      publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
                    }
                    else {
                      #ifndef ESP_ARDUINO_VERSION_MAJOR // ESP32 core v1.0.x  
                      LITTLEFS.remove(lock_filename);
                      #else
                      LittleFS.remove(lock_filename);
                      #endif
                      publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, "Removed opsmode freeze");
                    }
                    break;
  #ifdef PLATFORM_ESP32CAM
  case FS_SD_MMC:  if (frozen) {
                     file_lock = SD_MMC.open(lock_filename, "a+");
                     file_lock.write ((char)tm_this->opsmode);
                     file_lock.close ();
                     sprintf (buffer, "Frozen opsmode to '%s'", modeName[tm_this->opsmode]);
                     publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
                   }
                   else {
                     SD_MMC.remove(lock_filename);
                     publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, "Removed opsmode freeze");
                   }
                   break;
  #endif
  }
  return true;
}

bool cmd_replay_start (char* filename, bool realtime) {
    // TODO: implement
    return false;
}

bool cmd_replay_stop () {
    // TODO: implement
    return false;
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

int8_t sign (int16_t x) {
    return (x > 0) - (x < 0);
}

#endif