/*
 * Fli3d - Library (file system, wifi, TM/TC, comms functionality)
 * version: 2020-10-17
 */

#include "fli3d.h"
#include "secrets.h"
#ifdef PLATFORM_ESP32CAM
#include "FS.h"
#include "SD_MMC.h"
#include "SPI.h"
#endif

WiFiUDP wifiUDP;
WiFiClient wifiTCP;
FtpServer ftpSrv;
NTPClient timeClient(wifiUDP, config_network.ntp_server, 0);

#ifdef PLATFORM_ESP32
RH_ASK radio_tx (RADIO_BAUD, DUMMY_PIN1, RF433_TX_PIN, DUMMY_PIN2);
char radio_buffer[2*(sizeof(tm_radio_t)+2)];
#endif

char    buffer[JSON_MAX_SIZE];
char    serial_buffer[JSON_MAX_SIZE];
char    path_buffer[32];

#ifdef PLATFORM_ESP32
extern void ota_setup ();
#endif 

ccsds_hdr_t        ccsds_hdr;
ccsds_t            ccsds_buffer;

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
tc_esp32_t         *tc_this = &tc_esp32;
tc_esp32cam_t      *tc_other = &tc_esp32cam;
sts_esp32_t        *sts_this = &sts_esp32;
config_esp32_t     *config_this = &config_esp32;
var_esp32_t        *var_this = &var_esp32;
#endif
#ifdef PLATFORM_ESP32CAM
tm_esp32cam_t      *tm_this = &esp32cam;
tc_esp32cam_t      *tc_this = &tc_esp32cam;
tc_esp32_t         *tc_other = &tc_esp32;
sts_esp32cam_t     *sts_this = &sts_esp32cam;
config_esp32cam_t  *config_this = &config_esp32cam;
var_esp32cam_t     *var_this = &var_esp32cam;
#endif

const char pidName[NUMBER_OF_PID][13] =   { "sts_esp32", "sts_esp32cam", "tm_esp32", "tm_esp32cam", "tm_camera", "tm_gps", "tm_motion", "tm_pressure", "tm_radio", "tm_timer", "tc_esp32", "tc_esp32cam" };
const char eventName[7][9] =              { "init", "info", "warning", "error", "cmd", "cmd_resp", "cmd_fail" };
const char subsystemName[13][14] =        { "esp32", "esp32cam", "ov2640", "neo6mv2", "mpu6050", "bmp280", "radio", "sd", "separation", "timer", "fli3d", "ground", "any" };
const char modeName[7][10] =              { "init", "checkout", "ready", "thrust", "freefall", "parachute", "static" };
const char cameraModeName[4][7] =         { "init", "idle", "single", "stream" };
const char cameraResolutionName[11][10] = { "160x120", "invalid1", "invalid2", "240x176", "320x240", "400x300", "640x480", "800x600", "1024x768", "1280x1024", "1600x1200" };
const char dataEncodingName[3][8] =       { "CCSDS", "JSON", "ASCII" };
const char commLineName[9][13] =          { "serial", "wifi_udp", "wifi_yamcs", "wifi_cam", "sd_ccsds", "sd_json", "sd_cam", "fs", "radio" };
const char tcName[9][20] =                { "reboot", "reboot_fli3d", "config_reset", "config_load", "config_save", "get_packet", "set_opsmode", "set_parameter", "set_routing" };
const char gpsStatusName[5][10] =         { "none", "est", "time_only", "std", "dgps" }; 
const char ipProtocolName[2][4] =         { "UDP", "TCP" }; 

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
      // FS is full, delete old data (TODO: ensure there is a way to inhibit this for data recovery after flight, e.g. using separation status for ESP32 and TBD for ESP32CAM)
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
    sprintf (buffer, "Initialized FS (size: %d kB; free: %d kB)", LITTLEFS.totalBytes()/2024, fs_free());
    publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);
    return true;
  }
  else {
    if (LITTLEFS.begin(true)) {
      sprintf (buffer, "Formatted and initialized FS (size: %d kB; free: %d kB)", LITTLEFS.totalBytes()/1024, fs_free());
      publish_event (STS_THIS, SS_THIS, EVENT_WARNING, buffer);
      tm_this->fs_enabled = true;
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
    ftpSrv.begin(config_network.ftp_user, config_network.ftp_password);
    publish_event (STS_THIS, SS_THIS, EVENT_INIT, "Initialized FTP server");
  }
  else {
    publish_event (STS_THIS, SS_THIS, EVENT_ERROR, "Failed to initialize FTP server as no file system available");
  }
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
  config_esp32.yamcs_protocol = PROTO_UDP; // TODO: test PROTO_TCP
  config_esp32.debug_over_serial = true;
  config_esp32.ota_enable = false;
  strcpy (config_esp32cam.config_file, "/default.cfg");
  strcpy (config_esp32cam.routing_file, "/default.rt");
  config_esp32cam.wifi_enable = true;
  config_esp32cam.wifi_ap_enable = true;
  config_esp32cam.wifi_sta_enable = true;
  config_esp32cam.wifi_udp_enable = true;
  config_esp32cam.wifi_yamcs_enable = true;
  config_esp32cam.wifi_image_enable = true; 
  config_esp32cam.fs_enable = true;
  config_esp32cam.sd_enable = true;
  config_esp32cam.sd_json_enable = true;
  config_esp32cam.sd_ccsds_enable = true;
  config_esp32cam.sd_image_enable = true;
  config_esp32cam.serial_format = ENC_JSON; // TODO: put to ENC_CCSDS after debug phase
  config_esp32cam.yamcs_protocol = PROTO_UDP; // TODO: test TCP
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
  set_routing (routing_serial, (char*)rt_serial);
  set_routing (routing_yamcs, (char*)rt_yamcs);
  set_routing (routing_udp, (char*)rt_udp);
  set_routing (routing_fs, (char*)rt_fs);
  #endif
  #ifdef PLATFORM_ESP32CAM
  char rt_serial[40] =   " 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0 ";
  char rt_yamcs[40] =    " 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0 ";
  char rt_udp[40] =      " 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ";
  char rt_fs[40] =       " 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1 ";
  char rt_sd_json[40] =  " 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1 ";
  char rt_sd_ccsds[40] = " 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1 ";
  set_routing (routing_serial, (char*)rt_serial);
  set_routing (routing_yamcs, (char*)rt_yamcs);
  set_routing (routing_udp, (char*)rt_udp);
  set_routing (routing_fs, (char*)rt_fs);
  set_routing (routing_sd_json, (char*)rt_sd_json);
  set_routing (routing_sd_ccsds, (char*)rt_sd_ccsds);
  #endif
}

bool fs_load_settings() {
  char linebuffer[80];
  uint8_t value_start;
  File file = LITTLEFS.open ("/current.cfg");
  if (!file) {
    publish_event (STS_THIS, SS_THIS, EVENT_ERROR, "Failed to open configuration file '/current.cfg' from FS; using defaults");
    return false;
  }
  uint8_t i = 0; 
  while (file.available ()) {
    linebuffer[i] = file.read();
    if (linebuffer[i] == '=') {
      value_start = i + 1;
    }
    if (linebuffer[i] != '\r') { // ignore \r
      if (linebuffer[i] == '\n') { // full line read
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
  return true;
}

bool fs_load_config() {
  char linebuffer[80];
  uint8_t value_start;
  File file = LITTLEFS.open (config_this->config_file);
  if (!file) {
    sprintf (buffer, "Failed to open configuration file '%s' from FS; using defaults", config_this->config_file);
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
        if (String(linebuffer).startsWith("wifi_ssid")) {
          strcpy (config_network.wifi_ssid, String(linebuffer).substring(value_start).c_str());
          Serial.println ("Set wifi_ssid to " + String (config_network.wifi_ssid));
        }
        if (String(linebuffer).startsWith("wifi_password")) {
          strcpy (config_network.wifi_password, String(linebuffer).substring(value_start).c_str());
          Serial.println ("Set wifi_password to " + String (config_network.wifi_password));
        }
        if (String(linebuffer).startsWith("ap_ssid")) {
          strcpy (config_network.ap_ssid, String(linebuffer).substring(value_start).c_str());
           Serial.println ("Set ap_ssid to " + String (config_network.ap_ssid));
        }
        if (String(linebuffer).startsWith("ap_password")) {
          strcpy (config_network.ap_password, String(linebuffer).substring(value_start).c_str());
          Serial.println ("Set ap_password to " + String (config_network.ap_password));
        }
        if (String(linebuffer).startsWith("ftp_user")) {
          strcpy (config_network.ftp_user, String(linebuffer).substring(value_start).c_str());
          Serial.println ("Set ftp_user to " + String (config_network.ftp_user));
        }
        if (String(linebuffer).startsWith("ftp_password")) {
          strcpy (config_network.ftp_password, String(linebuffer).substring(value_start).c_str());
          Serial.println ("Set ftp_password to " + String (config_network.ftp_password));
        }
        if (String(linebuffer).startsWith("udp_server")) {
          strcpy (config_network.udp_server, String(linebuffer).substring(value_start).c_str());
          Serial.println ("Set udp_server to " + String (config_network.udp_server));
        }
        if (String(linebuffer).startsWith("yamcs_server")) {
          strcpy (config_network.yamcs_server, String(linebuffer).substring(value_start).c_str());
          Serial.println ("Set yamcs_server to " + String (config_network.yamcs_server));
        }
        if (String(linebuffer).startsWith("ntp_server")) {
          strcpy (config_network.ntp_server, String(linebuffer).substring(value_start).c_str());
          Serial.println ("Set ntp_server to " + String (config_network.ntp_server));
        }
        if (String(linebuffer).startsWith("udp_port")) {
          config_network.udp_port = String(linebuffer).substring(value_start).toInt();
          Serial.println ("Set udp_port to " + String (config_network.udp_port));
        }
        if (String(linebuffer).startsWith("yamcs_tm_port")) {
          config_network.yamcs_tm_port = String(linebuffer).substring(value_start).toInt();
          Serial.println ("Set yamcs_tm_port to " + String (config_network.yamcs_tm_port));
        }
        if (String(linebuffer).startsWith("yamcs_tc_port")) {
          config_network.yamcs_tc_port = String(linebuffer).substring(value_start).toInt();
          Serial.println ("Set yamcs_tc_port to " + String (config_network.yamcs_tc_port));
        }
        if (String(linebuffer).startsWith("radio_rate")) {
          config_this->radio_rate = String(linebuffer).substring(value_start).toInt();
          Serial.println ("Set radio_rate to " + String (config_this->radio_rate) + " Hz");
        }
        if (String(linebuffer).startsWith("pressure_rate")) {
          config_this->pressure_rate = String(linebuffer).substring(value_start).toInt();
          Serial.println ("Set pressure_rate to " + String (config_this->pressure_rate) + " Hz");
        }
        if (String(linebuffer).startsWith("motion_rate")) {
          config_this->motion_rate = String(linebuffer).substring(value_start).toInt();
          Serial.println ("Set motion_rate to " + String (config_this->motion_rate) + " Hz");
        }
        if (String(linebuffer).startsWith("gps_rate")) {
          config_this->gps_rate = String(linebuffer).substring(value_start).toInt();
          Serial.println ("Set gps_rate to " + String (config_this->gps_rate) + " Hz");
        }
        if (String(linebuffer).startsWith("wifi_enable")) {
          config_this->wifi_enable = String(linebuffer).substring(value_start).toInt();
          Serial.println ("Set wifi_enable to " + String(config_this->wifi_enable?"true":"false"));
        }
        if (String(linebuffer).startsWith("wifi_sta_enable")) {
          config_this->wifi_sta_enable = String(linebuffer).substring(value_start).toInt();
          Serial.println ("Set wifi_sta_enable to " + String(config_this->wifi_sta_enable?"true":"false"));
        }
        if (String(linebuffer).startsWith("wifi_ap_enable")) {
          config_this->wifi_ap_enable = String(linebuffer).substring(value_start).toInt();
          Serial.println ("Set wifi_ap_enable to " + String(config_this->wifi_ap_enable?"true":"false"));
        }
        if (String(linebuffer).startsWith("wifi_udp_enable")) {
          config_this->wifi_udp_enable = String(linebuffer).substring(value_start).toInt();
          Serial.println ("Set wifi_udp_enable to " + String(config_this->wifi_udp_enable?"true":"false"));
        }
        if (String(linebuffer).startsWith("wifi_yamcs_enable")) {
          config_this->wifi_yamcs_enable = String(linebuffer).substring(value_start).toInt();
          Serial.println ("Set wifi_yamcs_enable to " + String(config_this->wifi_yamcs_enable?"true":"false"));
        }
        if (String(linebuffer).startsWith("fs_enable")) {
          config_this->fs_enable = String(linebuffer).substring(value_start).toInt();
          Serial.println ("Set fs_enable to " + String(config_this->fs_enable?"true":"false"));
        }
        if (String(linebuffer).startsWith("radio_enable")) {
          config_this->radio_enable = String(linebuffer).substring(value_start).toInt();
          Serial.println ("Set radio_enable to " + String(config_this->radio_enable?"true":"false"));
        }
        if (String(linebuffer).startsWith("pressure_enable")) {
          config_this->pressure_enable = String(linebuffer).substring(value_start).toInt();
          Serial.println ("Set pressure_enable to " + String(config_this->pressure_enable?"true":"false"));
        }
        if (String(linebuffer).startsWith("motion_enable")) {
          config_this->motion_enable = String(linebuffer).substring(value_start).toInt();
          Serial.println ("Set motion_enable to " + String(config_this->motion_enable?"true":"false"));
        }
        if (String(linebuffer).startsWith("gps_enable")) {
          config_this->gps_enable = String(linebuffer).substring(value_start).toInt();
          Serial.println ("Set gps_enable to " + String(config_this->gps_enable?"true":"false"));
        }
        if (String(linebuffer).startsWith("camera_enable")) {
          config_this->camera_enable = String(linebuffer).substring(value_start).toInt();
          Serial.println ("Set camera_enable to " + String(config_this->camera_enable?"true":"false"));
        }
        if (String(linebuffer).startsWith("serial_format")) {
          config_this->serial_format = String(linebuffer).substring(value_start).toInt();
          Serial.println ("Set serial_format to " + String(dataEncodingName[config_this->serial_format]));
        }
        if (String(linebuffer).startsWith("yamcs_protocol")) {
          config_this->yamcs_protocol = String(linebuffer).substring(value_start).toInt();
          Serial.println ("Set yamcs_protocol to " + String(ipProtocolName[config_this->yamcs_protocol]));
        }        
        if (String(linebuffer).startsWith("debug_over_serial")) {
          config_this->debug_over_serial = String(linebuffer).substring(value_start).toInt();
          Serial.println ("Set debug_over_serial to " + String(config_this->debug_over_serial?"true":"false"));
        }
        if (String(linebuffer).startsWith("ota_enable")) {
          config_this->ota_enable = String(linebuffer).substring(value_start).toInt();
          Serial.println ("Set ota_enable to " + String(config_this->ota_enable?"true":"false"));
        }
        if (String(linebuffer).startsWith("mpu6050_accel_offset_x_sensor")) {
          mpu6050.a_z_rocket_offset = String(linebuffer).substring(value_start).toInt();
          Serial.println ("Set mpu6050.a_z_rocket_offset to " + String(mpu6050.a_z_rocket_offset));
        } 
        if (String(linebuffer).startsWith("mpu6050_accel_offset_y_sensor")) {
          mpu6050.a_x_rocket_offset = String(linebuffer).substring(value_start).toInt();
          Serial.println ("Set mpu6050.a_x_rocket_offset to " + String(mpu6050.a_x_rocket_offset));
        } 
        if (String(linebuffer).startsWith("mpu6050_accel_offset_z_sensor")) {
          mpu6050.a_y_rocket_offset = String(linebuffer).substring(value_start).toInt();
          Serial.println ("Set mpu6050.a_y_rocket_offset to " + String(mpu6050.a_y_rocket_offset));
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
  return true;
}

bool fs_load_routing() {
  char linebuffer[80], parameter[80];
  uint8_t value_start;
  File file = LITTLEFS.open (config_this->routing_file);
  if (!file) {
    sprintf (buffer, "Failed to open routing file '%s' from FS; using defaults", config_this->routing_file);
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
          Serial.println ("Set routing_serial to " + set_routing (routing_serial, (char*)parameter));
        }
        if (String(linebuffer).startsWith("rt_yamcs")) {
          Serial.println ("Set routing_yamcs to " + set_routing (routing_yamcs, (char*)parameter));
        }
        if (String(linebuffer).startsWith("rt_udp")) {
          Serial.println ("Set routing_udp to " + set_routing (routing_udp, (char*)parameter));
        }
        if (String(linebuffer).startsWith("rt_fs")) {
          Serial.println ("Set routing_fs to " + set_routing (routing_fs, (char*)parameter));
        }
        #ifdef PLATFORM_ESP32CAM
        if (String(linebuffer).startsWith("rt_sd_json")) {
          Serial.println ("Set routing_sd_json to " + set_routing (routing_sd_json, (char*)parameter));
        }
        if (String(linebuffer).startsWith("rt_sd_ccsds")) {
          Serial.println ("Set routing_sd_ccsds to " + set_routing (routing_sd_ccsds, (char*)parameter));
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
  return true;
}

String set_routing (char* routing_table, char* routing_string) {
  uint8_t PID;
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

// WIFI FUNCTIONALITY

bool wifi_setup () {
  WiFi.mode(WIFI_AP_STA); 
  if (config_this->wifi_ap_enable) {
    wifi_ap_setup ();
  }
  if (config_this->wifi_sta_enable) {
    return wifi_sta_setup ();
  }
  return false;
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
  if (!tm_this->time_current and timeClient.update()) {
    sprintf (buffer, "Time on %s set via NTP server %s: %s", subsystemName[SS_THIS], config_network.ntp_server, timeClient.getFormattedDate().c_str());
    publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);
    tm_this->time_current = true;
    fs_create_today_dir ();
    return true;
  }
  return false;
}

// TM/TC FUNCTIONALITY (serial/udp/yamcs/fs/sd)

void publish_packet (uint8_t PID) {
  static uint16_t ccsds_len, payload_len, payload_ctr;
  static bool payload_type;
  static byte* payload_ptr;
  static uint32_t start_millis;
  
  // prepare packet payload
  switch (PID) {
    case STS_ESP32:      payload_len = sts_esp32.message_size + 6;
                         payload_ptr = (byte*)&sts_esp32;
                         payload_type = PKT_TM;
                         break;
    case STS_ESP32CAM:   payload_len = sts_esp32cam.message_size + 6;
                         payload_ptr = (byte*)&sts_esp32cam;
                         payload_type = PKT_TM;
                         break;
    case TM_ESP32:       payload_len = sizeof(tm_esp32_t);
                         payload_ptr = (byte*)&esp32;
                         payload_type = PKT_TM;
                         break;
    case TM_ESP32CAM:    payload_len = sizeof(tm_esp32cam_t);
                         payload_ptr = (byte*)&esp32cam;
                         payload_type = PKT_TM;
                         break;                  
    case TM_CAMERA:      payload_len = sizeof(tm_camera_t) - 26 + strlen (ov2640.filename) + 1;
                         payload_ptr = (byte*)&ov2640;
                         payload_type = PKT_TM;
                         break;    
    case TM_GPS:         payload_len = sizeof(tm_gps_t);
                         payload_ptr = (byte*)&neo6mv2;
                         payload_type = PKT_TM;
                         break;              
    case TM_MOTION:      payload_len = sizeof(tm_motion_t);
                         payload_ptr = (byte*)&mpu6050;
                         payload_type = PKT_TM;
                         break;
    case TM_PRESSURE:    payload_len = sizeof(tm_pressure_t);
                         payload_ptr = (byte*)&bmp280;
                         payload_type = PKT_TM;
                         break;
    case TM_RADIO:       payload_len = sizeof(tm_radio_t);
                         payload_ptr = (byte*)&radio;
                         payload_type = PKT_TM;
                         break;   
    case TM_TIMER:       payload_len = sizeof(tm_timer_t);
                         payload_ptr = (byte*)&timer;
                         payload_type = PKT_TM;
                         break;
    case TC_ESP32:       payload_len = tc_esp32.json_size + 1;
                         payload_ptr = (byte*)&tc_esp32;
                         payload_type = PKT_TC;
                         break;
    case TC_ESP32CAM:    payload_len = tc_esp32cam.json_size + 1;
                         payload_ptr = (byte*)&tc_esp32cam;
                         payload_type = PKT_TC;
                         break;                                                                                                                            
  }
  payload_ctr = update_packet (PID);
  
  // prepare CCSDS packet
  ccsds_len = sizeof(ccsds_hdr_t) + payload_len;
  update_ccsds_hdr (PID, payload_type, payload_len);
  memcpy (&ccsds_buffer.ccsds_hdr, &ccsds_hdr, sizeof (ccsds_hdr_t));
  memcpy (&ccsds_buffer.blob, payload_ptr, payload_len);
  
  // save CCSDS packet to FS (which will also act as a buffer for replay)
  if (routing_fs[PID] and config_this->fs_enable) {
    start_millis = millis();
    publish_fs (PID, payload_ctr, &ccsds_buffer, ccsds_len);
    timer.publish_fs_duration += millis() - start_millis;
  }
  // serial
  if (routing_serial[PID]) {
    start_millis = millis();
    publish_serial (PID, payload_ctr, &ccsds_buffer, ccsds_len);
    timer.publish_serial_duration += millis() - start_millis;
  }
  // Yamcs
  if (routing_yamcs[PID] and config_this->wifi_enable and config_this->wifi_yamcs_enable) {
    start_millis = millis();
    publish_yamcs (PID, payload_ctr, &ccsds_buffer, ccsds_len);
    timer.publish_yamcs_duration += millis() - start_millis;
  }
  // UDP
  if (routing_udp[PID] and config_this->wifi_enable and config_this->wifi_udp_enable) {
    start_millis = millis();
    publish_udp_packet (PID, payload_ctr, &ccsds_buffer, ccsds_len);
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
    publish_sd_json (PID, payload_ctr, &ccsds_buffer, ccsds_len);
  }
  if (routing_sd_ccsds[PID] and config_this->sd_enable and config_this->sd_ccsds_enable) {
    publish_sd_ccsds (PID, payload_ctr, &ccsds_buffer, ccsds_len);
  }
  #endif
  reset_packet (PID);
}

void publish_event (uint8_t PID, uint8_t subsystem, uint8_t event_type, const char* event_message) {
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
                         sts_esp32.message_size = strlen (event_message) + 1;
                         break; 
    case STS_ESP32CAM:   sts_esp32cam.subsystem = subsystem;
                         sts_esp32cam.type = event_type;
                         strcpy (sts_esp32cam.message, event_message);
                         sts_esp32cam.message_size = strlen (event_message) + 1;
                         break;
    case TC_ESP32:       tc_esp32.cmd = subsystem;
                         strcpy (tc_esp32.json, event_message); 
                         tc_esp32.json_size = strlen(event_message);
                         break;
    case TC_ESP32CAM:    tc_esp32cam.cmd = subsystem;
                         strcpy (tc_esp32cam.json, event_message); 
                         tc_esp32cam.json_size = strlen(event_message);
                         break;
  }
  publish_packet (PID);
}

void publish_tc (uint8_t PID, uint8_t cmd, char* json_str) {
  publish_event (PID, cmd, EVENT_CMD, json_str);
}

bool publish_fs (uint8_t PID, uint16_t payload_ctr, ccsds_t* ccsds_ptr, uint16_t ccsds_len) {
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
    tm_this->fs_current = true;
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

bool publish_serial (uint8_t PID, uint16_t payload_ctr, ccsds_t* ccsds_ptr, uint16_t ccsds_len) {
  if (tm_this->serial_connected) {
    if (payload_ctr > var_this->serial_last_packet[PID] + 1 and tm_this->fs_enabled) {
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
        tm_this->fs_enabled = true;
        tm_this->fs_current = true;
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
  
bool publish_yamcs (uint8_t PID, uint16_t payload_ctr, ccsds_t* ccsds_ptr, uint16_t ccsds_len) { 
  if (tm_this->wifi_connected) {
    if (payload_ctr > var_this->yamcs_last_packet[PID] + 1 and tm_this->wifi_yamcs_enabled) {
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
        tm_this->wifi_enabled = true;
        tm_this->wifi_current = true;
        fs_ccsds.seek(ccsds_len * var_this->yamcs_last_packet[PID]);         
        while (payload_ctr > var_this->yamcs_last_packet[PID] and batch_count++ < BUFFER_RELEASE_BATCH_SIZE) {
          fs_ccsds.read((uint8_t*)ccsds_ptr, (size_t)ccsds_len); 
          if (get_ccsds_packet_ctr(ccsds_ptr) == ++var_this->yamcs_last_packet[PID]) {
            switch ((uint8_t)config_this->yamcs_protocol) {
              case PROTO_UDP: wifiUDP.beginPacket(config_network.yamcs_server, config_network.yamcs_tm_port);
                              wifiUDP.write ((const uint8_t*)ccsds_ptr, get_ccsds_len (ccsds_ptr) + sizeof(ccsds_hdr_t));
                              wifiUDP.endPacket();
                              break;
              case PROTO_TCP: if (!wifiTCP.connect(config_network.yamcs_server, config_network.yamcs_tm_port)) {
                                tm_this->err_yamcs_dataloss = true;
                                return false;
                              }
                              wifiTCP.write ((const uint8_t*)ccsds_ptr, ccsds_len);
                              break;
            }
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
      switch ((uint8_t)config_this->yamcs_protocol) {
        case PROTO_UDP: wifiUDP.beginPacket(config_network.yamcs_server, config_network.yamcs_tm_port);
                        wifiUDP.write ((const uint8_t*)ccsds_ptr, ccsds_len);
                        wifiUDP.endPacket();   
                        break;
        case PROTO_TCP: if (!wifiTCP.connect(config_network.yamcs_server, config_network.yamcs_tm_port)) {
                          tm_this->err_yamcs_dataloss = true;
                          return false;
                        }
                        wifiTCP.write ((const uint8_t*)ccsds_ptr, ccsds_len);
                        break;
      }
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

bool publish_udp_packet (uint8_t PID, uint16_t payload_ctr, ccsds_t* ccsds_ptr, uint16_t ccsds_len) { 
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

bool publish_udp_text (char* message) { 
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
bool sd_json_publish (uint8_t PID, uint16_t payload_ctr, ccsds_t* ccsds_ptr, uint16_t ccsds_len) {
  // TODO: TBW
}

bool sd_ccsds_publish (uint8_t PID, uint16_t payload_ctr, ccsds_t* ccsds_ptr, uint16_t ccsds_len) {
  // TODO: TBW
}

uint16_t sd_free () {
  return ((SD_MMC.totalBytes() - SD_MMC.usedBytes())/1024);
}
#endif

uint16_t get_ccsds_len (ccsds_t* ccsds_ptr) {
  return (256*(uint16_t)*((byte*)ccsds_ptr+4)+(uint16_t)*((byte*)ccsds_ptr+5)+1);
}


uint32_t get_ccsds_millis (ccsds_t* ccsds_ptr) {
  return (65536*(uint8_t)*((byte*)ccsds_ptr+8)+256*(uint8_t)*((byte*)ccsds_ptr+7)+(uint8_t)*((byte*)ccsds_ptr+6));
}

uint16_t get_ccsds_packet_ctr (ccsds_t* ccsds_ptr) {
  return (256*(uint8_t)*((byte*)ccsds_ptr+10)+(uint8_t)*((byte*)ccsds_ptr+9));
}

void update_ccsds_hdr (uint8_t PID, bool pkt_type, uint16_t pkt_len) {
  static uint16_t ccsds_ctr[NUMBER_OF_PID];
  ccsds_ctr[PID]++;
  ccsds_hdr.version = 0;
  ccsds_hdr.type = pkt_type;
  ccsds_hdr.sec_hdr = false;
  ccsds_hdr.apid_H = (uint8_t)((PID + 42) >> 8);
  ccsds_hdr.apid_L = (uint8_t)(PID + 42);
  ccsds_hdr.seq_flag = 3;
  ccsds_hdr.seq_ctr_H = (uint8_t)(ccsds_ctr[PID] >> 8);
  ccsds_hdr.seq_ctr_L = (uint8_t)ccsds_ctr[PID];
  ccsds_hdr.pkt_len_H = (uint8_t)((pkt_len - 1) >> 8);
  ccsds_hdr.pkt_len_L = (uint8_t)(pkt_len - 1);
}

void build_json_str (char* json_buffer, uint8_t PID, ccsds_t* ccsds_ptr) {
  switch (PID) {
    case STS_ESP32:      { 
                           sts_esp32_t* sts_esp32_ptr = (sts_esp32_t*)((byte*)ccsds_ptr + sizeof(ccsds_hdr_t));
                           sprintf (json_buffer, "{\"ctr\":%u,\"type\":\"%s\",\"ss\":\"%s\",\"msg\":\"%s\"}", sts_esp32_ptr->packet_ctr, eventName[sts_esp32_ptr->type], subsystemName[sts_esp32_ptr->subsystem], sts_esp32_ptr->message);
                         }
                         break;
    case STS_ESP32CAM:   { 
                           sts_esp32cam_t* sts_esp32cam_ptr = (sts_esp32cam_t*)((byte*)ccsds_ptr + sizeof(ccsds_hdr_t));
                           sprintf (json_buffer, "{\"ctr\":%u,\"type\":\"%s\",\"ss\":\"%s\",\"msg\":\"%s\"}", sts_esp32cam_ptr->packet_ctr, eventName[sts_esp32cam_ptr->type], subsystemName[sts_esp32cam_ptr->subsystem], sts_esp32cam_ptr->message);
                         }
                         break;                  
    case TM_ESP32:       {
                           tm_esp32_t* esp32_ptr = (tm_esp32_t*)((byte*)ccsds_ptr + sizeof(ccsds_hdr_t));
                           sprintf (json_buffer, "{\"ctr\":%u,\"opsmode\":\"%s\",\"err\":%u,\"warn\":%u,\"mem_free\":%u,\"fs_free\":%u,\"press_rate\":%u,\"motion_rate\":%u,\"cam_rate\":%u,\"gps_rate\":%u,\"radio_rate\":%u,\"sep_sts\":%d,\"enabled\":\"%d%d%d%d%d%d%d%d\",\"current\":\"%d%d%d%d%d\",\"conn\":\"%d%d\"}", 
                                    esp32_ptr->packet_ctr, modeName[esp32_ptr->opsmode], esp32_ptr->error_ctr, esp32_ptr->warning_ctr, esp32_ptr->mem_free, esp32_ptr->fs_free, esp32_ptr->pressure_rate, esp32_ptr->motion_rate, esp32_ptr->camera_rate, esp32_ptr->gps_rate, esp32_ptr->radio_rate, esp32_ptr->separation_sts,
                                    esp32_ptr->radio_enabled, esp32_ptr->pressure_enabled, esp32_ptr->motion_enabled, esp32_ptr->gps_enabled, esp32_ptr->camera_enabled, esp32_ptr->wifi_enabled, esp32_ptr->wifi_udp_enabled, esp32_ptr->wifi_yamcs_enabled, 
                                    esp32_ptr->radio_current, esp32_ptr->pressure_current, esp32_ptr->motion_current, esp32_ptr->gps_current, esp32_ptr->camera_current,
                                    esp32_ptr->serial_connected, esp32_ptr->wifi_connected); 
                         }
                         break;
    case TM_ESP32CAM:    {
                           tm_esp32cam_t* esp32cam_ptr = (tm_esp32cam_t*)((byte*)ccsds_ptr + sizeof(ccsds_hdr_t));
                           sprintf (json_buffer, "{\"ctr\":%u,\"err\":%u,\"warn\":%u,\"mem_free\":%u,\"fs_free\":%u,\"sd_free\":%u,\"img_rate\":%u,\"enabled\":\"%d%d%d%d%d%d%d%d%d\",\"current\":\"%d%d\",\"connected\":\"%d%d\"}",
                                    esp32cam_ptr->packet_ctr, esp32cam_ptr->error_ctr, esp32cam_ptr->warning_ctr, esp32cam_ptr->mem_free, esp32cam_ptr->fs_free, esp32cam_ptr->sd_free, esp32cam_ptr->camera_image_rate, 
                                    esp32cam_ptr->wifi_enabled, esp32cam_ptr->wifi_udp_enabled, esp32cam_ptr->wifi_yamcs_enabled, esp32cam_ptr->wifi_image_enabled, esp32cam_ptr->camera_enabled, esp32cam_ptr->sd_enabled, esp32cam_ptr->sd_image_enabled, esp32cam_ptr->sd_json_enabled, esp32cam_ptr->sd_ccsds_enabled,  
                                    esp32cam_ptr->sd_current, esp32cam_ptr->camera_current,
                                    esp32cam_ptr->serial_connected, esp32cam_ptr->wifi_connected);
                         }
                         break;
    case TM_CAMERA:      {
                           tm_camera_t* ov2640_ptr = (tm_camera_t*)((byte*)ccsds_ptr + sizeof(ccsds_hdr_t));
                           sprintf (json_buffer, "{\"ctr\":%u,\"mode\":\"%s\",\"res\":\"%s\",\"auto_res\":%d,\"file\":\"%s\",\"size\":%u,\"exp\":%u,\"sd\":%u,\"wifi\":%u}", 
                                    ov2640_ptr->packet_ctr, cameraModeName[ov2640_ptr->camera_mode], cameraResolutionName[ov2640_ptr->resolution], ov2640_ptr->auto_res, ov2640_ptr->filename, ov2640_ptr->filesize, ov2640_ptr->exposure_ms, ov2640_ptr->sd_ms, ov2640_ptr->wifi_ms);
                         }
                         break;
    case TM_GPS:         {
                           tm_gps_t* neo6mv2_ptr = (tm_gps_t*)((byte*)ccsds_ptr + sizeof(ccsds_hdr_t));
                           *json_buffer++ = '{';
                           json_buffer += sprintf (json_buffer, "\"ctr\":%u,\"sts\":\"%s\",\"sats\":%d", neo6mv2_ptr->packet_ctr, gpsStatusName[neo6mv2_ptr->status], neo6mv2_ptr->satellites);
                           if (neo6mv2_ptr->time_valid) {
                             json_buffer += sprintf (json_buffer, ",\"time\":\"%02d:%02d:%02d\"", neo6mv2_ptr->hours, neo6mv2_ptr->minutes, neo6mv2_ptr->seconds);
                           }
                           if (neo6mv2_ptr->location_valid) {
                             json_buffer += sprintf (json_buffer, ",\"lat\":%.6f,\"lon\":%.6f", neo6mv2_ptr->latitude, neo6mv2_ptr->longitude);
                           }
                           if (neo6mv2_ptr->altitude_valid) {
                             json_buffer += sprintf (json_buffer, ",\"alt\":%.2f", neo6mv2_ptr->altitude);
                           }
                           if (neo6mv2_ptr->speed_valid) {
                             json_buffer += sprintf (json_buffer, ",\"v_north\":%.2f,\"v_east\":%.2f,\"v_down\":%.2f", neo6mv2_ptr->v_north, neo6mv2_ptr->v_east, neo6mv2_ptr->v_down);
                           }
                           if (neo6mv2_ptr->hdop_valid) {
                             json_buffer += sprintf (json_buffer, ",\"hdop\":%d.%03d", neo6mv2_ptr->milli_hdop/1000, neo6mv2_ptr->milli_hdop%1000);
                           }
                           if (neo6mv2_ptr->hdop_valid) {
                             json_buffer += sprintf (json_buffer, ",\"vdop\":%d.%03d", neo6mv2_ptr->milli_vdop/1000, neo6mv2_ptr->milli_vdop%1000);
                           }
                           *json_buffer++ = '}';
                           *json_buffer++ = 0;
                         }
                         break;
    case TM_MOTION:      {
                           tm_motion_t* mpu6050_ptr = (tm_motion_t*)((byte*)ccsds_ptr + sizeof(ccsds_hdr_t));
                           sprintf (json_buffer, "{\"ctr\":%u,\"accel\":[\"X\":%.2f,\"Y\":%.2f,\"Z\":%.2f],\"gyro\":[\"X\":%.2f,\"Y\":%.2f,\"Z\":%.2f],\"tilt\":%.2f,\"g\":%.2f,\"a\":%.2f,\"rpm\":%.2f}", 
                                    mpu6050_ptr->packet_ctr, mpu6050_ptr->accel_x_rocket, mpu6050_ptr->accel_y_rocket, mpu6050_ptr->accel_z_rocket, mpu6050_ptr->gyro_x_rocket, mpu6050_ptr->gyro_y_rocket, mpu6050_ptr->gyro_z_rocket, mpu6050_ptr->tilt, mpu6050_ptr->g, mpu6050_ptr->a, mpu6050_ptr->rpm); 
                         }
                         break;
    case TM_PRESSURE:    {
                           tm_pressure_t* bmp280_ptr = (tm_pressure_t*)((byte*)ccsds_ptr + sizeof(ccsds_hdr_t));
                           sprintf (json_buffer, "{\"ctr\":%u,\"p\":%.2f,\"p0\":%.2f,\"T\":%.2f,\"h\":%.2f,\"v_v\":%.2f}", bmp280_ptr->packet_ctr, bmp280_ptr->pressure, bmp280_ptr->zero_level_pressure, bmp280_ptr->temperature, bmp280_ptr->altitude, bmp280_ptr->velocity_v);
                         }
                         break;
    case TM_RADIO:       {
                           tm_radio_t* radio_ptr = (tm_radio_t*)((byte*)ccsds_ptr + sizeof(ccsds_hdr_t));
                           build_hex_str ((unsigned char*)&radio, sizeof(tm_radio_t), (char*)&radio_buffer, 2*sizeof(tm_radio_t)+2); 
                           sprintf (json_buffer, "{\"ctr\":%u,\"data\":\"%s\"}", radio_ptr->packet_ctr, radio_buffer);
                         }
                         break;
    case TM_TIMER:       {
                           tm_timer_t* timer_ptr = (tm_timer_t*)((byte*)ccsds_ptr + sizeof(ccsds_hdr_t));
                           sprintf (json_buffer, "{\"ctr\":%u,\"radio\":%u,\"pressure\":%u,\"motion\":%u,\"gps\":%u,\"esp32cam\":%u}", 
                                    timer_ptr->packet_ctr, timer_ptr->radio_duration, timer_ptr->pressure_duration, timer_ptr->motion_duration, timer_ptr->gps_duration, timer_ptr->esp32cam_duration);
                         }
                         break;
    case TC_ESP32:       {
                           tc_esp32_t* tc_esp32_ptr = (tc_esp32_t*)((byte*)ccsds_ptr + sizeof(ccsds_hdr_t));
                           if (tc_esp32_ptr->json_size) {
                             sprintf (json_buffer, "{\"cmd\":\"%s\",%s", tcName[tc_esp32_ptr->cmd], (char*)(&tc_esp32_ptr->json+1));
                           }
                           else {
                             sprintf (json_buffer, "{\"cmd\":\"%s\"}", tcName[tc_esp32_ptr->cmd]); 
                           }
                         }
                         break;
    case TC_ESP32CAM:    {
                           tc_esp32cam_t* tc_esp32cam_ptr = (tc_esp32cam_t*)((byte*)ccsds_ptr + sizeof(ccsds_hdr_t));
                           if (tc_esp32cam_ptr->json_size) {
                             sprintf (json_buffer, "{\"cmd\":\"%s\",%s", tcName[tc_esp32cam_ptr->cmd], (char*)(&tc_esp32cam_ptr->json+1));
                           }
                           else {
                             sprintf (json_buffer, "{\"cmd\":\"%s\"}", tcName[tc_esp32cam_ptr->cmd]); 
                           }
                         }
                         break;  
  }
}

uint16_t update_packet (uint8_t PID) {
  static uint16_t packet_ctr;
  switch (PID) {
    #ifdef PLATFORM_ESP32
    case STS_ESP32:      sts_esp32.millis = millis();
                         packet_ctr = ++sts_esp32.packet_ctr;
                         break;                
    case TM_ESP32:       esp32.millis = millis();
                         packet_ctr = ++esp32.packet_ctr; 
                         esp32.mem_free = ESP.getFreeHeap()/1024;
                         esp32.fs_free = fs_free ();
                         // compensate for this packet being prepared before it is actually sent
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
    case TM_RADIO:       radio.millis = millis ();
                         packet_ctr = ++radio.packet_ctr;
                         radio.opsmode = esp32.opsmode;
                         radio.error_ctr = esp32.error_ctr + esp32cam.error_ctr;
                         radio.warning_ctr = esp32.warning_ctr + esp32cam.warning_ctr;
                         radio.esp32_yamcs_buffer = esp32.yamcs_buffer;
                         radio.esp32_serial_buffer = esp32.serial_buffer;
                         radio.esp32cam_yamcs_buffer = esp32cam.yamcs_buffer;
                         radio.esp32cam_serial_buffer = esp32cam.serial_buffer;
                         radio.separation_sts = esp32.separation_sts;
                         radio.pressure_altitude = max((uint8_t)0,(uint8_t)round(bmp280.altitude));
                         radio.pressure_velocity_v = (int8_t)round(bmp280.velocity_v);
                         radio.temperature = (uint8_t)round(bmp280.temperature);
                         radio.motion_tilt = (uint8_t)round(mpu6050.tilt);
                         radio.motion_g = (uint8_t)round(mpu6050.g*10);  
                         radio.motion_a = (int8_t)round(mpu6050.a);
                         radio.motion_rpm = (int8_t)round(mpu6050.rpm); 
                         radio.gps_satellites = neo6mv2.satellites;
                         radio.gps_altitude = max((uint8_t)0,(uint8_t)round(neo6mv2.altitude));
                         radio.gps_velocity_v = (int8_t)round(-neo6mv2.v_down/100);
                         radio.gps_velocity = (uint8_t)round(sqrt (neo6mv2.v_north*neo6mv2.v_north + neo6mv2.v_east*neo6mv2.v_east + neo6mv2.v_down*neo6mv2.v_down) / 1000000.0);   
                         radio.camera_image_ctr = ov2640.packet_ctr;
                         radio.camera_rate = esp32cam.camera_image_rate;
                         radio.sd_free = esp32cam.sd_free;
                         radio.serial_connected = esp32.serial_connected;
                         radio.esp32_wifi_connected = esp32.wifi_connected;
                         radio.esp32cam_wifi_connected = esp32cam.wifi_connected;
                         break;
    case TM_TIMER:       packet_ctr = ++timer.packet_ctr;
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
                         esp32cam.mem_free = ESP.getFreeHeap();
                         esp32cam.sd_free = sd_free();
                         // compensate for this packet being prepared before it is actually sent
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
                         break;
    case TM_CAMERA:      packet_ctr = ++ov2640.packet_ctr; 
                         break;        
    case TC_ESP32:       // do nothing
                         break;
    #endif
  }
  return (packet_ctr);
}

void reset_packet (uint8_t PID) {
  switch (PID) {
    #ifdef PLATFORM_ESP32
    case STS_ESP32:      sts_esp32.message_size = 0;
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
                         esp32.fs_ftp_enabled = false;
                         esp32.warn_serial_connloss = false;
                         esp32.warn_wifi_connloss = false;
                         esp32.err_serial_dataloss = false;
                         esp32.err_yamcs_dataloss = false;    
                         esp32.pressure_current = false;
                         esp32.motion_current = false;
                         esp32.camera_current = false;
                         esp32.gps_current = false; 
                         esp32.radio_current = false;
                         break;
    case TM_GPS:         esp32.gps_rate++;
                         break;
    case TM_MOTION:      esp32.motion_rate++;
                         esp32.motion_current = true;
                         radio.motion_current = true;
                         break;
    case TM_PRESSURE:    esp32.pressure_rate++;
                         esp32.pressure_current = true;
                         radio.pressure_current = true;
                         break;
    case TM_RADIO:       radio.pressure_current = false;
                         radio.motion_current = false;
                         radio.gps_current = false; 
                         radio.camera_current = false; 
                         esp32.radio_rate++;
                         esp32.radio_current = true;
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
                         timer.publish_fs_duration = 0;
                         timer.publish_serial_duration = 0;
                         timer.publish_yamcs_duration = 0;
                         timer.publish_udp_duration = 0;
                         break;
    case TC_ESP32CAM:    tc_esp32cam.json_size = 0;
                         break;
    #endif
    #ifdef PLATFORM_ESP32CAM
    case STS_ESP32CAM:   sts_esp32cam.message_size = 0;
                         break;
    case TM_ESP32CAM:    esp32cam.udp_rate = 0;
                         esp32cam.yamcs_rate = 0;
                         esp32cam.serial_in_rate = 0;
                         esp32cam.serial_out_rate = 0;
                         esp32cam.fs_rate = 0;
                         esp32cam.fs_ftp_enabled = false;
                         esp32cam.camera_image_rate = 0;
                         esp32cam.sd_current = false;
                         esp32cam.camera_current = false;
                         break;
    case TM_CAMERA:      strcpy (ov2640.filename, "");
                         ov2640.filesize = 0;
                         ov2640.wifi_ms = 0;
                         ov2640.sd_ms = 0;
                         ov2640.exposure_ms = 0;
                         esp32.camera_rate++;
                         esp32.camera_current = true;
                         radio.camera_current = true;                            
    case TC_ESP32:       tc_esp32.json_size = 0;
                         break;
    #endif
  }
}

// RADIO FUNCTIONALITY

bool radio_setup () {
  delay(1000);
  if (radio_tx.init()) {
    publish_event (STS_ESP32, SS_RADIO, EVENT_INIT, "Radio transmitter initialized");
    return true;
  }
  else {
    publish_event (STS_ESP32, SS_RADIO, EVENT_ERROR, "Failed to initialize radio transmitter");
    return false; 
  }
}

void publish_radio () {
  if (esp32.radio_enabled) {
    radio_tx.send((uint8_t *)&radio, sizeof(tm_radio_t));
  }
}

// SERIAL FUNCTIONALITY

uint16_t serial_check () {
  // gets characters from Serial, puts them in serial_buffer, and returns length of string that is ready for processing or false if string is not complete
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
        case 0x00: // start of CCSDS transmission
                   Serial.println ("Start of CCSDS transmission");
                   serial_status = SERIAL_CCSDS;
                   data_len = JSON_MAX_SIZE;
                   break;
        case '[':  // start of JSON-encoded data string
                   Serial.println ("Start of JSON transmission");
                   serial_status = SERIAL_JSON;
                   break;
        case 'O':  
        case 'o':  // keepalive or ASCII
                   Serial.println ("Start of KEEPALIVE transmission");
                   serial_status = SERIAL_KEEPALIVE;
                   break;
        default:   // ASCII
                   Serial.println ("Start of ASCII transmission");
                   serial_status = SERIAL_ASCII;
                   break;
      }
    }
    if (serial_status == SERIAL_CCSDS) {
      switch (serial_buffer_pos-1) {
        case 1:         pkt_apid = (uint8_t)(c); 
                        break;
        case 5:         data_len = min ((uint16_t)(serial_buffer[4] << 8) + (uint16_t)(serial_buffer[5]) + 6, JSON_MAX_SIZE); 
                        break;
        default:        if (serial_buffer_pos-1 == data_len) {
                          // CCSDS packet is complete
                          Serial.println ("CCSDS packet complete");
                          if (!tm_this->serial_connected) { announce_established_serial_connection (); }
                          tm_this->serial_in_rate++;
                          serial_status = SERIAL_COMPLETE;
                          serial_buffer_pos = 0;
                          return (data_len);
                        }
                        // happily build up CCSDS blob
                        break;
      }
    }
    else { // normal ASCII encoded transmission (JSON, SERIAL, KEEPALIVE)
      switch (c) {
        case 0x0A:  // received carriage return, EOL reached
                    if (serial_status == SERIAL_KEEPALIVE) {
                      switch (serial_buffer[0]) {
                        case 'O': // keep-alive confirming good connection by counterpart
                                  if (!tm_this->serial_connected) { announce_established_serial_connection (); }
                                  break;
                        case 'o': // keep-alive indicating counterpart not receiving over serial
                                  if (!config_this->debug_over_serial and tm_this->serial_connected) {
                                    tm_this->serial_connected = false;
                                    tm_this->warn_serial_connloss = true;
                                  }
                                  break;
                      }
                      Serial.println ("KEEPALIVE transmission completed");
                      serial_status = SERIAL_COMPLETE;
                      serial_buffer_pos = 0;
                      return false;
                    }
                    else {
                      serial_buffer[serial_buffer_pos-1] = 0x00;
                      if (!tm_this->serial_connected) { announce_established_serial_connection (); }
                      tm_this->serial_in_rate++;
                      Serial.print ("ASCII/JSON transmission completed: [");
                      Serial.print (strlen(serial_buffer));
                      Serial.print ("] ");
                      Serial.println (serial_buffer);
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
    if (serial_buffer_pos == JSON_MAX_SIZE) { // something is wrong: line too long // TODO: overall assessment of maximal buffer sizes
      serial_buffer[JSON_MAX_SIZE-1] = '\0';
      serial_buffer_pos = 0;
      Serial_print_charptr ((char*)&serial_buffer, JSON_MAX_SIZE);
      if (!config_this->debug_over_serial) {
        tm_this->serial_connected = false;
      }
      sprintf (buffer, "Received too long message from %s", subsystemName[SS_OTHER]);
      publish_event (STS_THIS, SS_OTHER, EVENT_ERROR, buffer);
      serial_status = SERIAL_UNKNOWN;
      return false; // invalid serial_buffer content, do not use
    }
  } // while Serial.available
  return false; // no more serial characters waiting
}

void announce_established_serial_connection () {
  tm_this->serial_connected = true;
  sprintf (buffer, "Established serial connection to %s", subsystemName[SS_OTHER]);
  publish_event (STS_THIS, SS_OTHER, EVENT_INIT, buffer);
} 

void announce_lost_serial_connection () {
  tm_this->serial_connected = false;
  tm_this->warn_serial_connloss = true;
}

void serial_parse (uint16_t data_len) {
  if (serial_buffer[0] == '[') {
    // JSON formatted message
    serial_parse_json ();
  }
  else if (serial_buffer[0] == 0x00) {
    // CCSDS formatted message
    serial_parse_ccsds (data_len);
  }
  else {
    // generic ASCII string: likely debug information
    publish_udp_text (serial_buffer);
  }
}

void serial_parse_json () { // TODO: is full coverage needed / useful?  What about millis?  Probably get rid of all this?
  static StaticJsonDocument<JSON_MAX_SIZE> obj;
  char * json_str = strchr (serial_buffer, '{');
  char * tag_str = strchr (serial_buffer, ' ') + 1;
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
                        publish_packet (STS_ESP32CAM);
                        break;                        
    case TM_ESP32CAM:   // {\"ctr\":%u,\"err\":%u,\"warn\":%u,\"mem_free\":%u,\"sd_free\":%u,\"img_rate\":%u,\"enabled\":\"%d%d%d%d%d%d%d%d%d\",\"current\":\"%d%d\",\"connected\":\"%d%d\",\"debug\":\"%d%d%d%d\"}
                        esp32cam.packet_ctr = obj["ctr"];
                        esp32cam.error_ctr = obj["err"];
                        esp32cam.warning_ctr = obj["warn"];
                        //esp32cam.tc_exec_ctr
                        //esp32cam.tc_fail_ctr
                        esp32cam.mem_free = obj["mem_free"];
                        esp32cam.fs_free = obj["fs_free"];
                        esp32cam.sd_free = obj["sd_free"];
                        esp32cam.camera_image_rate = obj["img_rate"];
                        esp32cam.wifi_enabled = (obj["enabled"][0] == '1')?1:0;     
                        esp32cam.wifi_udp_enabled = (obj["enabled"][1] == '1')?1:0;      
                        esp32cam.wifi_yamcs_enabled = (obj["enabled"][2] == '1')?1:0;    
                        esp32cam.wifi_image_enabled = (obj["enabled"][3] == '1')?1:0;    
                        esp32cam.camera_enabled = (obj["enabled"][4] == '1')?1:0;                           
                        esp32cam.sd_enabled = (obj["enabled"][5] == '1')?1:0; 
                        esp32cam.sd_image_enabled = (obj["enabled"][6] == '1')?1:0; 
                        esp32cam.sd_json_enabled = (obj["enabled"][7] == '1')?1:0;  
                        esp32cam.sd_ccsds_enabled = (obj["enabled"][8] == '1')?1:0; 
                        esp32cam.sd_current = (obj["current"][0] == '1')?1:0;           
                        esp32cam.camera_current = (obj["current"][1] == '1')?1:0; 
                        esp32cam.serial_connected = (obj["connected"][0] == '1')?1:0;
                        esp32cam.wifi_connected = (obj["connected"][1] == '1')?1:0; 
                        //esp32cam.udp_rate;
                        //esp32cam.yamcs_rate;
                        //esp32cam.serial_in_rate;
                        //esp32cam.serial_out_rate;
                        //esp32cam.yamcs_buffer;
                        //esp32cam.serial_buffer;
                        publish_packet (TM_ESP32CAM);
                        break;
    case TM_CAMERA:     // {\"ctr\":%u,\"mode\":\"%s\",\"res\":\"%s\",\"auto_res\":%d,\"file\":\"%s\",\"size\":%u,\"exp\":%u,\"sd\":%u,\"wifi\":%u}
                        ov2640.packet_ctr = obj["ctr"];
                        ov2640.camera_mode = id_of(obj["mode"], sizeof(cameraModeName[0]), (char*)&cameraModeName, sizeof(cameraModeName));
                        ov2640.resolution = id_of(obj["res"], sizeof(cameraResolutionName[0]), (char*)cameraResolutionName, sizeof(cameraResolutionName));
                        ov2640.auto_res = obj["auto_res"];
                        strcpy (ov2640.filename, obj["filename"]);
                        ov2640.filesize = obj["size"];
                        ov2640.exposure_ms = obj["exp"];
                        ov2640.sd_ms = obj["sd"];
                        ov2640.wifi_ms = obj["wifi"];
                        esp32.camera_current = true;
                        publish_packet (TM_CAMERA);
                        break;
    case TC_ESP32:      // execute command
                        if (!strcmp(obj["cmd"], "reboot")) {
                          cmd_reboot ();
                        }
                        else if (!strcmp(obj["cmd"], "reboot_fli3d")) {
                          cmd_reboot_fli3d ();
                        }
                        else if (!strcmp(obj["cmd"], "get_packet")) {
                          cmd_get_packet (json_str);
                        }
                        else if (!strcmp(obj["cmd"], "set_opsmode")) {
                          cmd_set_opsmode (json_str);
                        }
                        else if (!strcmp(obj["cmd"], "set_parameter")) {
                          cmd_set_parameter (json_str);
                        }
                        else if (!strcmp(obj["cmd"], "set_routing")) {
                          cmd_set_routing (json_str);
                        }
                        else {
                          publish_event (STS_ESP32, SS_ESP32, EVENT_ERROR, "JSON command to ESP32 not understood");
                        }
                        break;
    case TC_ESP32CAM:   // forward command
                        publish_tc (TC_ESP32CAM, obj["cmd"], json_str);
                        break;
    #endif
    #ifdef PLATFORM_ESP32CAM
    case STS_ESP32:     // {\"ctr\":%u,\"type\":\"%s\",\"ss\":\"%s\",\"msg\":\"%s\"}
                        sts_esp32.packet_ctr = obj["ctr"];
                        sts_esp32.type = id_of ((const char*)obj["type"], sizeof(eventName[0]), (char*)&eventName, sizeof(eventName));
                        sts_esp32.subsystem = id_of ((const char*)obj["ss"], sizeof(subsystemName[0]), (char*)&subsystemName, sizeof(subsystemName));
                        strcpy (sts_esp32.message, obj["msg"]);
                        publish_packet (STS_ESP32);
                        break;
    case TM_ESP32:      // {\"ctr\":%u,\"opsmode\":\"%s\",\"err\":%u,\"warn\":%u,\"mem_free\":%u,\"press_rate\":%u,\"motion_rate\":%u,\"cam_rate\":%u,\"gps_rate\":%u,\"radio_rate\":%u,\"sep_sts\":%d,\"enabled\":\"%d%d%d%d%d%d%d%d\",\"current\":\"%d%d%d%d%d\",\"conn\":\"%d%d\",\"debug\":\"%d%d%d%d%d%d%d\"}
                        esp32.packet_ctr = obj["ctr"];
                        esp32.opsmode = id_of(obj["opsmode"], sizeof(opsmodeName[0]), (char*)&opsmodeName, sizeof(opsmodeName));
                        esp32.error_ctr = obj["err"];
                        esp32.warning_ctr = obj["warn"];
                        //esp32.tc_exec_ctr
                        //esp32.tc_fail_ctr
                        esp32.mem_free = obj["mem_free"];
                        esp32.fs_free = obj["fs_free"];
                        esp32.pressure_rate = obj["press_rate"];
                        esp32.motion_rate = obj["motion_rate"];
                        esp32.camera_rate = obj["cam_rate"];
                        esp32.gps_rate = obj["gps_rate"];
                        esp32.radio_rate = obj["radio_rate"];
                        esp32.separation_sts = obj["sep_sts"];
                        esp32.radio_enabled = (obj["enabled"][0] == '1')?1:0;     
                        esp32.pressure_enabled = (obj["enabled"][1] == '1')?1:0;     
                        esp32.motion_enabled = (obj["enabled"][2] == '1')?1:0;     
                        esp32.gps_enabled = (obj["enabled"][3] == '1')?1:0;     
                        esp32.camera_enabled = (obj["enabled"][4] == '1')?1:0;     
                        esp32.wifi_enabled = (obj["enabled"][5] == '1')?1:0;     
                        esp32.wifi_udp_enabled = (obj["enabled"][6] == '1')?1:0;      
                        esp32.wifi_yamcs_enabled = (obj["enabled"][7] == '1')?1:0;    
                        esp32.radio_current = (obj["current"][0] == '1')?1:0;     
                        esp32.pressure_current = (obj["current"][1] == '1')?1:0;     
                        esp32.motion_current = (obj["current"][2] == '1')?1:0;     
                        esp32.gps_current = (obj["current"][3] == '1')?1:0;     
                        esp32.camera_current = (obj["current"][4] == '1')?1:0; 
                        esp32.serial_connected = (obj["conn"][5] == '1')?1:0;     
                        esp32.wifi_connected = (obj["conn"][6] == '1')?1:0;        
                        // udp_rate;
                        // yamcs_rate;
                        // serial_in_rate;
                        // serial_out_rate;
                        // yamcs_buffer;
                        // serial_buffer;
                        // radio_debug:1;           // 7
                        // pressure_debug:1;        //  6
                        // motion_debug:1;          //   5
                        // gps_debug:1;             //    4
                        // camera_debug:1;          //     3  
                        // wifi_debug:1;            //      2
                        // esp32_debug:1;           //        0
                        publish_packet (TM_ESP32);
                        break;
    case TM_GPS:        // {\"ctr\":%u,\"sts\":\"%s\",\"sats\":%d,\"time\":\"%02d:%02d:%02d\",\"lat\":%.6f,\"lon\":%.6f,\"alt\":%.2f,\"v_north\":%.2f,\"v_east\":%.2f,\"v_down\":%.2f,\"hdop\":%d.%03d,\"vdop\":%d.%03d}
                        neo6mv2.time_valid = false;
                        neo6mv2.location_valid = false;
                        neo6mv2.altitude_valid = false;
                        neo6mv2.speed_valid = false;
                        neo6mv2.hdop_valid = false;
                        neo6mv2.vdop_valid = false;
                        neo6mv2.packet_ctr = obj["ctr"];
                        neo6mv2.status = id_of(obj["sts"], sizeof(gpsName[0]), (char*)&gpsName, sizeof(gpsName));
                        neo6mv2.satellites = obj["sats"];
                        if (obj.containsKey("time")) {
                          neo6mv2.time_valid = true;
                          neo6mv2.hours = obj["time"].substring(0,1);
                          neo6mv2.minutes = obj["time"].substring(3,4);
                          neo6mv2.seconds = obj["time"].substring(6,7);
                        }
                        if (obj.containsKey("lat")) {
                          neo6mv2.location_valid = true;
                          neo6mv2.latitude = obj["lat"];
                          neo6mv2.longitude = obj["lon"];
                        }
                        if (obj.containsKey("alt")) {
                          neo6mv2.altitude_valid = true;
                          neo6mv2.altitude = obj["alt"];
                        }
                        if (obj.containsKey("v_north")) {
                          neo6mv2.speed_valid = true;
                          neo6mv2.v_north = obj["v_north"];
                          neo6mv2.v_east = obj["v_east"];
                          neo6mv2.v_down = obj["v_down"];
                        }
                        if (obj.containsKey("hdop")) {
                          neo6mv2.hdop_valid = true;
                          neo6mv2.milli_hdop = obj["hdop"]*1000;
                        }
                        if (obj.containsKey("vdop")) {
                          neo6mv2.vdop_valid = true;
                          neo6mv2.milli_vdop = obj["vdop"]*1000;
                        }
                        publish_packet (TM_GPS);
                        break;
    case TM_MOTION:     // {\"ctr\":%u,\"accel\":[\"X\":%.2f,\"Y\":%.2f,\"Z\":%.2f],\"gyro\":[\"X\":%.2f,\"Y\":%.2f,\"Z\":%.2f],\"tilt\":%.2f,\"g\":%.2f,\"a\":%.2f,\"rpm\":%.2f}
                        mpu6050.packet_ctr = obj["ctr"];
                        mpu6050.accel_x_rocket = obj["accel"]["X"];
                        mpu6050.accel_y_rocket = obj["accel"]["Y"];
                        mpu6050.accel_z_rocket = obj["accel"]["Z"];
                        mpu6050.gyro_x_rocket = obj["gyro"]["X"];
                        mpu6050.gyro_y_rocket = obj["gyro"]["Y"];
                        mpu6050.gyro_z_rocket = obj["gyro"]["Z"];
                        mpu6050.tilt = obj["tilt"];
                        mpu6050.g = obj["g"];
                        mpu6050.a = obj["a"];
                        mpu6050.rpm = obj["rpm"];
                        publish_packet (TM_MOTION);
                        break;
    case TM_PRESSURE:   // {\"ctr\":%u,\"p\":%.2f,\"p0\":%.2f,\"T\":%.2f,\"h\":%.2f,\"v_v\":%.2f}
                        bmp280.packet_ctr = obj["ctr"];
                        bmp280.pressure = obj["p"];
                        bmp280.zero_level_pressure = obj["p0"];
                        bmp280.altitude = obj["h"];
                        bmp280.velocity_v = obj["v_v"];
                        bmp280.temperature = obj["T"];
                        publish_packet (TM_PRESSURE);
                        break;
    case TM_RADIO:      // {\"ctr\":%u,\"data\":\"%s\"} 
                        radio.packet_ctr = obj["ctr"];
                        publish_packet (TM_RADIO);
                        break;
    case TC_ESP32:      // forward command  
                        publish_tc (TC_ESP32, obj["cmd"], json_str);
                        break;
    case TC_ESP32CAM:   // execute command
                        if (!strcmp(obj["cmd"], "reboot")) {
                          cmd_reboot ();
                        }
                        else if (!strcmp(obj["cmd"], "reboot_fli3d")) {
                          cmd_reboot_fli3d ();
                        }
                        else if (!strcmp(obj["cmd"], "config_reset")) {
                          cmd_config_reset ();
                        }
                        else if (!strcmp(obj["cmd"], "config_load")) {
                          cmd_config_load ();
                        }
                        else if (!strcmp(obj["cmd"], "config_save")) {
                          cmd_config_save ();
                        }
                        else if (!strcmp(obj["cmd"], "get_packet")) {
                          cmd_get_packet (json_str);
                        }
                        else if (!strcmp(obj["cmd"], "set_cammode")) {
                          cmd_set_cammode (json_str);
                        }
                        else if (!strcmp(obj["cmd"], "set_parameter")) {
                          cmd_set_parameter (json_str);
                        }
                        else if (!strcmp(obj["cmd"], "set_routing")) {
                          cmd_set_routing (json_str);
                        }
                        else {
                          publish_event (STS_ESP32CAM, SS_ESP32CAM, EVENT_ERROR, "JSON command to ESP32CAM not understood");
                        }
                        break;
    #endif
    default:            sprintf (buffer, "Ignored packet with PID %d received over serial", id_of (tag_str, sizeof(pidName[0]), (char*)pidName, sizeof(pidName)));
                        publish_event (STS_THIS, SS_THIS, EVENT_WARNING, buffer); 
                        break;
  }
}

void serial_parse_ccsds (uint16_t data_len) {
  // decode ccsds_hdr (populate structure and interpret)
  memcpy (&ccsds_hdr, &serial_buffer, 6);
  #ifdef PLATFORM_ESP32
  switch (ccsds_hdr.apid_L - 42) {
    case -42:            break;
    case STS_ESP32CAM:   memcpy (&sts_esp32cam, &serial_buffer + 6, sizeof(sts_esp32cam_t));
                         publish_packet (STS_ESP32CAM);
                         break;
    case TM_ESP32CAM:    memcpy (&esp32cam, &serial_buffer + 6, sizeof(tm_esp32cam_t));
                         publish_packet (TM_ESP32CAM);
                         break;
    case TM_CAMERA:      memcpy (&ov2640, &serial_buffer + 6, sizeof(tm_camera_t));
                         publish_packet (TM_CAMERA);
                         break;
    case TC_ESP32:       memcpy (&tc_esp32, &serial_buffer + 6, sizeof(tc_esp32_t));
                         switch (tc_esp32.cmd) {
                           case TC_REBOOT:             cmd_reboot ();
                                                       break;
                           case TC_REBOOT_FLI3D:       cmd_reboot_fli3d ();
                                                       break;                                           
                           case TC_GET_PACKET:         cmd_get_packet (tc_this->json);
                                                       break;                                                       
                           case TC_SET_OPSMODE:        cmd_set_opsmode (tc_this->json);
                                                       break;
                           case TC_SET_PARAMETER:      cmd_set_parameter (tc_this->json); 
                                                       break;
                           case TC_SET_ROUTING:        cmd_set_routing (tc_this->json); 
                                                       break;
                           default:                    publish_event (STS_ESP32, SS_ESP32, EVENT_CMD_FAIL, "CCSDS command to ESP32 not understood");
                                                       break;
                         }
                         break;
    default:             // should not receive this CCSDS packet over serial
                         sprintf (buffer, "Received unknown packet with PID %d over serial", ccsds_hdr.apid_L - 42);
                         publish_event (STS_THIS, SS_THIS, EVENT_ERROR, buffer);
                         break;
  }
  #endif
  #ifdef PLATFORM_ESP32CAM
  switch (ccsds_hdr.apid_L - 42) {
    case -42:            break;
    case STS_ESP32:      memcpy (&sts_esp32, &serial_buffer + 6, sizeof(sts_esp32_t));
                         publish_packet (STS_ESP32);
                         break;
    case TM_ESP32:       memcpy (&esp32, &serial_buffer + 6, sizeof(tm_esp32_t));
                         publish_packet (TM_ESP32);
                         break;
    case TM_GPS:         memcpy (&neo6mv2, &serial_buffer + 6, sizeof(tm_gps_t));
                         publish_packet (TM_GPS);
                         break;
    case TM_MOTION:      memcpy (&mpu6050, &serial_buffer + 6, sizeof(tm_motion_t));
                         publish_packet (TM_MOTION);
                         break;
    case TM_PRESSURE:    memcpy (&bmp280, &serial_buffer + 6, sizeof(tm_pressure_t));
                         publish_packet (TM_PRESSURE);
                         break;
    case TM_RADIO:       memcpy (&radio, &serial_buffer + 6, sizeof(tm_radio_t));
                         publish_packet (TM_RADIO);
                         break;
    case TM_TIMER:       memcpy (&timer, &serial_buffer + 6, sizeof(tm_timer_t));
                         publish_packet (TM_TIMER);
                         break;
    case TC_ESP32CAM:    memcpy (&tc_esp32cam, &serial_buffer + 6, sizeof(tc_esp32cam_t));
                         switch (tc_esp32cam.cmd) {
                           case TC_REBOOT:             cmd_reboot ();
                                                       break;
                           case TC_REBOOT_FLI3D:       cmd_reboot_fli3d ();
                                                       break;                                               
                           case TC_GET_PACKET:         cmd_get_packet (tc_this->json);
                                                       break;                                                       
                           case TC_SET_CAMMODE:        cmd_set_cammode (tc_this->json);
                                                       break;
                           case TC_SET_PARAMETER:      cmd_set_parameter (tc_this->json); 
                                                       break;
                           case TC_SET_ROUTING:        cmd_set_routing (tc_this->json); 
                                                       break;
                           default:                    publish_event (STS_ESP32CAM, SS_ESP32CAM, EVENT_CMD_FAIL, "CCSDS command to ESP32CAM not understood");
                                                       break;
                         }
                         break;
    default:             // should not receive this CCSDS packet over serial
                         sprintf (buffer, "Received unknown packet with PID %d over serial", ccsds_hdr.apid_L - 42);
                         publish_event (STS_THIS, SS_THIS, EVENT_ERROR, buffer);
                         break;
  }
  #endif
}

// COMMAND FUNCTIONALITY

bool cmd_reboot () {
  sprintf (buffer, "Rebooting %s subsystem", subsystemName[SS_THIS]);
  publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
  delay (1000);
  ESP.restart();   
}

bool cmd_reboot_fli3d () {
  sprintf (buffer, "Triggering reboot of %s subsystem", subsystemName[SS_OTHER]);
  publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
  publish_tc (TC_OTHER, TC_REBOOT, "{}");
  cmd_reboot ();
}

bool cmd_get_packet (char* json_str) {
  // json: {"pkt":"tm_gps"}
  StaticJsonDocument<JSON_MAX_SIZE> obj;
  deserializeJson(obj, json_str);
  if (obj.containsKey("pkt")) {
    uint8_t PID = id_of (obj["pkt"], sizeof(pidName[0]), (char*)pidName, sizeof(pidName));
    if (PID != 255) {
      sprintf (buffer, "Sending packet %s (PID %d) on request", pidName[PID], PID);
      publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
      publish_packet (PID);
      return true;
    }
    else {
      sprintf (buffer, "Packet %s unknown", obj["pkt"].as<char*>());
      publish_event (STS_THIS, SS_THIS, EVENT_CMD_FAIL, buffer);
      return false;          
    }
  }
  else {
    publish_event (STS_THIS, SS_THIS, EVENT_CMD_FAIL, "Command needs 'pkt' as string parameter");
    return false;    
  }
}

bool cmd_set_opsmode (char* json_str) {
  // json: {"opsmode":"ready"}
  StaticJsonDocument<JSON_MAX_SIZE> obj;
  deserializeJson(obj, json_str);
  if (obj.containsKey("opsmode")) {
    uint8_t opsmode = id_of (obj["opsmode"], sizeof(modeName[0]), (char*)modeName, sizeof(modeName));
    if (opsmode != 255) {
      sprintf (buffer, "Setting opsmode for ESP32 subsystem to '%s'", modeName[opsmode]);
      publish_event (STS_ESP32, SS_ESP32, EVENT_CMD_RESP, buffer);
      esp32.opsmode = opsmode;
      return true;
    }
    else {
      sprintf (buffer, "Opsmode '%s' unknown", obj["opsmode"].as<char*>());
      publish_event (STS_THIS, SS_THIS, EVENT_CMD_FAIL, buffer);
      return false;          
    }
  }
  else {
    publish_event (STS_THIS, SS_THIS, EVENT_CMD_FAIL, "Command needs 'opsmode' as string parameter");
    return false;    
  }  
}

bool cmd_set_parameter (char* json_str) {
  // json: {"wifi_ssid":"my_wifi","wifi_password":"my_password"}
  bool parameter_set = false;
  StaticJsonDocument<JSON_MAX_SIZE> obj;
  deserializeJson(obj, json_str);
  #ifdef PLATFORM_ESP32
  if (obj.containsKey("radio_rate")) { 
    config_esp32.radio_rate = obj["radio_rate"];
    var_timer.radio_interval = (1000 / config_esp32.radio_rate);
    sprintf (buffer, "Setting radio_rate to %d Hz", config_esp32.radio_rate);
    publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
    parameter_set = true;
  } 
  if (obj.containsKey("pressure_rate")) { 
    config_esp32.pressure_rate = obj["pressure_rate"];
    var_timer.pressure_interval = (1000 / config_esp32.pressure_rate);
    sprintf (buffer, "Setting pressure_rate to %d Hz", config_esp32.pressure_rate);
    publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
    parameter_set = true;
  } 
  if (obj.containsKey("motion_rate")) { 
    config_esp32.motion_rate = obj["motion_rate"];
    var_timer.motion_interval = (1000 / config_esp32.motion_rate);
    sprintf (buffer, "Setting motion_rate to %d Hz", config_esp32.motion_rate);
    publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
    parameter_set = true;
  } 
  if (obj.containsKey("gps_rate")) { 
    config_esp32.gps_rate = obj["gps_rate"];
    var_timer.gps_interval = (1000 / config_esp32.gps_rate);
    sprintf (buffer, "Setting gps_rate to %d Hz (save and reboot needed)", config_esp32.gps_rate);
    publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
    parameter_set = true;
  } 
  if (obj.containsKey("radio_enable")) { 
    config_esp32.radio_enable = obj["radio_enable"];
    publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, config_esp32.radio_enable?"Enabling radio transmitter (save and reboot needed)":"Disabling radio transmitter (save and reboot needed)");      
    parameter_set = true;
  } 
  if (obj.containsKey("pressure_enable")) { 
    config_esp32.pressure_enable = obj["pressure_enable"];
    publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, config_esp32.pressure_enable?"Enabling pressure sensor (save and reboot needed)":"Disabling pressure sensor (save and reboot needed)");      
    parameter_set = true;
  } 
  if (obj.containsKey("motion_enable")) { 
    config_esp32.motion_enable = obj["motion_enable"];
    publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, config_esp32.motion_enable?"Enabling accelerometer/gyroscope sensor (save and reboot needed)":"Disabling accelerometer/gyroscope sensor (save and reboot needed)");      
    parameter_set = true;
  } 
  if (obj.containsKey("gps_enable")) { 
    config_esp32.gps_enable = obj["gps_enable"];
    publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, config_esp32.gps_enable?"Enabling gps receiver (save and reboot needed)":"Disabling gps receiver (save and reboot needed)");      
    parameter_set = true;
  } 
  if (obj.containsKey("camera_enable")) { 
    config_esp32.gps_enable = obj["camera_enable"];
    publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, config_esp32.camera_enable?"Enabling camera (save and reboot needed)":"Disabling camera (save and reboot needed)");      
    parameter_set = true;
  } 
  if (obj.containsKey("ota_enable")) { 
    config_esp32.ota_enable = obj["ota_enable"];
    if (config_esp32.ota_enable) {
      ota_setup ();
    }
    publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, config_esp32.ota_enable?"Enabling OTA firmware update":"Disabling OTA firmware update");      
    parameter_set = true;
  }
  #endif
  #ifdef PLATFORM_ESP32CAM
  if (obj.containsKey("sd_enable")) { 
    config_esp32cam.sd_enable = obj["sd_enable"];
    publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, eeprom.esp32cam.sd_enable?"Enabling use of SD card (save and reboot needed)":"Disabling use of SD card (save and reboot needed)");      
    parameter_set = true;
  }
  if (obj.containsKey("sd_json_enable")) { 
    config_esp32cam.sd_json_enable = obj["sd_json_enable"];
    publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, eeprom.esp32cam.sd_json_enable?"Enabling logging to SD card in JSON format (save and reboot needed)":"Disabling logging to SD card in JSON (save and reboot needed)");      
    parameter_set = true;
  } 
  if (obj.containsKey("sd_ccsds_enable")) { 
    config_esp32cam.sd_ccsds_enable = obj["sd_ccsds_enable"];
    publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, eeprom.esp32cam.sd_ccsds_enable?"Enabling logging to SD card in CCSDS format (save and reboot needed)":"Disabling logging to SD card in CCSDS (save and reboot needed)");      
    parameter_set = true;
  } 
  if (obj.containsKey("sd_image_enable")) { 
    config_esp32cam.sd_image_enable = obj["sd_image_enable"];
    publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, eeprom.esp32cam.sd_image_enable?"Enabling storage of images to SD card (save and reboot needed)":"Disabling storage of images to SD card (save and reboot needed)");      
    parameter_set = true;
  }       
  #endif
  if (obj.containsKey("wifi_enable")) { 
    config_this->wifi_enable = obj["wifi_enable"];
    publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, config_this->wifi_enable?"Enabling wifi":"Disabling wifi");      
    parameter_set = true;
  } 
  if (obj.containsKey("wifi_udp_enable")) { 
    config_this->wifi_udp_enable = obj["wifi_udp_enable"];
    publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, config_this->wifi_udp_enable?"Enabling udp transmission over wifi":"Disabling udp transmission over wifi");      
    parameter_set = true;
  } 
  if (obj.containsKey("wifi_yamcs_enable")) { 
    config_this->wifi_yamcs_enable = obj["wifi_yamcs_enable"];
    publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, config_this->wifi_yamcs_enable?"Enabling connection to yamcs over wifi":"Disabling connection to yamcs over wifi");      
    parameter_set = true;
  }                   
  if (obj.containsKey("wifi_sta_enable")) { 
    config_this->wifi_sta_enable = obj["wifi_sta_enable"];
    publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, config_this->wifi_sta_enable?"Enabling wifi in STA mode (save and reboot needed)":"Disabling wifi in STA mode (save and reboot needed)");      
    parameter_set = true;
  }
  if (obj.containsKey("wifi_ap_enable")) { 
    config_this->wifi_ap_enable = obj["wifi_ap_enable"];
    publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, config_this->wifi_ap_enable?"Enabling wifi in AP mode (save and reboot needed)":"Disabling wifi in AP mode (save and reboot needed)");      
    parameter_set = true;
  }
  if (obj.containsKey("serial_ccsds")) { 
    config_this->serial_format = ENC_CCSDS;
    publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, "Setting serial transmission encoding to CCSDS");      
    parameter_set = true;
  }
  if (obj.containsKey("serial_json")) { 
    config_this->serial_format = ENC_JSON;
    publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, "Setting serial transmission encoding to JSON");      
    parameter_set = true;
  }
  if (obj.containsKey("debug_over_serial")) { 
    config_this->debug_over_serial = obj["debug_over_serial"];
    tm_this->serial_connected = true;
    publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, config_this->debug_over_serial?"Enabling debug over serial mode":"Disabling debug over serial mode");      
    parameter_set = true;
  }
  if (!parameter_set) {
    publish_event (STS_THIS, SS_THIS, EVENT_CMD_FAIL, "Command needs valid parameter/value json pairs");
    return false; 
  }
}

bool cmd_set_routing (char* json_str) {
  // json: {"table":"serial","load":"default","tm_gps":1,"tc_esp32":0}
 /*  static byte* table_ptr;
  uint8_t PID;
  bool table_updated = false;
  StaticJsonDocument<JSON_MAX_SIZE> obj;
  deserializeJson(obj, json_str);
  JsonObject entry = obj.as<JsonObject>();
  if (obj.containsKey("table")) {
    uint8_t table = id_of (obj["table"], sizeof(commLineName[0]), (char*)commLineName, sizeof(commLineName));
    switch (table) {
      case COMM_SERIAL:       if (obj.containsKey("load")) {
                                if (!strcmp(obj["load"], "default")) { routing_serial = (char*)&routing_serial_default; table_updated = true; }
                                if (!strcmp(obj["load"], "all")) { routing_serial = (char*)&routing_all; table_updated = true; }
                                if (!strcmp(obj["load"], "none")) { routing_serial = (char*)&routing_none; table_updated = true; }
                                if (!strcmp(obj["load"], "debug")) { routing_serial = (char*)&routing_serial_debug; table_updated = true; }
                              }
                              table_ptr = (byte*)routing_serial;
                              break; 
      case COMM_WIFI_UDP:     if (obj.containsKey("load")) {
                                if (!strcmp(obj["load"], "default")) { routing_udp = (char*)&routing_udp_default; table_updated = true; }
                                if (!strcmp(obj["load"], "all")) { routing_udp = (char*)&routing_all; table_updated = true; }
                                if (!strcmp(obj["load"], "none")) { routing_udp = (char*)&routing_none; table_updated = true; }
                                if (!strcmp(obj["load"], "debug")) { routing_udp = (char*)&routing_udp_debug; table_updated = true; }
                              }
                              table_ptr = (byte*)routing_udp;
                              break;
      case COMM_WIFI_YAMCS:   if (obj.containsKey("load")) {
                                if (!strcmp(obj["load"], "default")) { routing_yamcs = (char*)&routing_yamcs_default; table_updated = true; }
                                if (!strcmp(obj["load"], "all")) { routing_yamcs = (char*)&routing_all; table_updated = true; }
                                if (!strcmp(obj["load"], "none")) { routing_yamcs = (char*)&routing_none; table_updated = true; }
                                if (!strcmp(obj["load"], "debug")) { routing_yamcs = (char*)&routing_yamcs_debug; table_updated = true; }
                              }
                              table_ptr = (byte*)routing_yamcs;
                              break;
      case COMM_FS:           if (obj.containsKey("load")) {
                                if (!strcmp(obj["load"], "default")) { routing_fs = (char*)&routing_fs_default; table_updated = true; }
                                if (!strcmp(obj["load"], "all")) { routing_fs = (char*)&routing_all; table_updated = true; }
                                if (!strcmp(obj["load"], "none")) { routing_fs = (char*)&routing_none; table_updated = true; }
                                if (!strcmp(obj["load"], "debug")) { routing_fs = (char*)&routing_fs_debug; table_updated = true; }
                              }
                              table_ptr = (byte*)routing_fs;
                              break;                   
      #ifdef PLATFORM_ESP32CAM
      case COMM_SD_CCSDS:     if (obj.containsKey("load")) {
                                if (!strcmp(obj["load"], "default")) { routing_sd_ccsds = (char*)&routing_sd_ccsds_default; table_updated = true; }
                                if (!strcmp(obj["load"], "all")) { routing_sd_ccsds = (char*)&routing_all; table_updated = true; }
                                if (!strcmp(obj["load"], "none")) { routing_sd_ccsds = (char*)&routing_none; table_updated = true; }
                                if (!strcmp(obj["load"], "debug")) { routing_sd_ccsds = (char*)&routing_sd_ccsds_debug; table_updated = true; }
                              }
                              table_ptr = (byte*)routing_sd_ccsds;
                              break;
      case COMM_SD_JSON:      if (obj.containsKey("load")) {
                                if (!strcmp(obj["load"], "default")) { routing_sd_json = (char*)&routing_sd_json_default; table_updated = true; }
                                if (!strcmp(obj["load"], "all")) { routing_sd_json = (char*)&routing_all; table_updated = true; }
                                if (!strcmp(obj["load"], "none")) { routing_sd_json = (char*)&routing_none; table_updated = true; }
                                if (!strcmp(obj["load"], "debug")) { routing_sd_json = (char*)&routing_sd_json_debug; table_updated = true; }
                              }
                              table_ptr = (byte*)routing_sd_json;
                              break;
      #endif
      default:                sprintf (buffer, "Table '%s' unknown", obj["table"].as<char*>());
                              publish_event (STS_THIS, SS_THIS, EVENT_CMD_FAIL, buffer);
                              return false;
                              break; 
    }
    if (table_updated) {
      sprintf (buffer, "Loading '%s' settings for routing over %s", obj["load"].as<char*>(), obj["table"].as<char*>());
      publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
    }
    for (JsonPair pair : entry) {      
      PID = id_of (pair.key().c_str(), sizeof(pidName[0]), (char*)pidName, sizeof(pidName));
      if (PID != 255) {
        table_ptr[PID] = pair.value().as<bool>();
        sprintf (buffer, "Setting routing of '%s' in '%s' table to %s", pair.key().c_str(), obj["table"].as<char*>(), pair.value().as<bool>()?"on":"off");
        publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
        table_updated = true;
      }
    } 
    if (!table_updated) {
      sprintf (buffer, "No update to table %s applied; specify 'load' or packet", obj["table"].as<char*>());
      publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
    }  
  }
  else {
    publish_event (STS_THIS, SS_THIS, EVENT_CMD_FAIL, "Command needs 'table' as string parameter");
    return false;    
  } */
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

void Serial_print_charptr (char *ptr, uint8_t len) {
  for (uint8_t i=0; i<len; i++) {
    Serial.print (*(ptr+i), HEX);
    Serial.print (" "); 
  }
  Serial.println ();
}

void build_hex_str(unsigned char * in, size_t insz, char * out, size_t outsz) {
  unsigned char * pin = in;
  const char * hex = "0123456789ABCDEF";
  char * pout = out;
  for(; pin < in+insz; pout +=2, pin++){
    pout[0] = hex[(*pin>>4) & 0xF];
    pout[1] = hex[ *pin     & 0xF];
    if (pout + 2 - out > outsz){
      break;
    }
  }
  pout[-1] = 0;
}

//  build_hex_str ((unsigned char*)ccsds_ptr, 100, (char*)&buffer, sizeof(buffer));
//  Serial.println ("DEBUG: start of ccsds packet at build_json_str: " + String(buffer));