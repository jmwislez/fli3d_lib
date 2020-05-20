/*
 * Fli3d - Library (wifi, bus, TM, eeprom functionality)
 * version: 2020-04-14 edited on esp32
 */

#include "fli3d.h"
#include "secrets.h"
#ifdef PLATFORM_ESP32CAM
#include "FS.h"
#include "SD_MMC.h"
#include "SPI.h"
#endif

LinkedList<ccsds_t*> linkedlist_serial; 
LinkedList<ccsds_t*> linkedlist_udp; 
LinkedList<ccsds_t*> linkedlist_yamcs; 
LinkedList<ccsds_t*> linkedlist_sd_ccsds;
LinkedList<String>   linkedlist_sd_json;
WiFiUDP wifiUDP;
NTPClient timeClient(wifiUDP, eeprom_network.ntp_server, 0);
RH_ASK radio_tx (RADIO_BAUD, DUMMY_PIN1, RF433_TX_PIN, DUMMY_PIN2);

char buffer[TM_MAX_MSG_SIZE];
char bus_buffer[TM_MAX_MSG_SIZE + 25];
char radio_buffer[2*(sizeof(tm_radio_t)+2)];
char serial_buffer[TM_MAX_MSG_SIZE];

#ifdef PLATFORM_ESP32
extern void ota_setup ();
#endif 
#ifdef PLATFORM_ESP32CAM
extern char sd_dir[20];
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
debug_esp32_t      debug_esp32;
debug_esp32cam_t   debug_esp32cam;

eeprom_network_t   eeprom_network;
eeprom_esp32_t     eeprom_esp32;
eeprom_esp32cam_t  eeprom_esp32cam;

#ifdef PLATFORM_ESP32
tm_esp32_t         *tm_this = &esp32;
tc_esp32_t         *tc_this = &tc_esp32;
tc_esp32cam_t      *tc_other = &tc_esp32cam;
sts_esp32_t        *sts_this = &sts_esp32;
debug_esp32_t      *debug_this = &debug_esp32;
eeprom_esp32_t     *eeprom_this = &eeprom_esp32;
#endif
#ifdef PLATFORM_ESP32CAM
tm_esp32cam_t      *tm_this = &esp32cam;
tc_esp32cam_t      *tc_this = &tc_esp32cam;
tc_esp32_t         *tc_other = &tc_esp32;
sts_esp32cam_t     *sts_this = &sts_esp32cam;
debug_esp32cam_t   *debug_this = &debug_esp32cam;
eeprom_esp32cam_t  *eeprom_this = &eeprom_esp32cam;
#endif

const char pidName[NUMBER_OF_PID][16] =   { "sts_esp32", "sts_esp32cam", "tm_esp32", "tm_esp32cam", "tm_camera", "tm_gps", "tm_motion", "tm_pressure", "tm_radio", "tm_timer", "tc_esp32", "tc_esp32cam", "debug_esp32", "debug_esp32cam", "eeprom_network", "eeprom_esp32", "eeprom_esp32cam" };
const char eventName[8][9] =              { "init", "info", "warning", "error", "debug", "cmd", "cmd_resp", "cmd_fail" };
const char subsystemName[15][14] =        { "esp32", "esp32cam", "ov2640", "neo6mv2", "mpu6050", "bmp280", "radio", "sd", "separation", "timer", "esp32_wifi", "esp32cam_wifi", "fli3d", "ground", "any" };
const char modeName[7][10] =              { "init", "checkout", "ready", "thrust", "freefall", "parachute", "static" };
const char cameraModeName[4][7] =         { "init", "idle", "single", "stream" };
const char cameraResolutionName[11][10] = { "160x120", "invalid1", "invalid2", "240x176", "320x240", "400x300", "640x480", "800x600", "1024x768", "1280x1024", "1600x1200" };
const char dataEncodingName[3][8] =       { "CCSDS", "JSON", "ASCII" };
const char commLineName[7][11] =          { "serial", "wifi_udp", "wifi_yamcs", "sd_ccsds", "sd_json", "sd_cam", "radio" };
const char tcName[9][20] =                { "reboot", "reboot_fli3d", "eeprom_reset", "eeprom_load", "eeprom_save", "get_packet", "set_opsmode", "set_parameter", "set_routing" };
const char gpsStatusName[5][10] =         { "none", "est", "time_only", "std", "dgps" }; 

//                                              STS_ESP32 STS_ESP32CAM TM_ESP32 TM_ESP32CAM TM_CAMERA TM_GPS TM_MOTION TM_PRESSURE TM_RADIO TM_TIMER TC_ESP32 TC_ESP32CAM DEBUG_ESP32 DEBUG_ESP32CAM EEPROM_NETWORK EEPROM_ESP32 EEPROM_ESP32CAM

char routing_all[NUMBER_OF_PID] =              {   1,        1,          1,       1,          1,        1,     1,        1,          1,       1,       1,       1,             1,          1,              1,             1,           1        };
char routing_none[NUMBER_OF_PID] =             {   0,        0,          0,       0,          0,        0,     0,        0,          0,       0,       0,       0,             0,          0,              0,             0,           0        };
#ifdef PLATFORM_ESP32
char routing_serial_default[NUMBER_OF_PID] =   {   1,        0,          1,       0,          0,        1,     1,        1,          1,       0,       0,       1,             1,          0,              1,             1,           0        };
char routing_yamcs_default[NUMBER_OF_PID] =    {   1,        1,          1,       1,          1,        1,     1,        1,          1,       0,       0,       0,             0,          0,              0,             0,           0        };
char routing_udp_default[NUMBER_OF_PID] =      {   1,        1,          1,       1,          1,        1,     1,        1,          1,       0,       1,       1,             1,          1,              0,             0,           0        };
char routing_serial_debug[NUMBER_OF_PID] =     {   1,        0,          1,       0,          0,        1,     1,        1,          1,       0,       0,       1,             1,          0,              1,             1,           0        };
char routing_yamcs_debug[NUMBER_OF_PID] =      {   1,        1,          1,       1,          1,        1,     1,        1,          1,       0,       0,       0,             0,          0,              0,             0,           0        };
char routing_udp_debug[NUMBER_OF_PID] =        {   1,        1,          1,       1,          1,        1,     1,        1,          1,       0,       1,       1,             1,          1,              1,             1,           1        };
#endif
#ifdef PLATFORM_ESP32CAM
char routing_serial_default[NUMBER_OF_PID] =   {   0,        1,          0,       1,          1,        0,     0,        0,          0,       0,       1,       0,             0,          1,              1,             0,           1        };
char routing_yamcs_default[NUMBER_OF_PID] =    {   1,        1,          1,       1,          1,        1,     1,        1,          1,       0,       0,       0,             0,          0,              0,             0            0        };
char routing_udp_default[NUMBER_OF_PID] =      {   1,        1,          1,       1,          1,        1,     1,        1,          1,       0,       1,       1,             1,          1,              0,             0,           0        };
char routing_sd_json_default[NUMBER_OF_PID] =  {   1,        1,          1,       1,          1,        1,     1,        1,          1,       0,       1,       1,             1,          1,              0,             0,           0        };
char routing_sd_ccsds_default[NUMBER_OF_PID] = {   1,        1,          1,       1,          1,        1,     1,        1,          1,       0,       1,       1,             1,          1,              0,             0,           0        };
char routing_serial_debug[NUMBER_OF_PID] =     {   0,        1,          0,       1,          1,        0,     0,        0,          0,       0,       1,       0,             0,          1,              1,             0,           1        };
char routing_yamcs_debug[NUMBER_OF_PID] =      {   1,        1,          1,       1,          1,        1,     1,        1,          1,       0,       0,       0,             0,          0,              0,             0            0        };
char routing_udp_debug[NUMBER_OF_PID] =        {   1,        1,          1,       1,          1,        1,     1,        1,          1,       0,       1,       1,             1,          1,              0,             0,           0        };
char routing_sd_json_debug[NUMBER_OF_PID] =    {   1,        1,          1,       1,          1,        1,     1,        1,          1,       0,       1,       1,             1,          1,              0,             0,           0        };
char routing_sd_ccsds_debug[NUMBER_OF_PID] =   {   1,        1,          1,       1,          1,        1,     1,        1,          1,       0,       1,       1,             1,          1,              0,             0,           0        };
char* routing_sd_json = (char*)&routing_sd_json_default;
char* routing_sd_ccsds = (char*)&routing_sd_ccsds_default;
#endif
char* routing_serial = (char*)&routing_serial_default;
char* routing_udp = (char*)&routing_udp_default;
char* routing_yamcs = (char*)&routing_yamcs_default;

// EEPROM FUNCTIONALITY

void eeprom_setup () {
  EEPROM.begin(sizeof(eeprom_network) + sizeof(*eeprom_this));
  // initially enable all channels, so that buffering is properly performed; will be soon overwritten by settings on EEPROM 
  eeprom_this->wifi_enable = true;
  eeprom_this->wifi_yamcs_enable = true;
  eeprom_this->wifi_udp_enable = true;
  eeprom_this->udp_format = ENC_JSON;
  eeprom_this->serial_format = ENC_JSON;
  #ifdef PLATFORM_ESP32CAM
  eeprom_this->sd_enable = true;
  eeprom_this->sd_json_enable = true;
  eeprom_this->sd_ccsds_enable = true;
  #endif
}

void eeprom_load (uint8_t eeprom_id, uint16_t offset) {
  byte* struct_ptr;
  char* ptr;
  uint8_t struct_sizeof;
  switch (eeprom_id) {
    case EEPROM_NETWORK:  struct_ptr = (byte*)&eeprom_network;
                          struct_sizeof = sizeof(eeprom_network_t);
                          break;
    case EEPROM_ESP32:    struct_ptr = (byte*)&eeprom_esp32;
                          struct_sizeof = sizeof(eeprom_esp32_t);
                          break;
    case EEPROM_ESP32CAM: struct_ptr = (byte*)&eeprom_esp32cam;
                          struct_sizeof = sizeof(eeprom_esp32cam_t);
                          break;
  }
  uint8_t magic = (uint8_t)*struct_ptr;
  uint8_t version = (uint8_t)*(struct_ptr+1);
  uint8_t struct_size = (uint8_t)*(struct_ptr+2); 
  
  if (EEPROM.read(offset + 0) == magic) {
    if (EEPROM.read(offset + 1) == version) {
      if (EEPROM.read(offset + 2) == struct_size and struct_size == struct_sizeof) {
        for (uint8_t pos = 3; pos < struct_sizeof; pos++) {
          ptr = (char*)(struct_ptr + pos);
          *ptr = EEPROM.read (offset + pos);
        }
        sprintf (buffer, "Restored %s settings from EEPROM", pidName[eeprom_id]);
        bus_publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);
      }
      else {
        sprintf (buffer, "Size error in data structure for %s settings (size:[mem:%02x,struct:%02x,eeprom:%02x])", pidName[eeprom_id], struct_sizeof, struct_size, EEPROM.read(offset + 2));
        bus_publish_event (STS_THIS, SS_THIS, EVENT_ERROR, buffer);
        eeprom_reset (eeprom_id, offset);
      }
    }
    else {
      sprintf (buffer, "Old data structure version detected for %s settings on EEPROM (version:[struct:%02x,eeprom:%02x])", pidName[eeprom_id], version, EEPROM.read(offset + 1));
      bus_publish_event (STS_THIS, SS_THIS, EVENT_WARNING, buffer);
      eeprom_reset (eeprom_id, offset);
    }
  }
  else {
    sprintf (buffer, "EEPROM at offset %d does not contain %s settings (magic:[struct:%02x,eeprom:%02x])", offset, pidName[eeprom_id], magic, EEPROM.read(offset + 0));
    bus_publish_event (STS_THIS, SS_THIS, EVENT_WARNING, buffer);
    eeprom_reset (eeprom_id, offset);
  }
}

void eeprom_save (uint8_t eeprom_id, uint16_t offset) {
  byte* data_struct;
  uint16_t data_size;
  switch (eeprom_id) {
    case EEPROM_NETWORK:  //EEPROM.put (offset, eeprom_network);
                          data_struct = (byte*)&eeprom_network;
                          data_size = sizeof (eeprom_network);
                          break;
    case EEPROM_ESP32:    //EEPROM.put (offset, eeprom_esp32);
                          data_struct = (byte*)&eeprom_esp32;
                          data_size = sizeof (eeprom_esp32);
                          break;
    case EEPROM_ESP32CAM: //EEPROM.put (offset, eeprom_esp32cam);
                          data_struct = (byte*)&eeprom_esp32cam;
                          data_size = sizeof (eeprom_esp32cam);
                          break;
  }
  for (uint16_t i=0; i < data_size; i++) {
    EEPROM.write (i+offset, *(data_struct+i)); 
  }
  EEPROM.commit();
  sprintf (buffer, "Saved %s settings to EEPROM at offset %d", pidName[eeprom_id], offset);
  bus_publish_event (STS_THIS, SS_THIS, EVENT_INFO, buffer);
}

void eeprom_reset (uint8_t eeprom_id, uint16_t offset) {
  sprintf (buffer, "Resetting %s EEPROM to default values", pidName[eeprom_id]);
  bus_publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);
  switch (eeprom_id) {
    case EEPROM_NETWORK:  strcpy (eeprom_network.wifi_ssid, default_wifi_ssid); 
                          strcpy (eeprom_network.wifi_password, default_wifi_password);
                          strcpy (eeprom_network.ap_ssid, default_ap_ssid); 
                          strcpy (eeprom_network.ap_password, default_ap_password); 
                          strcpy (eeprom_network.udp_server, default_udp_server);
                          strcpy (eeprom_network.yamcs_server, default_yamcs_server); 
                          strcpy (eeprom_network.ntp_server, default_ntp_server);
                          eeprom_network.udp_port = default_udp_port; 
                          eeprom_network.yamcs_tm_port = default_yamcs_tm_port; 
                          eeprom_network.yamcs_tc_port = default_yamcs_tc_port; 
                          break;
    case EEPROM_ESP32:    eeprom_esp32.radio_rate = 2;
                          eeprom_esp32.pressure_rate = 10;
                          eeprom_esp32.motion_rate = 10;
                          eeprom_esp32.gps_rate = 10;
                          eeprom_esp32.radio_enable = true;
                          eeprom_esp32.pressure_enable = true;
                          eeprom_esp32.motion_enable = true;
                          eeprom_esp32.gps_enable = true;
                          eeprom_esp32.camera_enable = true;
                          eeprom_esp32.wifi_enable = true;
                          eeprom_esp32.wifi_udp_enable = true;
                          eeprom_esp32.wifi_yamcs_enable = true;
                          eeprom_esp32.radio_debug = false; 
                          eeprom_esp32.pressure_debug = false; 
                          eeprom_esp32.motion_debug = false; 
                          eeprom_esp32.gps_debug = false;
                          eeprom_esp32.camera_debug = false;  
                          eeprom_esp32.wifi_debug = false;
                          eeprom_esp32.timer_debug = true;
                          eeprom_esp32.esp32_debug = true;
                          eeprom_esp32.serial_format = ENC_JSON; // TODO: put to ENC_CCSDS after debug phase
                          eeprom_esp32.udp_format = ENC_JSON;
                          eeprom_esp32.wifi_sta_enable = true;
                          eeprom_esp32.wifi_ap_enable = true;
                          eeprom_esp32.debug_over_serial = false;
                          break;
    case EEPROM_ESP32CAM: eeprom_esp32cam.sd_enable = true;
    	                    eeprom_esp32cam.sd_json_enable = true;
                          eeprom_esp32cam.sd_ccsds_enable = true;
                          eeprom_esp32cam.sd_image_enable = true;
                          eeprom_esp32cam.wifi_enable = true;
                          eeprom_esp32cam.wifi_udp_enable = true;
                          eeprom_esp32cam.wifi_yamcs_enable = true;
                          eeprom_esp32cam.wifi_image_enable = true; 
                          eeprom_esp32cam.serial_format = ENC_JSON; // TODO: put to ENC_CCSDS after debug phase
                          eeprom_esp32cam.udp_format = ENC_JSON;
                          eeprom_esp32cam.esp32cam_debug = true;
                          eeprom_esp32cam.sd_debug = false;
                          eeprom_esp32cam.wifi_debug = false;
                          eeprom_esp32cam.camera_debug = false;
                          eeprom_esp32cam.timer_debug = false;
                          eeprom_esp32cam.wifi_ap_enable = true;
                          eeprom_esp32cam.wifi_sta_enable = true;
                          eeprom_esp32cam.debug_over_serial = false;
                          break;
  } 
  eeprom_save (eeprom_id, offset); 
}

void eeprom_print () {
  for (uint16_t i=0; i < sizeof(eeprom_network) + sizeof(*eeprom_this); i++) {
    if (i == sizeof(eeprom_network)) {
      Serial.println ();
    }
    Serial.print ("[");
    Serial.print (i,HEX);
    Serial.print ("|");
    Serial.print (EEPROM.read(i),HEX);
    //Serial.print ("|");
    //Serial.print ((char)EEPROM.read(i));
    Serial.print ("]");
  }
  Serial.println ();
}

bool wifi_setup () {
  WiFi.mode(WIFI_AP_STA); 
  if (eeprom_this->wifi_ap_enable) {
    WiFi.softAP(eeprom_network.ap_ssid, eeprom_network.ap_password);
    sprintf (buffer, "Started WiFi access point %s with IP %s", eeprom_network.ap_ssid, WiFi.softAPIP().toString().c_str());
    bus_publish_event (STS_THIS, SS_THIS_WIFI, EVENT_INIT, buffer);
  }
  if (eeprom_this->wifi_sta_enable) {
    WiFi.begin(eeprom_network.wifi_ssid, eeprom_network.wifi_password);  
    uint8_t i = 0;
    while (i++ < WIFI_TIMEOUT and WiFi.status() != WL_CONNECTED) {
      delay (500);
    } 
    if (i >= WIFI_TIMEOUT) {
      sprintf (buffer, "Failed to connect %s to %s WiFi", subsystemName[SS_THIS], eeprom_network.wifi_ssid);
      bus_publish_event (STS_THIS, SS_THIS_WIFI, EVENT_ERROR, buffer);
      return false;
    }
    else {
      sprintf (buffer, "%s WiFi connected to %s with IP %s", subsystemName[SS_THIS], eeprom_network.wifi_ssid, WiFi.localIP().toString().c_str());
      bus_publish_event (STS_THIS, SS_THIS_WIFI, EVENT_INIT, buffer);
      if (eeprom_this->wifi_udp_enable) {
        tm_this->wifi_udp_enabled = true;
      }
      if (eeprom_this->wifi_yamcs_enable) {
        tm_this->wifi_yamcs_enabled = true;
      }
      tm_this->wifi_connected = true;
      timeClient.begin();
      uint8_t j = 0;
      while (j++ < NTP_TIMEOUT and !timeClient.update()) {
        delay (500);
      }
      if (j >= NTP_TIMEOUT) {      
        sprintf (buffer, "Failed to connect %s to NTP server %s; continuing in background", subsystemName[SS_THIS], eeprom_network.ntp_server);
        bus_publish_event (STS_THIS, SS_THIS_WIFI, EVENT_WARNING, buffer);      
      }
      else {
        sprintf (buffer, "Time on %s set via NTP server %s: %s", subsystemName[SS_THIS], eeprom_network.ntp_server, timeClient.getFormattedDate().c_str());
        bus_publish_event (STS_THIS, SS_THIS_WIFI, EVENT_INIT, buffer);
      }
      return true;
    }
  }
}

// TM/TC FUNCTIONALITY (serial/udp/yamcs)

void bus_publish_pkt (uint8_t PID) {
  static uint16_t ccsds_len, packet_len;
  static bool packet_type;
  static byte* packet_ptr;
  switch (PID) {
    case STS_ESP32:      packet_len = sts_esp32.message_size + 6;
                         packet_ptr = (byte*)&sts_esp32;
                         packet_type = PKT_TM;
                         break;
    case STS_ESP32CAM:   packet_len = sts_esp32cam.message_size + 6;
                         packet_ptr = (byte*)&sts_esp32cam;
                         packet_type = PKT_TM;
                         break;
    case TM_ESP32:       packet_len = sizeof(tm_esp32_t);
                         packet_ptr = (byte*)&esp32;
                         packet_type = PKT_TM;
                         break;
    case TM_ESP32CAM:    packet_len = sizeof(tm_esp32cam_t);
                         packet_ptr = (byte*)&esp32cam;
                         packet_type = PKT_TM;
                         break;                  
    case TM_CAMERA:      packet_len = sizeof(tm_camera_t) - 26 + strlen (ov2640.filename) + 1;
                         packet_ptr = (byte*)&ov2640;
                         packet_type = PKT_TM;
                         break;    
    case TM_GPS:         packet_len = sizeof(tm_gps_t);
                         packet_ptr = (byte*)&neo6mv2;
                         packet_type = PKT_TM;
                         break;              
    case TM_MOTION:      packet_len = sizeof(tm_motion_t);
                         packet_ptr = (byte*)&mpu6050;
                         packet_type = PKT_TM;
                         break;
    case TM_PRESSURE:    packet_len = sizeof(tm_pressure_t);
                         packet_ptr = (byte*)&bmp280;
                         packet_type = PKT_TM;
                         break;
    case TM_RADIO:       packet_len = sizeof(tm_radio_t);
                         packet_ptr = (byte*)&radio;
                         packet_type = PKT_TM;
                         break;   
    case TM_TIMER:       packet_len = sizeof(tm_timer_t);
                         packet_ptr = (byte*)&timer;
                         packet_type = PKT_TM;
                         break;
    case TC_ESP32:       packet_len = tc_esp32.json_size + 1;
                         packet_ptr = (byte*)&tc_esp32;
                         packet_type = PKT_TC;
                         break;
    case TC_ESP32CAM:    packet_len = tc_esp32cam.json_size + 1;
                         packet_ptr = (byte*)&tc_esp32cam;
                         packet_type = PKT_TC;
                         break;                         
    case DEBUG_ESP32:    packet_len = debug_esp32.message_size + 5;
                         packet_ptr = (byte*)&debug_esp32;
                         packet_type = PKT_TM;
                         break;
    case DEBUG_ESP32CAM: packet_len = debug_esp32cam.message_size + 5;
                         packet_ptr = (byte*)&debug_esp32cam;
                         packet_type = PKT_TM;
                         break; 
    case EEPROM_NETWORK: packet_len = sizeof(eeprom_network_t);
                         packet_ptr = (byte*)&eeprom_network;
                         packet_type = PKT_TM;
                         break;
    case EEPROM_ESP32:   packet_len = sizeof(eeprom_esp32_t);
                         packet_ptr = (byte*)&eeprom_esp32;
                         packet_type = PKT_TM;
                         break;  
    case EEPROM_ESP32CAM:packet_len = sizeof(eeprom_esp32cam_t);
                         packet_ptr = (byte*)&eeprom_esp32cam;
                         packet_type = PKT_TM;
                         break;                                                                                                                              
  }
  update_pkt (PID); 
  buildJsonString (PID); // fills buffer
  sprintf (bus_buffer, "[%d] %s %s", millis(), pidName[PID], buffer);
  ccsds_len = sizeof(ccsds_hdr_t) + packet_len;
  update_ccsds_hdr (PID, packet_type, packet_len);
  memcpy (&ccsds_buffer.ccsds_hdr, &ccsds_hdr, sizeof (ccsds_hdr_t));
  memcpy (&ccsds_buffer.blob, packet_ptr, packet_len);
  // SERIAL
  if (routing_serial[PID]) {
    switch (eeprom_this->serial_format) {
      case ENC_JSON:  serial_publish ((char*)&bus_buffer, strlen (bus_buffer)); break;
      case ENC_CCSDS: serial_publish ((char*)&ccsds_buffer, ccsds_len); break;
    }
  }
  // UDP
  if (routing_udp[PID] and eeprom_this->wifi_enable and eeprom_this->wifi_udp_enable) {
    switch (eeprom_this->udp_format) {
      case ENC_JSON:  udp_publish ((char*)&bus_buffer, strlen (bus_buffer)); break;
      case ENC_CCSDS: udp_publish ((char*)&ccsds_buffer, ccsds_len); break;
    }
  }
  // Yamcs
  if (routing_yamcs[PID] and eeprom_this->wifi_enable and eeprom_this->wifi_yamcs_enable) {
    yamcs_publish ((char*)&ccsds_buffer, ccsds_len);
  }
  #ifdef PLATFORM_ESP32
  if (tm_this->radio_enabled and PID == TM_RADIO) {
    radio_publish ();
  }
  #endif
  #ifdef PLATFORM_ESP32CAM
  if (routing_sd_json[PID] and eeprom_this->sd_enable and eeprom_this->sd_json_enable) {
    sd_json_publish (bus_buffer);
  }
  if (routing_sd_ccsds[PID] and eeprom_this->sd_enable and eeprom_this->sd_ccsds_enable) {
    sd_ccsds_publish ((char*)&ccsds_buffer, ccsds_len);
  }
  #endif
  reset_pkt (PID);
}

void bus_publish_event (uint8_t PID, uint8_t subsystem, uint8_t event_type, const char* event_message) {
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
    case DEBUG_ESP32:    strcpy (debug_esp32.message, event_message);
                         debug_esp32.message_size = strlen (event_message) + 1;
                         break;
    case DEBUG_ESP32CAM: strcpy (debug_esp32cam.message, event_message);
                         debug_esp32cam.message_size = strlen (event_message) + 1;
                         break;
  }
  bus_publish_pkt (PID);
}

void bus_publish_tc (uint8_t PID, uint8_t cmd, char* json_str) {
  bus_publish_event (PID, cmd, EVENT_CMD, json_str);
}

bool serial_publish (char* packet_ptr, uint16_t packet_len) {
  static ccsds_t* linkedlist_serial_entry;
  uint8_t batch_count = 0;
  linkedlist_serial_entry = (ccsds_t*)malloc(sizeof(ccsds_t));
  if (linkedlist_serial_entry == NULL or (tm_this->mem_free < MIN_MEM_FREE and linkedlist_serial.size() > 10)) { 
    tm_this->serial_connected = true; 
    tm_this->err_serial_dataloss = true;
  }
  else {
    memcpy (linkedlist_serial_entry->blob, packet_ptr, packet_len);
    linkedlist_serial_entry->blob_size = packet_len;
    linkedlist_serial.add(linkedlist_serial_entry);
  }
  if (tm_this->serial_connected) {
    while (linkedlist_serial.size() and batch_count++ < BUFFER_RELEASE_BATCH_SIZE) {
      linkedlist_serial_entry = linkedlist_serial.remove(0);
      Serial.write ((const uint8_t*)linkedlist_serial_entry->blob, linkedlist_serial_entry->blob_size);
      Serial.println ();
      free (linkedlist_serial_entry);
      tm_this->serial_out_rate++;
    } 
  }
  tm_this->serial_buffer = linkedlist_serial.size();
  return true;  
}

bool udp_publish (char* packet_ptr, uint16_t packet_len) {
  static ccsds_t* linkedlist_udp_entry;
  uint8_t batch_count = 0;
  linkedlist_udp_entry = (ccsds_t*)malloc(sizeof(ccsds_t));
  if (linkedlist_udp_entry == NULL or (tm_this->mem_free < MIN_MEM_FREE and linkedlist_udp.size() > 10)) { 
    tm_this->wifi_connected = true; 
    tm_this->err_udp_dataloss = true;
  }
  else {
    memcpy (linkedlist_udp_entry->blob, packet_ptr, packet_len);
    linkedlist_udp_entry->blob_size = packet_len;
    linkedlist_udp.add(linkedlist_udp_entry);
  }
  if (tm_this->wifi_connected) {
    while (linkedlist_udp.size() and batch_count++ < BUFFER_RELEASE_BATCH_SIZE) {
      linkedlist_udp_entry = linkedlist_udp.remove(0);
      wifiUDP.beginPacket(eeprom_network.udp_server, eeprom_network.udp_port);
      wifiUDP.write ((const uint8_t*)linkedlist_udp_entry->blob, linkedlist_udp_entry->blob_size);
      wifiUDP.println ();
      wifiUDP.endPacket();
      free (linkedlist_udp_entry);
      tm_this->udp_rate++;
    } 
  }
  tm_this->udp_buffer = linkedlist_udp.size(); 
  return true;
}
  
bool yamcs_publish (char* packet_ptr, uint16_t packet_len) { 
  static ccsds_t* linkedlist_yamcs_entry;
  uint8_t batch_count = 0;
  linkedlist_yamcs_entry = (ccsds_t*)malloc(sizeof(ccsds_t));
  if (linkedlist_yamcs_entry == NULL or (tm_this->mem_free < MIN_MEM_FREE and linkedlist_yamcs.size() > 10)) { 
    tm_this->wifi_connected = true; 
    tm_this->err_yamcs_dataloss = true;
  }
  else {
    memcpy (linkedlist_yamcs_entry->blob, packet_ptr, packet_len);
    linkedlist_yamcs_entry->blob_size = packet_len;
    linkedlist_yamcs.add(linkedlist_yamcs_entry);
  }
  if (tm_this->wifi_connected) {
    while (linkedlist_yamcs.size() and batch_count++ < BUFFER_RELEASE_BATCH_SIZE) {
      linkedlist_yamcs_entry = linkedlist_yamcs.remove(0);
      wifiUDP.beginPacket(eeprom_network.yamcs_server, eeprom_network.yamcs_tm_port);
      wifiUDP.write ((const uint8_t*)&linkedlist_yamcs_entry->blob, linkedlist_yamcs_entry->blob_size);
      wifiUDP.endPacket();
      free (linkedlist_yamcs_entry);
      tm_this->yamcs_rate++;
    } 
  }
  tm_this->yamcs_buffer = linkedlist_yamcs.size();
  return true;  
}

#ifdef PLATFORM_ESP32CAM
bool sd_json_publish (char* message) {
  char path[30];
  uint8_t batch_count = 0;
  if (!linkedlist_sd_json.add(String(message)) or (tm_this->mem_free < MIN_MEM_FREE and linkedlist_sd_json.size() > 10)) {
    esp32cam.sd_enabled = true; 
    tm_this->err_sd_dataloss = true;
  }
  if (esp32cam.sd_enabled) {
    sprintf (path, "%s/json.log", sd_dir);
    File logfile_json = SD_MMC.open(path, FILE_APPEND);
    if (!logfile_json) {
      bus_publish_event (STS_ESP32CAM, SS_SD, EVENT_ERROR, "Failed to open JSON logfile in append mode");
      esp32cam.sd_json_enabled = false;
      return false;
    }
    else {
      esp32cam.sd_json_enabled = true;
      esp32cam.sd_current = true;
      while (linkedlist_sd_json.size() and batch_count++ < BUFFER_RELEASE_BATCH_SIZE) {
        logfile_json.println (linkedlist_sd_json.remove(0));
        esp32cam.sd_json_rate++;
      }
      logfile_json.close();
    }
  }
  tm_this->sd_ccsds_buffer = linkedlist_sd_json.size();
  return true;  
}

bool sd_ccsds_publish (char* packet_ptr, uint16_t packet_len) {
  char path[30];
  static ccsds_t* linkedlist_sd_ccsds_entry;
  uint8_t batch_count = 0;
  linkedlist_sd_ccsds_entry = (ccsds_t*)malloc(sizeof(ccsds_t));
  if (linkedlist_sd_ccsds_entry == NULL or (tm_this->mem_free < MIN_MEM_FREE and linkedlist_sd_ccsds.size() > 10)) { 
    tm_this->sd_enabled = true;
    tm_this->err_sd_dataloss = true;
  }
  else {
    memcpy (linkedlist_sd_ccsds_entry->blob, packet_ptr, packet_len);
    linkedlist_sd_ccsds_entry->blob_size = packet_len;
    linkedlist_sd_ccsds.add(linkedlist_sd_ccsds_entry);
  }
  if (esp32cam.sd_enabled) {
    sprintf (path, "%s/ccsds.log", sd_dir);
    File logfile_ccsds = SD_MMC.open(path, FILE_APPEND);
    if (!logfile_ccsds) {
      bus_publish_event (STS_ESP32CAM, SS_SD, EVENT_ERROR, "Failed to open CCSDS logfile in append mode");
      esp32cam.sd_ccsds_enabled = false;
      return false;
    }
    else {
      esp32cam.sd_ccsds_enabled = true;
      esp32cam.sd_current = true;
      while (linkedlist_sd_ccsds.size() and batch_count++ < BUFFER_RELEASE_BATCH_SIZE) {
        linkedlist_sd_ccsds_entry = linkedlist_sd_ccsds.remove(0);
        logfile_ccsds.write ((const uint8_t*)linkedlist_sd_ccsds_entry->blob, linkedlist_sd_ccsds_entry->blob_size);
        logfile_ccsds.println ();
        free (linkedlist_sd_ccsds_entry);
        esp32cam.sd_ccsds_rate++;
      }
      logfile_ccsds.close();
    }
  }
  tm_this->sd_ccsds_buffer = linkedlist_sd_ccsds.size();
  return true;  
}

uint32_t sd_free () {
  return (SD_MMC.totalBytes() - SD_MMC.usedBytes());
}
#endif

void update_ccsds_hdr (uint16_t PID, bool pkt_type, uint16_t pkt_len) {
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
/*  if (PID == 2) {
    Serial.print (pkt_len);
    Serial.print (":");
    Serial.print (ccsds_hdr.pkt_len_H, HEX);
    Serial.print (":");
    Serial.println (ccsds_hdr.pkt_len_L, HEX);
  } */
} 

void buildJsonString (uint8_t PID) {
  char * p = buffer;
  switch (PID) {
    case STS_ESP32:      sprintf (buffer, "{\"ctr\":%u,\"type\":\"%s\",\"ss\":\"%s\",\"msg\":\"%s\"}", sts_esp32.packet_ctr, eventName[sts_esp32.type], subsystemName[sts_esp32.subsystem], sts_esp32.message);
                         break;
    case STS_ESP32CAM:   sprintf (buffer, "{\"ctr\":%u,\"type\":\"%s\",\"ss\":\"%s\",\"msg\":\"%s\"}", sts_esp32cam.packet_ctr, eventName[sts_esp32cam.type], subsystemName[sts_esp32cam.subsystem], sts_esp32cam.message);
                         break;                  
    case TM_ESP32:       sprintf (buffer, "{\"ctr\":%u,\"opsmode\":\"%s\",\"err\":%u,\"warn\":%u,\"mem_free\":%u,\"press_rate\":%u,\"motion_rate\":%u,\"cam_rate\":%u,\"gps_rate\":%u,\"radio_rate\":%u,\"sep_sts\":%d,\"enabled\":\"%d%d%d%d%d%d%d%d\",\"current\":\"%d%d%d%d%d\",\"conn\":\"%d%d\"}", 
                                  esp32.packet_ctr, modeName[esp32.opsmode], esp32.error_ctr, esp32.warning_ctr, esp32.mem_free, esp32.pressure_rate, esp32.motion_rate, esp32.camera_rate, esp32.gps_rate, esp32.radio_rate, esp32.separation_sts,
                                  esp32.radio_enabled, esp32.pressure_enabled, esp32.motion_enabled, esp32.gps_enabled, esp32.camera_enabled, esp32.wifi_enabled, esp32.wifi_udp_enabled, esp32.wifi_yamcs_enabled, 
                                  esp32.radio_current, esp32.pressure_current, esp32.motion_current, esp32.gps_current, esp32.camera_current,
                                  esp32.serial_connected, esp32.wifi_connected); 
                         break;
    case TM_ESP32CAM:    sprintf (buffer, "{\"ctr\":%u,\"err\":%u,\"warn\":%u,\"mem_free\":%u,\"sd_free\":%u,\"img_rate\":%u,\"enabled\":\"%d%d%d%d%d%d%d%d%d\",\"current\":\"%d%d\",\"connected\":\"%d%d\"}",
                                  esp32cam.packet_ctr, esp32cam.error_ctr, esp32cam.warning_ctr, esp32cam.mem_free, esp32cam.sd_free, esp32cam.camera_image_rate, 
                                  esp32cam.wifi_enabled, esp32cam.wifi_udp_enabled, esp32cam.wifi_yamcs_enabled, esp32cam.wifi_image_enabled, esp32cam.camera_enabled, esp32cam.sd_enabled, esp32cam.sd_image_enabled, esp32cam.sd_json_enabled, esp32cam.sd_ccsds_enabled,  
                                  esp32cam.sd_current, esp32cam.camera_current,
                                  esp32cam.serial_connected, esp32cam.wifi_connected);
                         break;
    case TM_CAMERA:      sprintf (buffer, "{\"ctr\":%u,\"mode\":\"%s\",\"res\":\"%s\",\"auto_res\":%d,\"file\":\"%s\",\"size\":%u,\"exp\":%u,\"sd\":%u,\"wifi\":%u}", 
                                  ov2640.packet_ctr, cameraModeName[ov2640.camera_mode], cameraResolutionName[ov2640.resolution], ov2640.auto_res, ov2640.filename, ov2640.filesize, ov2640.exposure_ms, ov2640.sd_ms, ov2640.wifi_ms);
                         break;
    case TM_GPS:         *p++ = '{';
                         p += sprintf (p, "\"ctr\":%u,\"sts\":\"%s\",\"sats\":%d", neo6mv2.packet_ctr, gpsStatusName[neo6mv2.status], neo6mv2.satellites);
                         if (neo6mv2.time_valid) {
                           p += sprintf (p, ",\"time\":\"%02d:%02d:%02d\"", neo6mv2.hours, neo6mv2.minutes, neo6mv2.seconds);
                         }
                         if (neo6mv2.location_valid) {
                           p += sprintf (p, ",\"lat\":%.6f,\"lon\":%.6f", neo6mv2.latitude, neo6mv2.longitude);
                         }
                         if (neo6mv2.altitude_valid) {
                           p += sprintf (p, ",\"alt\":%.2f", neo6mv2.altitude);
                         }
                         if (neo6mv2.speed_valid) {
                           p += sprintf (p, ",\"v_north\":%.2f,\"v_east\":%.2f,\"v_down\":%.2f", neo6mv2.v_north, neo6mv2.v_east, neo6mv2.v_down);
                         }
                         if (neo6mv2.hdop_valid) {
                           p += sprintf (p, ",\"hdop\":%d.%03d", neo6mv2.milli_hdop/1000, neo6mv2.milli_hdop%1000);
                         }
                         if (neo6mv2.hdop_valid) {
                           p += sprintf (p, ",\"vdop\":%d.%03d", neo6mv2.milli_vdop/1000, neo6mv2.milli_vdop%1000);
                         }
                         *p++ = '}';
                         *p++ = 0;
                         break;
    case TM_MOTION:      sprintf (buffer, "{\"ctr\":%u,\"accel\":[\"X\":%.2f,\"Y\":%.2f,\"Z\":%.2f],\"gyro\":[\"X\":%.2f,\"Y\":%.2f,\"Z\":%.2f],\"tilt\":%.2f,\"g\":%.2f,\"a\":%.2f,\"rpm\":%.2f}", 
                                  mpu6050.packet_ctr, mpu6050.accel_x_rocket, mpu6050.accel_y_rocket, mpu6050.accel_z_rocket, mpu6050.gyro_x_rocket, mpu6050.gyro_y_rocket, mpu6050.gyro_z_rocket, mpu6050.tilt, mpu6050.g, mpu6050.a, mpu6050.rpm); 
                         break;
    case TM_PRESSURE:    sprintf (buffer, "{\"ctr\":%u,\"p\":%.2f,\"p0\":%.2f,\"T\":%.2f,\"h\":%.2f,\"v_v\":%.2f}", bmp280.packet_ctr, bmp280.pressure, bmp280.zero_level_pressure, bmp280.temperature, bmp280.altitude, bmp280.velocity_v);
                         break;
    case TM_RADIO:       tohex ((unsigned char*)&radio, sizeof(tm_radio_t), (char*)&radio_buffer, 2*sizeof(tm_radio_t)+2); 
                         sprintf (buffer, "{\"ctr\":%u,\"data\":\"%s\"}", radio.packet_ctr, radio_buffer);
                         break;
    case TM_TIMER:       sprintf (buffer, "{\"ctr\":%u,\"loop_ms\":%u,\"radio\":%d,\"radio_ms\":%u,\"pressure\":%d,\"pressure_ms\":%u,\"motion\":%d,\"motion_ms\":%u,\"gps\":%d,\"gps_ms\":%u,\"esp32cam_ms\":%u}", 
                                  timer.packet_ctr, timer.loop_duration, timer.do_radio, timer.radio_duration, timer.do_pressure, timer.pressure_duration, timer.do_motion, timer.motion_duration, timer.do_gps, timer.gps_duration, timer.esp32cam_duration);
                         break;
    case TC_ESP32:       if (tc_esp32.json_size) {
                           sprintf (buffer, "{\"cmd\":\"%s\",%s", tcName[tc_esp32.cmd], (char*)(&tc_esp32.json+1));
                         }
                         else {
                           sprintf (buffer, "{\"cmd\":\"%s\"}", tcName[tc_esp32.cmd]); 
                         }
                         break;
    case TC_ESP32CAM:    if (tc_esp32cam.json_size) {
                           sprintf (buffer, "{\"cmd\":\"%s\",%s", tcName[tc_esp32cam.cmd], (char*)(&tc_esp32cam.json+1));
                         }
                         else {
                           sprintf (buffer, "{\"cmd\":\"%s\"}", tcName[tc_esp32cam.cmd]); 
                         }
                         break;                    
    case DEBUG_ESP32:    sprintf (buffer, "{\"ctr\":%u,\"msg\":\"%s\"}", debug_esp32.packet_ctr, debug_esp32.message);
                         break;
    case DEBUG_ESP32CAM: sprintf (buffer, "{\"ctr\":%u,\"msg\":\"%s\"}", debug_esp32cam.packet_ctr, debug_esp32cam.message);
                         break;    
  }
}

void update_pkt (uint8_t PID) {
  switch (PID) {
    #ifdef PLATFORM_ESP32
    case STS_ESP32:      sts_esp32.millis = millis();
                         sts_esp32.packet_ctr++;
                         break;                
    case TM_ESP32:       esp32.millis = millis();
                         esp32.packet_ctr++; 
                         esp32.mem_free = ESP.getFreeHeap();
                         if (esp32.serial_connected) {
                           esp32.serial_out_rate++;
                         }
                         if (esp32.wifi_connected and esp32.wifi_udp_enabled) {
                           esp32.udp_rate++;
                         }
                         if (esp32.wifi_connected and esp32.wifi_yamcs_enabled) {
                           esp32.yamcs_rate++;
                         }
                         break;             
    case TM_GPS:         neo6mv2.packet_ctr++;
                         break;
    case TM_MOTION:      mpu6050.packet_ctr++;
                         break;
    case TM_PRESSURE:    bmp280.packet_ctr++;
                         break;
    case TM_RADIO:       radio.millis = millis ();
                         radio.packet_ctr++;
                         radio.opsmode = esp32.opsmode;
                         radio.error_ctr = esp32.error_ctr + esp32cam.error_ctr;
                         radio.warning_ctr = esp32.warning_ctr + esp32cam.warning_ctr;
                         radio.esp32_udp_buffer = esp32.udp_buffer;
                         radio.esp32_yamcs_buffer = esp32.yamcs_buffer;
                         radio.esp32_serial_buffer = esp32.serial_buffer;
                         radio.esp32cam_udp_buffer = esp32cam.udp_buffer;
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
    case TM_TIMER:       timer.packet_ctr++;
                         break;
    case TC_ESP32CAM:    // do nothing
                         break;
    case DEBUG_ESP32:    debug_esp32.millis = millis ();
                         debug_esp32.packet_ctr++;
                         break;
    #endif
    #ifdef PLATFORM_ESP32CAM
    case STS_ESP32CAM:   sts_esp32cam.millis = millis();
                         sts_esp32cam.packet_ctr++; 
                         break;
    case TM_ESP32CAM:    esp32cam.millis = millis();
                         esp32cam.packet_ctr++; 
                         esp32cam.mem_free = ESP.getFreeHeap();
                         esp32cam.sd_free = sd_free();
                         if (esp32cam.serial_connected) {
                           esp32cam.serial_out_rate++;
                         }
                         if (esp32cam.wifi_connected and esp32cam.wifi_udp_enabled) {
                           esp32cam.udp_rate++;
                         }
                         if (esp32cam.wifi_connected and esp32cam.wifi_yamcs_enabled) {
                           esp32cam.yamcs_rate++;
                         }
                         break;
    case TM_CAMERA:      ov2640.packet_ctr++; 
                         break;        
    case TC_ESP32:       // do nothing
                         break;
    case DEBUG_ESP32CAM: debug_esp32cam.millis = millis ();
                         debug_esp32cam.packet_ctr++;
                         break;
    #endif
  }
}

void reset_pkt (uint8_t PID) {
  switch (PID) {
    #ifdef PLATFORM_ESP32
    case STS_ESP32:      sts_esp32.message_size = 0;
                         break;
    case TM_ESP32:       esp32.radio_rate = 0;
                         esp32.pressure_rate = 0;
                         esp32.motion_rate = 0;
                         esp32.gps_rate = 0;
                         esp32.camera_rate = 0;
                         esp32.timer_rate = 0;
                         esp32.udp_rate = 0;
                         esp32.yamcs_rate = 0;
                         esp32.serial_out_rate = 0;    
                         esp32.serial_in_rate = 0;  
                         esp32.warn_serial_connloss = false;
                         esp32.warn_wifi_connloss = false;
                         esp32.err_serial_dataloss = false;
                         esp32.err_udp_dataloss = false;
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
    case TM_TIMER:       break;
    case TC_ESP32CAM:    tc_esp32cam.json_size = 0;
                         break;
    case DEBUG_ESP32:    debug_esp32.message_size = 0;
                         break;
    #endif
    #ifdef PLATFORM_ESP32CAM
    case STS_ESP32CAM:   sts_esp32cam.message_size = 0;
                         break;
    case TM_ESP32CAM:    esp32cam.timer_rate = 0;
                         esp32cam.udp_rate = 0;
                         esp32cam.yamcs_rate = 0;
                         esp32cam.serial_in_rate = 0;
                         esp32cam.serial_out_rate = 0;
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
    case DEBUG_ESP32CAM: debug_esp32cam.message_size = 0;
                         break;
    #endif
  }
}

// RADIO FUNCTIONALITY

bool radio_setup () {
  delay(1000);
  if (radio_tx.init()) {
    bus_publish_event (STS_ESP32, SS_RADIO, EVENT_INIT, "Radio transmitter initialized");
    return true;
  }
  else {
    bus_publish_event (STS_ESP32, SS_RADIO, EVENT_ERROR, "Failed to initialize radio transmitter");
    return false; 
  }
}

void radio_publish () {
  if (esp32.radio_enabled) {
    radio_tx.send((uint8_t *)&radio, sizeof(tm_radio_t));
  }
}

uint16_t serial_check () {
  // gets characters from Serial, puts them in serial_buffer, and returns length of string that is ready for processing or false if string is not complete
  static uint8_t serial_status;
  static uint16_t data_len;
  static uint8_t pkt_apid;
  static uint16_t serial_buffer_pos;
  while(Serial.available()) {
    timer.last_serial_in_millis = millis();
    char c = Serial.read();
    serial_buffer[serial_buffer_pos++] = c;
    if (serial_status == SERIAL_COMPLETE) {
      // new transmission started: find out what we're getting ...
      switch (c) {
        case 0x00: // start of CCSDS transmission
                   Serial.println ("Start of CCSDS transmission");
                   serial_status = SERIAL_CCSDS;
                   data_len = TM_MAX_MSG_SIZE;
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
        case 5:         data_len = min ((uint16_t)(serial_buffer[4] << 8) + (uint16_t)(serial_buffer[5]) + 6, TM_MAX_MSG_SIZE); 
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
                                  if (!eeprom_this->debug_over_serial and tm_this->serial_connected) {
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
                      bus_publish_event (STS_THIS, SS_OTHER, EVENT_WARNING, buffer);                    
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
    if (serial_buffer_pos == TM_MAX_MSG_SIZE) { // something is wrong: line too long // TODO: overall assessment of maximal buffer sizes
      serial_buffer[TM_MAX_MSG_SIZE-1] = '\0';
      serial_buffer_pos = 0;
      Serial_print_charptr ((char*)&serial_buffer, TM_MAX_MSG_SIZE);
      if (!eeprom_this->debug_over_serial) {
        tm_this->serial_connected = false;
      }
      sprintf (buffer, "Received too long message from %s", subsystemName[SS_OTHER]);
      bus_publish_event (STS_THIS, SS_OTHER, EVENT_ERROR, buffer);
      serial_status = SERIAL_UNKNOWN;
      return false; // invalid serial_buffer content, do not use
    }
  } // while Serial.available
  return false; // no more serial characters waiting
}

void announce_established_serial_connection () {
  tm_this->serial_connected = true;
  sprintf (buffer, "Established serial connection to %s", subsystemName[SS_OTHER]);
  bus_publish_event (STS_THIS, SS_OTHER, EVENT_INIT, buffer);
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
    #ifdef PLATFORM_ESP32
    if (eeprom_esp32cam.esp32cam_debug) {
      bus_publish_event (DEBUG_ESP32CAM, SS_ESP32CAM, EVENT_DEBUG, serial_buffer);
    }
    #endif
    #ifdef PLATFORM_ESP32CAM
    if (eeprom_esp32.esp32_debug) {
      bus_publish_event (DEBUG_ESP32, SS_ESP32, EVENT_DEBUG, serial_buffer);
    }
    #endif
  }
}

void serial_parse_json () { // TODO: is full coverage needed / useful?  What about millis?
  static StaticJsonDocument<TM_MAX_MSG_SIZE> obj;
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
                        bus_publish_pkt (STS_ESP32CAM);
                        break;                        
    case TM_ESP32CAM:   // {\"ctr\":%u,\"err\":%u,\"warn\":%u,\"mem_free\":%u,\"sd_free\":%u,\"img_rate\":%u,\"enabled\":\"%d%d%d%d%d%d%d%d%d\",\"current\":\"%d%d\",\"connected\":\"%d%d\",\"debug\":\"%d%d%d%d\"}
                        esp32cam.packet_ctr = obj["ctr"];
                        esp32cam.error_ctr = obj["err"];
                        esp32cam.warning_ctr = obj["warn"];
                        //esp32cam.tc_exec_ctr
                        //esp32cam.tc_fail_ctr
                        esp32cam.mem_free = obj["mem_free"];
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
                        //esp32cam.timer_rate;
                        //esp32cam.udp_rate;
                        //esp32cam.yamcs_rate;
                        //esp32cam.serial_in_rate;
                        //esp32cam.serial_out_rate;
                        //esp32cam.udp_buffer;
                        //esp32cam.yamcs_buffer;
                        //esp32cam.serial_buffer;
                        bus_publish_pkt (TM_ESP32CAM);
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
                        bus_publish_pkt (TM_CAMERA);
                        break;
    case TC_ESP32:      // execute command
                        if (!strcmp(obj["cmd"], "reboot")) {
                          cmd_reboot ();
                        }
                        else if (!strcmp(obj["cmd"], "reboot_fli3d")) {
                          cmd_reboot_fli3d ();
                        }
                        else if (!strcmp(obj["cmd"], "eeprom_reset")) {
                          cmd_eeprom_reset ();
                        }
                        else if (!strcmp(obj["cmd"], "eeprom_load")) {
                          cmd_eeprom_load ();
                        }
                        else if (!strcmp(obj["cmd"], "eeprom_save")) {
                          cmd_eeprom_save ();
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
                          bus_publish_event (STS_ESP32, SS_ESP32, EVENT_ERROR, "JSON command to ESP32 not understood");
                        }
                        break;
    case TC_ESP32CAM:   // forward command
                        bus_publish_tc (TC_ESP32CAM, obj["cmd"], json_str);
                        break;
    #endif
    #ifdef PLATFORM_ESP32CAM
    case STS_ESP32:     // {\"ctr\":%u,\"type\":\"%s\",\"ss\":\"%s\",\"msg\":\"%s\"}
                        sts_esp32.packet_ctr = obj["ctr"];
                        sts_esp32.type = id_of ((const char*)obj["type"], sizeof(eventName[0]), (char*)&eventName, sizeof(eventName));
                        sts_esp32.subsystem = id_of ((const char*)obj["ss"], sizeof(subsystemName[0]), (char*)&subsystemName, sizeof(subsystemName));
                        strcpy (sts_esp32.message, obj["msg"]);
                        bus_publish_pkt (STS_ESP32);
                        break;
    case TM_ESP32:      // {\"ctr\":%u,\"opsmode\":\"%s\",\"err\":%u,\"warn\":%u,\"mem_free\":%u,\"press_rate\":%u,\"motion_rate\":%u,\"cam_rate\":%u,\"gps_rate\":%u,\"radio_rate\":%u,\"sep_sts\":%d,\"enabled\":\"%d%d%d%d%d%d%d%d\",\"current\":\"%d%d%d%d%d\",\"conn\":\"%d%d\",\"debug\":\"%d%d%d%d%d%d%d\"}
                        esp32.packet_ctr = obj["ctr"];
                        esp32.opsmode = id_of(obj["opsmode"], sizeof(opsmodeName[0]), (char*)&opsmodeName, sizeof(opsmodeName));
                        esp32.error_ctr = obj["err"];
                        esp32.warning_ctr = obj["warn"];
                        //esp32.tc_exec_ctr
                        //esp32.tc_fail_ctr
                        esp32.mem_free = obj["mem_free"];
                        esp32.pressure_rate = obj["press_rate"];
                        esp32.motion_rate = obj["motion_rate"];
                        esp32.camera_rate = obj["cam_rate"];
                        esp32.gps_rate = obj["gps_rate"];
                        esp32.radio_rate = obj["radio_rate"];
                        //esp32.timer_rate
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
                        // udp_buffer;
                        // yamcs_buffer;
                        // serial_buffer;
                        // radio_debug:1;           // 7
                        // pressure_debug:1;        //  6
                        // motion_debug:1;          //   5
                        // gps_debug:1;             //    4
                        // camera_debug:1;          //     3  
                        // wifi_debug:1;            //      2
                        // timer_debug:1;           //       1
                        // esp32_debug:1;           //        0
                        bus_publish_pkt (TM_ESP32);
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
                        bus_publish_pkt (TM_GPS);
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
                        bus_publish_pkt (TM_MOTION);
                        break;
    case TM_PRESSURE:   // {\"ctr\":%u,\"p\":%.2f,\"p0\":%.2f,\"T\":%.2f,\"h\":%.2f,\"v_v\":%.2f}
                        bmp280.packet_ctr = obj["ctr"];
                        bmp280.pressure = obj["p"];
                        bmp280.zero_level_pressure = obj["p0"];
                        bmp280.altitude = obj["h"];
                        bmp280.velocity_v = obj["v_v"];
                        bmp280.temperature = obj["T"];
                        bus_publish_pkt (TM_PRESSURE);
                        break;
    case TM_RADIO:      // {\"ctr\":%u,\"data\":\"%s\"} 
                        radio.packet_ctr = obj["ctr"];
                        // TODO: function to map data string back to packet (if useful)
                        bus_publish_pkt (TM_RADIO);
                        break;
    case TC_ESP32:      // forward command  
                        bus_publish_tc (TC_ESP32, obj["cmd"], json_str);
                        break;
    case TC_ESP32CAM:   // execute command
                        if (!strcmp(obj["cmd"], "reboot")) {
                          cmd_reboot ();
                        }
                        else if (!strcmp(obj["cmd"], "reboot_fli3d")) {
                          cmd_reboot_fli3d ();
                        }
                        else if (!strcmp(obj["cmd"], "eeprom_reset")) {
                          cmd_eeprom_reset ();
                        }
                        else if (!strcmp(obj["cmd"], "eeprom_load")) {
                          cmd_eeprom_load ();
                        }
                        else if (!strcmp(obj["cmd"], "eeprom_save")) {
                          cmd_eeprom_save ();
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
                          bus_publish_event (STS_ESP32CAM, SS_ESP32CAM, EVENT_ERROR, "JSON command to ESP32CAM not understood");
                        }
                        break;
    #endif
    default:            sprintf (buffer, "Ignored packet with PID %d received over serial", id_of (tag_str, sizeof(pidName[0]), (char*)pidName, sizeof(pidName)));
                        bus_publish_event (STS_THIS, SS_THIS, EVENT_WARNING, buffer); 
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
                         bus_publish_pkt (STS_ESP32CAM);
                         break;
    case TM_ESP32CAM:    memcpy (&esp32cam, &serial_buffer + 6, sizeof(tm_esp32cam_t));
                         bus_publish_pkt (TM_ESP32CAM);
                         break;
    case TM_CAMERA:      memcpy (&ov2640, &serial_buffer + 6, sizeof(tm_camera_t));
                         bus_publish_pkt (TM_CAMERA);
                         break;
    case TC_ESP32:       memcpy (&tc_esp32, &serial_buffer + 6, sizeof(tc_esp32_t));
                         switch (tc_esp32.cmd) {
                           case TC_REBOOT:             cmd_reboot ();
                                                       break;
                           case TC_REBOOT_FLI3D:       cmd_reboot_fli3d ();
                                                       break;
                           case TC_EEPROM_RESET:       cmd_eeprom_reset ();
                                                       break;
                           case TC_EEPROM_LOAD:        cmd_eeprom_load ();
                                                       break;
                           case TC_EEPROM_SAVE:        cmd_eeprom_save ();
                                                       break;                                                       
                           case TC_GET_PACKET:         cmd_get_packet (tc_this->json);
                                                       break;                                                       
                           case TC_SET_OPSMODE:        cmd_set_opsmode (tc_this->json);
                                                       break;
                           case TC_SET_PARAMETER:      cmd_set_parameter (tc_this->json); 
                                                       break;
                           case TC_SET_ROUTING:        cmd_set_routing (tc_this->json); 
                                                       break;
                           default:                    bus_publish_event (STS_ESP32, SS_ESP32, EVENT_CMD_FAIL, "CCSDS command to ESP32 not understood");
                                                       break;
                         }
                         break;
    case DEBUG_ESP32CAM: memcpy (&debug_esp32cam, &serial_buffer + 6, sizeof(debug_esp32cam_t));
                         bus_publish_pkt (DEBUG_ESP32CAM);
                         break;
    case EEPROM_NETWORK: memcpy (&eeprom_network, &serial_buffer + 6, sizeof(eeprom_network_t));
                         bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, "Updated EEPROM network settings in memory");
                         break;    
    case EEPROM_ESP32:   memcpy (&eeprom_esp32, &serial_buffer + 6, sizeof(eeprom_esp32_t));
                         bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, "Updated EEPROM esp32 settings in memory");
                         break;    
    default:             // should not receive this CCSDS packet over serial
                         sprintf (buffer, "Received unknown packet with PID %d over serial", ccsds_hdr.apid_L - 42);
                         bus_publish_event (STS_THIS, SS_THIS, EVENT_ERROR, buffer);
                         break;
  }
  #endif
  #ifdef PLATFORM_ESP32CAM
  switch (ccsds_hdr.apid_L - 42) {
    case -42:            break;
    case STS_ESP32:      memcpy (&sts_esp32, &serial_buffer + 6, sizeof(sts_esp32_t));
                         bus_publish_pkt (STS_ESP32);
                         break;
    case TM_ESP32:       memcpy (&esp32, &serial_buffer + 6, sizeof(tm_esp32_t));
                         bus_publish_pkt (TM_ESP32);
                         break;
    case TM_GPS:         memcpy (&neo6mv2, &serial_buffer + 6, sizeof(tm_gps_t));
                         bus_publish_pkt (TM_GPS);
                         break;
    case TM_MOTION:      memcpy (&mpu6050, &serial_buffer + 6, sizeof(tm_motion_t));
                         bus_publish_pkt (TM_MOTION);
                         break;
    case TM_PRESSURE:    memcpy (&bmp280, &serial_buffer + 6, sizeof(tm_pressure_t));
                         bus_publish_pkt (TM_PRESSURE);
                         break;
    case TM_RADIO:       memcpy (&radio, &serial_buffer + 6, sizeof(tm_radio_t));
                         bus_publish_pkt (TM_RADIO);
                         break;
    case TM_TIMER:       memcpy (&timer, &serial_buffer + 6, sizeof(tm_timer_t));
                         bus_publish_pkt (TM_TIMER);
                         break;
    case TC_ESP32CAM:    memcpy (&tc_esp32cam, &serial_buffer + 6, sizeof(tc_esp32cam_t));
                         switch (tc_esp32cam.cmd) {
                           case TC_REBOOT:             cmd_reboot ();
                                                       break;
                           case TC_REBOOT_FLI3D:       cmd_reboot_fli3d ();
                                                       break;
                           case TC_EEPROM_RESET:       cmd_eeprom_reset ();
                                                       break;
                           case TC_EEPROM_LOAD:        cmd_eeprom_load ();
                                                       break;
                           case TC_EEPROM_SAVE:        cmd_eeprom_save ();
                                                       break;                                                       
                           case TC_GET_PACKET:         cmd_get_packet (tc_this->json);
                                                       break;                                                       
                           case TC_SET_CAMMODE:        cmd_set_cammode (tc_this->json);
                                                       break;
                           case TC_SET_PARAMETER:      cmd_set_parameter (tc_this->json); 
                                                       break;
                           case TC_SET_ROUTING:        cmd_set_routing (tc_this->json); 
                                                       break;
                           default:                    bus_publish_event (STS_ESP32CAM, SS_ESP32CAM, EVENT_CMD_FAIL, "CCSDS command to ESP32CAM not understood");
                                                       break;
                         }
                         break;
    case DEBUG_ESP32:    memcpy (&debug_esp32, &serial_buffer + 6, sizeof(debug_esp32_t));
                         bus_publish_pkt (DEBUG_ESP32);
                         break;
    case EEPROM_NETWORK: memcpy (&eeprom_network, &serial_buffer + 6, sizeof(eeprom_network_t));
                         bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, "Updated EEPROM network settings in memory");
                         break;
    case EEPROM_ESP32CAM:memcpy (&eeprom_esp32cam, &serial_buffer + 6, sizeof(eeprom_esp32cam_t));
                         bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, "Updated EEPROM esp32cam settings in memory");
    default:             // should not receive this CCSDS packet over serial
                         sprintf (buffer, "Received unknown packet with PID %d over serial", ccsds_hdr.apid_L - 42);
                         bus_publish_event (STS_THIS, SS_THIS, EVENT_ERROR, buffer);
                         break;
  }
  #endif
}

// COMMAND FUNCTIONALITY

bool cmd_reboot () {
  sprintf (buffer, "Rebooting %s subsystem", subsystemName[SS_THIS]);
  bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
  delay (1000);
  ESP.restart();   
}

bool cmd_reboot_fli3d () {
  sprintf (buffer, "Triggering reboot of %s subsystem", subsystemName[SS_OTHER]);
  bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
  bus_publish_tc (TC_OTHER, TC_REBOOT, "");
  cmd_reboot ();
}

bool cmd_eeprom_reset () {
  bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, "Resetting EEPROM values in memory to defaults");
  eeprom_reset (EEPROM_NETWORK, 0);
  eeprom_reset (EEPROM_THIS, sizeof(eeprom_network));
  return true;
}

bool cmd_eeprom_load () {
  bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, "Loading settings from EEPROM");
  eeprom_load (EEPROM_NETWORK, 0);
  eeprom_load (EEPROM_THIS, sizeof(eeprom_network));
  return true;
}

bool cmd_eeprom_save () {
  bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, "Saving settings to EEPROM");
  eeprom_save (EEPROM_NETWORK, 0);
  eeprom_save (EEPROM_THIS, sizeof(eeprom_network));
  return true;
}

bool cmd_get_packet (char* json_str) {
  // json: {"pkt":"tm_gps"}
  StaticJsonDocument<TM_MAX_MSG_SIZE> obj;
  deserializeJson(obj, json_str);
  if (obj.containsKey("pkt")) {
    uint8_t PID = id_of (obj["pkt"], sizeof(pidName[0]), (char*)pidName, sizeof(pidName));
    if (PID != 255) {
      sprintf (buffer, "Sending packet %s (PID %d) on request", pidName[PID], PID);
      bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
      bus_publish_pkt (PID);
      return true;
    }
    else {
      sprintf (buffer, "Packet %s unknown", obj["pkt"].as<char*>());
      bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_FAIL, buffer);
      return false;          
    }
  }
  else {
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_FAIL, "Command needs 'pkt' as string parameter");
    return false;    
  }
}

bool cmd_set_opsmode (char* json_str) {
  // json: {"opsmode":"ready"}
  StaticJsonDocument<TM_MAX_MSG_SIZE> obj;
  deserializeJson(obj, json_str);
  if (obj.containsKey("opsmode")) {
    uint8_t opsmode = id_of (obj["opsmode"], sizeof(modeName[0]), (char*)modeName, sizeof(modeName));
    if (opsmode != 255) {
      sprintf (buffer, "Setting opsmode for ESP32 subsystem to '%s'", modeName[opsmode]);
      bus_publish_event (STS_ESP32, SS_ESP32, EVENT_CMD_RESP, buffer);
      esp32.opsmode = opsmode;
      return true;
    }
    else {
      sprintf (buffer, "Opsmode '%s' unknown", obj["opsmode"].as<char*>());
      bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_FAIL, buffer);
      return false;          
    }
  }
  else {
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_FAIL, "Command needs 'opsmode' as string parameter");
    return false;    
  }  
}

bool cmd_set_parameter (char* json_str) {
  // json: {"wifi_ssid":"my_wifi","wifi_password":"my_password"}
  bool parameter_set = false;
  StaticJsonDocument<TM_MAX_MSG_SIZE> obj;
  deserializeJson(obj, json_str);
  if (obj.containsKey("wifi_ssid")) {
    strcpy (eeprom_network.wifi_ssid, obj["wifi_ssid"]);
    sprintf (buffer, "Setting wifi_ssid for %s to '%s' (save and reboot needed)", subsystemName[SS_THIS], eeprom_network.wifi_ssid);
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
    parameter_set = true;
  }
  if (obj.containsKey("wifi_password")) { 
    strcpy (eeprom_network.wifi_password, obj["wifi_password"]);
    sprintf (buffer, "Setting wifi_password for %s to '%s' (save and reboot needed)", subsystemName[SS_THIS], eeprom_network.wifi_password);
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
    parameter_set = true;
  }
  if (obj.containsKey("ap_ssid")) { 
    strcpy (eeprom_network.ap_ssid, obj["ap_ssid"]);
    sprintf (buffer, "Setting ap_ssid for %s to '%s' (save and reboot needed)", subsystemName[SS_THIS], eeprom_network.ap_ssid);
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
    parameter_set = true;
  }
  if (obj.containsKey("ap_password")) { 
    strcpy (eeprom_network.ap_password, obj["ap_password"]);
    sprintf (buffer, "Setting ap_password for %s to '%s' (save and reboot needed)", subsystemName[SS_THIS], eeprom_network.ap_password);
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
    parameter_set = true;
  }
  if (obj.containsKey("udp_server")) { 
    strcpy (eeprom_network.udp_server, obj["udp_server"]);
    sprintf (buffer, "Setting udp_server for %s to '%s'", subsystemName[SS_THIS], eeprom_network.udp_server);
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
    parameter_set = true;
  }  
  if (obj.containsKey("yamcs_server")) { 
    strcpy (eeprom_network.yamcs_server, obj["yamcs_server"]);
    sprintf (buffer, "Setting yamcs_server for %s to '%s'", subsystemName[SS_THIS], eeprom_network.yamcs_server);
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
    parameter_set = true;
  }
  if (obj.containsKey("ntp_server")) { 
    strcpy (eeprom_network.ntp_server, obj["ntp_server"]);
    sprintf (buffer, "Setting ntp_server for %s to '%s' (save and reboot needed)", subsystemName[SS_THIS], eeprom_network.ntp_server);
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
    parameter_set = true;
  }  
  if (obj.containsKey("udp_port")) { 
    eeprom_network.udp_port = obj["udp_port"];
    sprintf (buffer, "Setting udp_port for %s to '%d'", subsystemName[SS_THIS], eeprom_network.udp_port);
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
    parameter_set = true;
  }      
  if (obj.containsKey("yamcs_tm_port")) { 
    eeprom_network.yamcs_tm_port = obj["yamcs_tm_port"];
    sprintf (buffer, "Setting yamcs_tm_port for %s to '%d'", subsystemName[SS_THIS], eeprom_network.yamcs_tm_port);
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
    parameter_set = true;
  } 
  if (obj.containsKey("yamcs_tc_port")) { 
    eeprom_network.yamcs_tc_port = obj["yamcs_tc_port"];
    sprintf (buffer, "Setting yamcs_tc_port for %s to '%d'", subsystemName[SS_THIS], eeprom_network.yamcs_tc_port);
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
    parameter_set = true;
  }   
  #ifdef PLATFORM_ESP32
  if (obj.containsKey("radio_rate")) { 
    eeprom_esp32.radio_rate = obj["radio_rate"];
    timer.radio_interval = (1000 / eeprom_esp32.radio_rate);
    sprintf (buffer, "Setting radio_rate to %d Hz", eeprom_esp32.radio_rate);
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
    parameter_set = true;
  } 
  if (obj.containsKey("pressure_rate")) { 
    eeprom_esp32.pressure_rate = obj["pressure_rate"];
    timer.pressure_interval = (1000 / eeprom_esp32.pressure_rate);
    sprintf (buffer, "Setting pressure_rate to %d Hz", eeprom_esp32.pressure_rate);
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
    parameter_set = true;
  } 
  if (obj.containsKey("motion_rate")) { 
    eeprom_esp32.motion_rate = obj["motion_rate"];
    timer.motion_interval = (1000 / eeprom_esp32.motion_rate);
    sprintf (buffer, "Setting motion_rate to %d Hz", eeprom_esp32.motion_rate);
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
    parameter_set = true;
  } 
  if (obj.containsKey("gps_rate")) { 
    eeprom_esp32.gps_rate = obj["gps_rate"];
    timer.gps_interval = (1000 / eeprom_esp32.gps_rate);
    sprintf (buffer, "Setting gps_rate to %d Hz (save and reboot needed)", eeprom_esp32.gps_rate);
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
    parameter_set = true;
  } 
  if (obj.containsKey("radio_enable")) { 
    eeprom_esp32.radio_enable = obj["radio_enable"];
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, eeprom_esp32.radio_enable?"Enabling radio transmitter (save and reboot needed)":"Disabling radio transmitter (save and reboot needed)");      
    parameter_set = true;
  } 
  if (obj.containsKey("pressure_enable")) { 
    eeprom_esp32.pressure_enable = obj["pressure_enable"];
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, eeprom_esp32.pressure_enable?"Enabling pressure sensor (save and reboot needed)":"Disabling pressure sensor (save and reboot needed)");      
    parameter_set = true;
  } 
  if (obj.containsKey("motion_enable")) { 
    eeprom_esp32.motion_enable = obj["motion_enable"];
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, eeprom_esp32.motion_enable?"Enabling accelerometer/gyroscope sensor (save and reboot needed)":"Disabling accelerometer/gyroscope sensor (save and reboot needed)");      
    parameter_set = true;
  } 
  if (obj.containsKey("gps_enable")) { 
    eeprom_esp32.gps_enable = obj["gps_enable"];
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, eeprom_esp32.gps_enable?"Enabling gps receiver (save and reboot needed)":"Disabling gps receiver (save and reboot needed)");      
    parameter_set = true;
  } 
  if (obj.containsKey("camera_enable")) { 
    eeprom_esp32.gps_enable = obj["camera_enable"];
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, eeprom_esp32.camera_enable?"Enabling camera (save and reboot needed)":"Disabling camera (save and reboot needed)");      
    parameter_set = true;
  } 
  if (obj.containsKey("ota_enable")) { 
    eeprom_esp32.ota_enable = obj["ota_enable"];
    if (eeprom_esp32.ota_enable) {
      ota_setup ();
    }
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, eeprom_esp32.ota_enable?"Enabling OTA firmware update":"Disabling OTA firmware update");      
    parameter_set = true;
  }
  if (obj.containsKey("radio_debug")) { 
    eeprom_esp32.radio_debug = obj["radio_debug"];
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, eeprom_esp32.radio_debug?"Enabling radio debug":"Disabling radio debug");      
    parameter_set = true;
  }
  if (obj.containsKey("pressure_debug")) { 
    eeprom_esp32.pressure_debug = obj["pressure_debug"];
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, eeprom_esp32.pressure_debug?"Enabling pressure debug":"Disabling pressure debug");      
    parameter_set = true;
  }
    if (obj.containsKey("motion_debug")) { 
    eeprom_esp32.motion_debug = obj["motion_debug"];
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, eeprom_esp32.motion_debug?"Enabling motion debug":"Disabling motion debug");      
    parameter_set = true;
  }
    if (obj.containsKey("gps_debug")) { 
    eeprom_esp32.gps_debug = obj["gps_debug"];
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, eeprom_esp32.gps_debug?"Enabling gps debug":"Disabling gps debug");      
    parameter_set = true;
  }
    if (obj.containsKey("camera_debug")) { 
    eeprom_esp32.camera_debug = obj["camera_debug"];
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, eeprom_esp32.camera_debug?"Enabling camera debug":"Disabling camera debug");      
    parameter_set = true;
  }
    if (obj.containsKey("wifi_debug")) { 
    eeprom_esp32.wifi_debug = obj["wifi_debug"];
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, eeprom_esp32.wifi_debug?"Enabling wifi debug":"Disabling wifi debug");      
    parameter_set = true;
  }
    if (obj.containsKey("timer_debug")) { 
    eeprom_esp32.timer_debug = obj["timer_debug"];
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, eeprom_esp32.timer_debug?"Enabling timer debug":"Disabling timer debug");      
    parameter_set = true;
  }
    if (obj.containsKey("esp32_debug")) { 
    eeprom_esp32.esp32_debug = obj["esp32_debug"];
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, eeprom_esp32.esp32_debug?"Enabling esp32 debug":"Disabling esp32 debug");      
    parameter_set = true;
  }
  #endif
  #ifdef PLATFORM_ESP32CAM
  if (obj.containsKey("sd_enable")) { 
    eeprom_esp32cam.sd_enable = obj["sd_enable"];
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, eeprom.esp32cam.sd_enable?"Enabling use of SD card (save and reboot needed)":"Disabling use of SD card (save and reboot needed)");      
    parameter_set = true;
  }
  if (obj.containsKey("sd_json_enable")) { 
    eeprom_esp32cam.sd_json_enable = obj["sd_json_enable"];
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, eeprom.esp32cam.sd_json_enable?"Enabling logging to SD card in JSON format (save and reboot needed)":"Disabling logging to SD card in JSON (save and reboot needed)");      
    parameter_set = true;
  } 
  if (obj.containsKey("sd_ccsds_enable")) { 
    eeprom_esp32cam.sd_ccsds_enable = obj["sd_ccsds_enable"];
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, eeprom.esp32cam.sd_ccsds_enable?"Enabling logging to SD card in CCSDS format (save and reboot needed)":"Disabling logging to SD card in CCSDS (save and reboot needed)");      
    parameter_set = true;
  } 
  if (obj.containsKey("sd_image_enable")) { 
    eeprom_esp32cam.sd_image_enable = obj["sd_image_enable"];
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, eeprom.esp32cam.sd_image_enable?"Enabling storage of images to SD card (save and reboot needed)":"Disabling storage of images to SD card (save and reboot needed)");      
    parameter_set = true;
  }       
  //     esp32cam_debug:1;        // 7
  //     sd_debug:1;              //  6
  //     wifi_debug:1;            //   5
  //     camera_debug:1;          //    4
  //     timer_debug:1;           //     3  
  #endif
  if (obj.containsKey("wifi_enable")) { 
    eeprom_this->wifi_enable = obj["wifi_enable"];
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, eeprom_this->wifi_enable?"Enabling wifi":"Disabling wifi");      
    parameter_set = true;
  } 
  if (obj.containsKey("wifi_udp_enable")) { 
    eeprom_this->wifi_udp_enable = obj["wifi_udp_enable"];
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, eeprom_this->wifi_udp_enable?"Enabling udp transmission over wifi":"Disabling udp transmission over wifi");      
    parameter_set = true;
  } 
  if (obj.containsKey("wifi_yamcs_enable")) { 
    eeprom_this->wifi_yamcs_enable = obj["wifi_yamcs_enable"];
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, eeprom_this->wifi_yamcs_enable?"Enabling connection to yamcs over wifi":"Disabling connection to yamcs over wifi");      
    parameter_set = true;
  }                   
  if (obj.containsKey("wifi_sta_enable")) { 
    eeprom_this->wifi_sta_enable = obj["wifi_sta_enable"];
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, eeprom_this->wifi_sta_enable?"Enabling wifi in STA mode (save and reboot needed)":"Disabling wifi in STA mode (save and reboot needed)");      
    parameter_set = true;
  }
  if (obj.containsKey("wifi_ap_enable")) { 
    eeprom_this->wifi_ap_enable = obj["wifi_ap_enable"];
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, eeprom_this->wifi_ap_enable?"Enabling wifi in AP mode (save and reboot needed)":"Disabling wifi in AP mode (save and reboot needed)");      
    parameter_set = true;
  }
  if (obj.containsKey("serial_ccsds")) { 
    eeprom_this->serial_format = ENC_CCSDS;
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, "Setting serial transmission encoding to CCSDS");      
    parameter_set = true;
  }
  if (obj.containsKey("serial_json")) { 
    eeprom_this->serial_format = ENC_JSON;
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, "Setting serial transmission encoding to JSON");      
    parameter_set = true;
  }
  if (obj.containsKey("udp_ccsds")) { 
    eeprom_this->udp_format = ENC_CCSDS;
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, "Setting udp transmission encoding to CCSDS");      
    parameter_set = true;
  }
  if (obj.containsKey("udp_json")) { 
    eeprom_this->udp_format = ENC_JSON;
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, "Setting udp transmission encoding to JSON");      
    parameter_set = true;
  }        
  if (obj.containsKey("debug_over_serial")) { 
    eeprom_this->debug_over_serial = obj["debug_over_serial"];
    tm_this->serial_connected = true;
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, eeprom_this->debug_over_serial?"Enabling debug over serial mode":"Disabling debug over serial mode");      
    parameter_set = true;
  }
  if (!parameter_set) {
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_FAIL, "Command needs valid parameter/value json pairs");
    return false; 
  }
}

bool cmd_set_routing (char* json_str) {
  // json: {"table":"serial","load":"default","tm_gps":1,"tc_esp32":0}
  static byte* table_ptr;
  uint8_t PID;
  bool table_updated = false;
  StaticJsonDocument<TM_MAX_MSG_SIZE> obj;
  deserializeJson(obj, json_str);
  JsonObject entry = obj.as<JsonObject>();
  if (obj.containsKey("table")) {
    uint8_t table = id_of (obj["table"], sizeof(commLineName[0]), (char*)commLineName, sizeof(commLineName));
    switch (table) {
      case COMM_SERIAL:     if (obj.containsKey("load")) {
                              if (!strcmp(obj["load"], "default")) { routing_serial = (char*)&routing_serial_default; table_updated = true; }
                              if (!strcmp(obj["load"], "all")) { routing_serial = (char*)&routing_all; table_updated = true; }
                              if (!strcmp(obj["load"], "none")) { routing_serial = (char*)&routing_none; table_updated = true; }
                              if (!strcmp(obj["load"], "debug")) { routing_serial = (char*)&routing_serial_debug; table_updated = true; }
                            }
                            table_ptr = (byte*)routing_serial;
                            break; 
      case COMM_WIFI_UDP:   if (obj.containsKey("load")) {
                              if (!strcmp(obj["load"], "default")) { routing_udp = (char*)&routing_udp_default; table_updated = true; }
                              if (!strcmp(obj["load"], "all")) { routing_udp = (char*)&routing_all; table_updated = true; }
                              if (!strcmp(obj["load"], "none")) { routing_udp = (char*)&routing_none; table_updated = true; }
                              if (!strcmp(obj["load"], "debug")) { routing_udp = (char*)&routing_udp_debug; table_updated = true; }
                            }
                            table_ptr = (byte*)routing_udp;
                            break;
      case COMM_WIFI_YAMCS: if (obj.containsKey("load")) {
                              if (!strcmp(obj["load"], "default")) { routing_yamcs = (char*)&routing_yamcs_default; table_updated = true; }
                              if (!strcmp(obj["load"], "all")) { routing_yamcs = (char*)&routing_all; table_updated = true; }
                              if (!strcmp(obj["load"], "none")) { routing_yamcs = (char*)&routing_none; table_updated = true; }
                              if (!strcmp(obj["load"], "debug")) { routing_yamcs = (char*)&routing_yamcs_debug; table_updated = true; }
                            }
                            table_ptr = (byte*)routing_yamcs;
                            break;
      #ifdef PLATFORM_ESP32CAM
      case COMM_SD_CCSDS:   if (obj.containsKey("load")) {
                              if (!strcmp(obj["load"], "default")) { routing_sd_ccsds = (char*)&routing_sd_ccsds_default; table_updated = true; }
                              if (!strcmp(obj["load"], "all")) { routing_sd_ccsds = (char*)&routing_all; table_updated = true; }
                              if (!strcmp(obj["load"], "none")) { routing_sd_ccsds = (char*)&routing_none; table_updated = true; }
                              if (!strcmp(obj["load"], "debug")) { routing_sd_ccsds = (char*)&routing_sd_ccsds_debug; table_updated = true; }
                            }
                            table_ptr = (byte*)routing_sd_ccsds;
                            break;
      case COMM_SD_JSON:    if (obj.containsKey("load")) {
                              if (!strcmp(obj["load"], "default")) { routing_sd_json = (char*)&routing_sd_json_default; table_updated = true; }
                              if (!strcmp(obj["load"], "all")) { routing_sd_json = (char*)&routing_all; table_updated = true; }
                              if (!strcmp(obj["load"], "none")) { routing_sd_json = (char*)&routing_none; table_updated = true; }
                              if (!strcmp(obj["load"], "debug")) { routing_sd_json = (char*)&routing_sd_json_debug; table_updated = true; }
                            }
                            table_ptr = (byte*)routing_sd_json;
                            break;
      #endif
      default:              sprintf (buffer, "Table '%s' unknown", obj["table"].as<char*>());
                            bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_FAIL, buffer);
                            return false;
                            break; 
    }
    if (table_updated) {
      sprintf (buffer, "Loading '%s' settings for routing over %s", obj["load"].as<char*>(), obj["table"].as<char*>());
      bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
    }
    for (JsonPair pair : entry) {      
      PID = id_of (pair.key().c_str(), sizeof(pidName[0]), (char*)pidName, sizeof(pidName));
      if (PID != 255) {
        table_ptr[PID] = pair.value().as<bool>();
        sprintf (buffer, "Setting routing of '%s' in '%s' table to %s", pair.key().c_str(), obj["table"].as<char*>(), pair.value().as<bool>()?"on":"off");
        bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
        table_updated = true;
      }
    } 
    if (!table_updated) {
      sprintf (buffer, "No update to table %s applied; specify 'load' or packet", obj["table"].as<char*>());
      bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
    }  
  }
  else {
    bus_publish_event (STS_THIS, SS_THIS, EVENT_CMD_FAIL, "Command needs 'table' as string parameter");
    return false;    
  }
}

// SUPPORT FUNCTIONS

uint8_t id_of (const char* string2, uint8_t string_len, const char* array_of_strings, uint16_t array_len) { 
  //Serial.print ("Tag: [");
  //Serial.print (string2);
  //Serial.print ("] ");
  //Serial.print (string_len);
  //Serial.print (" [");
  //Serial.print (array_of_strings);
  //Serial.print ("] ");
  //Serial.println (array_len);
  char string[string_len]; 
  strcpy (string, string2);
  //Serial_print_charptr ((char*)&string, string_len);
  //Serial.println (string_len);
  //Serial.println (array_len);
  //Serial.println ((char*)&string);
  for (uint8_t id = 0; id < array_len / string_len; id++) {
    //Serial.println (id);
    //Serial_print_charptr ((char*)(array_of_strings + string_len*id), string_len);
    //Serial.println ((char*)(array_of_strings + string_len*id));
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

void tohex(unsigned char * in, size_t insz, char * out, size_t outsz) {
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
