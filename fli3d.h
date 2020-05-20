/*
 * Fli3d - Library (wifi, bus, TM, eeprom functionality)
 * version: 2020-04-14 edited on esp32
 */
 
#ifndef _FLI3D_H_
#define _FLI3D_H_

#ifdef ARDUINO_MH_ET_LIVE_ESP32MINIKIT
#define PLATFORM_ESP32
#endif
#ifdef ARDUINO_ESP32_DEV
#define PLATFORM_ESP32CAM
#endif

#include <EEPROM.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <LinkedList.h>
#include <Arduino.h>
#include <RH_ASK.h>
#include <ArduinoJson.h>

#define SerialBaud 115200
#define TM_MAX_MSG_SIZE 250
#define WIFI_TIMEOUT 30
#define NTP_TIMEOUT 5
#define RADIO_BAUD 2000
#define KEEPALIVE_INTERVAL 200
#define BUFFER_RELEASE_BATCH_SIZE 3
#define MIN_MEM_FREE 70000

// Pin assignment for ESP32 MH-ET minikit board
#define DUMMY_PIN1             12   // IO12; hack: RadioHead needs an RX pin to be set
#define I2C_SCL_PIN            SCL  // IO22; default SCL I2C pin on ESP32
#define I2C_SDA_PIN            SDA  // IO21; default SDA I2C pin on ESP32
#define SEP_STS_PIN            19   // IO19; 
#define RF433_TX_PIN           26   // IO26; 
#define DUMMY_PIN2             14   // IO14; hack: RadioHead needs a PTT pin to be set
#define GPS_RX_PIN             16   // IO16; second serial interface on ESP32 (Serial2)
#define GPS_TX_PIN             17   // IO17; second serial interface on ESP32 (Serial2)
#define ESP32CAM_RX_PIN        3    // IO3; RX / main serial interface on ESP32 (Serial)
#define ESP32CAM_TX_PIN        1    // IO1; TX / main serial interface on ESP32 (Serial)

//  MDB DEFINITION

#ifdef PLATFORM_ESP32
#define SS_THIS      SS_ESP32       // define default subsystem
#define SS_THIS_WIFI SS_ESP32_WIFI  // define default subsystem
#define SS_OTHER     SS_ESP32CAM    // define counterpart subsystem
#define TM_THIS      TM_ESP32       // define default system TM packet
#define TC_THIS      TC_ESP32       // define default system TC packet
#define TC_OTHER     TC_ESP32CAM    // define default TC packet destination
#define STS_THIS     STS_ESP32      // define default system STS packet
#define DEBUG_THIS   DEBUG_ESP32    // define default system DEBUG packet
#define EEPROM_THIS  EEPROM_ESP32   // define default system EEPROM packet
#endif
#ifdef PLATFORM_ESP32CAM
#define SS_THIS      SS_ESP32CAM    // define default subsystem
#define SS_THIS_WIFI SS_ESP32CAM_WIFI // define default subsystem
#define SS_OTHER     SS_ESP32       // define counterpart subsystem
#define TM_THIS      TM_ESP32CAM    // define default system TM packet
#define TC_THIS      TC_ESP32CAM    // define default system TC packet
#define TC_OTHER     TC_ESP32       // define default TC packet destination
#define STS_THIS     STS_ESP32CAM   // define default system STS packet
#define DEBUG_THIS   DEBUG_ESP32CAM // define default system DEBUG packet
#define EEPROM_THIS  EEPROM_ESP32CAM // define default system EEPROM packet
#endif

// PIDs
#define STS_ESP32              0
#define STS_ESP32CAM           1
#define TM_ESP32               2
#define TM_ESP32CAM            3
#define TM_CAMERA              4
#define TM_GPS                 5
#define TM_MOTION              6
#define TM_PRESSURE            7
#define TM_RADIO               8
#define TM_TIMER               9
#define TC_ESP32               10
#define TC_ESP32CAM            11
#define DEBUG_ESP32            12
#define DEBUG_ESP32CAM         13
#define EEPROM_NETWORK         14
#define EEPROM_ESP32           15
#define EEPROM_ESP32CAM        16
#define NUMBER_OF_PID          17
extern const char pidName[NUMBER_OF_PID][16];

// event_types
#define EVENT_INIT             0
#define EVENT_INFO             1
#define EVENT_WARNING          2
#define EVENT_ERROR            3
#define EVENT_DEBUG            4
#define EVENT_CMD              5
#define EVENT_CMD_RESP         6
#define EVENT_CMD_FAIL         7
extern const char eventName[8][9];

// subsystem
#define SS_ESP32               0
#define SS_ESP32CAM            1
#define SS_OV2640              2
#define SS_NEO6MV2             3
#define SS_MPU6050             4
#define SS_BMP280              5
#define SS_RADIO               6
#define SS_SD                  7
#define SS_SEPARATION          8
#define SS_TIMER               9
#define SS_ESP32_WIFI          10
#define SS_ESP32CAM_WIFI       11
#define SS_FLI3D               12
#define SS_GROUND              13
#define SS_ANY                 14 
extern const char subsystemName[15][14];

// opsmode
#define MODE_INIT              0
#define MODE_CHECKOUT          1
#define MODE_READY             2
#define MODE_THRUST            3
#define MODE_FREEFALL          4
#define MODE_PARACHUTE         5
#define MODE_STATIC            6
extern const char modeName[7][10];

// cammode
#define CAM_INIT               0
#define CAM_IDLE               1
#define CAM_SINGLE             2
#define CAM_STREAM             3
extern const char cameraModeName[4][7];

// cam resolutions
#define RES_160x120            0
#define RES_240x176            1
#define RES_240x176            2
#define RES_240x176            3
#define RES_320x240            4 
#define RES_400x300            5
#define RES_640x480            6
#define RES_800x600            7
#define RES_1024x768           8
#define RES_1280x1024          9
#define RES_1600x1200          10           
extern const char cameraResolutionName[11][10];

// encoding
#define ENC_CCSDS              0
#define ENC_JSON               1
#define ENC_ASCII              2
extern const char dataEncodingName[3][8];

// packet types
#define PKT_TM                 0
#define PKT_TC                 1

#define COMM_SERIAL            0
#define COMM_WIFI_UDP          1
#define COMM_WIFI_YAMCS        2
#define COMM_SD_CCSDS          3
#define COMM_SD_JSON           4
#define COMM_SD_CAM            5
#define COMM_RADIO             6
extern const char commLineName[7][11];

#define TC_REBOOT              0
#define TC_REBOOT_FLI3D        1
#define TC_EEPROM_RESET        2
#define TC_EEPROM_LOAD         3
#define TC_EEPROM_SAVE         4
#define TC_GET_PACKET          5
#define TC_SET_OPSMODE         6
#define TC_SET_PARAMETER       7
#define TC_SET_ROUTING         8
extern const char tcName[9][20];

// serial buffer status
#define SERIAL_UNKNOWN         0
#define SERIAL_CR              1
#define SERIAL_LF              2
#define SERIAL_JSON            3
#define SERIAL_CCSDS           4
#define SERIAL_ASCII           5
#define SERIAL_KEEPALIVE       6
#define SERIAL_COMPLETE        7

extern const char gpsStatusName[5][10]; 

// TM/TC packet definitions

typedef struct __attribute__ ((packed)) ccsds_hdr_t {
  uint8_t  apid_H:3;                // 0:5
  bool     sec_hdr:1;               // 0: 4
  bool     type:1;                  // 0:  3
  uint8_t  version:3;               // 0:   0
  uint8_t  apid_L;                  // 1
  uint8_t  seq_ctr_H:6;             // 2:2
  uint8_t  seq_flag:2;              // 2: 0
  uint8_t  seq_ctr_L;               // 3
  uint8_t  pkt_len_H;               // 4
  uint8_t  pkt_len_L;               // 5
}; 

typedef struct __attribute__ ((packed)) ccsds_t {
  char     ccsds_hdr[6];
  char     blob[TM_MAX_MSG_SIZE];
  uint8_t  blob_size;
};

typedef struct __attribute__ ((packed)) sts_esp32_t { // APID: 42 (2a)
  uint32_t millis:24;
  uint16_t packet_ctr;
  uint8_t  type:4;                  // 4
  uint8_t  subsystem:4;             //  0
  char     message[TM_MAX_MSG_SIZE];
  uint8_t  message_size;
};

typedef struct __attribute__ ((packed)) sts_esp32cam_t { // APID: 43 (2b)
  uint32_t millis:24;
  uint16_t packet_ctr;
  uint8_t  type:4;                  // 4
  uint8_t  subsystem:4;             //  0
  char     message[TM_MAX_MSG_SIZE];
  uint8_t  message_size;
}; 

typedef struct __attribute__ ((packed)) tm_esp32_t { // APID: 44 (2c) size: 33 bytes
  uint32_t millis:24;
  uint16_t packet_ctr;
  uint8_t  opsmode:4;               // 4
  uint8_t  free:4;                  //  0
  uint8_t  error_ctr;
  uint8_t  warning_ctr;
  uint8_t  tc_exec_ctr;
  uint8_t  tc_fail_ctr;
  uint8_t  radio_rate;
  uint8_t  pressure_rate;
  uint8_t  motion_rate;
  uint8_t  gps_rate;
  uint8_t  camera_rate;
  uint32_t timer_rate:24;
  uint8_t  udp_rate;
  uint8_t  yamcs_rate;
  uint8_t  serial_in_rate;
  uint8_t  serial_out_rate;
  uint8_t  udp_buffer;
  uint8_t  yamcs_buffer;
  uint8_t  serial_buffer;
  uint32_t mem_free;
  bool     radio_enabled:1;         // 7
  bool     pressure_enabled:1;      //  6
  bool     motion_enabled:1;        //   5
  bool     gps_enabled:1;           //    4
  bool     camera_enabled:1;        //     3
  bool     wifi_enabled:1;          //      2
  bool     wifi_udp_enabled:1;      //       1
  bool     wifi_yamcs_enabled:1;    //        0
  bool     warn_serial_connloss:1;  // 7
  bool     warn_wifi_connloss:1;    //  6
  bool     err_serial_dataloss:1;   //   5
  bool     err_udp_dataloss:1;      //    4
  bool     err_yamcs_dataloss:1;    //     3
  bool     free_12:1;               //      2
  bool     free_11:1;               //       1
  bool     free_10:1;               //        0
  bool     radio_current:1;         // 7
  bool     pressure_current:1;      //  6
  bool     motion_current:1;        //   5
  bool     gps_current:1;           //    4
  bool     camera_current:1;        //     3
  bool     serial_connected:1;      //      2
  bool     wifi_connected:1;        //       1 
  bool     separation_sts:1;        //        0  
};

typedef struct __attribute__ ((packed)) tm_esp32cam_t { // APID: 45
  uint32_t millis:24;
  uint16_t packet_ctr;
  uint8_t  error_ctr;
  uint8_t  warning_ctr;
  uint8_t  tc_exec_ctr;
  uint8_t  tc_fail_ctr;
  uint32_t timer_rate:24;
  uint8_t  udp_rate;
  uint8_t  yamcs_rate;
  uint8_t  serial_in_rate;
  uint8_t  serial_out_rate;
  uint8_t  sd_json_rate;
  uint8_t  sd_ccsds_rate;
  uint8_t  camera_image_rate;
  uint8_t  udp_buffer;
  uint8_t  yamcs_buffer;
  uint8_t  serial_buffer;
  uint8_t  sd_json_buffer;
  uint8_t  sd_ccsds_buffer;
  uint32_t mem_free;
  uint32_t sd_free;
  bool     sd_enabled:1;            // 7 
  bool     sd_image_enabled:1;      //  6
  bool     sd_json_enabled:1;       //   5
  bool     sd_ccsds_enabled:1;      //    4 
  bool     wifi_enabled:1;          //     3
  bool     wifi_udp_enabled:1;      //      2
  bool     wifi_yamcs_enabled:1;    //       1
  bool     wifi_image_enabled:1;    //        0
  bool     camera_enabled:1;        // 7
  bool     sd_current:1;            //  6
  bool     camera_current:1;        //   5
  bool     wifi_connected:1;        //    4
  bool     serial_connected:1;      //     3
  bool     free_12:1;               //      2
  bool     free_11:1;               //       1
  bool     free_10:1;               //        0  
};

typedef struct __attribute__ ((packed)) tm_camera_t { // APID: 46
  uint32_t millis:24;
  uint16_t packet_ctr;
  uint8_t  camera_mode:4;           // 4
  uint8_t  resolution:4;            //  0
  uint32_t filesize:24; 
  uint8_t  wifi_ms; 
  uint8_t  sd_ms;
  uint8_t  exposure_ms;
  bool     auto_res;                // 7
  bool     free_06:1;               //  6
  bool     free_05:1;               //   5
  bool     free_04:1;               //    4
  bool     free_03:1;               //     3
  bool     free_02:1;               //      2
  bool     free_01:1;               //       1
  bool     free_00:1;               //        0
  char     filename[26]; 
};

typedef struct __attribute__ ((packed)) tm_gps_t { // APID: 47
  uint32_t millis:24;
  uint16_t packet_ctr;
  uint8_t  status:4;                // 4
  uint8_t  satellites:4;            //  0
  uint8_t  hours;
  uint8_t  minutes;
  uint8_t  seconds;
  float    latitude;
  float    longitude;
  float    altitude;
  float    v_north;
  float    v_east;
  float    v_down;
  uint16_t milli_hdop;
  uint16_t milli_vdop;
  bool     time_valid:1;            // 7
  bool     location_valid:1;        //  6
  bool     altitude_valid:1;        //   5
  bool     speed_valid:1;           //    4
  bool     hdop_valid:1;            //     3
  bool     vdop_valid:1;            //      2
  bool     free_01:1;               //       1
  bool     free_00:1;               //        0
};

typedef struct __attribute__ ((packed)) tm_motion_t { // APID: 48
  uint32_t millis:24;
  uint16_t packet_ctr;
  float    accel_x_rocket;
  float    accel_y_rocket;
  float    accel_z_rocket;
  float    gyro_x_rocket;
  float    gyro_y_rocket;
  float    gyro_z_rocket;
  float    tilt;
  float    g;
  float    a;
  float    rpm;
  int16_t  a_x_offset;
  int16_t  a_y_offset;
  int16_t  a_z_offset;
  int16_t  g_x_offset;
  int16_t  g_y_offset;
  int16_t  g_z_offset;
}; 

typedef struct __attribute__ ((packed)) tm_pressure_t { // APID: 49
  uint32_t millis:24;
  uint16_t packet_ctr;
  float    pressure;
  float    zero_level_pressure;
  float    altitude;
  float    velocity_v;
  float    temperature;
}; 

typedef struct __attribute__ ((packed)) tm_radio_t { // APID: 50
  uint32_t millis:24;
  uint16_t packet_ctr;
  uint8_t  opsmode:4;               // 4
  uint8_t  free:4;                  //  0
  uint8_t  error_ctr;
  uint8_t  warning_ctr;
  uint8_t  esp32_udp_buffer;
  uint8_t  esp32_yamcs_buffer;
  uint8_t  esp32_serial_buffer;
  uint8_t  esp32cam_udp_buffer;
  uint8_t  esp32cam_yamcs_buffer;
  uint8_t  esp32cam_serial_buffer;
  bool     separation_sts:1;        // 7
  bool     pressure_current:1;      //  6
  bool     motion_current:1;        //   5
  bool     gps_current:1;           //    4
  bool     camera_current:1;        //     3
  bool     serial_connected:1;      //      2
  bool     esp32_wifi_connected:1;  //       1 
  bool     esp32cam_wifi_connected:1; //        0
  uint8_t  pressure_altitude;       // m               (0 - 255 m)
  int8_t   pressure_velocity_v;     // m/s             (-125 - 126 m/s)
  int8_t   temperature;             // degC            (-125 - 126 degC)
  uint8_t  motion_tilt;             // deg             (0 - 180 deg)
  int8_t   motion_g;                // g / 10          (-12.5 - 12.6 g)  
  int8_t   motion_a;                // m/s2            (-125 - 126 m/s2)
  int8_t   motion_rpm;              //                 (-125 - 126 rpm)  
  uint8_t  gps_satellites;
  int8_t   gps_velocity_v;          // m/s             (-125 - 126 m/s)
  uint8_t  gps_velocity;            // m/s             (0 - 255 m/s)
  uint8_t  gps_altitude;            // m               (0 - 255 m)
  uint16_t camera_image_ctr;
  uint8_t  camera_rate;
  uint16_t sd_free;
}; 

typedef struct __attribute__ ((packed)) tm_timer_t { // APID: 51
  uint32_t millis:24;
  uint16_t packet_ctr;
  uint8_t  radio_duration;
  uint8_t  pressure_duration;
  uint8_t  motion_duration;
  uint8_t  gps_duration;
  uint8_t  esp32cam_duration;
  uint8_t  loop_duration;
  bool     do_radio:1;
  bool     do_pressure:1;
  bool     do_motion:1;
  bool     do_gps:1; 
  bool     free_03:1;
  bool     free_02:1;
  bool     free_01:1;
  bool     free_00:1;
  uint32_t next_second;
  uint32_t next_radio_time;
  uint32_t next_pressure_time;
  uint32_t next_motion_time;
  uint32_t next_gps_time;
  uint32_t last_serial_in_millis;
  uint32_t last_serial_out_millis;
  uint32_t last_wifi_check;
  uint16_t radio_interval;
  uint16_t pressure_interval;
  uint16_t motion_interval;
  uint16_t gps_interval;
}; 

typedef struct __attribute__ ((packed)) tc_esp32_t { // APID: 52
  uint8_t  cmd;
  char     json[TM_MAX_MSG_SIZE];
  uint8_t  json_size;
}; 

typedef struct __attribute__ ((packed)) tc_esp32cam_t { // APID: 53
  uint8_t  cmd;
  char     json[TM_MAX_MSG_SIZE];
  uint8_t  json_size;
}; 

typedef struct __attribute__ ((packed)) debug_esp32_t { // APID: 54
  uint32_t millis:24;
  uint16_t packet_ctr;
  char     message[TM_MAX_MSG_SIZE];
  uint8_t  message_size;
}; 

typedef struct __attribute__ ((packed)) debug_esp32cam_t { // APID: 55
  uint32_t millis:24;
  uint16_t packet_ctr;
  char     message[TM_MAX_MSG_SIZE];
  uint8_t  message_size;
}; 

typedef struct __attribute__ ((packed)) eeprom_network_t { // APID: 56
  uint8_t  magic = 0x46;
  uint8_t  version = 4; 
  uint8_t  struct_size = 149;
  char     wifi_ssid[20]; 
  char     wifi_password[20];
  char     ap_ssid[20]; 
  char     ap_password[20];
  char     udp_server[20];
  char     yamcs_server[20]; 
  char     ntp_server[20];
  uint16_t udp_port; 
  uint16_t yamcs_tm_port;
  uint16_t yamcs_tc_port;
};

typedef struct __attribute__ ((packed)) eeprom_esp32_t { // APID: 57
  uint8_t  magic = 0x45;
  uint8_t  version = 6; 
  uint8_t  struct_size = 10;
  uint8_t  radio_rate = 2;          // Hz (valid: 2 or less)
  uint8_t  pressure_rate = 10;      // Hz (up to 157 Hz, highest resolution up to 23 Hz); reached 176 Hz on ESP8266
  uint8_t  motion_rate = 10;        // Hz (up to 400)
  uint8_t  gps_rate = 10;           // Hz (valid: 1,5,10,16)
  bool     radio_enable:1;          // 7
  bool     pressure_enable:1;       //  6
  bool     motion_enable:1;         //   5
  bool     gps_enable:1;            //    4
  bool     camera_enable:1;         //     3
  bool     wifi_enable:1;           //      2
  bool     wifi_udp_enable:1;       //       1
  bool     wifi_yamcs_enable:1;     //        0
  bool     radio_debug:1;           // 7
  bool     pressure_debug:1;        //  6
  bool     motion_debug:1;          //   5
  bool     gps_debug:1;             //    4
  bool     camera_debug:1;          //     3  
  bool     wifi_debug:1;            //      2
  bool     timer_debug:1;           //       1
  bool     esp32_debug:1;           //        0
  bool     serial_format:1;         // 7
  bool     udp_format:1;            //  6
  bool     wifi_sta_enable:1;       //   5
  bool     wifi_ap_enable:1;        //    4
  bool     debug_over_serial:1;     //     3
  bool     ota_enable:1;            //      2
  bool     free_21:1;               //       1
  bool     free_20:1;               //        0
};

typedef struct __attribute__ ((packed)) eeprom_esp32cam_t { // APID: 58
  uint8_t  magic = 0x43;
  uint8_t  version = 5;
  uint8_t  struct_size = 6;
  bool     sd_json_enable:1;        // 7
  bool     sd_ccsds_enable:1;       //  6
  bool     wifi_enable:1;           //   5
  bool     wifi_udp_enable:1;       //    4
  bool     wifi_yamcs_enable:1;     //     3
  bool     wifi_image_enable:1;     //      2 
  bool     serial_format:1;         //       1
  bool     udp_format:1;            //        0
  bool     sd_image_enable:1;       // 7
  bool     debug_over_serial:1;     //  6
  bool     free_15:1;               //   5
  bool     free_14:1;               //    4
  bool     free_13:1;               //     3
  bool     free_12:1;               //      2
  bool     free_11:1;               //       1
  bool     free_10:1;               //        0
  bool     esp32cam_debug:1;        // 7
  bool     sd_debug:1;              //  6
  bool     wifi_debug:1;            //   5
  bool     camera_debug:1;          //    4
  bool     timer_debug:1;           //     3
  bool     wifi_sta_enable:1;       //      2
  bool     wifi_ap_enable:1;        //       1
  bool     sd_enable:1;             //        0  
};

extern ccsds_hdr_t        ccsds_hdr;

extern sts_esp32_t        sts_esp32;
extern sts_esp32cam_t     sts_esp32cam;
extern tm_esp32_t         esp32;
extern tm_esp32cam_t      esp32cam;
extern tm_camera_t        ov2640;
extern tm_gps_t           neo6mv2;
extern tm_motion_t        mpu6050;
extern tm_pressure_t      bmp280;
extern tm_radio_t         radio;
extern tm_timer_t         timer;
extern debug_esp32_t      debug_esp32;
extern debug_esp32cam_t   debug_esp32cam;

#ifdef PLATFORM_ESP32
extern tm_esp32_t*        tm_this;
extern sts_esp32_t*       sts_this;
extern debug_esp32_t*     debug_this;
extern eeprom_esp32_t*    eeprom_this;
#endif
#ifdef PLATFORM_ESP32CAM
extern tm_esp32cam_t*     tm_this;
extern sts_esp32cam_t*    sts_this;
extern debug_esp32cam_t*  debug_this;
extern eeprom_esp32cam_t* eeprom_this;
#endif

extern eeprom_network_t   eeprom_network;
extern eeprom_esp32_t     eeprom_esp32;
extern eeprom_esp32cam_t  eeprom_esp32cam;

extern LinkedList<ccsds_t*> linkedlist_serial;
extern LinkedList<ccsds_t*> linkedlist_udp;
extern LinkedList<ccsds_t*> linkedlist_yamcs;
extern WiFiUDP wifiUDP;
extern char buffer[TM_MAX_MSG_SIZE];
extern char bus_buffer[TM_MAX_MSG_SIZE + 25];
extern char radio_buffer[2*(sizeof(tm_radio_t)+2)];

extern void eeprom_setup ();
extern void eeprom_load (uint8_t eeprom_id, uint16_t offset);
extern void eeprom_save (uint8_t eeprom_id, uint16_t offset);
extern void eeprom_reset (uint8_t eeprom_id, uint16_t offset);
extern void eeprom_print ();

extern bool wifi_setup ();

extern void bus_publish_pkt (uint8_t PID);
extern void bus_publish_event (uint8_t PID, uint8_t subsystem, uint8_t event_type, const char* event_message);
extern void bus_publish_tc (uint8_t PID, uint8_t cmd, char* json);
extern bool serial_publish (char* packet_ptr, uint16_t packet_len);
extern bool udp_publish (char* packet_ptr, uint16_t packet_len);
extern bool yamcs_publish (char* packet_ptr, uint16_t packet_len);
#ifdef PLATFORM_ESP32CAM
extern bool sd_json_publish (char* message);
extern bool sd_ccsds_publish (char* packet_ptr, uint16_t packet_len);
extern uint32_t sd_free ();
#endif

extern void update_ccsds_hdr (uint16_t PID, bool pkt_type, uint16_t pkt_len);
extern void buildJsonString (uint8_t PID);

extern void update_pkt (uint8_t PID);
extern void reset_pkt (uint8_t PID);

#ifdef PLATFORM_ESP32
extern bool radio_setup ();
extern void radio_publish ();
#endif

extern uint16_t serial_check ();
extern void announce_established_serial_connection ();
extern void announce_lost_serial_connection ();
extern void serial_parse (uint16_t data_len);
extern void serial_parse_json (); 
extern void serial_parse_ccsds (uint16_t data_len);

extern bool cmd_reboot ();
extern bool cmd_reboot_fli3d ();
extern bool cmd_eeprom_reset ();
extern bool cmd_eeprom_load ();
extern bool cmd_eeprom_save ();
extern bool cmd_get_packet (char* json_str);
extern bool cmd_set_opsmode (char* json_str);
extern bool cmd_set_parameter (char* json_str);
extern bool cmd_set_routing (char* json_str);

extern uint8_t id_of (const char* string, uint8_t string_len, const char* array_of_strings, uint16_t array_len);
extern void tohex (unsigned char * in, size_t insz, char * out, size_t outsz);
extern void Serial_print_charptr (char *ptr, uint8_t len);

#endif // _FLI3D_H_
