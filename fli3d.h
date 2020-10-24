/*
 * Fli3d - Library (file system, wifi, TM/TC, comms functionality)
 * version: 2020-10-23
 */
 
#ifndef _FLI3D_H_
#define _FLI3D_H_

#ifdef ARDUINO_MH_ET_LIVE_ESP32MINIKIT
#define PLATFORM_ESP32
#endif
#ifdef ARDUINO_ESP32_DEV
#define PLATFORM_ESP32CAM
#endif

//#define ASYNCUDP // uncomment to use AsyncUDP for commanding

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#ifdef ASYNCUDP
#include <AsyncUDP.h>
#endif
#include <ESPFtpServer.h>
#include <NTPClient.h>
#include <LinkedList.h>
#include <Arduino.h>
#include <RH_ASK.h>
#include <ArduinoJson.h>
#include <LITTLEFS.h>
#include "FS.h"

#define SerialBaud                115200
#define JSON_MAX_SIZE             512
#define WIFI_TIMEOUT              120    // s
#define NTP_TIMEOUT               60     // s
#define WIFI_POLL                 5      // s
#define RADIO_BAUD                2000   // transmission speed over 433 MHz radio
#define KEEPALIVE_INTERVAL        200    // ms for keep-alive of serial connection between ESP32 and ESP32cam
#define BUFFER_RELEASE_BATCH_SIZE 3      // TM buffer is released by this number of packets at a time
#define MIN_MEM_FREE              70000  // TM buffering stops when memory is below this value 

// Pin assignment for ESP32 MH-ET minikit board
#define DUMMY_PIN1                12   // IO12; hack: RadioHead needs an RX pin to be set
#define I2C_SCL_PIN               SCL  // IO22; default SCL I2C pin on ESP32
#define I2C_SDA_PIN               SDA  // IO21; default SDA I2C pin on ESP32
#define SEP_STS_PIN               19   // IO19; 
#define RF433_TX_PIN              26   // IO26; 
#define DUMMY_PIN2                14   // IO14; hack: RadioHead needs a PTT pin to be set
#define GPS_RX_PIN                16   // IO16; second serial interface on ESP32 (Serial2)
#define GPS_TX_PIN                17   // IO17; second serial interface on ESP32 (Serial2)
#define ESP32CAM_RX_PIN           3    // IO3; RX / main serial interface on ESP32 (Serial)
#define ESP32CAM_TX_PIN           1    // IO1; TX / main serial interface on ESP32 (Serial)

//  MDB DEFINITION

#ifdef PLATFORM_ESP32
#define SS_THIS      SS_ESP32       // define default subsystem
#define SS_OTHER     SS_ESP32CAM    // define counterpart subsystem
#define TM_THIS      TM_ESP32       // define default system TM packet
#define TM_OTHER     TM_ESP32CAM    // define counterpart system TM packet
#define TC_THIS      TC_ESP32       // define default system TC packet
#define TC_OTHER     TC_ESP32CAM    // define default TC packet destination
#define STS_THIS     STS_ESP32      // define default system STS packet
#define STS_OTHER    STS_ESP32CAM   // define counterpart system STS packet
#endif
#ifdef PLATFORM_ESP32CAM
#define SS_THIS      SS_ESP32CAM    // define default subsystem
#define SS_OTHER     SS_ESP32       // define counterpart subsystem
#define TM_THIS      TM_ESP32CAM    // define default system TM packet
#define TM_OTHER     TM_ESP32       // define counterpart system TM packet
#define TC_THIS      TC_ESP32CAM    // define default system TC packet
#define TC_OTHER     TC_ESP32       // define default TC packet destination
#define STS_THIS     STS_ESP32CAM   // define default system STS packet
#define STS_OTHER    STS_ESP32      // define counterpart system STS packet
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
#define NUMBER_OF_PID          12
extern const char pidName[NUMBER_OF_PID][13];

// event_types
#define EVENT_INIT             0
#define EVENT_INFO             1
#define EVENT_WARNING          2
#define EVENT_ERROR            3
#define EVENT_CMD              4
#define EVENT_CMD_ACK          5
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
#define SS_FLI3D               10
#define SS_GROUND              11
#define SS_ANY                 12 
extern const char subsystemName[13][14];

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
#define RES_INVALID1           2
#define RES_INVALID2           3
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

// data channels
#define COMM_SERIAL            0
#define COMM_WIFI_UDP          1
#define COMM_WIFI_YAMCS        2
#define COMM_WIFI_CAM          3
#define COMM_SD_CCSDS          4
#define COMM_SD_JSON           5
#define COMM_SD_CAM            6
#define COMM_FS                7
#define COMM_RADIO             8
extern const char commLineName[9][13];

#define TC_REBOOT              42
#define TC_SET_OPSMODE         43
#define TC_LOAD_CONFIG         44
#define TC_LOAD_ROUTING        45
#define TC_SET_PARAMETER       46
extern const char tcName[5][20];

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

struct __attribute__ ((packed)) ccsds_hdr_t {
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

struct __attribute__ ((packed)) ccsds_t {
  char     ccsds_hdr[6];
  char     blob[JSON_MAX_SIZE];
  uint8_t  blob_size;
  uint8_t  PID;
};

struct __attribute__ ((packed)) ccsds_tc_t {
  char     ccsds_hdr[6];
  uint8_t  cmd_id;
  uint8_t  int_parameter;
  char     str_parameter[JSON_MAX_SIZE];
};

struct __attribute__ ((packed)) sts_esp32_t { // APID: 42 (2a)
  uint32_t millis:24;
  uint16_t packet_ctr;
  uint8_t  type:4;                  // 4
  uint8_t  subsystem:4;             //  0
  char     message[JSON_MAX_SIZE];
  uint8_t  message_size;
};

struct __attribute__ ((packed)) sts_esp32cam_t { // APID: 43 (2b)
  uint32_t millis:24;
  uint16_t packet_ctr;
  uint8_t  type:4;                  // 4
  uint8_t  subsystem:4;             //  0
  char     message[JSON_MAX_SIZE];
  uint8_t  message_size;
}; 

struct __attribute__ ((packed)) tm_esp32_t { // APID: 44 (2c)
  uint32_t millis:24;
  uint16_t packet_ctr;
  uint8_t  opsmode:4;               // 4
  uint8_t  free:4;                  //  0 - free to assign
  uint8_t  error_ctr;
  uint8_t  warning_ctr;
  uint8_t  tc_exec_ctr;
  uint8_t  tc_fail_ctr;
  uint8_t  radio_rate;
  uint8_t  pressure_rate;
  uint8_t  motion_rate;
  uint8_t  gps_rate;
  uint8_t  camera_rate;
  uint8_t  udp_rate;
  uint8_t  yamcs_rate;
  uint8_t  serial_in_rate;
  uint8_t  serial_out_rate;
  uint8_t  fs_rate;
  uint16_t yamcs_buffer;
  uint16_t serial_buffer;
  uint16_t mem_free;
  uint16_t fs_free;
  bool     radio_enabled:1;         // 7
  bool     pressure_enabled:1;      //  6
  bool     motion_enabled:1;        //   5
  bool     gps_enabled:1;           //    4
  bool     camera_enabled:1;        //     3
  bool     wifi_enabled:1;          //      2
  bool     wifi_udp_enabled:1;      //       1
  bool     wifi_yamcs_enabled:1;    //        0
  bool     fs_enabled:1;            // 7
  bool     fs_ftp_enabled:1;        //  6
  bool     time_set:1;              //   5
  bool     free_24:1;               //    4 - free to assign
  bool     free_23:1;               //     3 - free to assign
  bool     free_22:1;               //      2 - free to assign
  bool     free_21:1;               //       1 - free to assign
  bool     free_20:1;               //        0 - free to assign
  bool     serial_connected:1;      // 7
  bool     wifi_connected:1;        //  6 
  bool     warn_serial_connloss:1;  //   5        
  bool     warn_wifi_connloss:1;    //    4       
  bool     err_serial_dataloss:1;   //     3
  bool     err_yamcs_dataloss:1;    //      2
  bool     err_fs_dataloss:1;       //       1
  bool     separation_sts:1;        //        0  
  bool     radio_active:1;          // 7
  bool     pressure_active:1;       //  6
  bool     motion_active:1;         //   5
  bool     gps_active:1;            //    4
  bool     camera_active:1;         //     3
  bool     fs_active:1;             //      2
  bool     ftp_active:1;            //       1
  bool     ota_enabled:1;            //        0 
};

struct __attribute__ ((packed)) tm_esp32cam_t { // APID: 45 (2d)
  uint32_t millis:24;
  uint16_t packet_ctr;
  uint8_t  error_ctr;
  uint8_t  warning_ctr;
  uint8_t  tc_exec_ctr;
  uint8_t  tc_fail_ctr;
  uint8_t  udp_rate;
  uint8_t  yamcs_rate;
  uint8_t  serial_in_rate;
  uint8_t  serial_out_rate;
  uint8_t  fs_rate;
  uint8_t  sd_json_rate;
  uint8_t  sd_ccsds_rate;
  uint8_t  sd_image_rate;
  uint8_t  camera_image_rate;
  uint8_t  yamcs_buffer;
  uint8_t  serial_buffer;
  uint16_t mem_free;
  uint16_t fs_free;
  uint16_t sd_free;
  bool     wifi_enabled:1;          // 7
  bool     wifi_udp_enabled:1;      //  6
  bool     wifi_yamcs_enabled:1;    //   5
  bool     wifi_image_enabled:1;    //    4
  bool     camera_enabled:1;        //     3   
  bool     time_set:1;              //      2
  bool     free_11:1;               //       1 - free to assign
  bool     free_10:1;               //        0 - free to assign
  bool     fs_enabled:1;            // 7 
  bool     fs_ftp_enabled:1;        //  6      
  bool     sd_enabled:1;            //   5 
  bool     sd_image_enabled:1;      //    4
  bool     sd_json_enabled:1;       //     3
  bool     sd_ccsds_enabled:1;      //      2 
  bool     free_21:1;               //       1 - free to assign
  bool     free_20:1;               //        0 - free to assign  
  bool     serial_connected:1;      // 7
  bool     wifi_connected:1;        //  6 
  bool     warn_serial_connloss:1;  //   5        
  bool     warn_wifi_connloss:1;    //    4       
  bool     err_serial_dataloss:1;   //     3
  bool     err_yamcs_dataloss:1;    //      2
  bool     err_fs_dataloss:1;       //       1
  bool     err_sd_dataloss:1;       //        0           
  bool     camera_active:1;         // 7
  bool     fs_active:1;             //  6
  bool     sd_active:1;             //   5
  bool     ftp_active:1;            //    4
  bool     ota_enabled:1;            //     3
  bool     free_42:1;               //      2 - free to assign 
  bool     free_41:1;               //       1 - free to assign 
  bool     free_40:1;               //        0 - free to assign 
};

struct __attribute__ ((packed)) tm_camera_t { // APID: 46 (2e)
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

struct __attribute__ ((packed)) tm_gps_t { // APID: 47 (2f)
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
  float    latitude_zero;
  float    longitude_zero;
  float    altitude_zero;  
  int16_t  x;
  int16_t  y;
  int16_t  z;
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

struct __attribute__ ((packed)) tm_motion_t { // APID: 48 (30)
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
  int16_t  a_x_rocket_offset;
  int16_t  a_y_rocket_offset;
  int16_t  a_z_rocket_offset;
  int16_t  g_x_rocket_offset;
  int16_t  g_y_rocket_offset;
  int16_t  g_z_rocket_offset;
}; 

struct __attribute__ ((packed)) tm_pressure_t { // APID: 49 (31)
  uint32_t millis:24;
  uint16_t packet_ctr;
  float    pressure;
  float    zero_level_pressure;
  float    altitude;
  float    velocity_v;
  float    temperature;
}; 

struct __attribute__ ((packed)) tm_radio_t { // APID: 50 (32)
  uint32_t millis:24;
  uint16_t packet_ctr;
  uint8_t  opsmode:4;               // 4
  bool     pressure_active:1;       //  3
  bool     motion_active:1;         //   2
  bool     gps_active:1;            //    1
  bool     camera_active:1;         //     0
  uint8_t  error_ctr;
  uint8_t  warning_ctr;
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
  bool     esp32_serial_connected:1;        // 7
  bool     esp32_wifi_connected:1;          //  6 
  bool     esp32_warn_serial_connloss:1;    //   5        
  bool     esp32_warn_wifi_connloss:1;      //    4       
  bool     esp32_err_serial_dataloss:1;     //     3
  bool     esp32_err_yamcs_dataloss:1;      //      2
  bool     esp32_err_fs_dataloss:1;         //       1  
  bool     separation_sts:1;                //        0
  bool     esp32cam_serial_connected:1;     // 7
  bool     esp32cam_wifi_connected:1;       //  6
  bool     esp32cam_warn_serial_connloss:1; //   5        
  bool     esp32cam_warn_wifi_connloss:1;   //    4       
  bool     esp32cam_err_serial_dataloss:1;  //     3
  bool     esp32cam_err_yamcs_dataloss:1;   //      2
  bool     esp32cam_err_fs_dataloss:1;      //       1
  bool     esp32cam_err_sd_dataloss:1;      //        0
}; 

struct __attribute__ ((packed)) tm_timer_t { // APID: 51 (33)
  uint32_t  millis:24;
  uint16_t  packet_ctr;
  uint16_t  radio_duration;
  uint16_t  pressure_duration;
  uint16_t  motion_duration;
  uint16_t  gps_duration;
  uint16_t  esp32cam_duration;
  uint16_t  serial_duration;
  uint16_t  ota_duration;
  uint16_t  ftp_duration;
  uint16_t  wifi_duration;
  uint16_t  tc_duration;
  uint16_t  idle_duration;
  uint16_t  publish_fs_duration;
  uint16_t  publish_serial_duration;
  uint16_t  publish_yamcs_duration;
  uint16_t  publish_udp_duration;
};

struct __attribute__ ((packed)) var_timer_t {
  uint32_t next_second;
  uint32_t next_radio_time;
  uint32_t next_pressure_time;
  uint32_t next_motion_time;
  uint32_t next_gps_time;
  uint32_t next_wifi_time;
  uint32_t next_ntp_time;
  uint32_t last_serial_in_millis;
  uint32_t last_serial_out_millis;
  uint16_t radio_interval;
  uint16_t pressure_interval;
  uint16_t motion_interval;
  uint16_t gps_interval;
  bool     do_radio:1;
  bool     do_pressure:1;
  bool     do_motion:1;
  bool     do_gps:1; 
  bool     do_time:1;
  bool     do_wifi:1;
  bool     do_ntp:1;
}; 

struct __attribute__ ((packed)) tc_esp32_t { // APID: 52 (34)
  uint8_t  cmd_id;
  uint8_t  int_parameter;
  char     str_parameter[JSON_MAX_SIZE];
  uint16_t str_parameter_size;
}; 

struct __attribute__ ((packed)) tc_esp32cam_t { // APID: 53 (35)
  uint8_t  cmd_id;
  uint8_t  int_parameter;
  char     str_parameter[JSON_MAX_SIZE];
  uint16_t str_parameter_size;
}; 

struct __attribute__ ((packed)) config_network_t {
  char     wifi_ssid[20]; 
  char     wifi_password[20];
  char     ap_ssid[20]; 
  char     ap_password[20];
  char     ftp_user[20];
  char     ftp_password[20];
  char     udp_server[20];
  char     yamcs_server[20]; 
  char     ntp_server[20];
  uint16_t udp_port; 
  uint16_t yamcs_tm_port;
  uint16_t yamcs_tc_port;
};

struct __attribute__ ((packed)) config_esp32_t { 
  uint8_t  radio_rate;              // Hz (valid: 2 or less)
  uint8_t  pressure_rate;           // Hz (up to 157 Hz, highest resolution up to 23 Hz); reached 176 Hz on ESP8266
  uint8_t  motion_rate;             // Hz (up to 400)
  uint8_t  gps_rate;                // Hz (valid: 1,5,10,16)
  char     config_file[20];
  char     routing_file[20];
  bool     radio_enable:1;          
  bool     pressure_enable:1;       
  bool     motion_enable:1;         
  bool     gps_enable:1;            
  bool     camera_enable:1;         
  bool     wifi_enable:1;           
  bool     wifi_sta_enable:1;       
  bool     wifi_ap_enable:1;  
  bool     wifi_udp_enable:1;       
  bool     wifi_yamcs_enable:1;     
  bool     fs_enable:1;
  bool     serial_format:1;         
  bool     debug_over_serial:1;     
  bool     ota_enable:1;
};

struct __attribute__ ((packed)) var_esp32_t { 
  char     today_dir[20];
  uint16_t fs_last_packet[NUMBER_OF_PID];
  uint16_t serial_last_packet[NUMBER_OF_PID];
  uint16_t yamcs_last_packet[NUMBER_OF_PID];
};

struct __attribute__ ((packed)) config_esp32cam_t {
  char     config_file[20];
  char     routing_file[20];
  bool     wifi_enable:1;           
  bool     wifi_sta_enable:1;       
  bool     wifi_ap_enable:1;     
  bool     wifi_udp_enable:1;       
  bool     wifi_yamcs_enable:1;     
  bool     wifi_image_enable:1;
  bool     fs_enable:1;
  bool     sd_enable:1;               
  bool     sd_json_enable:1;        
  bool     sd_ccsds_enable:1;       
  bool     sd_image_enable:1;  
  bool     serial_format:1;         
  bool     debug_over_serial:1;
  bool     ota_enable:1;
};

struct __attribute__ ((packed)) var_esp32cam_t { 
  char     today_dir[20];
  uint16_t fs_last_packet[NUMBER_OF_PID];
  uint16_t serial_last_packet[NUMBER_OF_PID];
  uint16_t yamcs_last_packet[NUMBER_OF_PID];
  uint16_t sd_ccsds_last_packet[NUMBER_OF_PID];
};

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
extern tc_esp32_t         tc_esp32;
extern tc_esp32cam_t      tc_esp32cam;

extern var_timer_t        var_timer;

extern config_network_t   config_network;
extern config_esp32_t     config_esp32;
extern config_esp32cam_t  config_esp32cam;

#ifdef PLATFORM_ESP32
extern tm_esp32_t*        tm_this;
extern tm_esp32cam_t*     tm_other;
extern sts_esp32_t*       sts_this;
extern sts_esp32cam_t*    sts_other;
extern tc_esp32_t*        tc_this;
extern tc_esp32cam_t*     tc_other;
extern config_esp32_t*    config_this;
#endif
#ifdef PLATFORM_ESP32CAM
extern tm_esp32cam_t*     tm_this;
extern tm_esp32_t*        tm_other;
extern sts_esp32cam_t*    sts_this;
extern sts_esp32_t*       sts_other;
extern tc_esp32cam_t*     tc_this;
extern tc_esp32_t*        tc_other;
extern config_esp32cam_t* config_this;
#endif

extern char buffer[JSON_MAX_SIZE];
#ifdef PLATFORM_ESP32
extern char radio_buffer[2*(sizeof(tm_radio_t)+2)];
#endif 

// FS FUNCTIONALITY
extern bool fs_setup ();
extern bool ftp_setup ();
extern bool ftp_check ();
extern uint16_t fs_free ();
#ifdef PLATFORM_ESP32CAM
extern uint16_t sd_free ();
#endif

// CONFIGURATION FUNCTIONALITY
extern void load_default_config ();
extern bool fs_load_settings ();
extern bool fs_load_config (const char* filename);
extern bool fs_load_routing (const char* filename);
extern String set_routing (char* routing_table, const char* routing_string);
extern bool set_parameter (const char* parameter, const char* value);

// WIFI FUNCTIONALITY
extern bool wifi_setup ();
extern bool wifi_ap_setup ();
extern bool wifi_sta_setup ();
extern bool wifi_check ();
extern bool time_check ();

// TM/TC FUNCTIONALITY
extern void publish_packet (uint8_t PID);
extern void publish_event (uint8_t PID, uint8_t subsystem, uint8_t event_type, const char* event_message);
extern bool publish_fs (uint8_t PID, uint16_t payload_ctr, ccsds_t* ccsds_ptr, uint16_t ccsds_len);
extern bool publish_serial (uint8_t PID, uint16_t payload_ctr, ccsds_t* ccsds_ptr, uint16_t ccsds_len);
extern bool publish_yamcs (uint8_t PID, uint16_t payload_ctr, ccsds_t* ccsds_ptr, uint16_t ccsds_len);
extern bool publish_udp (uint8_t PID, uint16_t payload_ctr, ccsds_t* ccsds_ptr, uint16_t ccsds_len);
extern bool publish_udp_text (const char* message);
#ifdef PLATFORM_ESP32CAM
extern bool publish_sd_ccsds (uint8_t PID, uint16_t payload_ctr, ccsds_t* ccsds_ptr, uint16_t ccsds_len);
extern bool publish_sd_json (uint8_t PID, uint16_t payload_ctr, ccsds_t* ccsds_ptr, uint16_t ccsds_len);
#endif
extern bool yamcs_tc_setup ();
#ifndef ASYNCUDP
extern bool yamcs_tc_check ();
#endif
extern uint16_t update_packet (uint8_t PID);
extern void reset_packet (uint8_t PID);

// CCSDS FUNCTIONALITY
extern bool valid_ccsds_hdr (ccsds_t* ccsds_ptr, bool pkt_type);
extern uint16_t get_ccsds_apid (ccsds_t* ccsds_ptr);
extern uint16_t get_ccsds_packet_ctr (ccsds_t* ccsds_ptr);
extern uint16_t get_ccsds_len (ccsds_t* ccsds_ptr);
extern uint32_t get_ccsds_millis (ccsds_t* ccsds_ptr);
extern void update_ccsds_hdr (uint8_t PID, bool pkt_type, uint16_t pkt_len);
extern void parse_ccsds (ccsds_t* ccsds_ptr);

// JSON FUNCTIONALITY
extern void build_json_str (char* json_buffer, uint8_t PID, ccsds_t* ccsds_ptr);
extern bool parse_json (const char* json_string);

// RADIO FUNCTIONALITY
#ifdef PLATFORM_ESP32
extern bool radio_setup ();
extern void publish_radio ();
#endif

// SERIAL FUNCTIONALITY
extern uint16_t serial_check ();
extern void serial_parse ();

// COMMANDS
extern bool cmd_reboot (uint8_t subsystem);
extern bool cmd_set_opsmode (uint8_t opsmode);
extern bool cmd_load_config (const char* filename);
extern bool cmd_load_routing (const char* filename);
extern bool cmd_set_parameter (const char* parameter, const char* value);
extern bool cmd_toggle_routing (uint8_t PID, const char interface);

// SUPPORT FUNCTIONS
extern uint8_t id_of (const char* string, uint8_t string_len, const char* array_of_strings, uint16_t array_len);
extern String get_hex_str (char* blob, uint16_t length);
extern void hex_to_bin (byte* destination, char* hex_input);

#endif // _FLI3D_H_
