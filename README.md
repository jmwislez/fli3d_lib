# fli3d_lib

## Install this library
Install these files as a library in the Arduino IDE by going into ```Arduino\libraries```, and doing ```git clone https://github.com/jmwislez/fli3d_lib```.
Note that ESP32 core 1.0.6 was used (no success yet on 2.0.4).

## Install other dependencies

The following other libraries need to be installed:

- via Arduino IDE "Manage libraries...":
  - ArduinoJson
  - LittleFS_esp32
  - SerialTransfer
  - ESPdht (optional, for extra thermal sensor in weather balloon version)
- via ```git clone``` in ```Arduino/libraries/```:
  - NTPClient (https://github.com/taranais/NTPClient)
    This version of NTPClient contains getFormattedDate
  - LinkedList (https://github.com/ivanseidel/LinkedList)
  - RadioHead (https://github.com/adafruit/RadioHead)
    In ```RH_ASK.cpp```, change
    ```c++
    #if (RH_PLATFORM == RH_PLATFORM_ESP8266)
    // interrupt handler and related code must be in RAM on ESP8266,
    // according to issue #46.
    #define INTERRUPT_ATTR ICACHE_RAM_ATTR
    #else
    #define INTERRUPT_ATTR
    #endif
    ```
    to 
    ```c++
    #if (RH_PLATFORM == RH_PLATFORM_ESP8266 or RH_PLATFORM == RH_PLATFORM_ESP32)
    // interrupt handler and related code must be in RAM on ESP8266 or ESP32, 
    // according to issue #46.
    #define INTERRUPT_ATTR ICACHE_RAM_ATTR
    #else
    #define INTERRUPT_ATTR
    #endif
    ```
  - I2Cdev (subdirectory ```Arduino/I2Cdev/``` of https://github.com/jrowberg/i2cdevlib)
  - MPU6050 (subdirectory ```Arduino/MPU6050/``` of https://github.com/jrowberg/i2cdevlib)
  - MPU9250 (subdirectory ```Arduino/MPU9250/``` of https://github.com/jrowberg/i2cdevlib) - instead of MPU6050 in weather balloon version
  - NeoGPS (https://github.com/SlashDevin/NeoGPS)
    In ```src/GPSfix_cfg.h```, ensure the following configuration:
    ```c++
    #define GPS_FIX_DATE
    #define GPS_FIX_TIME
    #define GPS_FIX_LOCATION
    //#define GPS_FIX_LOCATION_DMS
    #define GPS_FIX_ALTITUDE
    #define GPS_FIX_SPEED
    #define GPS_FIX_VELNED
    #define GPS_FIX_HEADING
    #define GPS_FIX_SATELLITES
    #define GPS_FIX_HDOP
    #define GPS_FIX_VDOP
    #define GPS_FIX_PDOP
    #define GPS_FIX_LAT_ERR
    #define GPS_FIX_LON_ERR
    #define GPS_FIX_ALT_ERR
    //#define GPS_FIX_SPD_ERR
    //#define GPS_FIX_HDG_ERR
    //#define GPS_FIX_TIME_ERR
    //#define GPS_FIX_GEOID_HEIGHT
    ```
    In ```NMEAGPS_cfg.h```, ensure the following configuration:
    ```c++
    #define NMEAGPS_PARSE_GGA
    #define NMEAGPS_PARSE_GLL
    #define NMEAGPS_PARSE_GSA
    #define NMEAGPS_PARSE_GSV
    #define NMEAGPS_PARSE_GST
    #define NMEAGPS_PARSE_RMC
    #define NMEAGPS_PARSE_VTG
    #define NMEAGPS_PARSE_ZDA
    
    #define LAST_SENTENCE_IN_INTERVAL NMEAGPS::NMEA_ZDA
    
    #define NMEAGPS_PARSING_SCRATCHPAD
    ```
  - ESPFtpServer (https://github.com/jmwislez/ESPFtpServer)
  
## Configure library

Create ```secrets.h``` based on ```secrets.h.example```.

## Use fli3d_lib

This library is needed to compile https://github.com/jmwislez/fli3d_ESP32.git and https://github.com/jmwislez/fli3d_ESP32cam.git.
