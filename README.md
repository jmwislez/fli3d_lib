# fli3d_lib

Install these files as libraries in the Arduino IDE by copying them to a subdirectory of ```libraries/```.

The following other libraries need to be installed:

- via Arduino IDE "Manage libraries...":
  - ArduinoJson
  - ArduinoOTA
- manually installed in ```libraries/```:
  - NTPClient (https://github.com/taranais/NTPClient)
    This version of NTPClient contains getFormattedDate
  - LinkedList (https://github.com/ivanseidel/LinkedList)
  - RadioHead (https://github.com/adafruit/RadioHead)
    In ```RH_AK.cpp```, change
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
  - I2Cdev (Arduino/I2Cdev/ of https://github.com/jrowberg/i2cdevlib)
  - MPU6050 (Arduino/MPU6050/ of https://github.com/jrowberg/i2cdevlib)
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
    //#define GPS_FIX_HEADING
    #define GPS_FIX_SATELLITES
    #define GPS_FIX_HDOP
    #define GPS_FIX_VDOP
    //#define GPS_FIX_PDOP
    #define GPS_FIX_LAT_ERR
    #define GPS_FIX_LON_ERR
    #define GPS_FIX_ALT_ERR
    #define GPS_FIX_SPD_ERR
    //#define GPS_FIX_HDG_ERR
    //#define GPS_FIX_TIME_ERR
    //#define GPS_FIX_GEOID_HEIGHT
    ```
