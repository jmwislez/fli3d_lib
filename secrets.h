/*
 * Fli3d - secrets
 * version: 2020-04-14 edited on esp32
 */

#ifdef PLATFORM_ESP32
const char     default_wifi_ssid[20]     = "wlan-27"; 
const char     default_wifi_password[20] = "VDhradCdrQ7upH6"; 
const char     default_ap_ssid[20]       = "fli3d_esp32"; 
const char     default_ap_password[20]   = "fli3d_backdoor"; 
const char     default_udp_server[20]    = "192.168.2.50";
const char     default_yamcs_server[20]  = "192.168.2.50"; 
const char     default_ntp_server[20]    = "ntp.telenet.be";
const uint16_t default_udp_port          = 4242; 
const uint16_t default_yamcs_tm_port     = 10042;
const uint16_t default_yamcs_tc_port     = 10052;
#endif
#ifdef PLATFORM_ESP32CAM
const char     default_wifi_ssid[20]     = "wlan-27"; 
const char     default_wifi_password[20] = "VDhradCdrQ7upH6"; 
const char     default_ap_ssid[20]       = "fli3d_esp32cam"; 
const char     default_ap_password[20]   = "fli3d_backdoor";
const char     default_udp_server[20]    = "192.168.2.50";
const char     default_yamcs_server[20]  = "192.168.2.50"; 
const char     default_ntp_server[20]    = "ntp.telenet.be";
const uint16_t default_udp_port          = 4243; 
const uint16_t default_yamcs_tm_port     = 10043; 
const uint16_t default_yamcs_tc_port     = 10053; 
#endif
