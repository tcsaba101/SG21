/* OpenSprinkler Unified (AVR/RPI/BBB/LINUX) Firmware
 * Copyright (C) 2015 by Ray Wang (ray@opensprinkler.com)
 *
 * OpenSprinkler macro defines and hardware pin assignments
 * Feb 2015 @ OpenSprinkler.com
 *
 * This file is part of the OpenSprinkler library
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#ifndef _DEFINES_H
#define _DEFINES_H

#if !defined(ARDUINO) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284__)
  #define TMP_BUFFER_SIZE      	255   	// scratch buffer size
  #define POST_BUFFER_SIZE	 	1000 	// http POST message buffer
#else
  #define TMP_BUFFER_SIZE      128   // scratch buffer size
#endif

/** Firmware version, hardware version, and maximal values */
#define OS_FW_VERSION  217  // Firmware version: 217 means 2.1.7
                            // if this number is different from the one stored in non-volatile memory
                            // a device reset will be automatically triggered

#define OS_FW_MINOR      0  // Firmware minor version

// SMART GREEN version identifier
#define OS_SGHW_VERSION		20	// SB2 sensor attachment: ACS712 current, inputs: 2 soil, 1 flow, 1 rain + program switch
#define OS_SGFW_VERSION 	21	// SG Firmware version

#if defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284__)
/*
*/	#define SRDATA_PIN23_D7		//uncomment if SR_DATA relocated to MCUpin23 (LCD D7), comment when it is connected to MCUpin 21 (LCD D5)
	#define DEBUG_JTAG_ICE		//must use pin23 and should be uncommented for JTAG debugging, see more at line 390	
	#define SERIAL_DEBUG		//uncomment for debug prints see also at line 449
	
#endif


/** Hardware version base numbers */
#define OS_HW_VERSION_BASE   0x00
#define OSPI_HW_VERSION_BASE 0x40
#define OSBO_HW_VERSION_BASE 0x80
#define SIM_HW_VERSION_BASE  0xC0

/** Hardware type macro defines */
#define HW_TYPE_AC           0xAC   // standard 24VAC for 24VAC solenoids only, with triacs
#define HW_TYPE_DC           0xDC   // DC powered, for both DC and 24VAC solenoids, with boost converter and MOSFETs
#define HW_TYPE_LATCH        0x1A   // DC powered, for DC latching solenoids only, with boost converter and H-bridges
#define HW_TYPE_UNIV         0x15   // DC powered, universal driver

/** File names */
#define WEATHER_OPTS_FILENAME "wtopts.txt"    // weather options file
#define STATION_ATTR_FILENAME "stns.dat"      // station attributes data file
#define STATION_SPECIAL_DATA_SIZE  (TMP_BUFFER_SIZE - 8)
#define IFTTT_KEY_FILENAME "ifkey.txt"
#define IFTTT_KEY_MAXSIZE   128

#define FLOWCOUNT_RT_WINDOW   15    // flow count window (for computing real-time flow rate), 30 seconds

/** Station type macro defines */
#define STN_TYPE_STANDARD    0x00
#define STN_TYPE_RF          0x01
#define STN_TYPE_REMOTE      0x02
#define STN_TYPE_GPIO        0x03	// Support for raw connection of station to GPIO pin
#define STN_TYPE_HTTP        0x04	// Support for HTTP Get connection
#define STN_TYPE_OTHER       0xFF

#define IFTTT_PROGRAM_SCHED   0x01
#define IFTTT_RAINSENSOR      0x02
#define IFTTT_FLOWSENSOR      0x04
#define IFTTT_WEATHER_UPDATE  0x08
#define IFTTT_REBOOT          0x10
#define IFTTT_STATION_RUN     0x20

/** Sensor type macro defines */
#define SENSOR_TYPE_NONE    0x00
	// rainsensor/program switch input
//#define SENSOR_TYPE_RAIN   0x01  // rain sensor
#define SENSOR_TYPE_RAIN    0x02  // cheat rain sensor till UI can handle
#define SENSOR_TYPE_PSWITCH 0xF0  // program switch
	//flow sensor input
#define SENSOR_TYPE_FLOW    0x01  // flow sensor
	// soil sensor inputs
#define SENSOR_TYPE_SOILDIG		0x01  // soil sensor digital
#define SENSOR_TYPE_SOILAN1		0x02  // soil sensor analogue Vegetronix fitted
#define SENSOR_TYPE_SOILAN2		0x02  // soil sensor analogue Other linear
	
#define SENSOR_TYPE_OTHER   0xFF

/** Non-volatile memory (NVM) defines */
#if defined(ARDUINO)

/** 2KB NVM (ATmega644) data structure:
  * |         |     |  ---STRING PARAMETERS---      |           |   ----STATION ATTRIBUTES-----      |          |
  * | PROGRAM | CON | PWD | LOC | JURL | WURL | KEY | STN_NAMES | MAS | IGR | MAS2 | DIS | SEQ | SPE | OPTIONS  |
  * |  (986)  |(12) |(36) |(48) | (40) | (40) |(24) |   (768)   | (6) | (6) |  (6) | (6) | (6) | (6) |  (58)    |
  * |         |     |     |     |      |      |     |           |     |     |      |     |     |     |          |
  * 0        986  998   1034  1082  1122   1162   1186        1954  1960  1966   1972  1978  1984  1990      2048
  */

/** 4KB NVM (ATmega1284) data structure:
*******
********** in SGxx the CHANGED NVM structure see Tables_SG_20_vxx.xls file
******
  * |         |     |  ---STRING PARAMETERS---      |           |   ----STATION ATTRIBUTES-----      |          |
  * | PROGRAM | CON | PWD | LOC | JURL | WURL | KEY | STN_NAMES | MAS | IGR | MAS2 | DIS | SEQ | SPE | OPTIONS  |
  * |  (2433) |(12) |(36) |(48) | (48) | (48) |(24) |   (1344)  | (7) | (7) |  (7) | (7) | (7) | (7) |   (61)   |
  * |         |     |     |     |      |      |     |           |     |     |      |     |     |     |          |
  * 0       2433  2445   2481  2529  2577   2625   2649        3993  4000  4007   4014  4021  4028  4035      4096
  */

  #if defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284__) // for 4KB NVM
	//*****
	//due to the space needed for sensor data in nvm space the max no of stations are 48
	//and the max number of programs are 16
	//****
	#define MAX_EXT_BOARDS    5  // maximum number of exp. boards (each expands 8 stations)
    #define MAX_NUM_STATIONS  ((1+MAX_EXT_BOARDS)*8)  // maximum number of stations

    #define NVM_SIZE            4096  // For AVR, nvm data is stored in EEPROM, ATmega1284 has 4K EEPROM
    #define STATION_NAME_SIZE   24    // maximum number of characters in each station name

    #define MAX_PROGRAMDATA     2100  // program data
    #define MAX_NVCONDATA       12    // non-volatile controller data
    #define MAX_USER_PASSWORD   36    // user password
    #define MAX_LOCATION        48    // location string
    #define MAX_JAVASCRIPTURL   48    // javascript url
    #define MAX_WEATHERURL      48    // weather script url
    #define MAX_WEATHER_KEY     24    // weather api key
	#define MAX_CLOUDURL        48    // Cloud Server url

  #else

    #define MAX_EXT_BOARDS    5  // maximum number of exp. boards (each expands 8 stations)
    #define MAX_NUM_STATIONS  ((1+MAX_EXT_BOARDS)*8)  // maximum number of stations

    #define NVM_SIZE            2048  // For AVR, nvm data is stored in EEPROM, ATmega644 has 2K EEPROM
    #define STATION_NAME_SIZE   16    // maximum number of characters in each station name

    #define MAX_PROGRAMDATA     986   // program data
    #define MAX_NVCONDATA       12     // non-volatile controller data
    #define MAX_USER_PASSWORD   36    // user password
    #define MAX_LOCATION        48    // location string
    #define MAX_JAVASCRIPTURL   40    // javascript url
    #define MAX_WEATHERURL      40    // weather script url
    #define MAX_WEATHER_KEY     24    // weather api key,

  #endif

#else // NVM defines for RPI/BBB/LINUX

  // These are kept the same as AVR for compatibility reasons
  // But they can be increased if needed
  #define NVM_FILENAME        "nvm.dat" // for RPI/BBB, nvm data is stored in a file

  #define MAX_EXT_BOARDS    5  // maximum number of exp. boards (each expands 8 stations)
  #define MAX_NUM_STATIONS  ((1+MAX_EXT_BOARDS)*8)  // maximum number of stations

  #define NVM_SIZE            4096
  #define STATION_NAME_SIZE   24    // maximum number of characters in each station name

  #define MAX_PROGRAMDATA     2433  // program data
  #define MAX_NVCONDATA       12     // non-volatile controller data
  #define MAX_USER_PASSWORD   36    // user password
  #define MAX_LOCATION        48    // location string
  #define MAX_JAVASCRIPTURL   48    // javascript url
  #define MAX_WEATHERURL      48    // weather script url
  #define MAX_WEATHER_KEY     24    // weather api key

#endif  // end of NVM defines

/** NVM data addresses */
#define ADDR_NVM_PROGRAMS      (0)   // program starting address
#define ADDR_NVM_NVCONDATA     (ADDR_NVM_PROGRAMS+MAX_PROGRAMDATA)
#define ADDR_NVM_PASSWORD      (ADDR_NVM_NVCONDATA+MAX_NVCONDATA)
#define ADDR_NVM_LOCATION      (ADDR_NVM_PASSWORD+MAX_USER_PASSWORD)
#define ADDR_NVM_JAVASCRIPTURL (ADDR_NVM_LOCATION+MAX_LOCATION)
#define ADDR_NVM_WEATHERURL    (ADDR_NVM_JAVASCRIPTURL+MAX_JAVASCRIPTURL)
#define ADDR_NVM_CLOUDURL      (ADDR_NVM_JAVASCRIPTURL+MAX_CLOUDURL)
#define ADDR_NVM_WEATHER_KEY   (ADDR_NVM_WEATHERURL+MAX_WEATHERURL)
#define ADDR_NVM_STN_NAMES     (ADDR_NVM_WEATHER_KEY+MAX_WEATHER_KEY)
#define ADDR_NVM_MAS_OP        (ADDR_NVM_STN_NAMES+MAX_NUM_STATIONS*STATION_NAME_SIZE) // master op bits
#define ADDR_NVM_IGNRAIN       (ADDR_NVM_MAS_OP+(MAX_EXT_BOARDS+1))  // ignore rain bits
#define ADDR_NVM_MAS_OP_2      (ADDR_NVM_IGNRAIN+(MAX_EXT_BOARDS+1)) // master2 op bits
#define ADDR_NVM_STNDISABLE    (ADDR_NVM_MAS_OP_2+(MAX_EXT_BOARDS+1))// station disable bits
#define ADDR_NVM_STNSEQ        (ADDR_NVM_STNDISABLE+(MAX_EXT_BOARDS+1))// station sequential bits
#define ADDR_NVM_STNSPE        (ADDR_NVM_STNSEQ+(MAX_EXT_BOARDS+1)) // station special bits (i.e. non-standard stations)
// soil sensors data
#define ADDR_NVM_SSENSOR_1     (ADDR_NVM_STNSPE+(MAX_EXT_BOARDS+1))		// station attach soil_sensor_1 bits
#define ADDR_NVM_SSENSOR_2     (ADDR_NVM_SSENSOR_1+(MAX_EXT_BOARDS+1))	// station attach soil_sensor_2 bits
// flow 
#define ADDR_NVM_ST_EXCLUDE_AL (ADDR_NVM_SSENSOR_2+(MAX_EXT_BOARDS+1)) // station exclude of alarm handling
#define ADDR_NVM_ALARM_FATAL   (ADDR_NVM_ST_EXCLUDE_AL+(MAX_EXT_BOARDS+1)) // station disable in case of fatal alarm
#define ADDR_NVM_FLOW_REFS     (ADDR_NVM_ALARM_FATAL+(MAX_EXT_BOARDS+1)) // Station flow reference 2byte LSB: 0,125 gallon/minute
#define ADDR_NVM_CURR_REFS     (ADDR_NVM_FLOW_REFS+(MAX_NUM_STATIONS)*2) // station current reference 1 byte LSB: 4mA
#define ADDR_NVM_IMPULSES	   (ADDR_NVM_CURR_REFS+(MAX_NUM_STATIONS))  // 0:last_prog_impulses, 2:day_impulses, 4:last_day_impulses each 2 bytes
#define ADDR_NVM_OPTIONS       (ADDR_NVM_IMPULSES+6)  // options

/** Default password, location string, weather key, script urls */
#define DEFAULT_PASSWORD          "a6d82bced638de3def1e9bbb4983225c"  // md5 of 'opendoor'
#define DEFAULT_LOCATION          "Budapest"
#define DEFAULT_WEATHER_KEY       ""
#define DEFAULT_JAVASCRIPT_URL    "https://ui.opensprinkler.com/js"
#define DEFAULT_WEATHER_URL       "weather.opensprinkler.com"
#define DEFAULT_IFTTT_URL         "maker.ifttt.com"
#define DEFAULT_CLOUD_URL         "client.sandz.hu"   //development server address: "http://client.sandz.hu/sr"

/** Macro define of each option
  * Refer to OpenSprinkler.cpp for details on each option
  */
typedef enum {
OPTION_FW_VERSION = 0,
OPTION_TIMEZONE,
OPTION_USE_NTP,
OPTION_USE_DHCP,
OPTION_STATIC_IP1,
OPTION_STATIC_IP2,
OPTION_STATIC_IP3,
OPTION_STATIC_IP4,
OPTION_GATEWAY_IP1,
OPTION_GATEWAY_IP2,
OPTION_GATEWAY_IP3,
OPTION_GATEWAY_IP4,
OPTION_HTTPPORT_0,
OPTION_HTTPPORT_1,
OPTION_HW_VERSION,
OPTION_EXT_BOARDS,
OPTION_SEQUENTIAL_RETIRED,
OPTION_STATION_DELAY_TIME,
OPTION_MASTER_STATION,
OPTION_MASTER_ON_ADJ,
OPTION_MASTER_OFF_ADJ,
OPTION_RSENSOR_TYPE,
OPTION_RAINSENSOR_TYPE,
OPTION_WATER_PERCENTAGE,
OPTION_DEVICE_ENABLE,
OPTION_IGNORE_PASSWORD,
OPTION_DEVICE_ID,
OPTION_LCD_CONTRAST,
OPTION_LCD_BACKLIGHT,
OPTION_LCD_DIMMING,
OPTION_BOOST_TIME,
OPTION_USE_WEATHER,
OPTION_NTP_IP1,
OPTION_NTP_IP2,
OPTION_NTP_IP3,
OPTION_NTP_IP4,
OPTION_ENABLE_LOGGING,
OPTION_MASTER_STATION_2,
OPTION_MASTER_ON_ADJ_2,
OPTION_MASTER_OFF_ADJ_2,
OPTION_FW_MINOR,
OPTION_PULSE_RATE_0,
OPTION_PULSE_RATE_1,
OPTION_REMOTE_EXT_MODE,
OPTION_DNS_IP1,
OPTION_DNS_IP2,
OPTION_DNS_IP3,
OPTION_DNS_IP4,
OPTION_SPE_AUTO_REFRESH,
OPTION_IFTTT_ENABLE,
//new options in SGxx version see Tables_SG_20_vxx file for details
OPTION_FSENSOR_TYPE,
OPTION_FLOWUNIT_GAL,
OPTION_FLOW_ALARM,
OPTION_FLOW_ALARM_RANGE,
OPTION_FREEFLOW_QUANTITY,
OPTION_FREEFLOW_TIME,
OPTION_SSENSOR_1,
OPTION_SOILSENSOR1_TYPE,
OPTION_SSENSOR_2,
OPTION_SOILSENSOR2_TYPE,
OPTION_CURRENT,
OPTION_CURR_ALARM,
OPTION_CURR_RANGE,
OPTION_CAL_REQUEST,
OPTION_SEND_LOGFILES,
OPTION_FATAL_ALARM,
OPTION_SGHW_VERSION,
OPTION_SGFW_VERSION,
OPTION_CLIENT_MODE,
OPTION_CLOUDREFRESH_DEF,
OPTION_CLOUDREFRESH_FAST,
OPTION_STATUS_REPORT,
OPTION_FLASH_MEMORY,
OPTION_LCD_SIZE,
OPTION_LANGUAGE_LCD,
OPTION_RESET,
NUM_OPTIONS	// total number of options
} OS_OPTION_t;

/** Cloud Message Type */
#define SEND_CLOUD_OPTIONS		0
#define SEND_CLOUD_SETTINGS		1
#define SEND_CLOUD_PROGRAMS		2
#define SEND_CLOUD_STATIONS		3
#define SEND_CLOUD_STATUS_SPEC	4
#define SEND_CLOUD_LOG			5


/** Log Data Type */
//Classic
#define LOGDATA_STATION					0x00
#define LOGDATA_RAINSENSE				0x01
#define LOGDATA_RAINDELAY				0x02
#define LOGDATA_WATERLEVEL				0x03
#define LOGDATA_PROGFLOW				0x04  //was LOGDATA_FLOWSENSE

//Smart
#define LOGDATA_RAINSENSE2				0x05
#define LOGDATA_RAINDELAY2				0x06
#define LOGDATA_PROGFLOW2				0x07
#define LOGDATA_DAYFLOW					0x08
#define LOGDATA_CALIBRATED				0x09  // a zone has been calibrated

#define LOGDATA_FREEFLOW_END			0x0A  // freeflow ended

// soil log options
#define LOGDATA_SOIL1					0x30
#define LOGDATA_SOIL2					0x31
#define LOGDATA_SOIL1_PROG_CANCEL		0x32
#define LOGDATA_SOIL2_PROG_CANCEL		0x33
#define LOGDATA_SOIL1_STATION_CANCEL	0x34
#define LOGDATA_SOIL2_STATION_CANCEL	0x35

//flow and rain station cancel log options
#define LOGDATA_FATAL_STATION_CANCEL	0x36
#define LOGDATA_RAIN_STATION_CANCEL		0x37

// alert log data types
#define LOGDATA_ALARM_FLOW_STOPPED		0x10
#define LOGDATA_ALARM_FF_QUANTITY		0x11
#define LOGDATA_ALARM_FF_TIME			0x12
#define LOGDATA_ALARM_LEAKAGE_START		0x13
#define LOGDATA_ALARM_LEAKAGE_END		0x14
//flow alarms
#define LOGDATA_ALARM_FLOW_HIGH			0x15
#define LOGDATA_ALARM_FLOW_LOW			0x16
#define LOGDATA_ALARM_FATAL_FLOW		0x17
//station current alarms
#define LOGDATA_ALARM_CURRENT_HIGH		0x18
#define LOGDATA_ALARM_CURRENT_LOW		0x19

//Admin log
#define LOGDATA_FAILED_STATE			0x20
	
	
#undef OS_HW_VERSION

/** Hardware defines */
#if defined(ARDUINO)

  #if F_CPU==8000000L
    #define OS_HW_VERSION (OS_HW_VERSION_BASE+20)
  #elif F_CPU==12000000L
    #define OS_HW_VERSION (OS_HW_VERSION_BASE+21)
  #elif F_CPU==16000000L
    #if defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284__)
      #define OS_HW_VERSION (OS_HW_VERSION_BASE+23)
      #define PIN_FREE_LIST		{2,10,12,13,14,15,18,19}
    #else
      #define OS_HW_VERSION (OS_HW_VERSION_BASE+22)
    #endif
  #endif

  // hardware pins
  #define PIN_BUTTON_1      31    // button 1
  #define PIN_BUTTON_2      30    // button 2
  #define PIN_BUTTON_3      29    // button 3
  #define PIN_RF_DATA       28    // RF data pin
  #define PORT_RF        PORTA
  #define PINX_RF        PINA3
  #define PIN_SR_LATCH       3    // shift register latch pin
  #define PIN_SR_CLOCK      22    // shift register clock pin
  #define PIN_SR_OE          1    // shift register output enable pin

  // regular 16x2 LCD pin defines
  // JTAG development needs the pins: 18,19,20,21
  // On the PCB the SR_DATA have to cut and reconnect to pin 23 (LCD_D7) It will work normally with this definition
  // We have to change the LCD display to an I2C model to use JTAG development
  // just connect and the OS identify it automatically  

#if defined(DEBUG_JTAG_ICE)
  #define PIN_SR_DATA       23    // shift register data pin when JTAG-ICE is used, it has to be hacked on the pcb
  #define PIN_LCD_EN        22    // dummy definition on an unused pin
  #define PIN_LCD_RS        22    // dummy definition on an unused pin
  #define PIN_LCD_D4        22    // dummy definition on an unused pin
  #define PIN_LCD_D5        22    // dummy definition on an unused pin
  #define PIN_LCD_D6        22    // dummy definition on an unused pin
  #define PIN_LCD_D7        22    // dummy definition on an unused pin
#else
  // LCD and shift register pins normal operation
  #define PIN_SR_DATA       21    // shift register data pin normal operation
	
#if defined(SRDATA_PIN23_D7) 
  #define PIN_SR_DATA       23    // shift register data pin hacked
#endif
	
  #define PIN_LCD_EN        18    // LCD enable pin
  #define PIN_LCD_RS        19    // LCD rs pin
  #define PIN_LCD_D4        20    // LCD d4 pin
  #define PIN_LCD_D5        21    // LCD d5 pin
  #define PIN_LCD_D6        22    // LCD d6 pin
  #define PIN_LCD_D7        23    // LCD d7 pin
#endif
	
  #define PIN_LCD_BACKLIGHT 12    // LCD backlight pin
  #define PIN_LCD_CONTRAST  13    // LCD contrast pin

  // DC controller pin defines
  #define PIN_BOOST         20    // booster pin
  #define PIN_BOOST_EN      23    // boost voltage enable pin

  #define PIN_ETHER_CS       4    // Ethernet controller chip select pin
  #define PIN_SD_CS          0    // SD card chip select pin

  #define PIN_RAINSENSOR    2    // rain sensor/programswitch input is connected to pin D3/INT2
  #define PIN_FLOWSENSOR     11    // flow sensor (INT1)
  #define PIN_FLOWSENSOR_INT 1    // flow sensor interrupt pin (INT1)
  #define PIN_EXP_SENSE      4    // expansion board sensing pin (A4)
  #define PIN_CURR_SENSE     6    // current sensing pin (A6)
  #define PIN_CURR_DIGITAL  25    // digital pin index for A6
  #define PIN_SOILSENSOR_1	24	  // digital SoilSensor 1 (A7)
  #define PIN_SOILSENSOR_2	26	  // digital SoilSensor 2 (A5)
  // Ethernet buffer size
  #if defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284__)
    #define ETHER_BUFFER_SIZE   1400 // ATmega1284 has 16K RAM, so use a bigger buffer
  #else
    #define ETHER_BUFFER_SIZE   950  // ATmega644 has 4K RAM, so use a smaller buffer
  #endif

  #define 	wdt_reset()   __asm__ __volatile__ ("wdr")  // watchdog timer reset

  //#define SERIAL_DEBUG  it is relocated to line 53
  #if defined(SERIAL_DEBUG) /** Serial debug functions */

    #define DEBUG_BEGIN(x)   Serial.begin(x)
    #define DEBUG_PRINT(x)   Serial.print(x)
    #define DEBUG_PRINTLN(x) Serial.println(x)
    #define DEBUG_PRINTIP(x) ether.printIp("IP:",x)

  #else

    #define DEBUG_BEGIN(x)   {}
    #define DEBUG_PRINT(x)   {}
    #define DEBUG_PRINTLN(x) {}
    #define DEBUG_PRINTIP(x) {}

  #endif
  typedef unsigned char   uint8_t;
  typedef unsigned int    uint16_t;
  typedef int int16_t;

#else // Hardware defines for RPI/BBB

  /** OSPi pin defines */
  #if defined(OSPI)

  #define OS_HW_VERSION    OSPI_HW_VERSION_BASE
  #define PIN_SR_LATCH      22    // shift register latch pin
  #define PIN_SR_DATA       27    // shift register data pin
  #define PIN_SR_DATA_ALT   21    // shift register data pin (alternative, for RPi 1 rev. 1 boards)
  #define PIN_SR_CLOCK       4    // shift register clock pin
  #define PIN_SR_OE         17    // shift register output enable pin
  #define PIN_RAINSENSOR    14    // rain sensor
  #define PIN_FLOWSENSOR    14    // flow sensor (currently shared with rain sensor, change if using a different pin)
  #define PIN_RF_DATA       15    // RF transmitter pin
  #define PIN_BUTTON_1      23    // button 1
  #define PIN_BUTTON_2      24    // button 2
  #define PIN_BUTTON_3      25    // button 3

  #define PIN_FREE_LIST		{5,6,7,8,9,10,11,12,13,16,18,19,20,21,23,24,25,26}

  /** BBB pin defines */
  #elif defined(OSBO)

  #define OS_HW_VERSION    OSBO_HW_VERSION_BASE
  // these are gpio pin numbers, refer to
  // https://github.com/mkaczanowski/BeagleBoneBlack-GPIO/blob/master/GPIO/GPIOConst.cpp
  #define PIN_SR_LATCH      60    // P9_12, shift register latch pin
  #define PIN_SR_DATA       30    // P9_11, shift register data pin
  #define PIN_SR_CLOCK      31    // P9_13, shift register clock pin
  #define PIN_SR_OE         50    // P9_14, shift register output enable pin
  #define PIN_RAINSENSOR    48    // P9_15, rain sensor is connected to pin D3
  #define PIN_FLOWSENSOR    48    // flow sensor (currently shared with rain sensor, change if using a different pin)
  #define PIN_RF_DATA       51    // RF transmitter pin

  #else
    // For Linux or other software simulators
    // use fake hardware pins
    #if defined(DEMO)
      #define OS_HW_VERSION 255   // assign hardware number 255 to DEMO firmware
    #else
      #define OS_HW_VERSION SIM_HW_VERSION_BASE
    #endif
    #define PIN_SR_LATCH    0
    #define PIN_SR_DATA     0
    #define PIN_SR_CLOCK    0
    #define PIN_SR_OE       0
    #define PIN_RAINSENSOR  0
    #define PIN_FLOWSENSOR  0
    #define PIN_RF_DATA     0
	#define PIN_FREE_LIST	{}

  #endif

  #define ETHER_BUFFER_SIZE   16384

  #define DEBUG_BEGIN(x)          {}  /** Serial debug functions */
  #define ENABLE_DEBUG
  #if defined(ENABLE_DEBUG)
    inline  void DEBUG_PRINT(int x) {printf("%d", x);}
    inline  void DEBUG_PRINT(const char*s) {printf("%s", s);}
    #define DEBUG_PRINTLN(x)        {DEBUG_PRINT(x);printf("\n");}
  #else
    #define DEBUG_PRINT(x) {}
    #define DEBUG_PRINTLN(x) {}
  #endif

  inline void itoa(int v,char *s,int b)   {sprintf(s,"%d",v);}
  inline void ultoa(unsigned long v,char *s,int b) {sprintf(s,"%lu",v);}
  #define now()       time(0)

  /** Re-define avr-specific (e.g. PGM) types to use standard types */
  #define pgm_read_byte(x) *(x)
  #define PSTR(x)      x
  #define strcat_P     strcat
  #define strcpy_P     strcpy
  #define PROGMEM
  typedef const char* PGM_P;
  typedef unsigned char   uint8_t;
  typedef short           int16_t;
  typedef unsigned short  uint16_t;
  typedef bool boolean;

#endif  // end of Hardawre defines

/** Other defines */
// button values
#define BUTTON_1            0x01
#define BUTTON_2            0x02
#define BUTTON_3            0x04

// button status values
#define BUTTON_NONE         0x00  // no button pressed
#define BUTTON_MASK         0x0F  // button status mask
#define BUTTON_FLAG_HOLD    0x80  // long hold flag
#define BUTTON_FLAG_DOWN    0x40  // down flag
#define BUTTON_FLAG_UP      0x20  // up flag

// button timing values
#define BUTTON_DELAY_MS        1  // short delay (milliseconds)
#define BUTTON_HOLD_MS      1000  // long hold expiration time (milliseconds)

// button mode values
#define BUTTON_WAIT_NONE       0  // do not wait, return value immediately
#define BUTTON_WAIT_RELEASE    1  // wait until button is release
#define BUTTON_WAIT_HOLD       2  // wait until button hold time expires

#define DISPLAY_MSG_MS      2000  // message display time (milliseconds)

typedef unsigned char byte;
typedef unsigned long ulong;

#endif  // _DEFINES_H


