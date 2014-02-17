// Constants used in defining the layout of message structures used by the Davis
// instrument Vantage Pro2 / Vantage Vue weather station consoles
//
// This is part of the DavisRFM69 library from https://github.com/dekay/DavisRFM69
// (C) DeKay 2014 dekaymail@gmail.com
// and released under the MIT License (http://opensource.org/licenses/mit-license.php)

#ifndef DAVISDEF_h
#define DAVISDEF_h

// LOOP data packet length and structure.
// See http://www.davisnet.com/support/weather/download/VantageSerialProtocolDocs_v230.pdf
#define LOOP_PACKET_LENGTH 99
#define BAR_TREND 3
#define PACKET_TYPE 4
#define NEXT_RECORD_LSB 5
#define NEXT_RECORD_MSB 6
#define BAROMETER_LSB 7
#define BAROMETER_MSB 8
#define INSIDE_TEMPERATURE_LSB 9
#define INSIDE_TEMPERATURE_MSB 10
#define INSIDE_HUMIDITY 11
#define OUTSIDE_TEMPERATURE_LSB 12
#define OUTSIDE_TEMPERATURE_MSB 13
#define WIND_SPEED 14
#define TEN_MIN_AVG_WIND_SPEED 15
#define WIND_DIRECTION_LSB 16
#define WIND_DIRECTION_MSB 17
#define EXTRA_TEMPERATURES_BASE 18
#define SOIL_TEMPERATURE_BASE 25
#define LEAF_TEMPERATURE_BASE 29
#define OUTSIDE_HUMIDITY 33
#define EXTRA_HUMIDITIES_BASE 34
#define RAIN_RATE_LSB 41
#define RAIN_RATE_MSB 42
#define UV 43
#define SOLAR_RADIATION_LSB 44
#define SOLAR_RADIATION_MSB 45
#define STORM_RAIN_LSB 46
#define STORM_RAIN_MSB 47
#define START_DATE_OF_CURRENT_STORM_LSB 48
#define START_DATE_OF_CURRENT_STORM_MSB 49
#define DAY_RAIN_LSB 50
#define DAY_RAIN_MSB 51
#define MONTH_RAIN_LSB 52
#define MONTH_RAIN_MSB 53
#define YEAR_RAIN_LSB 54
#define YEAR_RAIN_MSB 55
#define DAY_ET_LSB 56
#define DAY_ET_MSB 57
#define MONTH_ET_LSB 58
#define MONTH_ET_MSB 59
#define YEAR_ET_LSB 60
#define YEAR_ET_MSB 61
#define SOIL_MOISTURES_BASE 62
#define LEAF_WETNESS_BASE 66
#define INSIDE_ALARMS 70
#define RAIN_ALARMS 71
#define OUTSIDE_ALARMS_LSB 72
#define OUTSIDE_ALARMS_MSB 73
#define EXTRA_TEMP_HUM_ALARMS_BASE 74
#define SOIL_LEAF_ALARMS_BASE 82
#define TRANSMITTER_BATTERY_STATUS 86
#define CONSOLE_BATTERY_VOLTAGE_LSB 87
#define CONSOLE_BATTERY_VOLTAGE_MSB 88
#define FORECAST_ICONS 89
#define FORECAST_RULE_NUMBER 90
#define TIME_OF_SUNRISE_LSB 91
#define TIME_OF_SUNRISE_MSB 92
#define TIME_OF_SUNSET_LSB 93
#define TIME_OF_SUNSET_MSB 94
#define CRC_MSB 97
#define CRC_LSB 98

// For the packet stats array used in response to the RXCHECK command
#define PACKET_STATS_LENGTH 5
#define PACKETS_RECEIVED 0
#define PACKETS_MISSED 1
#define NUM_RESYNCS 2
#define RECEIVED_STREAK 3
#define CRC_ERRORS 4

// For the HiLows array.
#define HI_LOWS_LENGTH 436

#endif  // DAVISDEF_h