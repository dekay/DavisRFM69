/*
 this file documents the davis ISS protocol messages used on rf inbetween a sensor suite and the base unit indoors

content copied over from https://github.com/dekay/DavisRFM69/wiki/Message-Protocol
*/

#ifndef DAVISMESSAGE
#define DAVISMESSAGE

//byte0 message type and status
//byte0 high nibble
#define cap_volt 0x20 // vue only
#define unknown0 0x30 // ??
#define uv_index 0x40
#define rainrate 0x50
#define solarrad 0x60
#define sol_volt 0x70 // vue only
#define temp 0x80
#define windgust 0x90
#define humi 0xA0
#define rain 0xE0

//byte0 low nibble
#define batlow 0x08
#define tr_id 0x07
//byte1 wind speed in miles
//byte2 wind direction
//byte3-5 data
//byte6+7 16bit crc


//data from byte3-5:
//cap_volt: (vue only)
//voltage = ((Byte3 * 4) + ((Byte4 && 0xC0) / 64)) / 100
//uv_index:
//UVIndex = ((Byte3 << 8) + Byte4) >> 6) / 50.0
// if byte3==0xff then no_sensor
//rainrate:
//no rain     if Byte3 == 0xFF
//solarrad:
//Solar radiation = (((Byte3 << 8) + Byte4) >> 6) * 1.757936
// if byte3==0xff then no_sensor
//temp:
//tempF = ((Byte3 * 256 + Byte4) / 160
//windgust:
//gust = Byte3
//gust_index = Byte5 >> 4
//humi:
//humidity = (((Byte4 >> 4) << 8) + Byte3) / 10.0
//rain:
//byte3=buckettips
//overflow at 127


#endif
