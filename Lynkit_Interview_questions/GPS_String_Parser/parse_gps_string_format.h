#ifndef __PARSE_GPS_STRING_FORMAT_H__
#define __PARSE_GPS_STRING_FORMAT_H__

//enum for GPGGA packet fields 
enum
{
    GPGGA_MESSAGE_ID = -1,
	GPGGA_UTC_POSITION_FIX=0,
    GPGGA_LATITUDE ,
    GPGGA_LATITUDE_DIRECTION,
	GPGGA_LONGITUDE,
    GPGGA_LONGITUDE_DIRECTION,
    GPGGA_GPS_QUALITY,
	GPGGA_NO_OF_SV,
    GPGGA_HDOP,
	GPGGA_ORTHOMETRIC_HEIGHT,
	GPGGA_ORTHOMETRIC_HEIGHT_UNIT,
	GPGGA_GEOID_SEPARATION,
	GPGGA_GEOID_SEPARATION_UNIT,
	GPGGA_DIFFERENTIAL_DATA_RECORD_AGE,
	GPGGA_REFERENCE_STATION_ID,
	GPGGA_CHECKSUM
};


//enum for GPRMC packet fields 
enum
{
    GPRMC_MESSAGE_ID = -1,
	GPRMC_UTC_POSITION_FIX=0,
	GPRMC_STATUS,
    GPRMC_LATITUDE ,
    GPRMC_LATITUDE_DIRECTION,
	GPRMC_LONGITUDE,
    GPRMC_LONGITUDE_DIRECTION,
    GPRMC_SPEED_OVER_GROUND,
	GPRMC_TRACK_ANGLE,
	GPRMC_DATE,
	GPRMC_MAGNETIC_VARIATION,
	GPGGA_CHECKSUM
};


#endif
