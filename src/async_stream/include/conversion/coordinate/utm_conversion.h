#pragma once

#include <cstdint>
#include <cmath>
#include <limits>
#include <angles/angles.h>

/** Convert WGS 84 geodetic point to UTM point.
 *
 *  Equations from USGS Bulletin 1532
 *
 *  @param from WGS 84 point message.
 *  @param to UTM point.
 */

// WGS84 Parameters
#define WGS84_A   6378137.0   // major axis
#define WGS84_B   6356752.31424518  // minor axis
#define WGS84_F   0.0033528107    // ellipsoid flattening
#define WGS84_E   0.0818191908    // first eccentricity
#define WGS84_EP  0.0820944379    // second eccentricity

// UTM Parameters
#define UTM_K0    0.9996      // scale factor
#define UTM_FE    500000.0    // false easting
#define UTM_FN_N  0.0           // false northing, northern hemisphere
#define UTM_FN_S  10000000.0    // false northing, southern hemisphere
#define UTM_E2    (WGS84_E*WGS84_E) // e^2
#define UTM_E4    (UTM_E2*UTM_E2)   // e^4
#define UTM_E6    (UTM_E4*UTM_E2)   // e^6
#define UTM_EP2   (UTM_E2/(1-UTM_E2)) // e'^2

class GeoPoint{
public:
  GeoPoint():
    latitude(0.0),
    longitude(0.0),
    altitude(0.0) {}

  double latitude;
  double longitude;
  double altitude;
};

class UTMPoint{
public:

  UTMPoint():
    easting(0.0),
    northing(0.0),
    altitude(std::numeric_limits<double>::quiet_NaN()),
    zone(0),
    band(' ') {}

  double easting;           ///< easting within grid zone [meters]
  double northing;          ///< northing within grid zone [meters] 
  double altitude;          ///< altitude above ellipsoid [meters] or NaN
  uint8_t zone;             ///< UTM longitude zone number
  char   band; 
};

class UTM{
public:
	char UTMBand(double Lat, double Lon);
	void WGS84ToUTM(const GeoPoint &from, UTMPoint &to);
};