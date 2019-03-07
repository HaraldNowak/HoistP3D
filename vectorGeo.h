#ifndef VECTORGEO_H_
#define VECTORGEO_H_

#include <Types.h>
#include <Pdk.h>
#include <math.h>
//#include <RenderingTypes.h>

using namespace P3D;

#define PI                  3.141592653589793238462643
#define PI_OVER_2           (PI / 2)
#define TWO_PI              (2*PI)
#define RadToDeg(rad)       ((rad) * (360/TWO_PI))
#define DegToRad(deg)       ((deg) * (TWO_PI/360))

#define FeetToMeters(feet)  ((feet)*(0.3048))
#define MetersToFeet(feet)  ((feet)/(0.3048))

inline double calcDist(DXYZ vXYZ, DXYZ endPosXYZ)
{
	double dx = vXYZ.dX - endPosXYZ.dX;
	double dy = vXYZ.dY - endPosXYZ.dY;
	double dz = vXYZ.dZ - endPosXYZ.dZ;

	return sqrt(dx*dx + dy*dy + dz*dz);
}

inline DXYZ convertLonAltLat2XYZ(DXYZ lonAltLat, double earthRadius) // rad_ft_rad, ECF
{
	double r = earthRadius + lonAltLat.dY;
	double lon = lonAltLat.dX;
	double lat = lonAltLat.dZ;

	DXYZ rc;
	rc.dX = r * cos(lat) * cos(lon);
	rc.dY = r * cos(lat) * sin(lon);
	rc.dZ = r * sin(lat);

	return rc;
}

inline DXYZ convertLonAltLat2XYZ(LLADegreesMeters lonAltLat, double earthRadius) // rad_ft_rad, ECF
{
	DXYZ xyz;
	xyz.dX = DegToRad(lonAltLat.Longitude);
	xyz.dY = MetersToFeet(lonAltLat.Altitude);
	xyz.dZ = DegToRad(lonAltLat.Latitude);

	return convertLonAltLat2XYZ(xyz, earthRadius);
}

// Convert to local ENU system (tangential plane to earth at given lat/lon)

inline DXYZ convertLonAltLat2XYZLocal(DXYZ lonAltLat, double earthRadius, double refLon, double refLat) // rad_ft_rad, locally aligned: x: north/south, y:alt, z: east/west - approximated
{
	double r = earthRadius + lonAltLat.dY;
	double lon = lonAltLat.dX;
	double lat = lonAltLat.dZ;
	double dlon = lon - refLon ;
	double dlat = lat - refLat ;

	DXYZ rc;
	rc.dX = r*cos(lat) * dlon;
	rc.dY = lonAltLat.dY;
	rc.dZ = r * dlat;

	return rc;
}

inline DXYZ convertLonAltLat2XYZLocal(LLADegreesMeters lonAltLat, double earthRadius, double refLon, double refLat) // rad_ft_rad, locally aligned
{
	DXYZ xyz;
	xyz.dX = DegToRad(lonAltLat.Longitude);
	xyz.dY = MetersToFeet(lonAltLat.Altitude);
	xyz.dZ = DegToRad(lonAltLat.Latitude);

	return convertLonAltLat2XYZLocal(xyz, earthRadius, refLon, refLat);
}

inline DXYZ convertXYZLocal2LonAltLat(DXYZ xyz, double earthRadius, double refLon, double refLat) // rad_ft_rad, locally aligned: x: north/south, y:alt, z: east/west - approximated
{
	double r = earthRadius + xyz.dY;

	DXYZ lonAltLat;
	
	lonAltLat.dY = xyz.dY;
	lonAltLat.dZ = xyz.dZ / r + refLat ;

	lonAltLat.dX = xyz.dX / (r*cos(lonAltLat.dZ)) + refLon;

	return lonAltLat;
}

inline DXYZ vectorSubtract(DXYZ v1, DXYZ v2)
{
	DXYZ rc;

	rc.dX = v2.dX - v1.dX;
	rc.dY = v2.dY - v1.dY;
	rc.dZ = v2.dZ - v1.dZ;

	return rc;
}

inline double getVectorLen(DXYZ v)
{
	return sqrt(v.dX*v.dX + v.dY*v.dY + v.dZ*v.dZ);
}

#endif // VECTORGEO_H_