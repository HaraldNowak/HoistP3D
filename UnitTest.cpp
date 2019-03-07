#include <CppUnitTest.h>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

#include <RenderingTypes.h>
#include "vectorGeo.h"
#include "HoistP3DBase.h"
#include "ObjectSimCfgProps.h"

TEST_MODULE_INITIALIZE(ModuleInitialize)
{
	Logger::WriteMessage("In Module Initialize");
}

TEST_MODULE_CLEANUP(ModuleCleanup)
{
	Logger::WriteMessage("In Module Cleanup");
}

TEST_CLASS(TestClassName)
{
	double earthRadius = MetersToFeet(6371000); // in ft

public:
	TEST_METHOD(findSimCfgs)
	{
		//HoistP3DBase::find_file_recursive(const path & dir_path, string file_name, function<void(string)> func)
		AllocConsole();
		freopen("CON", "w", stdout);

		CacheObjectSimCfgProps objectSimProps;

		path dir_path("g:\\Prepar3DV4\\SimObjects\\Misc\\");

		HoistP3DBase::find_file_recursive(dir_path, "sim.cfg", [this, &objectSimProps](std::string path)
		{
			char achBuf[256];
			sprintf(achBuf,"Found sim.cfg under %s\n", path.c_str());
			Logger::WriteMessage(achBuf);
			ObjectSimCfgProps props = objectSimProps.getObjectSimCfg(path.c_str());

			Logger::WriteMessage(props.title.c_str());
		});

	}

	TEST_METHOD(TestMethodName)
	{
		// Run a function under test here.
		//Assert::AreEqual(expectedValue, actualValue, L"message", LINE_INFO());

		Assert::AreEqual(42, 42, L"42 == 42", LINE_INFO());
	}

	TEST_METHOD(testENUVectors)
	{
		DXYZ lonAltLat;

		/* Vienna
		Country	Austria
		Latitude	48.210033
		Longitude	16.363449
		DMS Lat	48° 12' 36.1188'' N
		DMS Long	16° 21' 48.4164'' E
		UTM Easting	601,292.59
		UTM Northing	5,340,543.63
		UTM Zone	33U
		Elevation (m)	188 m
		Elevation (f)	617 feet
		*/

		lonAltLat.dX = DegToRad(16.363449);
		lonAltLat.dY = MetersToFeet(188);
		lonAltLat.dZ = DegToRad(48.210033);

		double lonRef = DegToRad(16);
		double latRef = DegToRad(48);

		DXYZ enuXYZ = convertLonAltLat2XYZLocal(lonAltLat, earthRadius, lonRef, latRef); // rad_ft_rad, ECF

		DXYZ lonAltLatInverse = convertXYZLocal2LonAltLat(enuXYZ, earthRadius, lonRef, latRef); // rad_ft_rad, ECF

		Assert::AreEqual(lonAltLat.dX, lonAltLatInverse.dX, L"longitude after enu conversion and backconversion", LINE_INFO());
		Assert::AreEqual(lonAltLat.dY, lonAltLatInverse.dY, L"altitude after enu conversion and backconversion", LINE_INFO());
		Assert::AreEqual(lonAltLat.dZ, lonAltLatInverse.dZ, L"latitude after enu conversion and backconversion", LINE_INFO());
	}
};