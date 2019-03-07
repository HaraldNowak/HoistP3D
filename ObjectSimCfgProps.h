//#pragma once
#include <Types.h>
#include <Pdk.h>
#include <vector>
#include "ReadConfig.h"

using namespace P3D;

// As long as we have no information where to get this from, we read it ourselfes:
// [HoistPoint]
// position = -0.5, 0, 1.82; (feet)Longitudinal, lateral, vertical position in
class ObjectSimCfgProps {
public:
	std::string title;

	DXYZ hoistAttachLoad ;

	ObjectSimCfgProps() {
		hoistAttachLoad = { 0, 0, 0 };
	}
};

class CacheObjectSimCfgProps
{
	std::map<std::string, ObjectSimCfgProps> cachedSimCfgs;
public:
	std::vector<ObjectSimCfgProps *> getAllObjectSimCfg()
	{
		std::vector<ObjectSimCfgProps *> rc;
		for (std::map<std::string, ObjectSimCfgProps>::iterator itr = cachedSimCfgs.begin(); itr != cachedSimCfgs.end(); ++itr) {
			rc.push_back(&itr->second);
		}
		return rc;
	}

	ObjectSimCfgProps getObjectSimCfg(const char *pszPath)
	{
		std::string path(pszPath);
		if (cachedSimCfgs.find(path) != cachedSimCfgs.end())
		{
			return cachedSimCfgs[path];
		}
		else
		{
			ReadConfig configReaders(true); // wide char files!
			bool inHoistPointSection = false;

			configReaders.addEntry("title", [this,path](char *pszConfig)
			{
				printf("title section.\n");
				char achTitle[256];
				sscanf(pszConfig, "%s", achTitle);
				cachedSimCfgs[path].title = achTitle;
			});

			configReaders.addEntry("[HoistPoint]", [&inHoistPointSection](char *pszConfig)
			{
				printf("hoistpoint section.\n");
				inHoistPointSection = true;
			});
			configReaders.addEntry("position", [path, this, &inHoistPointSection](char *pszConfig)
			{
				//if (inHoistPointSection) {
				printf("hoistpoint ha:%s %d\n", pszConfig, (int)inHoistPointSection);
				DXYZ &hoistAttachLoad = cachedSimCfgs[path].hoistAttachLoad;
				sscanf(pszConfig, "%lf, %lf, %lf", &hoistAttachLoad.dX, &hoistAttachLoad.dY, &hoistAttachLoad.dZ);
				//}
			});

			configReaders.readFile(pszPath);
			if (cachedSimCfgs.find(path) != cachedSimCfgs.end()) {
				return cachedSimCfgs[path];
			}
			else {
				printf("Could not get hoistAttachLoad - returning empty one!\n");
				return cachedSimCfgs[path];
			}
		}
	}
};
