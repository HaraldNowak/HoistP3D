// Copyright (c) 2019 Harald Nowak. All rights reserved.
// HoistP3D
// Description: This example uses the PDK to render an alternative P3D Hoist

#define  _CRT_SECURE_NO_WARNINGS

#include "initpdk.h"
#include <WinError.h>
#include "PdkPlugin.h"
#include <map>
#include <functional>
#include <ctime>
#include <filesystem>
#include <set>

#include "ObjectSimCfgProps.h"

using namespace std;

#include <string>

#include "vectorGeo.h"
#include "HoistP3DBase.h"

using namespace P3D;

class HoistP3DPlugin : public PdkPlugin, HoistP3DBase, IForceElement
{
	bool m_bEnabled = false;

	double lengthFt = 5;
	double maxLengthFt = 100;
	double minLengthFt = 5;
	double hoistSpeed = 1; // in feet per second
	clock_t lastTickHoistMove = 0;

	bool bCaptVis = false;

	float pickupradius = 1;

	DXYZ hoistAttach = { 0, 0, 0 };
	DXYZ hoistAttachLoad = { 0, 0, 0 };
	
	double massLoad = 200;
	double bankMomentOfInertia = 110000;
	double yawMomentOfInertia = 110000;
	double pitchMomentOfInertia = 250000;

	float fRopeMass = 0.2;
	float fStaticCGHeight = 5;

	float fElevation = 0;
	float fSlingEndRadarAltitude = 0; 

	double lastRopeForce = 0; // to calculate 2st derivative
	double lastRopeLen = 0 ;
	double kRope = 20 ;
	double diffRopeLen = 0;
	double damping = 1;
	double speedforce = 1;

	ObjectWorldTransform objTrans ; // current helicopter pos
	P3DDXYZ endPos; // dynamic - end of open sling
	ObjectWorldTransform worldObjectPlacement; // dynamic - pos of last object placement

	bool m_bDrawUserSimObjects;
	bool m_bHoistUp;
	bool m_bHoistDown;
	bool m_bForcePhysics = false ;
	bool m_bUntieObject = false;

	int m_uOneSecondTick;

	double earthRadius = MetersToFeet(6371000); // in ft

	boolean bInitRope = true;

	IRopeSimulationV420 *pRope = NULL;

	UINT objectToPickUpID = 0;
	UINT objectPlaced = 0; // ID of last placed object (just needed for debug marker)

	string hoistConfigPath = "hoistConfig.txt" ;

	CacheObjectSimCfgProps objectSimProps;

	string pickupObjectTitle = "PPL_Swimmer1_sm" ;
	//std::set<string> pickupObjectTitles;
	string pickupObjectDirectory = "";

	bool m_bPlaceAtHoist = true ;
	double m_PlaceDistanceInM = 0.0 ;

	CComPtr<P3D::IGlobalDataV430> ptrGlobalData;
	CComPtr<IObjectRendererV400>  m_spRenderService = NULL;

	int bDebug = false; // int on purpose (makes conf reading easier)

	int topHoistMenu = 0;

	///----------------------------------------------------------------------------
	///  Menu Items
	///----------------------------------------------------------------------------
	CComPtr<P3D::IMenuItemV410> m_spMenuTop;

	CComPtr<P3D::IMenuItemV410> m_spMenuDrawObjs;
	CComPtr<P3D::IMenuItemV410> m_spMenuHoistDown;
	CComPtr<P3D::IMenuItemV410> m_spMenuHoistUp;
	CComPtr<P3D::IMenuItemV410> m_spMenuCaptureSizeUp;
	CComPtr<P3D::IMenuItemV410> m_spMenuCaptureSizeDown;
	CComPtr<P3D::IMenuItemV410> m_spMenuCaptureVisible;
	CComPtr<P3D::IMenuItemV410> m_spMenuReadConfig;
	CComPtr<P3D::IMenuItemV410> m_spSwitchToForcePhysics;
	CComPtr<P3D::IMenuItemV410> m_spUntieObject;
	
	CComPtr<P3D::IMenuItemV410> m_spCreateAndAttachObject;

	CComPtr<P3D::IMenuItemV410> m_spPickupObjectsPlacementParentMenu;
	CComPtr<P3D::IMenuItemV410> m_spPickupObjectsPlacementAtHoist;
	CComPtr<P3D::IMenuItemV410> m_spPickupObjectsPlacementUnderHeli;
	CComPtr<P3D::IMenuItemV410> m_spPickupObjectsPlacement5mAhead;
	CComPtr<P3D::IMenuItemV410> m_spPickupObjectsPlacement10mAhead;
	CComPtr<P3D::IMenuItemV410> m_spPickupObjectsPlacement100mAhead;
	CComPtr<P3D::IMenuItemV410> m_spPickupObjectsPlacement1nmAhead;
	CComPtr<P3D::IMenuItemV410> m_spPickupObjectsPlacement5nmAhead;
	CComPtr<P3D::IMenuItemV410> m_spPickupObjectsPlacement10nmAhead;

	CComPtr<P3D::IMenuItemV410> m_spPickupObjectsParentMenu;
	vector<CComPtr<P3D::IMenuItemV410> > m_VecMenuPickupObject;

	CComPtr<P3D::IMenuItemV410> m_spMenuAddon;

	enum CALLBACK_IDS {
		DRAW_GRID,
		DRAW_OBJS,
		HOIST_DOWN,
		HOIST_UP,
		CAPTURE_SIZE_UP,
		CAPTURE_SIZE_DOWN,
		CAPTURE_VIS,
		READ_CONFIG,
		FORCE_PHYSICS,
		UNTIE_OBJECT,

		PICKUP_OBJECT_PLACEPARENT,
		PICKUP_OBJECT_PLACE_ATTHOIST,
		PICKUP_OBJECT_PLACE_UNDER,
		PICKUP_OBJECT_PLACE_5mAHEAD,
		PICKUP_OBJECT_PLACE_10mAHEAD,
		PICKUP_OBJECT_PLACE_100mAHEAD,
		PICKUP_OBJECT_PLACE_1nmAHEAD,
		PICKUP_OBJECT_PLACE_5nmAHEAD,
		PICKUP_OBJECT_PLACE_10nmAHEAD,

		CREATE_AND_ATTACH_OBJECT,

		ENABLE,

		PICKUP_OBJECT_BASE = 100
	};

	class MenuCallback : public P3D::ICallbackV400
	{
		std::string name;
	public:
		MenuCallback(CALLBACK_IDS eventID)
			: m_EventID(eventID), m_RefCount(1)
		{}

		MenuCallback(CALLBACK_IDS eventID, const std::string &name_)
			: m_EventID(eventID), m_RefCount(1)
		{
			name = name_;
		}

		virtual void Invoke(P3D::IParameterListV400* pParams) override;

		CALLBACK_IDS m_EventID;

		DEFAULT_REFCOUNT_INLINE_IMPL()
			DEFAULT_IUNKNOWN_QI_INLINE_IMPL(MenuCallback, P3D::IID_ICallbackV400)
	};


public:

	HoistP3DPlugin() : PdkPlugin()
    {
		CreateAddonsMenu();

		m_bHoistUp = false;
		m_bHoistDown = false;

		m_uOneSecondTick = 0;
    }

	void BuildMenuAndConfig()
	{
		printf("BuildMenuAndConfig\n");
		bool bRc = ReadAircraftConfigFile();
		RemoveTopLevelMenu();
		if (bRc == true) // no hoistConfig.txt found? no hoist menu!
		{			
			topHoistMenu = CreateTopLevelMenu();
		} else {
			removeRopeAndPickedupObject(); // should there be any rope or object present, get rid of that too
		}
	}

	void RemoveTopLevelMenu()
	{
		printf("RemoveTopLevelMenu\n");
		if (topHoistMenu != 0) {
			printf("RemoveTopLevelMenu action\n"); 
			HRESULT hRes = P3D::PdkServices::GetMenuService()->RemoveItem(topHoistMenu, NO_PARENT);
			topHoistMenu = 0;
			printf("RemoveTopLevelMenu action done %d\n", hRes);
		}
	}

	void CreateAddonsMenu()
	{
		printf("CreateAddonsMenu\n");
		m_spMenuAddon.Attach(P3D::PdkServices::GetMenuService()->CreateMenuItem());
		m_spMenuAddon->SetType(P3D::MenuTypePdk::MENU_CHECK_ITEM);
		m_spMenuAddon->SetChecked(false);
		m_spMenuAddon->SetText(L"HoistP3D Enable");
		MenuCallback * callback = new MenuCallback(ENABLE);
		m_spMenuAddon->RegisterCallback(callback);
		HRESULT hRes = P3D::PdkServices::GetMenuService()->AddItem(m_spMenuAddon->GetId(), TOOLS_ITEM_ID, 0);
		printf("CreateAddonsMenu complete %d\n", hRes);
	}

	int CreateTopLevelMenu()
	{
		int iHoistMenuId = 0;

		m_spMenuTop.Attach(P3D::PdkServices::GetMenuService()->CreateMenuItem());
		m_spMenuTop->SetType(P3D::MenuTypePdk::MENU_ITEM);
		m_spMenuTop->SetText(L"Hoist");
		P3D::PdkServices::GetMenuService()->AddItem(iHoistMenuId=m_spMenuTop->GetId(), NO_PARENT, 1);


		m_spMenuDrawObjs.Attach(P3D::PdkServices::GetMenuService()->CreateMenuItem());
		m_spMenuDrawObjs->SetType(P3D::MenuTypePdk::MENU_CHECK_ITEM);
		m_spMenuDrawObjs->SetChecked(true);
		m_spMenuDrawObjs->SetText(L"Attach/Detach Hoist");
		MenuCallback * callback2 = new MenuCallback(DRAW_OBJS);
		m_spMenuDrawObjs->RegisterCallback(callback2);
		P3D::PdkServices::GetMenuService()->AddItem(m_spMenuDrawObjs->GetId(), m_spMenuTop->GetId(), 0);

		m_spMenuHoistDown.Attach(P3D::PdkServices::GetMenuService()->CreateMenuItem());
		m_spMenuHoistDown->SetType(P3D::MenuTypePdk::MENU_CHECK_ITEM);
		m_spMenuHoistDown->SetChecked(false);
		m_spMenuHoistDown->SetText(L"Hoist down");
		m_spMenuHoistDown->RegisterCallback(new MenuCallback(HOIST_DOWN));
		P3D::PdkServices::GetMenuService()->AddItem(m_spMenuHoistDown->GetId(), m_spMenuTop->GetId(), 0);

		m_spMenuHoistUp.Attach(P3D::PdkServices::GetMenuService()->CreateMenuItem());
		m_spMenuHoistUp->SetType(P3D::MenuTypePdk::MENU_CHECK_ITEM);
		m_spMenuHoistUp->SetChecked(false);
		m_spMenuHoistUp->SetText(L"Hoist up");
		m_spMenuHoistUp->RegisterCallback(new MenuCallback(HOIST_UP));
		P3D::PdkServices::GetMenuService()->AddItem(m_spMenuHoistUp->GetId(), m_spMenuTop->GetId(), 0);

		createPulldownMenu(m_spMenuCaptureSizeUp, L"Increase capture size", CAPTURE_SIZE_UP, false);
		createPulldownMenu(m_spMenuCaptureSizeDown, L"Decrease capture size", CAPTURE_SIZE_DOWN, false);

		createPulldownMenu(m_spMenuCaptureVisible, L"Capture sphere visible", CAPTURE_VIS, true);

		createPulldownMenu(m_spMenuReadConfig, L"Read config", READ_CONFIG, true);

		createPulldownMenu(m_spSwitchToForcePhysics, L"Force simulation", FORCE_PHYSICS, true);

		createPulldownMenu(m_spUntieObject, L"Untie object from cable", UNTIE_OBJECT, true);

		createPulldownMenu(m_spCreateAndAttachObject, L"Create and attach object", CREATE_AND_ATTACH_OBJECT, false);

		createPulldownMenu(m_spPickupObjectsPlacementParentMenu, L"Place Objects...", PICKUP_OBJECT_PLACEPARENT, false);
		createPulldownMenu(m_spPickupObjectsPlacementAtHoist,	 L"Attached to hoist", PICKUP_OBJECT_PLACE_ATTHOIST, true, m_spPickupObjectsPlacementParentMenu, "hoist");
		createPulldownMenu(m_spPickupObjectsPlacementUnderHeli,  L"Under aircraft", PICKUP_OBJECT_PLACE_UNDER, true, m_spPickupObjectsPlacementParentMenu, "0m" );
		createPulldownMenu(m_spPickupObjectsPlacement10mAhead, L"10m ahead", PICKUP_OBJECT_PLACE_10mAHEAD, true, m_spPickupObjectsPlacementParentMenu, "10m");
		createPulldownMenu(m_spPickupObjectsPlacement5mAhead, L"50m ahead", PICKUP_OBJECT_PLACE_5mAHEAD, true, m_spPickupObjectsPlacementParentMenu, "50m" );
		createPulldownMenu(m_spPickupObjectsPlacement100mAhead, L"100m ahead", PICKUP_OBJECT_PLACE_100mAHEAD, true, m_spPickupObjectsPlacementParentMenu, "100m" );
		createPulldownMenu(m_spPickupObjectsPlacement1nmAhead,   L"1nm ahead", PICKUP_OBJECT_PLACE_1nmAHEAD, true, m_spPickupObjectsPlacementParentMenu, "1nm");
		createPulldownMenu(m_spPickupObjectsPlacement5nmAhead, L"5nm ahead", PICKUP_OBJECT_PLACE_5nmAHEAD, true, m_spPickupObjectsPlacementParentMenu, "5nm" );
		createPulldownMenu(m_spPickupObjectsPlacement10nmAhead, L"10nm ahead", PICKUP_OBJECT_PLACE_10nmAHEAD, true, m_spPickupObjectsPlacementParentMenu, "10nm");

		createPulldownMenu(m_spPickupObjectsParentMenu, L"Objects...", PICKUP_OBJECT_BASE, false);

		InitPickupObjectsMenu();

		return iHoistMenuId;
	}

	void InitPickupObjectsMenu()
	{
		if (pickupObjectDirectory.length() > 0) {

			path dir_path(pickupObjectDirectory);

			find_file_recursive(dir_path, "sim.cfg", [this](string path) 
								{ 
									printf("Found sim.cfg under %s\n", path.c_str()); 
									objectSimProps.getObjectSimCfg(path.c_str());  
								});
		}

		P3D::PdkServices::GetMenuService()->RemoveItem(ALL_CHILDREN, m_spPickupObjectsParentMenu->GetId());
		int iObjNum = 0;
		for (ObjectSimCfgProps *pProps : objectSimProps.getAllObjectSimCfg()) {

			printf("About to add menu for %s\n", pProps->title.c_str());

			CComPtr<P3D::IMenuItemV410> ptrObjectMenu;
			m_VecMenuPickupObject.push_back(ptrObjectMenu);

			wchar_t wachMenuTitle[256];
			mbstowcs(wachMenuTitle, pProps->title.c_str(), 256 ) ;

			createPulldownMenu(ptrObjectMenu, wachMenuTitle, (CALLBACK_IDS)(((int)PICKUP_OBJECT_BASE) + ++iObjNum), false, m_spPickupObjectsParentMenu, pProps->title)  ;
		}
	}

	void createPulldownMenu(CComPtr<P3D::IMenuItemV410> &menu, wchar_t *pszText, CALLBACK_IDS callbackid, bool bCheckItem)
	{
		createPulldownMenu(menu, pszText, callbackid, bCheckItem, m_spMenuTop, string(""));
	}

	void createPulldownMenu( CComPtr<P3D::IMenuItemV410> &menu, wchar_t *pszText, CALLBACK_IDS callbackid, bool bCheckItem, CComPtr<P3D::IMenuItemV410> &m_spParentMenu, const string &name)
	{
		menu.Attach(P3D::PdkServices::GetMenuService()->CreateMenuItem());
		menu->SetType(bCheckItem?P3D::MenuTypePdk::MENU_CHECK_ITEM: P3D::MenuTypePdk::MENU_ITEM);
		if (bCheckItem) {
			menu->SetChecked(false);
		}
		menu->SetText(pszText);
		menu->RegisterCallback(new MenuCallback(callbackid, name));
		P3D::PdkServices::GetMenuService()->AddItem(menu->GetId(), m_spParentMenu->GetId(), 0);
	}

    ///----------------------------------------------------------------------------
    ///  PdkPlugin Callback overrides
    ///----------------------------------------------------------------------------

    virtual void OnCustomRender(IParameterListV400* pParams) override
    {
		if (m_bEnabled == false) return;

        // Get the Object Renderer service from the callback params
		CComPtr<IVisualEffectManagerV430> ptrVisEffect = NULL;
		CComPtr<ISimObjectManagerV440> ptrSimObjectManager = NULL;

		HRESULT hResRender = S_OK;
		if (m_spRenderService == NULL) {
			hResRender = pParams->GetServiceProvider()->QueryService(SID_ObjectRenderer, IID_IObjectRendererV400, (void**)&m_spRenderService);
		}

		HRESULT hResVisEff = pParams->GetServiceProvider()->QueryService(SID_VisualEffectManager, IID_IVisualEffectManagerV430, (void**)&ptrVisEffect);
		if (!SUCCEEDED(hResVisEff)) {
			printf("IID_IVisualEffectManagerV430:%d\n", hResVisEff);
		}

		if (ptrGlobalData == NULL) {
			HRESULT hGlobData = pParams->GetServiceProvider()->QueryService(SID_GlobalData, IID_IGlobalDataV430, (void**)&ptrGlobalData);
			if (!SUCCEEDED(hResVisEff)) {
				printf("IID_IGlobalDataV430:%d\n", hGlobData);
			}
		}		

		/*
		HRESULT hResSimObj = pParams->GetServiceProvider()->QueryService(SID_SimObjectManager, IID_ISimObjectManagerV440, (void**)&ptrSimObjectManager);
		if (!SUCCEEDED(hResSimObj)) {
			printf("IID_ISimObjectManagerV440:%d\n", hResSimObj);
		}
		*/

        if( SUCCEEDED(hResRender) && SUCCEEDED(hResVisEff) /*&& SUCCEEDED(hResSimObj)*/)
        {
            if (m_bDrawUserSimObjects)
            {
                DrawUserSimObjects(ptrVisEffect, pParams->GetServiceProvider());
			}
			else {
				// printf("Rope will be reinited...\n");
				bInitRope = true; // force reinit of rope!
				removeRopeAndPickedupObject();
			}
        }
    }

	void removeRopeAndPickedupObject()
	{
		objectToPickUpID = 0; // let go of objects
		if (pRope) {
			pRope->Release();
			pRope = NULL;
		}
	}

    virtual void OnOneHz(IParameterListV400* pParams) override
    {
        // keep track count of seconds. This is used for alternating object color.
        m_uOneSecondTick++;
    }

	bool ReadAircraftConfigFile()
	{
		CComPtr<IBaseObjectV400> spUserObject = NULL;
		PdkServices::GetSimObjectManager()->GetUserObject((IBaseObjectV400**)&spUserObject);
		wchar_t wachPath[512];
		spUserObject->GetCfgDir(wachPath, 512);
		char achPath[512];
		wcstombs(achPath, wachPath, 512);
		hoistConfigPath = string(achPath) + "\\hoistConfig.txt";
		printf("aircraft hoistConfig path: %s\n", hoistConfigPath.c_str()); // ööö to be moved to airplane load event!
		bool bRc = ReadConfigFile();

		return bRc;
	}

protected:

    ///----------------------------------------------------------------------------
    ///  Object drawing functions
    ///----------------------------------------------------------------------------

    void DrawUserSimObjects(IVisualEffectManagerV430 *pVisEffect, /* ISimObjectManagerV440 *pSimObjectManager*/ IServiceProvider* pServiceProvider)
    {
        CComPtr<IWindowV400> spWindow = NULL;
        CComPtr<IBaseObjectV400> spUserObject = NULL;
		
        ObjectWorldTransform cameraTrans;

        // Get current window
		/*
        spWindow = PdkServices::GetWindowPluginSystem()->GetCurrentWindow();
        if (spWindow != nullptr && spWindow->GetCameraSystem() != nullptr)
        {
            // get lla and pbh of camera (degrees/meters)
            spWindow->GetCameraSystem()->GetLLA(cameraTrans.LLA.Latitude, cameraTrans.LLA.Longitude, cameraTrans.LLA.Altitude);
            spWindow->GetCameraSystem()->GetPBH(cameraTrans.PBH.Pitch, cameraTrans.PBH.Bank, cameraTrans.PBH.Heading);
        }
		*/

		P3DFXYZ vStart;
		vStart.fX = 0;
		vStart.fY = 0;
		vStart.fZ = 0;

        // Get user sim from sim object manager
        PdkServices::GetSimObjectManager()->GetUserObject((IBaseObjectV400**)&spUserObject);
        if (spUserObject != nullptr)
        {
            // get position and orientation and convert from radians/feet to degrees/meters
            P3D::P3DDXYZ objLal, objPhb, lalVel, pbhVel = { 0 };
            spUserObject->GetPosition(objLal, objPhb, lalVel, pbhVel);
            
			objTrans.LLA.Latitude	= RadToDeg(objLal.dZ);
            objTrans.LLA.Longitude	= RadToDeg(objLal.dX);
            objTrans.LLA.Altitude	= FeetToMeters(objLal.dY);
            objTrans.PBH.Pitch		= (float)RadToDeg(objPhb.dX);
            objTrans.PBH.Bank		= (float)RadToDeg(objPhb.dZ);
            objTrans.PBH.Heading	= (float)RadToDeg(objPhb.dY);

			spUserObject->GetSurfaceElevation(fElevation, NULL);
			
			if (bInitRope || pRope == NULL) {

				bool bRc = ReadAircraftConfigFile(); // do we still need this here?
				//if( bRc == tr)

				P3DFXYZ vEnd = vStart;

				float fRestingRopeLength = (float)minLengthFt;
				float fRelativeGroundPosition = 0;
				REFIID ropeRiid = IID_IRopeSimulationV420;

				HRESULT hResRope = pVisEffect->CreateRopeSimulation(vStart, vEnd, fRopeMass, fRestingRopeLength, fRelativeGroundPosition, ropeRiid, (void **)&pRope);
				if (!SUCCEEDED(hResRope)) {
					printf("Rope error: %d\n", hResRope);
				}
				else {
					printf("Rope created!\n");
					pRope->SetRopeColor(16, 16, 16, 255);
					pRope->SetFixed(true, true);   // startpoint fixed
					pRope->SetFixed(false, false); // endpoint swinging free
					
					pRope->SetRenderWorldPosition(objLal);

					pRope->SetRelativeGroundPosition(true /*bool 	bCheckGround*/, (float)-minLengthFt);
					pRope->AddRef(); int iRefs = pRope->Release(); // get the current ref count

					printf("pRope refcount: %d, pos: %lf,%lf,%lf\n", iRefs, objLal.dX, objLal.dY, objLal.dZ );

					pRope->SetStart(vStart);
					P3DFXYZ end = pRope->GetEnd();
					printf("pRope end: %f,%f,%f\n", end.fX, end.fY, end.fZ ) ;
				}
				bInitRope = false;
			}
			else {				
				ObjectLocalTransform toSling(FeetToMeters(hoistAttach.dX), FeetToMeters(hoistAttach.dY), FeetToMeters(hoistAttach.dZ), 0, 0, 0);
				ObjectWorldTransform slingPos ;
				m_spRenderService->ApplyBodyRelativeOffset(objTrans, toSling, slingPos);
				
				//result: LLADegreesMeters:
				//slingPos.LLA.Altitude;
				//slingPos.LLA.Latitude;
				//slingPos.LLA.Longitude;

				//vPos, where X=longitude (radians), Y=altitude (feet), Z=latitude (radians)
				P3DDXYZ slingPoint;
				slingPoint.dX = DegToRad(slingPos.LLA.Longitude);
				slingPoint.dY = MetersToFeet(slingPos.LLA.Altitude);
				slingPoint.dZ = DegToRad(slingPos.LLA.Latitude);				
				pRope->SetRenderWorldPosition(slingPoint);
				
				pRope->SetStart(vStart);
				
				P3DFXYZ end = pRope->GetEnd();

				P3DFXYZ vWind;
				vWind.fX = lalVel.dX;
				vWind.fY = lalVel.dY;
				vWind.fZ = lalVel.dZ;

				pRope->SetRelativeWind(vWind);

				fSlingEndRadarAltitude = /*objLal.dY*/slingPoint.dY - fElevation ;
				pRope->SetRelativeGroundPosition(true /*bool 	bCheckGround*/, -fSlingEndRadarAltitude);

				double dt = 0;
				if (m_bHoistUp || m_bHoistDown || m_bForcePhysics ) {
					if (clock() > lastTickHoistMove) {
						clock_t clkDiff = clock() - lastTickHoistMove;
						if (clkDiff > 0) {
							dt = (double)clkDiff / CLOCKS_PER_SEC;
						} else { // wrapped around?
							// leave that beat out?
							dt = 0;
						}
						lastTickHoistMove = clock();
					}
				}
				else {
					lastTickHoistMove = clock() ;
				}

				if (m_bHoistUp) { // decrease length down until minLength:
					/*if (m_bForcePhysics) {
						if (lastRopeLen >= minLengthFt) {
							if (dt > 0) {
								lastRopeLen -= hoistSpeed*dt;
							}
						}
					} else*/ {
						if (lengthFt >= minLengthFt) {
							if (dt > 0) {
								lengthFt -= hoistSpeed*dt;
								//printf("up Len: %lf\n", lengthFt);
							}
						}
					}
				}
				else if (m_bHoistDown) { // increase length until maxLength:
					/*if (m_bForcePhysics) {
						if (lastRopeLen <= maxLengthFt) {
							if (dt > 0) {
								lengthFt += hoistSpeed*dt;
							}
						}
					} else*/ {
						if (lengthFt <= maxLengthFt) {
							if (dt > 0) {
								lengthFt += hoistSpeed*dt;
								//printf("dn Len: %lf\n", lengthFt);
							}
						}
					}
				}

				pRope->SetRopeLength((float)lengthFt);
				/*
				P3DFXYZ start = pRope->GetStart();
				printf("start: %f,%f,%f - ", start.fX, start.fY, start.fZ);
				P3DFXYZ end = pRope->GetEnd();
				printf("end: %f,%f,%f\n", end.fX, end.fY, end.fZ);				
				*/

				//Draw hoisted object
				
				DrawSimObjects(spUserObject, objTrans, cameraTrans, end, objPhb, hoistAttach, lalVel, slingPoint.dY+end.fY - fElevation, slingPoint, dt);
			}
        }
    }

	ObjectWorldTransform getHoistAttachPointInWorldCoords( const CComPtr<IBaseObjectV400> &ptrObject)
	{
		DXYZ vLonAltLat;
		DXYZ vPHB;
		DXYZ vLonLatAltVel;
		DXYZ vPHBVel;
		ptrObject->GetPosition(vLonAltLat, vPHB, vLonLatAltVel, vPHBVel);

		// go to world attachment point:
		ObjectWorldTransform posObj;
		posObj.LLA.Latitude = RadToDeg(vLonAltLat.dZ);
		posObj.LLA.Longitude = RadToDeg(vLonAltLat.dX);
		posObj.LLA.Altitude = FeetToMeters(vLonAltLat.dY);
		posObj.PBH.Pitch = (float)RadToDeg(vPHB.dX);
		posObj.PBH.Bank = (float)RadToDeg(vPHB.dZ);
		posObj.PBH.Heading = (float)RadToDeg(vPHB.dY);
		ObjectLocalTransform toLoadAttach(FeetToMeters(-hoistAttachLoad.dX), FeetToMeters(-hoistAttachLoad.dY), FeetToMeters(-hoistAttachLoad.dZ), 0, 0, 0);
		ObjectWorldTransform toAttachLoadWorld;
		m_spRenderService->ApplyBodyRelativeOffset(posObj, toLoadAttach, toAttachLoadWorld);

		return toAttachLoadWorld ;
	}

	/**
	 * world in meters/degrees, result in feet/rad
	 */
	DXYZ extractPosFromWorldDegMeters(ObjectWorldTransform &world)
	{
		DXYZ vLonAltLat;
		vLonAltLat.dX = DegToRad(world.LLA.Longitude);
		vLonAltLat.dY = MetersToFeet(world.LLA.Altitude);
		vLonAltLat.dZ = DegToRad(world.LLA.Latitude);
		return vLonAltLat;
	}

	void setForcePhysics(boolean b)
	{
		m_bForcePhysics = b;
		m_spSwitchToForcePhysics->SetChecked(m_bForcePhysics);
		if (b == true) {
			diffRopeLen = 0;

			/*
			CComPtr<IBaseObjectV400> spUserObject = NULL;
			PdkServices::GetSimObjectManager()->GetUserObject((IBaseObjectV400**)&spUserObject);
			if (spUserObject != nullptr)
			{				
				IForceMomentsV01 *pForceMoments = NULL;
				HRESULT hRes = spUserObject->QueryService(SID_ForceMoments, IID_IForceMomentsV01, (void **)&pForceMoments);
				//HRESULT hRes = spUserObject->QueryInterface(IID_IForceMomentsV01, (void **)&pForceMoments);
				if (!SUCCEEDED(hRes)) {
					printf("QueryService IID_IForceMoments failed: %d\n", hRes);
				} else {
					printf("ForceElement about to be registered\n");
					pForceMoments->RegisterElement(this);
					pForceMoments->Release();

					printf("ForceElement registered\n");
				}
			}
			*/
		} else {
			/*
			CComPtr<IBaseObjectV400> spUserObject = NULL;
			PdkServices::GetSimObjectManager()->GetUserObject((IBaseObjectV400**)&spUserObject);
			if (spUserObject != nullptr)
			{
				IForceMomentsV01 *pForceMoments = NULL;
				HRESULT hRes = spUserObject->QueryService(SID_ForceMoments, IID_IForceMomentsV01, (void **)&pForceMoments);
				if (!SUCCEEDED(hRes)) {
					printf("QueryService IID_IForceMoments failed: %d\n", hRes);
				} else {
					pForceMoments->UnRegisterElement(this);
					pForceMoments->Release();

					printf("ForceElement unregistered\n");
				}
			}
			*/
		}
	}

	void CreateAndAttachObject()
	{
		ISimObjectManagerV440 *pObjectManager = PdkServices::GetSimObjectManager();
			
		wchar_t wObjectTitle[256];
		mbstowcs(wObjectTitle, pickupObjectTitle.c_str(), 512);
		
		UINT objectID;
		HRESULT hRes = pObjectManager->CreateObject(wObjectTitle, objectID) ;
		if (SUCCEEDED(hRes)) {
			CComPtr<IBaseObjectV400> ptrObject;
			hRes = pObjectManager->GetObject(objectID, &ptrObject);
			if (SUCCEEDED(hRes)) {

				double lfCg = -1;
				HRESULT hRes = ptrObject->GetProperty(L"STATIC CG TO GROUND", L"feet", lfCg, 0);
				if (SUCCEEDED(hRes)) {
					printf("setting Cg: %lf\n", lfCg);
					fStaticCGHeight = (float)lfCg;
				}

				// get the sim.cfgs [HoistPoint] position, and shuffle as needed:
				wchar_t wachPath[512];
				ptrObject->GetCfgFilePath(wachPath, sizeof(wachPath));

				char achPath[512];
				wcstombs(achPath, wachPath, 512);
				ObjectSimCfgProps props = objectSimProps.getObjectSimCfg(achPath); // caches relevant sim cfg values

				DXYZ vdOrient = { 0, 0, 0 };
				DXYZ vel = { 0,0,0 };
				DXYZ velRot = { 0,0,0 };

				if (m_bPlaceAtHoist) { // place it and endpos, hoist will pick it up automatically

					hoistAttachLoad.dX = -props.hoistAttachLoad.dY;
					hoistAttachLoad.dY = -props.hoistAttachLoad.dZ;
					hoistAttachLoad.dZ = -props.hoistAttachLoad.dX;
					massLoad = props.mass;
					printf("setting hoistpoint: %lf %lf %lf setting mass %lf\n", hoistAttachLoad.dX, hoistAttachLoad.dY, hoistAttachLoad.dZ, massLoad);

					this->objectToPickUpID = objectID;
					ptrObject->SetPosition(endPos, vdOrient, vel, velRot, false, 0);
				} 
				else
				{
					this->objectPlaced = objectID;
					// current heli pos in objTrans - translate locally for the m given:
					ObjectLocalTransform toObjectPlacement(0, 0, m_PlaceDistanceInM, 0, 0, 0); // m_PlaceDistanceInM ahead					
					m_spRenderService->ApplyBodyRelativeOffset(objTrans, toObjectPlacement, worldObjectPlacement);

					// Get Elevation there:
					P3DDXYZ objPos = extractPosFromWorldDegMeters(worldObjectPlacement);

					//objPos.dX = DegToRad(worldObjectPlacement.LLA.Longitude);
					//objPos.dY = MetersToFeet(worldObjectPlacement.LLA.Altitude);
					//objPos.dZ = DegToRad(worldObjectPlacement.LLA.Latitude);
					if (bDebug) {
						printf("Placing object at %lf,%lf,%lf\n", objPos.dX, objPos.dY, objPos.dZ);
					}					
					ptrObject->SetPosition(objPos, vdOrient, vel, velRot, false, 0);

					//float fElevationFeet = 0;
					
					/*
					P3DFXYZ offset = { 0, 0, MetersToFeet(m_PlaceDistanceInM) };
					CComPtr<IBaseObjectV400> spUserObject = NULL;
					PdkServices::GetSimObjectManager()->GetUserObject((IBaseObjectV400**)&spUserObject);
					if (spUserObject != nullptr)
					{
						spUserObject->GetSurfaceElevation(fElevation, &offset);
						if (!SUCCEEDED(hRes)) {
							printf("Get elevation failed: %d\n", hRes);
						}
						if (bDebug) {
							printf("Object at elevation %f\n", fElevationFeet);
						}
					}
					*/

					/*
					HRESULT hRes = ptrObject->GetSurfaceElevation(fElevationFeet, &offset);
					if (!SUCCEEDED(hRes)) {
						printf("Get elevation failed: %d\n", hRes);
					}
					if (bDebug) {
						printf("Object at elevation %f\n", fElevationFeet);
					}
					*/

					/*
					double lfGroundAlt = -1;
					hRes = ptrObject->GetProperty(L"GROUND ALTITUDE", L"feet", lfGroundAlt, 0);
					if (SUCCEEDED(hRes)) {
						printf("setting ground alt: %lf\n", lfGroundAlt);
						//fStaticCGHeight = (float)lfCg;
					}
					*/

					// Set it softly to the ground
					objPos.dY = fElevation + fStaticCGHeight + 1 ; // 1ft above current altitude - getElevation for offste or for object delivers nonsense here? Maybe bc we are in Menu handler?
					if (bDebug) {
						printf("Correcting object to %lf,%lf,%lf\n", objPos.dX, objPos.dY, objPos.dZ);

						worldObjectPlacement.LLA.Altitude = FeetToMeters(objPos.dY);
					}
					ptrObject->SetPosition(objPos, vdOrient, vel, velRot, true, 0);
				}
				
			}
			else {
				printf("Could not get Object %d %s\n", objectToPickUpID, pickupObjectTitle.c_str());
			}
		}
		else {
			printf("Could not load %s\n", pickupObjectTitle.c_str());
		}
	}

	STDMETHOD_(void, GetForceMoment(__out DXYZ& vForces, __out DXYZ& vMoment))  const  /**< Gets a force moment */
	{
		if (m_bForcePhysics) {
			vForces.dX = 0;
			vForces.dY = MetersToFeet(9.81) * 100;
			vForces.dY = 0;

			vMoment.dX = 0;
			vMoment.dY = 0;
			vMoment.dZ = 0;
			printf("GetForceMoment phys\n");
		} else {
			printf("GetForceMoment no phys\n");
		}
	}
	
	STDMETHOD(GetOffset(float& fXFeet, float& fYFeet, float& fZFeet))          const /**< Gets the offset of a force moment */
	{
		fXFeet = 0;
		fYFeet = 0;
		fZFeet = 0;

		return S_OK;
	}
	
	STDMETHOD(SetForceMultiplier(float scalar))                                      /**< Sets a force multiplier */
	{
		return S_OK;
	}

    void DrawSimObjects(const CComPtr<IBaseObjectV400> &spUserObject, ObjectWorldTransform& objTrans, ObjectWorldTransform& cameraTrans, P3DFXYZ end, P3DDXYZ orient, 
						DXYZ hoistAttach, DXYZ lalVel, double dEndOverGround, P3DDXYZ slingStartPoint, double dt)
    {
		double ft2m = 0.3048;

        //Draw a sphere around user object.
		//printf("end: %f,%f,%f\n", end.fZ, end.fY, end.fX);		
		//ObjectLocalTransform toSling(FeetToMeters(hoistAttach.dX), FeetToMeters(hoistAttach.dY), FeetToMeters(hoistAttach.dZ), 0, 0, 0);

		//ObjectWorldTransform objTransTemp2 = objTrans;
		//ObjectWorldTransform offsetEndOfSling;
		//pRenderer->ApplyBodyRelativeOffset(objTrans, toSling, objTransTemp2);
		
		DXYZ endInWorldCoord;
		endInWorldCoord.dX = end.fX;
		endInWorldCoord.dY = end.fY;
		endInWorldCoord.dZ = end.fZ;
		DXYZ endInBodyCoord;
		spUserObject->RotateWorldToBody(endInWorldCoord, endInBodyCoord);

		ObjectLocalTransform toEndOfSling(endInBodyCoord.dX*ft2m + FeetToMeters(hoistAttach.dX), endInBodyCoord.dY*ft2m + FeetToMeters(hoistAttach.dY), endInBodyCoord.dZ*ft2m + FeetToMeters(hoistAttach.dZ), 0, 0, 0);
		ObjectWorldTransform endOfSling;
		ObjectWorldTransform objTransTemp = objTrans;
		//objTransTemp.PBH.Bank = 0; // Important! The rope his ots orientation relative to world, not relative to the heli! Nope, now that we rotated to body coords, it's fine.
		//objTransTemp.PBH.Heading = 0;
		//objTransTemp.PBH.Pitch = 0;
		m_spRenderService->ApplyBodyRelativeOffset(objTransTemp, toEndOfSling, endOfSling); // can only be calculated from object center (as we use "BodyRelativeOffset"), while end is relativ to slingPos

		// endOfSling.LLA now in degree/m/degree
				
		// Hmm?
		//ObjectWorldTransform finalEndOfSling;
		//pRenderer->ApplyBodyRelativeOffset(endOfSling, toSling, finalEndOfSling);

		endPos.dX = endOfSling.LLA.Longitude / 180 * PI; // rad
		endPos.dY = endOfSling.LLA.Altitude / ft2m;		 // ft
		endPos.dZ = endOfSling.LLA.Latitude / 180 * PI;  // rad

		if (bDebug && objectToPickUpID == 0 && objectPlaced != 0) {
			// bug sphere to show the object from far away during debugging:
			ARGBColor colorSphere3(32, 12, 12, 200);
			m_spRenderService->DrawSphere(worldObjectPlacement, max(20,m_PlaceDistanceInM/20), colorSphere3);
		}

		if (!m_bUntieObject)
		{
			if (objectToPickUpID == 0) { // ready to pickup

				if (bCaptVis) {
					ARGBColor colorSphere3(32, 200, 12, 200);
					m_spRenderService->DrawSphere(endOfSling, pickupradius, colorSphere3);
				}

				unsigned int numOfCloseObjects = 200;
				unsigned int objids[200];
				PdkServices::GetSimObjectManager()->GetObjectsInRadius(endPos, 3 * (pickupradius / ft2m), numOfCloseObjects, objids); // radius ignored if less than 2 feet? Oh my! endPos in rad_feet_rad?

				//printf("%d objects to pickup\n", numOfCloseObjects);

				for (unsigned int i = 0; i < numOfCloseObjects; i++) {
					UINT objId = objids[i];
					CComPtr<IBaseObjectV400> ptrObject;
					PdkServices::GetSimObjectManager()->GetObject(objId, &ptrObject);
					if (ptrObject) {
						if (!ptrObject->IsUser()) {
							wchar_t achTitle[256];
							ptrObject->GetTitle(achTitle, sizeof(achTitle));
							wchar_t wachPath[512];
							ptrObject->GetCfgFilePath(wachPath, sizeof(wachPath));

							char achPath[512];
							wcstombs(achPath, wachPath, 512);
							ObjectSimCfgProps props = objectSimProps.getObjectSimCfg(achPath); // caches relevant sim cfg values
							//if (pProps) {
							hoistAttachLoad.dX = -props.hoistAttachLoad.dY;
							hoistAttachLoad.dY = -props.hoistAttachLoad.dZ;
							hoistAttachLoad.dZ = -props.hoistAttachLoad.dX;

							//printf("setting hoistpoint for %d: %lf %lf %lf\n", objId, hoistAttachLoad.dX, hoistAttachLoad.dY, hoistAttachLoad.dZ);
						//}

						/*
						DXYZ vLonAltLat;
						DXYZ vPHB;
						DXYZ vLonLatAltVel;
						DXYZ vPHBVel;
						ptrObject->GetPosition(vLonAltLat, vPHB, vLonLatAltVel, vPHBVel);

						// go to world attachment point:
						ObjectWorldTransform posObj;
						posObj.LLA.Latitude = RadToDeg(vLonAltLat.dZ);
						posObj.LLA.Longitude = RadToDeg(vLonAltLat.dX);
						posObj.LLA.Altitude = FeetToMeters(vLonAltLat.dY);
						posObj.PBH.Pitch = (float)RadToDeg(vPHB.dX);
						posObj.PBH.Bank = (float)RadToDeg(vPHB.dZ);
						posObj.PBH.Heading = (float)RadToDeg(vPHB.dY);
						ObjectLocalTransform toLoadAttach(FeetToMeters(-hoistAttachLoad.dX), FeetToMeters(-hoistAttachLoad.dY), FeetToMeters(-hoistAttachLoad.dZ), 0, 0, 0);
						ObjectWorldTransform toAttachLoadWorld;
						pRenderer->ApplyBodyRelativeOffset(posObj, toLoadAttach, toAttachLoadWorld);
						*/

							ObjectWorldTransform toAttachLoadWorld = getHoistAttachPointInWorldCoords(ptrObject);

							if (bCaptVis) {
								ARGBColor colorSphere3(32, 12, 200, 12);
								m_spRenderService->DrawSphere(toAttachLoadWorld, 1, colorSphere3);
							}

							DXYZ vLonAltLat = extractPosFromWorldDegMeters(toAttachLoadWorld);
							/*vLonAltLat.dX = DegToRad(toAttachLoadWorld.LLA.Longitude);
							vLonAltLat.dY = MetersToFeet(toAttachLoadWorld.LLA.Altitude);
							vLonAltLat.dZ = DegToRad(toAttachLoadWorld.LLA.Latitude);*/

							DXYZ vXYZ = convertLonAltLat2XYZ(vLonAltLat, earthRadius); // rad_ft_rad
							DXYZ endPosXYZ = convertLonAltLat2XYZ(endPos, earthRadius);     // rad_ft_rad

							/*
							CComPtr<IBaseObjectV430> ptrObject43;
							ptrObject->QueryInterface(IID_IBaseObjectV430, (void **)&ptrObject43);
							wchar_t achCat[512] = L"";
							if (ptrObject43) {
								ptrObject43->GetCategoryName(achCat, sizeof(achCat));
								//if (wcscmp(achCat, L"Avatar") == 0) {
								//	objectToPickUpID = objId;
								//}
							}
							*/

							double dist = calcDist(vXYZ, endPosXYZ); // in feet!
							if (dist*ft2m < pickupradius) {
								objectToPickUpID = objId;
								printf("Picking up object %d dist %lf %lf,%lf,%lf vs %lf,%lf,%lf title %ls path %ls\n", objectToPickUpID, dist, vXYZ.dX, vXYZ.dY, vXYZ.dZ, endPosXYZ.dX, endPosXYZ.dY, endPosXYZ.dZ, achTitle, wachPath);

								setForcePhysics(true);

								double lfCg = -1;
								HRESULT hRes = ptrObject->GetProperty(L"STATIC CG TO GROUND", L"feet", lfCg, 0);
								if (SUCCEEDED(hRes)) {
									printf("setting Cg: %lf\n", lfCg);
									fStaticCGHeight = (float)lfCg;
								}

								// get the sim.cfgs [HoistPoint] position, and shuffle as needed:
								//if (pProps) {
								hoistAttachLoad.dX = -props.hoistAttachLoad.dY;
								hoistAttachLoad.dY = -props.hoistAttachLoad.dZ;
								hoistAttachLoad.dZ = -props.hoistAttachLoad.dX;
								massLoad = props.mass;

								printf("setting hoistpoint: %lf %lf %lf mass %lf\n", hoistAttachLoad.dX, hoistAttachLoad.dY, hoistAttachLoad.dZ, massLoad);
								//}
								//else {
								//	printf("Keeping hoistConfig hoistpoint: %lf %lf %lf\n", hoistAttachLoad.dX, hoistAttachLoad.dY, hoistAttachLoad.dZ);
								//}							
							}
							else {
								// printf("Not picking up object %d dist %lf %ls ", objId, dist, achTitle );
							}
						}
					}
				}
			}
			else { // we do have an object picked up:

				/*if (bCaptVis) {
					ARGBColor colorSphere(0, 12, 200, 12);
					pRenderer->DrawSphere(endOfSling, pickupradius, colorSphere);
				}*/
				// Object picked up already: render it:			
				CComPtr<IBaseObjectV400> ptrPickupObject;

				HRESULT hRes = PdkServices::GetSimObjectManager()->GetObject(objectToPickUpID, &ptrPickupObject);
				if (!SUCCEEDED(hRes)) {
					printf("GetObject error: %d\n", hRes);
				}
				if (ptrPickupObject != nullptr)
				{
					P3DDXYZ vel = lalVel; // same velocity as helicopter
					P3DDXYZ velRot = { 0, 0, 0 };

					double pitchLoadInRad = -atan(sqrt(end.fX*end.fX + end.fZ*end.fZ) / end.fY);

					// Hoist point must be rotatet if load is pitched in the loads coordinate system
					double yAttach = hoistAttachLoad.dY;
					double zAttach = hoistAttachLoad.dZ;

					DXYZ hoistAttachLoadRot;
					hoistAttachLoadRot.dX = hoistAttachLoad.dX;
					hoistAttachLoadRot.dY = yAttach*cos(pitchLoadInRad) - zAttach*sin(pitchLoadInRad);
					hoistAttachLoadRot.dZ = yAttach*sin(pitchLoadInRad) + zAttach*cos(pitchLoadInRad);

					// DXYZ finalEndPos = endPos; // rad_ft_rad
					DXYZ hoistAttachLoadInWorld;
					DXYZ hoistAttachInHeli;
					ptrPickupObject->RotateBodyToWorld(hoistAttachLoadRot, hoistAttachLoadInWorld); // from pickup body to world system
					spUserObject->RotateWorldToBody(hoistAttachLoadInWorld, hoistAttachInHeli);    // from world to helicopter system

					ObjectLocalTransform toAttachedLoad(FeetToMeters(hoistAttachInHeli.dX), FeetToMeters(hoistAttachInHeli.dY), FeetToMeters(hoistAttachInHeli.dZ), 0, 0, 0);
					ObjectWorldTransform hoistAttachLoad;
					m_spRenderService->ApplyBodyRelativeOffset(endOfSling, toAttachedLoad, hoistAttachLoad); // can only be calculated from object center (as we use "BodyRelativeOffset"), while end is relativ to slingPos

					P3DDXYZ finalEndPos;
					finalEndPos.dX = hoistAttachLoad.LLA.Longitude / 180 * PI; // rad
					finalEndPos.dY = hoistAttachLoad.LLA.Altitude / ft2m;	   // ft
					finalEndPos.dZ = hoistAttachLoad.LLA.Latitude / 180 * PI;  // rad

					P3DDXYZ vdOrient;
					vdOrient.dX = pitchLoadInRad;
					vdOrient.dY = orient.dY;
					vdOrient.dZ = 0;

					if (m_bForcePhysics) // take the current distance between load hoist point and load attach point on heli as d0; applay force = (d-d0)*k into the direction of that vector
					{
						if (!ptrGlobalData->IsPaused())
						{
							ObjectWorldTransform toAttachLoadWorld = getHoistAttachPointInWorldCoords(ptrPickupObject);
							DXYZ vLonAltLatHoistAttach = extractPosFromWorldDegMeters(toAttachLoadWorld);

							// reference latitude and longitude for local ENU coordinate system - why not put it to the hoist attach point!
							double refLon = vLonAltLatHoistAttach.dX;
							double refLat = vLonAltLatHoistAttach.dZ;

							if (bDebug) {
								ARGBColor colorSphereHoistLoad(64, 12, 200, 12);
								m_spRenderService->DrawSphere(toAttachLoadWorld, 1, colorSphereHoistLoad);
							}

							// ObjectLocalTransform toSling(FeetToMeters(hoistAttach.dX), FeetToMeters(hoistAttach.dY), FeetToMeters(hoistAttach.dZ), 0, 0, 0);
							ObjectLocalTransform toSling(FeetToMeters(hoistAttach.dX), FeetToMeters(hoistAttach.dY), FeetToMeters(hoistAttach.dZ), 0, 0, 0);
							ObjectWorldTransform objHostAttachInWorld = objTrans;
							m_spRenderService->ApplyBodyRelativeOffset(objTrans, toSling, objHostAttachInWorld);
							DXYZ vLonAltLatHoistAttachOnHelo = extractPosFromWorldDegMeters(objHostAttachInWorld);

							if (bDebug) {
								ARGBColor colorSphereHoistHeli(64, 12, 12, 200);
								m_spRenderService->DrawSphere(objHostAttachInWorld, 1, colorSphereHoistHeli);
							}

							DXYZ vXYZHostAttachECF = convertLonAltLat2XYZLocal(vLonAltLatHoistAttach, earthRadius, refLon, refLat); // rad_ft_rad to xyz local LonAltLat local system
							DXYZ vXYZAttachOnHeloECF = convertLonAltLat2XYZLocal(vLonAltLatHoistAttachOnHelo, earthRadius, refLon, refLat);

							DXYZ delta = vectorSubtract(vXYZHostAttachECF, vXYZAttachOnHeloECF); // delta is now exactly the rope vector in ECF
							double len = getVectorLen(delta);
							DXYZ dDeltaN; // direction of cable
							dDeltaN.dX = delta.dX / len;
							dDeltaN.dY = delta.dY / len;
							dDeltaN.dZ = delta.dZ / len;

							if (lastRopeLen == 0) {
								lastRopeLen = len;
							}

							//diffRopeLen = len - lastRopeLen;
							diffRopeLen = len - lengthFt;
							if (diffRopeLen < 0) {
								diffRopeLen = 0; // rope does not contract!
							}
							double ropeforce = diffRopeLen * kRope;
							double deltaRopeforce = dt > 0 ? (ropeforce - lastRopeForce) / dt : 0;
							lastRopeForce = ropeforce;

							DXYZ posWorld;
							DXYZ orient;
							DXYZ vel;
							DXYZ orientspeed;
							ptrPickupObject->GetPosition(posWorld, orient, vel, orientspeed);
							DXYZ posLocal = convertLonAltLat2XYZLocal(posWorld, earthRadius, refLon, refLat);

							DXYZ force; // actually the accel (already divided by load mass; ropeforce should by ropeforce/m, we assume the rope is accordingly thick for each mass: ropeforece->ropeforce*m)
							force.dX = dDeltaN.dX * (ropeforce - damping*deltaRopeforce) - vel.dX*speedforce;
							force.dY = dDeltaN.dY * (ropeforce - damping*deltaRopeforce) - vel.dY*speedforce; // - MetersToFeet(9.81) nope, sim does that already!
							force.dZ = dDeltaN.dZ * (ropeforce - damping*deltaRopeforce) - vel.dZ*speedforce;

							posLocal.dX += (vel.dX*dt + force.dX*dt*dt / 2);
							posLocal.dY += (vel.dY*dt + force.dY*dt*dt / 2);
							posLocal.dZ += (vel.dZ*dt + force.dZ*dt*dt / 2);

							vel.dX += (force.dX * dt);
							vel.dY += (force.dY * dt);
							vel.dZ += (force.dZ * dt);

							posWorld = convertXYZLocal2LonAltLat(posLocal, earthRadius, refLon, refLat);

							if (posLocal.dY - fElevation >= this->fStaticCGHeight)
							{
								// Pitch oriented relative to invisible string:
								//vdOrient.dX = -atan( delta.dX / delta.dY );
								//vdOrient.dZ = -atan( delta.dZ / delta.dY );

								ptrPickupObject->SetPosition(posWorld, vdOrient, vel, velRot, false, 0); // orientation still like in no physcis case? Or oriented to invisible cable?
							}
							else  // on ground? no deeper!
							{
								vdOrient.dX = 0; // no pitch on ground!
								vdOrient.dZ = 0; // no bank on ground!
								posWorld.dY = this->fStaticCGHeight + fElevation; // set it on the ground
								ptrPickupObject->SetPosition(posWorld, vdOrient, vel, velRot, true, 0); // orientation plane to ground
							}

							//pRope->SetFixed(true, true);   // startpoint fixed
							pRope->SetFixed(false, true); // endpoint fixed too

							P3DFXYZ deltaEnd;
							deltaEnd.fX = -delta.dX;
							deltaEnd.fY = -delta.dY;
							deltaEnd.fZ = -delta.dZ;
							pRope->SetEnd(deltaEnd);
							//P3DFXYZ end = pRope->GetEnd();

							// ------------------------------------
							// Apply backforce to helicopter:
							// ------------------------------------
							spUserObject->GetPosition(posWorld, orient, vel, orientspeed);
							posLocal = convertLonAltLat2XYZLocal(posWorld, earthRadius, refLon, refLat);

							double mLoad = massLoad; /* 200;
							hRes = ptrPickupObject->GetProperty(L"EMPTY WEIGHT", L"lbs", mLoad, 0);*/
							printf("Load mass %lf\n", mLoad);

							force.dX = -dDeltaN.dX *ropeforce*mLoad;  //dDeltaN.dX * (ropeforce - damping*deltaRopeforce) - vel.dX*speedforce;
							force.dY = -dDeltaN.dY *ropeforce*mLoad; // dDeltaN.dY * (ropeforce - damping*deltaRopeforce) - MetersToFeet(9.81) - vel.dY*speedforce;
							force.dZ = -dDeltaN.dZ *ropeforce*mLoad; // dDeltaN.dZ * (ropeforce - damping*deltaRopeforce) - vel.dZ*speedforce;

							DXYZ forceInHeliCoord;
							spUserObject->RotateWorldToBody(force, forceInHeliCoord);    // from world to helicopter system

							double heliMass = 100;
							hRes = spUserObject->GetProperty(L"TOTAL WEIGHT", L"lbs", heliMass, 0);
							printf("Heli mass %lf\n", heliMass);

							// Linear forces:
							DXYZ accel;
							accel.dX = force.dX / heliMass;
							accel.dY = force.dY / heliMass;
							accel.dZ = force.dZ / heliMass;

							posLocal.dX += (vel.dX*dt + accel.dX*dt*dt / 2);
							posLocal.dY += (vel.dY*dt + accel.dY*dt*dt / 2);
							posLocal.dZ += (vel.dZ*dt + accel.dZ*dt*dt / 2);

							vel.dX += (accel.dX * dt);
							vel.dY += (accel.dY * dt);
							vel.dZ += (accel.dZ * dt);

							// angular forces (i.e. moments): (assuming reference position = center of gravity)

							// forceInHeliCoord.dX and forceInHeliCoord.dZ are the forces creating a moment on bank and pitch (if hoist is on symmetry axis)
							DXYZ moment = crossProduct(forceInHeliCoord, -hoistAttach);
							
							double bankMoment  = moment.dZ; // forceInHeliCoord.dX * hoistAttach.dY + forceInHeliCoord.dY * hoistAttach.dX;
							double pitchMoment = moment.dX; // forceInHeliCoord.dZ *hoistAttach.dY;
							double yawMoment =   moment.dY; // forceInHeliCoord.dX * hoistAttach.dY + forceInHeliCoord.dY * hoistAttach.dX;

							double bankAccel = bankMomentOfInertia!=0? bankMoment / SlugsToLbs(bankMomentOfInertia) : 0 ;
							double pitchAccel = pitchMomentOfInertia!=0? pitchMoment / SlugsToLbs(pitchMomentOfInertia) : 0 ;
							double yawAccel = yawMomentOfInertia != 0 ? yawMoment / SlugsToLbs(yawMomentOfInertia) : 0;

							// orientation is PHB:
							orient.dX += (orientspeed.dX*dt + pitchAccel*dt*dt / 2);
							orient.dY += (orientspeed.dY*dt + yawAccel*dt*dt / 2);
							orient.dZ += (orientspeed.dZ*dt + bankAccel*dt*dt / 2);

							orientspeed.dX += (pitchAccel * dt);
							orientspeed.dY += (yawAccel * dt);
							orientspeed.dZ += (bankAccel * dt);

							posWorld = convertXYZLocal2LonAltLat(posLocal, earthRadius, refLon, refLat);
							spUserObject->SetPosition(posWorld, orient, vel, orientspeed, false, 0); // orientation still like in no physcis case? Or oriented to invisible cable?
						}
					}
					else
					{
						pRope->SetFixed(false, false); // endpoint swinging free

						if (dEndOverGround > -this->hoistAttachLoad.dY + this->fStaticCGHeight)
						{
							ptrPickupObject->SetPosition(finalEndPos, vdOrient, vel, velRot, false, 0);

							// printf("endOverGround: %lf\n", dEndOverGround);
						}
						else {
							// Not yet lifting (or sinking into the ground)
							// printf("nonlift endOverGround: %lf\n", dEndOverGround);

							finalEndPos.dY = this->fStaticCGHeight + fElevation; // set it on the ground
							ptrPickupObject->SetPosition(finalEndPos, vdOrient, vel, velRot, true, 0);
						}
					}
				}
				else {
					printf("GetObject failed for %d\n", objectToPickUpID);
					m_bForcePhysics = false;
					objectToPickUpID = 0;
					pRope->SetFixed(false, false); // endpoint free if we lost the object
				}
			}
		}

		// Textout
		IObjectRendererV440 *pV440Render = NULL;
		m_spRenderService->QueryInterface(IID_IObjectRendererV440, (void **)&pV440Render);
		if (pV440Render) {
			wchar_t szText[1024];
			ShowConfig(szText, 1024);

			ARGBColor colorText(255, 200, 12, 12);
			TextDescription textDescr;
			textDescr.HorizontalAlignment = HORIZONTAL_ALIGNMENT_LEFT;
			textDescr.Font = TEXT_FONT_DEFAULT;
			RenderFlags renderFlags = { 0 };
			pV440Render->DrawText2D(25, 25, szText, colorText, textDescr, renderFlags);

			pV440Render->Release();
		}
    }

	void UntieObject()
	{
		m_bUntieObject = true;
		objectToPickUpID = 0;
		setForcePhysics(false);

		pRope->SetFixed(false, false); // endpoint free
	}

	void SetObjectPlacement(CALLBACK_IDS m_EventID)
	{
		// clear checks:
		m_spPickupObjectsPlacementAtHoist->SetChecked(false);
		m_spPickupObjectsPlacementUnderHeli->SetChecked(false);
		m_spPickupObjectsPlacement5mAhead->SetChecked(false);
		m_spPickupObjectsPlacement10mAhead->SetChecked(false);
		m_spPickupObjectsPlacement100mAhead->SetChecked(false);
		m_spPickupObjectsPlacement1nmAhead->SetChecked(false);
		m_spPickupObjectsPlacement5nmAhead->SetChecked(false);
		m_spPickupObjectsPlacement10nmAhead->SetChecked(false);

		switch (m_EventID)
		{
		case PICKUP_OBJECT_PLACE_ATTHOIST:
			m_bPlaceAtHoist = true;
			m_PlaceDistanceInM = 0;
			m_spPickupObjectsPlacementAtHoist->SetChecked(true);
			break;

		case PICKUP_OBJECT_PLACE_UNDER:
			m_bPlaceAtHoist = false;
			m_PlaceDistanceInM = 0;
			m_spPickupObjectsPlacementUnderHeli->SetChecked(true);
			break;

		case PICKUP_OBJECT_PLACE_5mAHEAD:
			m_bPlaceAtHoist = false;
			m_PlaceDistanceInM = 50; // nope, 5 is nonsens, 50 is better
			m_spPickupObjectsPlacement5mAhead->SetChecked(true);
			break;

		case PICKUP_OBJECT_PLACE_10mAHEAD:
			m_bPlaceAtHoist = false;
			m_PlaceDistanceInM = 10;
			m_spPickupObjectsPlacement10mAhead->SetChecked(true);
			break;

		case PICKUP_OBJECT_PLACE_100mAHEAD:
			m_bPlaceAtHoist = false;
			m_PlaceDistanceInM = MetersToFeet(100);
			m_spPickupObjectsPlacement100mAhead->SetChecked(true);
			break;

		case PICKUP_OBJECT_PLACE_1nmAHEAD:
			m_bPlaceAtHoist = false;
			m_PlaceDistanceInM = MilesToMeters(1);
			m_spPickupObjectsPlacement1nmAhead->SetChecked(true);
			break;

		case PICKUP_OBJECT_PLACE_5nmAHEAD:
			m_bPlaceAtHoist = false;
			m_PlaceDistanceInM = MilesToMeters(5);
			m_spPickupObjectsPlacement5nmAhead->SetChecked(true);
			break;

		case PICKUP_OBJECT_PLACE_10nmAHEAD:
			m_bPlaceAtHoist = false;
			m_PlaceDistanceInM = MilesToMeters(10);
			m_spPickupObjectsPlacement10nmAhead->SetChecked(true);
			break;
		}
	}		

	void ShowConfig( wchar_t *szText, size_t bufSize )
	{
		swprintf_s(szText, bufSize, L"ropelen: %lf diffRopeLen: %lf aboveground: %.2f, elev: %.2f, Hoist length: %.2lf ft, pickup radius: %.2f m objid %d attach: %.2lf %.2lf %.2lf mass: %.2f cg: %.2f", 
							lastRopeLen, diffRopeLen, fSlingEndRadarAltitude, fElevation, lengthFt, pickupradius, objectToPickUpID, hoistAttach.dX, hoistAttach.dY, hoistAttach.dZ, fRopeMass, fStaticCGHeight);
	}

	bool ReadConfigFile()
	{
		ReadConfig configReaders(false); // no wide char

		configReaders.addEntry("hoistAttach",		[this](char *pszConfig) { printf("ha:%s\n", pszConfig); sscanf(pszConfig, "%lf %lf %lf", &hoistAttach.dX, &hoistAttach.dY, &hoistAttach.dZ); } );
		configReaders.addEntry("hoistAttachLoad",	[this](char *pszConfig) { printf("hal:%s\n", pszConfig); sscanf(pszConfig, "%lf %lf %lf", &hoistAttachLoad.dX, &hoistAttachLoad.dY, &hoistAttachLoad.dZ); }) ;
		configReaders.addEntry("ropeMass",			[this](char *pszConfig) { printf("m:%s\n", pszConfig); sscanf(pszConfig, "%f", &fRopeMass); }) ;
		configReaders.addEntry("static_cg_height",	[this](char *pszConfig) { printf("cg:%s\n", pszConfig); sscanf(pszConfig, "%f", &fStaticCGHeight); }) ;
		configReaders.addEntry("rope_k",			[this](char *pszConfig) { printf("k:%s\n", pszConfig); sscanf(pszConfig, "%lf", &kRope); });
		configReaders.addEntry("damping",			[this](char *pszConfig) { printf("damp:%s\n", pszConfig); sscanf(pszConfig, "%lf", &damping); });
		configReaders.addEntry("speedforce",		[this](char *pszConfig) { printf("sf:%s\n", pszConfig); sscanf(pszConfig, "%lf", &speedforce); });
		configReaders.addEntry("pickupObject",		[this](char *pszConfig) { printf("po:%s\n", pszConfig); char achPickupObject[256] = "";  sscanf(pszConfig, "%s", achPickupObject); pickupObjectTitle = achPickupObject; });
		configReaders.addEntry("pickupObjectDir",   [this](char *pszConfig) { printf("podir:%s\n", pszConfig); char achPickupObjectDir[256] = "";  sscanf(pszConfig, "%s", achPickupObjectDir); pickupObjectDirectory = achPickupObjectDir; });
		configReaders.addEntry("debug",				[this](char *pszConfig) { printf("db:%s\n", pszConfig); sscanf(pszConfig, "%d", &bDebug ); });

		configReaders.addEntry("empty_weight_pitch_MOI", [this](char *pszConfig) { printf("empty_weight_pitch_MOI:%s\n", pszConfig); sscanf(pszConfig, "%lf", &pitchMomentOfInertia); });
		configReaders.addEntry("empty_weight_roll_MOI", [this](char *pszConfig) { printf("empty_weight_roll_MOI:%s\n", pszConfig); sscanf(pszConfig, "%lf", &bankMomentOfInertia); });
		configReaders.addEntry("empty_weight_yaw_MOI", [this](char *pszConfig) { printf("empty_weight_yaw_MOI:%s\n", pszConfig); sscanf(pszConfig, "%lf", &yawMomentOfInertia); });
		
		return configReaders.readFile(hoistConfigPath.c_str());
	}
};


///----------------------------------------------------------------------------
/// Prepar3D DLL start and end entry points
///----------------------------------------------------------------------------

static HoistP3DPlugin *s_pCustomObjectsPlugin = nullptr;

void __stdcall DLLStart(__in __notnull IPdk* pPdk)
{
    PdkServices::Init(pPdk);
    s_pCustomObjectsPlugin = new HoistP3DPlugin();
}

void __stdcall DLLStop(void)
{
    if (s_pCustomObjectsPlugin != nullptr)
    {
        delete s_pCustomObjectsPlugin;
    }
    PdkServices::Shutdown();
}

void HoistP3DPlugin::MenuCallback::Invoke(P3D::IParameterListV400 * pParams)
{
	if (m_EventID > PICKUP_OBJECT_BASE) {
		//printf("Abot to create %s\n", this->name.c_str());

		s_pCustomObjectsPlugin->pickupObjectTitle = this->name;
		s_pCustomObjectsPlugin->CreateAndAttachObject();
		return;
	}

	switch (m_EventID)
	{
	case ENABLE:
		s_pCustomObjectsPlugin->m_bEnabled = !s_pCustomObjectsPlugin->m_bEnabled;		
		if (s_pCustomObjectsPlugin->m_bEnabled) {
			s_pCustomObjectsPlugin->BuildMenuAndConfig(); // will create a hoist menu if hoistConfig.txt is present
		} else {
			s_pCustomObjectsPlugin->removeRopeAndPickedupObject(); // make sling disappear
			s_pCustomObjectsPlugin->RemoveTopLevelMenu();

			s_pCustomObjectsPlugin->objectSimProps.clear();
		}
		s_pCustomObjectsPlugin->m_spMenuAddon->SetChecked(s_pCustomObjectsPlugin->m_bEnabled);
		break;

	case PICKUP_OBJECT_PLACE_ATTHOIST:
	case PICKUP_OBJECT_PLACE_UNDER:
	case PICKUP_OBJECT_PLACE_5mAHEAD:
	case PICKUP_OBJECT_PLACE_10mAHEAD:
	case PICKUP_OBJECT_PLACE_100mAHEAD:
	case PICKUP_OBJECT_PLACE_1nmAHEAD:
	case PICKUP_OBJECT_PLACE_5nmAHEAD:
	case PICKUP_OBJECT_PLACE_10nmAHEAD:

		s_pCustomObjectsPlugin->SetObjectPlacement(m_EventID);
		break;

	case CREATE_AND_ATTACH_OBJECT:
	{
		s_pCustomObjectsPlugin->CreateAndAttachObject();
		break;
	}
	case UNTIE_OBJECT:
	{
		s_pCustomObjectsPlugin->m_bUntieObject = !s_pCustomObjectsPlugin->m_bUntieObject;
		if (s_pCustomObjectsPlugin->m_bUntieObject == true) {
			s_pCustomObjectsPlugin->UntieObject();
		}
		s_pCustomObjectsPlugin->m_spUntieObject->SetChecked(s_pCustomObjectsPlugin->m_bUntieObject);
		break;
	}
	case FORCE_PHYSICS:
	{
		s_pCustomObjectsPlugin->setForcePhysics(!s_pCustomObjectsPlugin->m_bForcePhysics);
		// s_pCustomObjectsPlugin->lastRopeLen = 0;
		break;
	}
	case READ_CONFIG:
	{
		s_pCustomObjectsPlugin->ReadConfigFile();
		s_pCustomObjectsPlugin->InitPickupObjectsMenu();
		break;
	}

	case CAPTURE_VIS:
	{
		s_pCustomObjectsPlugin->bCaptVis = !s_pCustomObjectsPlugin->bCaptVis;
		s_pCustomObjectsPlugin->m_spMenuCaptureVisible->SetChecked(s_pCustomObjectsPlugin->bCaptVis);
		break;
	}
	case CAPTURE_SIZE_UP:
	{
		s_pCustomObjectsPlugin->pickupradius *= 1.25;
		break;
	}
	case CAPTURE_SIZE_DOWN:
	{
		s_pCustomObjectsPlugin->pickupradius /= 1.25;
		break;
	}
	case DRAW_OBJS:
	{
		s_pCustomObjectsPlugin->m_bDrawUserSimObjects = !s_pCustomObjectsPlugin->m_bDrawUserSimObjects;
		s_pCustomObjectsPlugin->m_spMenuDrawObjs->SetChecked(s_pCustomObjectsPlugin->m_bDrawUserSimObjects);
		break;
	}

	case HOIST_UP:
	{
		s_pCustomObjectsPlugin->m_bHoistUp = !s_pCustomObjectsPlugin->m_bHoistUp;
		if (s_pCustomObjectsPlugin->m_bHoistUp) {
			s_pCustomObjectsPlugin->m_bHoistDown = false;
		}
		s_pCustomObjectsPlugin->m_spMenuHoistUp->SetChecked(s_pCustomObjectsPlugin->m_bHoistUp);
		s_pCustomObjectsPlugin->m_spMenuHoistDown->SetChecked(s_pCustomObjectsPlugin->m_bHoistDown);
		break;
	}
	case HOIST_DOWN:
	{
		s_pCustomObjectsPlugin->m_bHoistDown = !s_pCustomObjectsPlugin->m_bHoistDown;
		if (s_pCustomObjectsPlugin->m_bHoistDown) {
			s_pCustomObjectsPlugin->m_bHoistUp = false;
		}
		s_pCustomObjectsPlugin->m_spMenuHoistUp->SetChecked(s_pCustomObjectsPlugin->m_bHoistUp);
		s_pCustomObjectsPlugin->m_spMenuHoistDown->SetChecked(s_pCustomObjectsPlugin->m_bHoistDown);
		break;
	}

	default:
		break;
	}
}
