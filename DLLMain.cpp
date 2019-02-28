// Copyright (c) 2010-2018 Lockheed Martin Corporation. All rights reserved.
// Use of this file is bound by the PREPAR3D® SOFTWARE DEVELOPER KIT END USER LICENSE AGREEMENT 

// Custom PDK Objects
// Description: This example uses the PDK to render custom PDK Objects.

#include "initpdk.h"
#include <WinError.h>
#include "PdkPlugin.h"
#include <ctime>

#include <string>

#define PI                  3.141592653589793238462643
#define PI_OVER_2           (PI / 2)
#define TWO_PI              (2*PI)
#define RadToDeg(rad)       ((rad) * (360/TWO_PI))
#define DegToRad(deg)       ((deg) * (TWO_PI/360))

#define FeetToMeters(feet)  ((feet)*(0.3048))
#define MetersToFeet(feet)  ((feet)/(0.3048))

using namespace P3D;

class CustomPDKObjectsPlugin : public PdkPlugin
{
	double lengthFt = 5;
	double maxLengthFt = 100;
	double minLengthFt = 5;
	double hoistSpeed = 1; // in feet per second
	clock_t lastTickHoistMove = 0;

	bool bCaptVis = false;

	float pickupradius = 1;

	DXYZ hoistAttach = { 0, 0, 0 };
	DXYZ hoistAttachLoad = { 0, 0, 0 };

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

	enum CALLBACK_IDS {
		DRAW_GRID,
		DRAW_OBJS,
		HOIST_DOWN,
		HOIST_UP,
		CAPTURE_SIZE_UP,
		CAPTURE_SIZE_DOWN,
		CAPTURE_VIS,
		READ_CONFIG
	};

public:

    CustomPDKObjectsPlugin() : PdkPlugin()
    {
        m_bDrawGroundObjectGrid = true;
		m_bHoistUp = false;
		m_bHoistDown = false;

        m_uOneSecondTick = 0;

        m_spMenuTop.Attach(P3D::PdkServices::GetMenuService()->CreateMenuItem());
        m_spMenuTop->SetType(P3D::MenuTypePdk::MENU_ITEM);
        m_spMenuTop->SetText(L"Hoist");
        P3D::PdkServices::GetMenuService()->AddItem(m_spMenuTop->GetId(), NO_PARENT, 1);


        m_spMenuDrawObjs.Attach(P3D::PdkServices::GetMenuService()->CreateMenuItem());
        m_spMenuDrawObjs->SetType(P3D::MenuTypePdk::MENU_CHECK_ITEM);
        m_spMenuDrawObjs->SetChecked(true);
        m_spMenuDrawObjs->SetText(L"Attach/Detach Hoist");
        CustomPDKObjectsPlugin::MenuCallback * callback2 = new CustomPDKObjectsPlugin::MenuCallback(DRAW_OBJS);
        m_spMenuDrawObjs->RegisterCallback(callback2);
        P3D::PdkServices::GetMenuService()->AddItem(m_spMenuDrawObjs->GetId(), m_spMenuTop->GetId(), 0);

		m_spMenuHoistDown.Attach(P3D::PdkServices::GetMenuService()->CreateMenuItem());
		m_spMenuHoistDown->SetType(P3D::MenuTypePdk::MENU_CHECK_ITEM);
		m_spMenuHoistDown->SetChecked(false);
		m_spMenuHoistDown->SetText(L"Hoist down");
		m_spMenuHoistDown->RegisterCallback(new CustomPDKObjectsPlugin::MenuCallback(HOIST_DOWN));
		P3D::PdkServices::GetMenuService()->AddItem(m_spMenuHoistDown->GetId(), m_spMenuTop->GetId(), 0);

		m_spMenuHoistUp.Attach(P3D::PdkServices::GetMenuService()->CreateMenuItem());
		m_spMenuHoistUp->SetType(P3D::MenuTypePdk::MENU_CHECK_ITEM);
		m_spMenuHoistUp->SetChecked(false);
		m_spMenuHoistUp->SetText(L"Hoist up");
		m_spMenuHoistUp->RegisterCallback(new CustomPDKObjectsPlugin::MenuCallback(HOIST_UP));
		P3D::PdkServices::GetMenuService()->AddItem(m_spMenuHoistUp->GetId(), m_spMenuTop->GetId(), 0);

		createPulldownMenu(m_spMenuCaptureSizeUp,   L"Increase capture size", CAPTURE_SIZE_UP, false);
		createPulldownMenu(m_spMenuCaptureSizeDown, L"Decrease capture size", CAPTURE_SIZE_DOWN, false);

		createPulldownMenu(m_spMenuCaptureVisible, L"Capture sphere visible", CAPTURE_VIS, true);

		createPulldownMenu(m_spMenuReadConfig, L"Read config", READ_CONFIG, true);
    }

	void createPulldownMenu( CComPtr<P3D::IMenuItemV410> &menu, wchar_t *pszText, CustomPDKObjectsPlugin::CALLBACK_IDS callbackid, bool bCheckItem )
	{
		menu.Attach(P3D::PdkServices::GetMenuService()->CreateMenuItem());
		menu->SetType(bCheckItem?P3D::MenuTypePdk::MENU_CHECK_ITEM: P3D::MenuTypePdk::MENU_ITEM);
		if (bCheckItem) {
			menu->SetChecked(false);
		}
		menu->SetText(pszText);
		menu->RegisterCallback(new CustomPDKObjectsPlugin::MenuCallback(callbackid));
		P3D::PdkServices::GetMenuService()->AddItem(menu->GetId(), m_spMenuTop->GetId(), 0);
	}

    ///----------------------------------------------------------------------------
    ///  PdkPlugin Callback overrides
    ///----------------------------------------------------------------------------

	boolean bInitRope = true ;

	IRopeSimulationV420 *pRope = NULL;

	UINT objectToPickUpID = 0;

    virtual void OnCustomRender(IParameterListV400* pParams) override
    {
        // Get the Object Renderer service from the callback params
        CComPtr<IObjectRendererV400>    spRenderService = NULL;
		CComPtr<IVisualEffectManagerV430> ptrVisEffect = NULL;
		CComPtr<ISimObjectManagerV440> ptrSimObjectManager = NULL;

		HRESULT hResRender = pParams->GetServiceProvider()->QueryService(SID_ObjectRenderer, IID_IObjectRendererV400, (void**)&spRenderService);

		HRESULT hResVisEff = pParams->GetServiceProvider()->QueryService(SID_VisualEffectManager, IID_IVisualEffectManagerV430, (void**)&ptrVisEffect);
		if (!SUCCEEDED(hResVisEff)) {
			printf("IID_IVisualEffectManagerV430:%d\n", hResVisEff);
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
                DrawUserSimObjects(spRenderService, ptrVisEffect, pParams->GetServiceProvider());
			}
			else {
				printf("Rope will be reinited...\n");
				bInitRope = true; // force reinit of rope!
				objectToPickUpID = 0; // let go of objects

				if (pRope) {
					pRope->Release();
				}
			}
        }
    }

    virtual void OnOneHz(IParameterListV400* pParams) override
    {
        // keep track count of seconds. This is used for alternating object color.
        m_uOneSecondTick++;
    }

protected:

    ///----------------------------------------------------------------------------
    ///  Object drawing functions
    ///----------------------------------------------------------------------------

    void DrawUserSimObjects(IObjectRendererV400* pRenderer, IVisualEffectManagerV430 *pVisEffect, /* ISimObjectManagerV440 *pSimObjectManager*/ IServiceProvider* pServiceProvider)
    {
        CComPtr<IWindowV400> spWindow = NULL;
        CComPtr<IBaseObjectV400> spUserObject = NULL;
		
        ObjectWorldTransform objTrans, cameraTrans;

        // Get current window
        spWindow = PdkServices::GetWindowPluginSystem()->GetCurrentWindow();
        if (spWindow != nullptr && spWindow->GetCameraSystem() != nullptr)
        {
            // get lla and pbh of camera (degrees/meters)
            spWindow->GetCameraSystem()->GetLLA(cameraTrans.LLA.Latitude, cameraTrans.LLA.Longitude, cameraTrans.LLA.Altitude);
            spWindow->GetCameraSystem()->GetPBH(cameraTrans.PBH.Pitch, cameraTrans.PBH.Bank, cameraTrans.PBH.Heading);
        }

		P3DFXYZ vStart;
		vStart.fX = 0;
		vStart.fY = 0;
		vStart.fZ = 0;

		if (hoistAttach.dX == 0) {
			hoistAttach.dX = 4.7;  //left/right
			hoistAttach.dY = 4.95; // up/down (down is < 0)
			hoistAttach.dZ = 1.9; // fore or aft
		}

        // Get user sim from sim object manager
        PdkServices::GetSimObjectManager()->GetUserObject((IBaseObjectV400**)&spUserObject);
        if (spUserObject != nullptr)
        {
            // get position and orientation and convert from radians/feet to degrees/meters
            P3D::P3DDXYZ objLal, objPhb, lalVel, pbhVel = { 0 };
            spUserObject->GetPosition(objLal, objPhb, lalVel, pbhVel);
            objTrans.LLA.Latitude = RadToDeg(objLal.dZ);
            objTrans.LLA.Longitude = RadToDeg(objLal.dX);
            objTrans.LLA.Altitude = FeetToMeters(objLal.dY);
            objTrans.PBH.Pitch = (float)RadToDeg(objPhb.dX);
            objTrans.PBH.Bank = (float)RadToDeg(objPhb.dZ);
            objTrans.PBH.Heading = (float)RadToDeg(objPhb.dY);

			float fElevation;
			spUserObject->GetSurfaceElevation(fElevation, NULL);
			
			if (bInitRope || pRope == NULL) {

				P3DFXYZ vEnd = vStart;
				//vEnd.fZ += 30;

				float fRopeMass = 5;
				float fRestingRopeLength = minLengthFt;
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
					
					/*
					DXYZ body;
					spUserObject->RotateWorldToBody(objLal, body);
					body.dX += 0.2;
					body.dY += 4.95;
					body.dZ += 4.3;
					spUserObject->RotateBodyToWorld(body, objLal);
					*/
					pRope->SetRenderWorldPosition(objLal);

					pRope->SetRelativeGroundPosition(true /*bool 	bCheckGround*/, -minLengthFt);
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
				pRenderer->ApplyBodyRelativeOffset(objTrans, toSling, slingPos);
				
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
				
				//pRope->SetRenderWorldPosition(objLal);

				pRope->SetStart(vStart);

				P3DFXYZ end = pRope->GetEnd();

				P3DFXYZ vWind;
				vWind.fX = lalVel.dX;
				vWind.fY = lalVel.dY;
				vWind.fZ = lalVel.dZ;

				pRope->SetRelativeWind(vWind);

				float dRadarAltitude = objLal.dY - fElevation;
				pRope->SetRelativeGroundPosition(true /*bool 	bCheckGround*/, -dRadarAltitude);

				double dt = 0;
				if (m_bHoistUp || m_bHoistDown) {
					if (clock() > lastTickHoistMove) {
						clock_t clkDiff = clock() - lastTickHoistMove;
						if (lastTickHoistMove > 0) {
							dt = (double)clkDiff / CLOCKS_PER_SEC;
						}
						lastTickHoistMove = clock();
					}
				}
				else {
					lastTickHoistMove = 0;
				}
				if (m_bHoistUp) { // decrease length down until minLength:
					if (lengthFt >= minLengthFt) {
						if (dt > 0) {
							lengthFt -= hoistSpeed*dt;
							//printf("up Len: %lf\n", lengthFt);
						}
					}
				}
				else if (m_bHoistDown) { // increase length until maxLength:
					if (lengthFt <= maxLengthFt) {
						if (dt > 0) {
							lengthFt += hoistSpeed*dt;
							//printf("dn Len: %lf\n", lengthFt);
						}
					}
				}

				pRope->SetRopeLength(lengthFt);

				/*
				P3DFXYZ start = pRope->GetStart();
				printf("start: %f,%f,%f - ", start.fX, start.fY, start.fZ);
				P3DFXYZ end = pRope->GetEnd();
				printf("end: %f,%f,%f\n", end.fX, end.fY, end.fZ);				
				*/

				//Draw hoisted object
				
				DrawSimObjects(spUserObject, pRenderer, objTrans, cameraTrans, end, objPhb, hoistAttach);
			}
        }
    }

	double calcDist(DXYZ vXYZ, DXYZ endPosXYZ)
	{
		double dx = vXYZ.dX - endPosXYZ.dX;
		double dy = vXYZ.dY - endPosXYZ.dY;
		double dz = vXYZ.dZ - endPosXYZ.dZ;

		return sqrt(dx*dx + dy*dy + dz*dz);
	}

	DXYZ convertLonAltLat2XYZ(LLADegreesMeters lonAltLat)
	{
		DXYZ xyz;
		xyz.dX = DegToRad(lonAltLat.Longitude);
		xyz.dY = MetersToFeet(lonAltLat.Altitude);
		xyz.dZ = DegToRad(lonAltLat.Latitude);

		return convertLonAltLat2XYZ(xyz);
	}

	DXYZ convertLonAltLat2XYZ( DXYZ lonAltLat) // rad_ft_rad
	{
		double earthRadius = MetersToFeet(6500000); // in ft
		double r = earthRadius + lonAltLat.dY;
		double lon = lonAltLat.dX;
		double lat = lonAltLat.dZ;

		DXYZ rc;
		rc.dX = r * cos(lat) * cos(lon) ;
		rc.dY = r * cos(lat) * sin(lon) ;
		rc.dZ = r * sin(lat) ;

		return rc;
	}

	DXYZ vectorSubtract(DXYZ v1, DXYZ v2) 
	{
		DXYZ rc;

		rc.dX = v2.dX - v1.dX;
		rc.dY = v2.dY - v1.dY;
		rc.dZ = v2.dZ - v1.dZ;

		return rc;
	}

    void DrawSimObjects(const CComPtr<IBaseObjectV400> &spUserObject, IObjectRendererV400* pRenderer, ObjectWorldTransform& objTrans, ObjectWorldTransform& cameraTrans, P3DFXYZ end, P3DDXYZ orient, DXYZ hoistAttach)
    {
        //Apply body relative offsets for rectangle placement.
        /*
		ObjectLocalTransform toRight(10, 0, 0, 0, 0, 0);
        ObjectLocalTransform toLeft(-10, 0, 0, 0, 0, 0);
        ObjectLocalTransform toLeftTop(-10, 5, 0, 0, 0, 0);
        ObjectLocalTransform toRightTop(10, 5, 0, 0, 0, 0);
        ObjectWorldTransform rightOfObject;
        ObjectWorldTransform leftOfObject;
        ObjectWorldTransform leftTopOfObject;
        ObjectWorldTransform rightTopOfObject;
        pRenderer->ApplyBodyRelativeOffset(objTrans, toRight, rightOfObject);
        pRenderer->ApplyBodyRelativeOffset(objTrans, toLeft, leftOfObject);
        pRenderer->ApplyBodyRelativeOffset(objTrans, toLeftTop, leftTopOfObject);
        pRenderer->ApplyBodyRelativeOffset(objTrans, toRightTop, rightTopOfObject);
		*/

		double ft2m = 0.3048;

        //Draw a sphere around user object.
		//printf("end: %f,%f,%f\n", end.fZ, end.fY, end.fX);		
		ObjectLocalTransform toSling(FeetToMeters(hoistAttach.dX), FeetToMeters(hoistAttach.dY), FeetToMeters(hoistAttach.dZ), 0, 0, 0);

		//ObjectWorldTransform objTransTemp2 = objTrans;
		//ObjectWorldTransform offsetEndOfSling;
		//pRenderer->ApplyBodyRelativeOffset(objTrans, toSling, objTransTemp2);
		
		/*ObjectWorldTransform slingPosTemp = slingPos;
		slingPosTemp.PBH.Bank = 0;
		slingPosTemp.PBH.Heading = 0;
		slingPosTemp.PBH.Pitch = 0;
		//pRenderer->ApplyBodyRelativeOffset(objTrans, toEndOfSling, endOfSling);

		/*
		DXYZ xyzSlingECF = convertLonAltLat2XYZ(slingPos.LLA);
		DXYZ xyzObjTransECF = convertLonAltLat2XYZ(objTrans.LLA);
		DXYZ delta = vectorSubtract(xyzObjTransECF, xyzSlingECF);
		*/

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
		pRenderer->ApplyBodyRelativeOffset(objTransTemp, toEndOfSling, endOfSling); // can only be calculated from object center (as we use "BodyRelativeOffset"), while end is relativ to slingPos

		// endOfSling.LLA now in degree/m/degree
				
		// Hmm?
		//ObjectWorldTransform finalEndOfSling;
		//pRenderer->ApplyBodyRelativeOffset(endOfSling, toSling, finalEndOfSling);

		P3DDXYZ endPos;
		endPos.dX = endOfSling.LLA.Longitude / 180 * PI; // rad
		endPos.dY = endOfSling.LLA.Altitude / ft2m;		 // ft
		endPos.dZ = endOfSling.LLA.Latitude / 180 * PI;  // rad

		if (objectToPickUpID == 0) { // ready to pickup

			if (bCaptVis) {
				//ARGBColor colorSphere(32, 200, 12, 12);
				//pRenderer->DrawSphere(finalEndOfSling, pickupradius, colorSphere);

				ARGBColor colorSphere3(32, 200, 12, 200);
				pRenderer->DrawSphere(endOfSling, pickupradius, colorSphere3);

				//ARGBColor colorSphere2(64, 12, 12, 200);
				//pRenderer->DrawSphere(objTransTemp2, pickupradius, colorSphere2);
			}

			IObjectRendererV440 *pV440Render = NULL;
			pRenderer->QueryInterface(IID_IObjectRendererV440, (void **)&pV440Render );
			if (pV440Render) {
				wchar_t szText[256];
				swprintf_s(szText, 256, L"Hoist length: %lf ft, pickup radius: %f m attach: %lf %lf %lf", lengthFt, pickupradius, hoistAttach.dX, hoistAttach.dY, hoistAttach.dZ);
				ARGBColor colorText(255, 200, 12, 12);
				TextDescription textDescr;
				textDescr.HorizontalAlignment = HORIZONTAL_ALIGNMENT_LEFT;
				textDescr.Font = TEXT_FONT_DEFAULT;
				RenderFlags renderFlags = { 0 };
				pV440Render->DrawText2D(50, 50, szText, colorText, textDescr, renderFlags);

				pV440Render->Release();
			}
			
			unsigned int numOfCloseObjects = 200;
			unsigned int objids[200];
			PdkServices::GetSimObjectManager()->GetObjectsInRadius(endPos, pickupradius/ft2m, numOfCloseObjects, objids); // radius ignored if less than 2 feet? Oh my! endPos in rad_feet_rad?

			//printf("%d objects to pickup\n", numOfCloseObjects);

			for( int i=0 ; i<numOfCloseObjects ; i++ ) {
				UINT objId = objids[i];
				CComPtr<IBaseObjectV400> ptrObject;
				PdkServices::GetSimObjectManager()->GetObject(objId, &ptrObject);
				if (ptrObject) {
					if (!ptrObject->IsUser()) {						
						wchar_t achTitle[256];
						ptrObject->GetTitle(achTitle, sizeof(achTitle));
						wchar_t achPath[512];
						ptrObject->GetCfgFilePath(achPath, sizeof(achPath));

						DXYZ vLonAltLat;
						DXYZ vPHB;
						DXYZ vLonLatAltVel;
						DXYZ vPHBVel;
						ptrObject->GetPosition(vLonAltLat, vPHB, vLonLatAltVel, vPHBVel);

						DXYZ vXYZ      = convertLonAltLat2XYZ(vLonAltLat); // rad_ft_rad
						DXYZ endPosXYZ = convertLonAltLat2XYZ(endPos);     // rad_ft_rad

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
						if (dist*ft2m< pickupradius) {
							objectToPickUpID = objId;
							printf("Picking up object %d dist %lf %lf,%lf,%lf vs %lf,%lf,%lf title %ls path %ls\n", objectToPickUpID, dist, vXYZ.dX, vXYZ.dY, vXYZ.dZ, endPosXYZ.dX, endPosXYZ.dY, endPosXYZ.dZ, achTitle, achPath);
						}
						else {
							printf("Not picking up object %d dist %lf %ls ", objId, dist, achTitle );
						}
					}
				}
			}
		}
		else {

			if (bCaptVis) {
				ARGBColor colorSphere(0, 12, 200, 12);
				pRenderer->DrawSphere(endOfSling, pickupradius, colorSphere);
			}
			// Object picked up already: render it:			
			CComPtr<IBaseObjectV400> ptrPickupObject;

			HRESULT hRes = PdkServices::GetSimObjectManager()->GetObject(objectToPickUpID, &ptrPickupObject);
			if (!SUCCEEDED(hRes)) {
				printf("GetObject error: %d\n", hRes ) ;
			}
			if (ptrPickupObject != nullptr)
			{
				P3DDXYZ vdOrient ;
				vdOrient.dX = 0;
				vdOrient.dY = 0;// orient.dY;
				vdOrient.dZ = 0;

				P3DDXYZ vel = { 0 };
				P3DDXYZ velRot = vel;

				// DXYZ finalEndPos = endPos; // rad_ft_rad
				ObjectLocalTransform toAttachedLoad(FeetToMeters(hoistAttachLoad.dX), FeetToMeters(hoistAttachLoad.dY), FeetToMeters(hoistAttachLoad.dZ), 0, 0, 0);
				ObjectWorldTransform hoistAttachLoad;
				pRenderer->ApplyBodyRelativeOffset(endOfSling, toAttachedLoad, hoistAttachLoad); // can only be calculated from object center (as we use "BodyRelativeOffset"), while end is relativ to slingPos

				P3DDXYZ finalEndPos;
				finalEndPos.dX = hoistAttachLoad.LLA.Longitude / 180 * PI; // rad
				finalEndPos.dY = hoistAttachLoad.LLA.Altitude / ft2m;	   // ft
				finalEndPos.dZ = hoistAttachLoad.LLA.Latitude / 180 * PI;  // rad

				ptrPickupObject->SetPosition(finalEndPos, vdOrient, vel, velRot, false, 0);
			}
			else {
				printf("GetObject failed for %d\n", objectToPickUpID);
			}
		}
		
		/*
		ARGBColor colorSphere2(255, 200, 200, 12);
		pRenderer->DrawSphere(rightOfObject, 0.3, colorSphere2);
		*/

		/*
        //Draw a rectangle to the left of the user object.
        ARGBColor colorRectangleLeft(64, 0, 0, 125);
        pRenderer->DrawRectangle(leftOfObject, 4, 4, 4, colorRectangleLeft);

        //Draw a cylinder to the right of the user object.
        ARGBColor colorCylinder(64, 125, 0, 125);
        pRenderer->DrawCylinder(rightOfObject, 4, 4, colorCylinder);

        //Draw a line between top left and top right of user object.
        ARGBColor colorLine(64, 0, 125, 0);
        pRenderer->DrawLine(leftTopOfObject.LLA, rightTopOfObject.LLA, 4, 4, colorLine);

        //Draw a triangle above the rectangle object
        ARGBColor colorTriangle(64, 125, 125, 0);
        pRenderer->DrawTriangle(leftTopOfObject, 4, 4, 4, colorTriangle);
		*/
    }


    bool m_bDrawGroundObjectGrid;
    bool m_bDrawUserSimObjects;
	bool m_bHoistUp;
	bool m_bHoistDown;

    int m_uOneSecondTick;

private:
    class MenuCallback : public P3D::ICallbackV400
    {
    public:
        MenuCallback(CALLBACK_IDS eventID)
            : m_EventID(eventID), m_RefCount(1)
        {

        }


        virtual void Invoke(P3D::IParameterListV400* pParams) override;

        CALLBACK_IDS m_EventID;

        DEFAULT_REFCOUNT_INLINE_IMPL()
            DEFAULT_IUNKNOWN_QI_INLINE_IMPL(MenuCallback, P3D::IID_ICallbackV400)
    };
};


///----------------------------------------------------------------------------
/// Prepar3D DLL start and end entry points
///----------------------------------------------------------------------------
static CustomPDKObjectsPlugin* s_pCustomObjectsPlugin = nullptr;

void __stdcall DLLStart(__in __notnull IPdk* pPdk)
{
    PdkServices::Init(pPdk);
    s_pCustomObjectsPlugin = new CustomPDKObjectsPlugin();
}

void __stdcall DLLStop(void)
{
    if (s_pCustomObjectsPlugin != nullptr)
    {
        delete s_pCustomObjectsPlugin;
    }
    PdkServices::Shutdown();
}

void CustomPDKObjectsPlugin::MenuCallback::Invoke(P3D::IParameterListV400 * pParams)
{
    switch (m_EventID)
    {
	case READ_CONFIG:
	{
		FILE *fp = fopen("hoistConfigDummy.txt", "w+");
		if (fp) {
			fprintf(fp, "dummy");
			fclose(fp);
		}
		else {
			printf("hoistConfigDummy could not be opened!\n");
		}

#define HOISTATTACH_TOKEN "hoistAttach="
#define HOISTATTACHLOAD_TOKEN "hoistAttachLoad="

		fp = fopen("hoistConfig.txt", "r") ;
		if (fp) {
			do {
				char achLine[1024];
				achLine[0] = '\0';
				fgets(achLine, sizeof(achLine), fp);
				printf("Line: %s", achLine);
				
				if (strncmp(achLine, HOISTATTACH_TOKEN, strlen(HOISTATTACH_TOKEN)) == 0) {
					char *pszCoords = achLine + strlen(HOISTATTACH_TOKEN);
					sscanf(pszCoords, "%lf %lf %lf", &s_pCustomObjectsPlugin->hoistAttach.dX, &s_pCustomObjectsPlugin->hoistAttach.dY, &s_pCustomObjectsPlugin->hoistAttach.dZ);
				}
				else if (strncmp(achLine, HOISTATTACHLOAD_TOKEN, strlen(HOISTATTACHLOAD_TOKEN)) == 0) {
					char *pszCoords = achLine + strlen(HOISTATTACHLOAD_TOKEN);
					sscanf(pszCoords, "%lf %lf %lf", &s_pCustomObjectsPlugin->hoistAttachLoad.dX, &s_pCustomObjectsPlugin->hoistAttachLoad.dY, &s_pCustomObjectsPlugin->hoistAttachLoad.dZ);
				}

			} while (!feof(fp));
			fclose(fp);
		}
		else {
			printf("hoistConfig could not be opened!\n");
		}
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

		/*
		wchar_t achMenuText[256];
		swprintf_s(achMenuText, 256, L"Increase capture size (current: %f)", s_pCustomObjectsPlugin->pickupradius);
		s_pCustomObjectsPlugin->m_spMenuCaptureSizeDown->SetText(achMenuText);
		*/
		break;
	}
	case CAPTURE_SIZE_DOWN:
	{
		s_pCustomObjectsPlugin->pickupradius /= 1.25;

		/*
		wchar_t achMenuText[256];
		swprintf_s(achMenuText, 256, L"Decrease capture size (current: %f)", s_pCustomObjectsPlugin->pickupradius);
		s_pCustomObjectsPlugin->m_spMenuCaptureSizeDown->SetText(achMenuText);
		*/
		break;
	}
	/*
    case DRAW_GRID:
    {
        s_pCustomObjectsPlugin->m_bDrawGroundObjectGrid = !s_pCustomObjectsPlugin->m_bDrawGroundObjectGrid;
        s_pCustomObjectsPlugin->m_spMenuDrawGrid->SetChecked(s_pCustomObjectsPlugin->m_bDrawGroundObjectGrid);
        break;
    }
	*/
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
