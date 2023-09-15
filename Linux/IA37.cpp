//Copyright (c) Matías Saibene
//Licenced under the MIT Licence

//===============================================================
//                  ORBITER MODULE: IA37
//
//IA37.cpp
// Control module for IA37 vessel class
//===========================================================

#define ORBITER_MODULE
#include <cstring>
#include <cstdint>
#include "IA37.h"
#include <algorithm>
#include <cstdio>

bool LND_GEAR;

// 1. vertical lift component (code from DeltaGlider)

void VLiftCoeff (VESSEL *v, double aoa, double M, double Re, void *context, double *cl, double *cm, double *cd)
{
	const int nabsc = 9;
	static const double AOA[nabsc] = {-180*RAD,-60*RAD,-30*RAD, -2*RAD, 15*RAD,20*RAD,25*RAD,60*RAD,180*RAD};
	static const double CL[nabsc]  = {       0,      0,   -0.4,      0,    0.7,     1,   0.8,     0,      0};
	static const double CM[nabsc]  = {       0,      0,  0.014, 0.0039, -0.006,-0.008,-0.010,     0,      0};
	int i;
	for (i = 0; i < nabsc-1 && AOA[i+1] < aoa; i++);
	if (i < nabsc - 1) {
		double f = (aoa - AOA[i]) / (AOA[i + 1] - AOA[i]);
		*cl = CL[i] + (CL[i + 1] - CL[i]) * f;  // aoa-dependent lift coefficient
		*cm = CM[i] + (CM[i + 1] - CM[i]) * f;  // aoa-dependent moment coefficient
	}
	else {
		*cl = CL[nabsc - 1];
		*cm = CM[nabsc - 1];
	}
	double saoa = sin(aoa);
	double pd = 0.015 + 0.4*saoa*saoa;  // profile drag
	*cd = pd + oapiGetInducedDrag (*cl, 1.5, 0.7) + oapiGetWaveDrag (M, 0.75, 1.0, 1.1, 0.04);
	// profile drag + (lift-)induced drag + transonic/supersonic wave (compressibility) drag
}

// 2. horizontal lift component (code from DeltaGlider)

void HLiftCoeff (VESSEL *v, double beta, double M, double Re, void *context, double *cl, double *cm, double *cd)
{
	int i;
	const int nabsc = 8;
	static const double BETA[nabsc] = {-180*RAD,-135*RAD,-90*RAD,-45*RAD,45*RAD,90*RAD,135*RAD,180*RAD};
	static const double CL[nabsc]   = {       0,    +0.3,      0,   -0.3,  +0.3,     0,   -0.3,      0};
	for (i = 0; i < nabsc-1 && BETA[i+1] < beta; i++);
	if (i < nabsc - 1) {
		*cl = CL[i] + (CL[i + 1] - CL[i]) * (beta - BETA[i]) / (BETA[i + 1] - BETA[i]);
	}
	else {
		*cl = CL[nabsc - 1];
	}
	*cm = 0.0;
	*cd = 0.015 + oapiGetInducedDrag (*cl, 1.5, 0.6) + oapiGetWaveDrag (M, 0.75, 1.0, 1.1, 0.04);
}

//Constructor
IA37::IA37(OBJHANDLE hVessel, int flightmodel) : VESSEL4(hVessel, flightmodel){
    landing_gear_proc = 0.0;

    landing_gear_status = GEAR_DOWN;

    DefineAnimations();

    IA37_vc = oapiLoadMeshGlobal("IA37_cockpit");

    LND_GEAR = true;

}

//Destructor
IA37::~IA37(){

}


//Define animations
void IA37::DefineAnimations(void){

    //Landing gear
    static unsigned int FrontLandingGearGrp[2] = {2, 3};
    static MGROUP_ROTATE FrontLandingGearRotate(
        0,
        FrontLandingGearGrp,
        2,
        _V(0.0024, -0.4404, 2.0604),
        _V(1, 0, 0),
        (float)(1.0)
    );

    static unsigned int RearLandingGearGrp[3] = {5, 6, 7};
    static MGROUP_ROTATE RearLandingGearRotate(
        0,
        RearLandingGearGrp,
        3,
        _V(0.0024, -0.4404, -1.7396),
        _V(1, 0, 0),
        (float)(-1.0)
    );

    //Identify landing gear animation
    anim_landing_gear = CreateAnimation(0.0);
    AddAnimationComponent(anim_landing_gear, 0, 1, &FrontLandingGearRotate);
    AddAnimationComponent(anim_landing_gear, 0, 1, &RearLandingGearRotate);

    //Control surfaces
    static unsigned int RudderGrp[1] = {8};
    static MGROUP_ROTATE Rudder(
        0,
        RudderGrp,
        1,
        _V(-0.0108, 1.0781, -5.4007),
        _V(0, 1, 0),
        (float)(0.2617)
    );
    anim_rudder = CreateAnimation(0.5);
    AddAnimationComponent(anim_rudder, 0, 1, &Rudder);

    static unsigned int ElevatorGrp[2] = {11,12};
    static MGROUP_ROTATE Elevator(
        0,
        ElevatorGrp,
        2,
        _V(-0.0108, 0.0062, -4.9072),
        _V(1, 0, 0),
        (float)(0.2617)
    );
    anim_elevator = CreateAnimation(0.5);
    AddAnimationComponent(anim_elevator, 0, 1, &Elevator);

    static unsigned int ElevatorTrimGrp[2] = {11, 12};
    static MGROUP_ROTATE ElevatorTrim(
        0,
        ElevatorTrimGrp,
        2,
        _V(-0.0108, 0.0062, -4.9072),
        _V(1, 0, 0),
        (float)(0.1221)
    );
    anim_elevator_trim = CreateAnimation(0.5);
    AddAnimationComponent(anim_elevator_trim, 0, 1, &ElevatorTrim);

    static unsigned int LeftAileronGrp[1] = {11};
    static MGROUP_ROTATE LAileron(
        0,
        LeftAileronGrp,
        1,
        _V(-3.6393, 0.0062, -4.9072),
        _V(1, 0, 0),
        (float)(-0.2617)
    );
    anim_laileron = CreateAnimation(0.5);
    AddAnimationComponent(anim_laileron, 0, 1, &LAileron);

    static unsigned int RightAileronGrp[1] = {12};
    static MGROUP_ROTATE RAileron(
        0,
        RightAileronGrp,
        1,
        _V(3.6649, 0.0062, -4.9072),
        _V(1, 0, 0),
        (float)(0.2617)
    );
    anim_raileron = CreateAnimation(0.5);
    AddAnimationComponent(anim_raileron, 0, 1, &RAileron);

}


// Overloaded callback functions
// Set the capabilities of the vessel class
void IA37::clbkSetClassCaps(FILEHANDLE cfg){

    //Define thrusters
    THRUSTER_HANDLE th_main[2];
    THGROUP_HANDLE thg_main;

    //Physical vessel resources
    SetSize(IA37_SIZE);
    SetEmptyMass(IA37_EMPTYMASS);
    SetCrossSections(IA37_CS);
    SetMaxWheelbrakeForce (2e5);

    //Propellant resources
    PROPELLANT_HANDLE JP1 = CreatePropellantResource(IA37_FUELMASS);


    //Main engine
    th_main[0] = CreateThruster(_V(-0.7349, 0.0060, -5.7835), _V(0, 0, 1), IA37_MAXMAINTH, JP1, IA37_ISP);
    th_main[1] = CreateThruster(_V(0.7351, 0.0060, -5.7835), _V(0, 0, 1), IA37_MAXMAINTH, JP1, IA37_ISP);
    thg_main = CreateThrusterGroup(th_main, 2, THGROUP_MAIN);

    SURFHANDLE exhaust_tex = oapiRegisterExhaustTexture("Exhaust");
    AddExhaust(th_main[0], 5, 0.5, exhaust_tex);
    AddExhaust(th_main[1], 5, 0.5, exhaust_tex);
    PARTICLESTREAMSPEC exhaust_main = {
		0, 2.0, 20, 200, 0.05, 0.1, 8, 1.0, PARTICLESTREAMSPEC::EMISSIVE,
		PARTICLESTREAMSPEC::LVL_SQRT, 0, 1,
		PARTICLESTREAMSPEC::ATM_PLOG, 1e-5, 0.1
	};
    AddExhaustStream (th_main[0], _V(0,0.3,-5), &exhaust_main);
    AddExhaustStream (th_main[1], _V(0,0.3,-5), &exhaust_main);


    //Add mesh
    AddMesh("IA37");

    //Add the mesh for the virtual cockpit
    mesh_Cockpit = AddMesh(IA37_vc, &IA37_COCKPIT_OFFSET);
    SetMeshVisibilityMode(mesh_Cockpit, MESHVIS_COCKPIT);


    hlaileron = CreateControlSurface3(AIRCTRL_AILERON, 1.25, 0.2, _V(-3.9179, 0.0054, -5.3924), AIRCTRL_AXIS_XNEG, 1.0, anim_raileron);
    hraileron = CreateControlSurface3(AIRCTRL_AILERON, 1.25, 0.2, _V(3.9211, 0.0054, -5.3924), AIRCTRL_AXIS_XPOS, 1.0, anim_laileron);

    CreateControlSurface3(AIRCTRL_ELEVATOR, 3.56, 0.2, _V(0.0016, 0.0053, -5.3407), AIRCTRL_AXIS_XPOS, 1.0, anim_elevator);
    CreateControlSurface3(AIRCTRL_ELEVATORTRIM, 3.56, 0.1, _V(0.0016, 0.0053, -5.3407), AIRCTRL_AXIS_XPOS, anim_elevator_trim);

    CreateControlSurface3(AIRCTRL_RUDDER, 0.35, 0.2, _V(-0.0104, 0.9376, -5.6396), AIRCTRL_AXIS_YPOS, 1.0, anim_rudder);


    //Code from DeltaGlider
    hwing = CreateAirfoil3 (LIFT_VERTICAL, _V(0,0,0), VLiftCoeff, 0, IA37_VLIFT_C, IA37_VLIFT_S, IA37_VLIFT_A);
	// wing and body lift+drag components

	CreateAirfoil3 (LIFT_HORIZONTAL, _V(0,0,-4), HLiftCoeff, 0, 5, 15, 1.5);
	// vertical stabiliser and body lift and drag components
}


//Load landing gear status from scenario file
void IA37::clbkLoadStateEx(FILEHANDLE scn, void *vs){

    char *line;

    while(oapiReadScenario_nextline(scn, line)){
        if(!strncasecmp(line, "GEAR", 4)){
            sscanf(line+4, "%d%lf", (int *)&landing_gear_status, &landing_gear_proc);
            SetAnimation(anim_landing_gear, landing_gear_proc);
        } else {
            ParseScenarioLineEx(line, vs);
        }
    }
}

void IA37::clbkSaveState(FILEHANDLE scn){

    char cbuf[256];

    SaveDefaultState(scn);
    sprintf(cbuf, "%d %0.4f", landing_gear_status, landing_gear_proc);
    oapiWriteScenario_string(scn, "GEAR", cbuf);
}

///////////////Logic for landing gear

void IA37::SetGearDown(void){
    ActivateLandingGear((landing_gear_status == GEAR_DOWN || landing_gear_status == GEAR_DEPLOYING) ?
        GEAR_STOWING : GEAR_DEPLOYING);
}

void IA37::ActivateLandingGear(LandingGearStatus action){
    landing_gear_status = action;
}

/////////Runnings animations...

void IA37::clbkPostStep(double simt, double simdt, double mjd){
    UpdateLandingGearAnimation(simdt);
}

///////////Functions for landing gear

void IA37::UpdateLandingGearAnimation(double simdt) {
    if (landing_gear_status >= GEAR_DEPLOYING) {
        double da = simdt * LANDING_GEAR_OPERATING_SPEED;
        if (landing_gear_status == GEAR_DEPLOYING) {
            if (landing_gear_proc > 0.0) landing_gear_proc = std::max(0.0, landing_gear_proc - da);
            else landing_gear_status = GEAR_DOWN;
            LND_GEAR = true;
            //SetTouchdownPoints(tdvtx_geardown, ntdvtx_geardown);
        } else {
            if (landing_gear_proc < 1.0) landing_gear_proc = std::min(1.0, landing_gear_proc + da);
            else landing_gear_status = GEAR_UP;
            LND_GEAR = false;
            //SetTouchdownPoints(tdvtx_gearup, ntdvtx_gearup);
        }
        SetAnimation(anim_landing_gear, landing_gear_proc);
    }
    if(LND_GEAR == true){
        SetTouchdownPoints(tdvtx_geardown, ntdvtx_geardown);
    }else if (LND_GEAR == false){
        SetTouchdownPoints(tdvtx_gearup, ntdvtx_gearup);
    }
}


int IA37::clbkConsumeBufferedKey(int key, bool down, char *kstate){

    if(key == OAPI_KEY_G && down){
        SetGearDown();
        return 1;
    }
    return 0;
}

bool IA37::clbkLoadVC(int id){
    switch(id){
        case 0 : SetCameraOffset(_V(0.0016, 0.0153, -0.1937));
        SetCameraDefaultDirection(_V(0, 0, 1));
        SetCameraRotationRange(RAD*120, RAD*120, RAD*60, RAD*60);
		break;
    }
    return true;
}

////////////////////////////////////
DLLCLBK void InitModule(MODULEHANDLE hModule){

}

DLLCLBK void ExitModule(MODULEHANDLE *hModule){

}

/////////Vessel initialization

DLLCLBK VESSEL *ovcInit(OBJHANDLE hvessel, int flightmodel){

    return new IA37(hvessel, flightmodel);

}

////////////Vessel memory cleanup

DLLCLBK void ovcExit(VESSEL *vessel){

    if(vessel) delete(IA37*)vessel;

}