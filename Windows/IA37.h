#ifndef __IA37_H
#define __IA37_H

#define STRICT 1
#include "OrbiterAPI.h"
#include "Orbitersdk.h"
#include "VesselAPI.h"

const double IA37_SIZE = 10.5; //Mean radius in meters.

const double IA37_EMPTYMASS = 3300; //Empty mass in kg.

const double IA37_FUELMASS = 1500;  //Fuel mass in kg.

const double IA37_ISP = 25e3; //Fuel-specific impulse in m/s.

const double IA37_MAXMAINTH = 16e3; //Max main thrust.

const double LANDING_GEAR_OPERATING_SPEED = 0.25;

const VECTOR3 IA37_CS = {4.0023, 46.5483, 2.6094}; //Cross sections in m^2.

const double IA37_VLIFT_C = 2.4005; //Chord lenght in meters.

const double IA37_VLIFT_S = 46.5483;  //Wing area in m^2.

const double IA37_VLIFT_A = 2.0821; //Wing aspect ratio.


const double IA37_HLIFT_C = 0.689; //Chord lenght in meters.

const double IA37_HLIFT_S = 4.4252;  //Wing area in m^2.

const double IA37_HLIFT_A = 0.3644; //Wing aspect ratio.


const VECTOR3 IA37_COCKPIT_OFFSET = {0.0011, 0.0052, 2.6594};


//Define impact convex hull
//Touchdown points for gear up

static const int ntdvtx_gearup = 7;
static TOUCHDOWNVTX tdvtx_gearup[ntdvtx_gearup] = {
    {_V(0.0028, -0.5800, 0.2794), 1e7, 1e5, 3.0, 3.0},
    {_V(0.0028, -0.2800, -6.0706), 1e7, 1e5, 3.0, 3.0},
    {_V(-4.9247, -0.0098, -5.6777), 1e7, 1e5, 3.0, 3.0},
    {_V(-0.0103, 1.8772, -5.6777), 1e7, 1e5, 3.0, 3.0},
    {_V(0.0009, 0.5900, 0.1679), 1e7, 1e5, 3.0, 3.0},
    {_V(0.0009, -0.0001, 4.2095), 1e7, 1e5, 3.0, 3.0},
    {_V(4.8753, -0.0098, -5.6777), 1e7, 1e5, 3.0, 3.0}
};

//For gear down

static const int ntdvtx_geardown = 10;
static TOUCHDOWNVTX tdvtx_geardown[ntdvtx_geardown] = {
    {_V(-0.0004, -1.7648, 1.6610), 1e6, 1e5, 1.6, 0.1},
    {_V(-4.8480, -1.5333, -5.6311), 1e6, 1e5, 1.6, 0.1},
    {_V(4.8535, -1.5333, -5.6311), 1e6, 1e5, 1.6, 0.1},
    {_V(0.0028, -0.5800, 0.2794), 1e7, 1e5, 3.0, 3.0},
    {_V(0.0028, -0.2800, -6.0706), 1e7, 1e5, 3.0, 3.0},
    {_V(-4.9247, -0.0098, -5.6777), 1e7, 1e5, 3.0, 3.0},
    {_V(-0.0103, 1.8772, -5.6777), 1e7, 1e5, 3.0, 3.0},
    {_V(0.0009, 0.5900, 0.1679), 1e7, 1e5, 3.0, 3.0},
    {_V(0.0009, -0.0001, 4.2095), 1e7, 1e5, 3.0, 3.0},
    {_V(4.8753, -0.0098, -5.6777), 1e7, 1e5, 3.0, 3.0}
};

//IA37 class interface
class IA37: public VESSEL4{
    public:
        enum LandingGearStatus {GEAR_DOWN, GEAR_UP, GEAR_DEPLOYING, GEAR_STOWING} landing_gear_status;

        IA37(OBJHANDLE hVessel, int flightmodel);
        virtual ~IA37();

        void DefineAnimations(void);
        void ActivateLandingGear(LandingGearStatus action);
        void SetGearDown(void);
        void UpdateLandingGearAnimation(double);

        void clbkSetClassCaps(FILEHANDLE cfg) override;
        void clbkLoadStateEx(FILEHANDLE scn, void *vs) override;
        void clbkSaveState(FILEHANDLE scn) override;
        void clbkPostStep(double, double, double) override;
        int clbkConsumeBufferedKey(DWORD, bool, char *) override;
        bool clbkLoadVC(int id) override;

        static void hlift(VESSEL *v, double beta, double M, double Re, void *context, double *cl, double *cm, double *cd);
		static void vlift(VESSEL *v, double aoa, double M, double Re, void *context, double *cl, double *cm, double *cd);

        MESHHANDLE IA37_vc;
        unsigned int mesh_Cockpit;

    
    private:
        unsigned int anim_landing_gear;
        unsigned int anim_rudder;
        unsigned int anim_elevator;
        unsigned int anim_laileron;
        unsigned int anim_raileron;
        unsigned int anim_elevator_trim;

        double landing_gear_proc;

        bool HUD_on;

        AIRFOILHANDLE hwing;
        CTRLSURFHANDLE hlaileron, hraileron;
};

#endif //!__IA37_H