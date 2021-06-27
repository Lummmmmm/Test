/*
******************************************************************************
**  CarMaker - Version 8.0.2
**  Vehicle Dynamics Simulation Toolkit
**
**  Copyright (C)   IPG Automotive GmbH
**                  Bannwaldallee 60             Phone  +49.721.98520.0
**                  76185 Karlsruhe              Fax    +49.721.98520.99
**                  Germany                      WWW    www.ipg-automotive.com
******************************************************************************
**
** Functions
** ---------
**
** Initialization
**
**	User_Init_First ()
**	User_PrintUsage ()
**	User_ScanCmdLine ()
**
**	User_AppLogFilter ()
**
**	User_Init ()
**	User_Register ()
**	User_DeclQuants ()
**
**	User_Param_Add ()
**	User_Param_Get ()
**
**
** Main TestRun Start/End:
**
**	User_TestRun_Start_atBegin ()
**	User_TestRun_Start_atEnd ()
**	User_TestRun_Start_StaticCond_Calc ()
**	User_TestRun_Start_Finalize ()
**	User_TestRun_RampUp ()
**
**	User_TestRun_End_First ()
**	User_TestRun_End ()
**
**
** Main Cycle:
**
**	User_In ()
**
**	User_DrivMan_Calc ()
** 	User_Traffic_Calc ()
**	User_VehicleControl_Calc ()
**	User_Brake_Calc ()           in Vhcl_Calc ()
**	User_Calc ()
**	User_Check_IsIdle ()
**
**	User_Out ()
**
**
** APO Communication:
**
**	User_ApoMsg_Eval ()
**	User_ApoMsg_Send ()
**
**	User_ShutDown ()
**	User_End ()
**	User_Cleanup ()
**
**
******************************************************************************
*/

#include <Global.h>

#if defined(WIN32)
# include <windows.h>
#endif

#include <stdlib.h>
#include <string.h>
#include <math.h>

#if defined(XENO)
# include <mio.h>
#endif

#include <CarMaker.h>
#include <Car/Vehicle_Car.h>

#include <ADASRP.h>

#include <rbs.h>

#include "IOVec.h"
#include "User.h"

/* @@PLUGIN-BEGIN-INCLUDE@@ - Automatically generated code - don't edit! */
/* @@PLUGIN-END@@ */

//TRAFFIC_OBJ_DEV_ANGLE
int test11=2;

//define the number of traffic objects here
#define NO_TRAFFIC_OBJS	1

tRoadEval *re;
tRoadRouteSpeedEval *rse;

tRoadRouteIn rin;
tRoadRouteOut rout;


tRoadLinkIn lin;
tRoadLinkOut lout;

tRoadLaneIn lain;
tRoadLaneOut laout;

tRoadMarkerOut *Mout;


tRoadGeoIn rgin;
tRoadGeoOut rgout;

double DirAngle[NO_TRAFFIC_OBJS],trf_obj_yaw[NO_TRAFFIC_OBJS],DevAng[NO_TRAFFIC_OBJS];

int UserCalcCalledByAppTestRunCalc = 0;

tUser	User;



/*
** User_Init_First ()
**
** First, low level initialization of the User module
**
** Call:
** - one times at start of program
** - no realtime conditions
**
*/

int
User_Init_First (void)
{
    memset (&User, 0, sizeof(User));


    return 0;
}



/*
** User_PrintUsage ()
**
** Print the user/application specific programm arguments
*/

void
User_PrintUsage (const char *Pgm)
{
    /* REMARK: 1 log statement for each usage line, no line breaks */
    LogUsage("\n");
    LogUsage("Usage: %s [options] [testrun]\n", Pgm);
    LogUsage("Options:\n");

#if defined(CM_HIL)
    {
	if (IO_GetDefault() != NULL)
	    printf(" -io %-12s Default I/O configuration\n", IO_GetDefault());
	const tIOConfig *cf;
	for (cf=IO_GetConfigurations(); cf->Name!=NULL; cf++)
	    LogUsage(" -io %-12s %s\n", cf->Name, cf->Description);
    }
#endif
}



/*
** User_ScanCmdLine ()
**
** Scan application specific command line arguments
**
** Return:
** - argv: last unscanned argument
** - NULL: error or unknown argument
*/

char **
User_ScanCmdLine (int argc, char **argv)
{
    const char *Pgm = argv[0];

    /* I/O configuration to be used in case no configuration was
       specified on the command line. */
    IO_SelectDefault("none" /* or "demoapp", "demorbs,demofr" etc. */);

    while (*++argv) {
	if (strcmp(*argv, "-io") == 0 && argv[1] != NULL) {
	    if (IO_Select(*++argv) != 0)
		return NULL;
	} else if (strcmp(*argv, "-h") == 0 || strcmp(*argv, "-help") == 0) {
	    User_PrintUsage(Pgm);
	    SimCore_PrintUsage(Pgm); /* Possible exit(), depending on CM-platform! */
	    return  NULL;
	} else if ((*argv)[0] == '-') {
	    LogErrF(EC_General, "Unknown option '%s'", *argv);
	    return NULL;
	} else {
	    break;
	}
    }

    return argv;
}



/*
** User_Init ()
**
** Basic initialization of the module User.o
**
** Call:
** - once at program start
** - no realtime conditions
*/

int
User_Init (void)
{
    return 0;
}



int
User_Register (void)
{

    /* @@PLUGIN-BEGIN-REGISTER@@ - Automatically generated code - don't edit! */
    /* @@PLUGIN-END@@ */

    return 0;
}



/*
** User_DeclQuants ()
**
** Add user specific quantities to the dictionary
**
** Call:
** - once at program start
** - no realtime conditions
*/

void
User_DeclQuants (void)
{
    int i;

    for (i=0; i<N_USEROUTPUT; i++) {
	char sbuf[32];
	sprintf (sbuf, "UserOut_%02d", i);
	DDefDouble (NULL, sbuf, "", &User.Out[i], DVA_IO_Out);
    }


    //TRAFFIC_OBJ_DEV_ANGLE
    for (i=0; i<NO_TRAFFIC_OBJS; i++) {
	char sbuf[64],sbuf1[64],sbuf2[64];
	sprintf (sbuf,"Yaw_Trfobj_%d", i);
	sprintf (sbuf1,"DirAngle_Trfobj_%d", i);
	sprintf (sbuf2,"DevAngle_Trfobj_%d", i);

	DDefDouble (NULL, sbuf, "rad", &trf_obj_yaw[i], DVA_IO_Out);
	DDefDouble (NULL, sbuf1, "rad", &DirAngle[i], DVA_IO_Out);
	DDefDouble (NULL, sbuf2, "rad", &DevAng[i], DVA_IO_Out);
    }


#if !defined(LABCAR)
    RBS_DeclQuants();
#endif
}


/*
** User_Param_Add ()
**
** Update all modified application specific parameters in the test stand
** parameter file (ECUParameters).
**
** If the variable SimCore.TestRig.ECUParam.Modified set to 1 somewhere else
** CarMaker calls this function to let the user add or change all necessary
** entries before the file is written.
** So, if writing the ECUParam file is necessary, set ECUParam.Modified to 1.
** The next TestRun start or end, CarMaker calls this function and writes
** the file to the harddisk.
**
** Call:
** - in a separate thread (no realtime contitions)
** - when starting a new test run
*/

int
User_Param_Add (void)
{
#if defined(CM_HIL)
    /* ECU parameters */
    if (SimCore.TestRig.ECUParam.Inf == NULL)
	return -1;
#endif

    return 0;
}



/*
** User_Param_Get ()
**
** Update all modified application specific parameters from the test stand
** parameter file (ECUParameters).
**
** Call:
** - in a separate thread (no realtime conditions)
** - if User_Param_Get() wasn't called
** - when starting a new test run, if
**   - the files SimParameters and/or
**   - ECUParameters
**   are modified since last reading
**
** return values:
**  0	ok
** -1	no testrig parameter file
** -2	testrig parameter error
** -3	i/o configuration specific error
** -4	no simulation parameters
** -5	simulation parameters error
** -6	FailSafeTester parameter/init error
*/

int
User_Param_Get (void)
{
    int rv = 0;

#if defined(CM_HIL)
    /*** testrig / ECU parameters */
    if (SimCore.TestRig.ECUParam.Inf == NULL)
	return -1;

    if (IO_Param_Get(SimCore.TestRig.ECUParam.Inf) != 0)
	rv = -2;
#endif

    /*** simulation parameters */
    if (SimCore.TestRig.SimParam.Inf == NULL)
	return -4;

    return rv;
}



/*
** User_TestRun_Start_atBegin ()
**
** Special things before a new simulation starts like
** - reset user variables to their default values
** - reset counters
** - ...
**
** Call:
** - in separate thread (no realtime conditions)
** - when starting a new test run
** - after (standard) infofiles are read in
** - before reading parameters for Environment, DrivMan, Car, ...
**   the models are NOT in the simulation-can-start-now state
**   (after Start(), before StaticCond())
*/

int
User_TestRun_Start_atBegin (void)
{
    int rv = 0;
    int i;

    for (i=0; i<N_USEROUTPUT; i++)
	User.Out[i] = 0.0;


    if (IO_None)
	return rv;

#if defined(CM_HIL)
    if (FST_New(SimCore.TestRig.ECUParam.Inf) != 0)
	rv = -6;
#endif

    return rv;
}




/*
** User_TestRun_Start_atEnd ()
**
** Special things before a new simulation starts like
** - reset user variables to there default values
** - reset counters
** - ...
**
** Call:
** - in separate thread (no realtime conditions)
** - when starting a new test run
** - at the end, behind reading parameters for Environment, DrivMan,
**   Car, ...
**   the models are NOT in the simulation-can-start-now state
**   (after Start(), before StaticCond())
*/

int
User_TestRun_Start_atEnd (void)

{
	//TRAFFIC_OBJ_DEV_ANGLE
	re = RoadNewRoadEval (Env.Road,ROAD_BUMP_NONE,ROAD_OT_SUV|ROAD_OT_LANES,NULL);
	return 0;
}



/*
** User_TestRun_Start_StaticCond_Calc ()
**
** called in non RT context
*/

int
User_TestRun_Start_StaticCond_Calc (void)
{
    return 0;
}



/*
** User_TestRun_Start_Finalize ()
**
** called in RT context
*/

int
User_TestRun_Start_Finalize (void)
{
    return 0;
}



/*
** User_TestRun_RampUp ()
**
** Perform a smooth transition of variables (e.g. I/O)
** from their current state  to the new testrun.
** This function is called repeatedly, once during each cycle, until
** it returns true (or issues an error message), so the function should
** return true if transitioning is done, false otherwise.
**
** In case of an error the function should issue an apropriate
** error message and return false;
**
** Called in RT context, in state SCState_StartSim,
** after preprocessing is done, before starting the engine.
** Please note, that in this early initialization state no calculation
** of the vehicle model takes place.
*/

int
User_TestRun_RampUp (double dt)
{
    int IsReady = 1;

    return IsReady;
}



/*
** User_TestRun_End_First ()
**
** Invoked immediately after the end of a simulation is initiated,
** but before data storage ends and before transitioning into SCState_Idle.
** - Send Scratchpad-note
** - ...
**
** Call:
** - in main task, in the main loop (real-time conditions!)
** - when a test run is finished (SimCore.State is SCState_End)
*/

int
User_TestRun_End_First (void)
{


	//TRAFFIC_OBJ_DEV_ANGLE
	if(re!=NULL){
	RoadDeleteRoadEval(re);
	re=NULL;
	}
	return 0;
}



/*
** User_TestRun_End ()
**
** Special things after the end of a simulation like
** - switch off an air compressor
** - Write something to a file
** - ...
**
** Call:
** - in separate thread (no realtime conditions)
** - when a test run is finished (SimCore.State is SCState_End<xyz>)
*/

int
User_TestRun_End (void)
{

	return 0;
}



/*
** User_In ()
**
** Assign quantities of the i/o vector to model variables
**
** Call:
** - in the main loop
** - pay attention to realtime condition
** - just after IO_In()
*/

void
User_In (const unsigned CycleNo)
{
    if (SimCore.State != SCState_Simulate)
	return;
}



/*
** User_DrivMan_Calc ()
**
** called
** - in RT context
** - after DrivMan_Calc()
*/

int
User_DrivMan_Calc (double dt)
{
    /* Rely on the Vehicle Operator within DrivMan module to get
       the vehicle in driving state using the IPG's
       PowerTrain Control model 'Generic' or similar */
    if (Vehicle.OperationState != OperState_Driving)
	return 0;

    return 0;
}


/*
** User_VehicleControl_Calc ()
**
** called
** - in RT context
** - after VehicleControl_Calc()
*/

int
User_VehicleControl_Calc (double dt)
{
    /* Rely on the Vehicle Operator within DrivMan module to get
       the vehicle in driving state using the IPG's
       PowerTrain Control model 'Generic' or similar */
    if (Vehicle.OperationState != OperState_Driving)
	return 0;

    return 0;
}



/*
** User_Brake_Calc ()
**
** called
** - in RT context
** - after Brake_Calc() in Vhcl_Calc()
*/

int
User_Brake_Calc (double dt)
{
    /* Modify the total brake torque from the brake system model Brake.Trq_tot[]
       or the target drive source torque from the brake control unit
       Brake.HydBrakeCU_IF.Trq_DriveSrc_trg[]
    */

    return 0;
}



/*
** User_Traffic_Calc ()
**
** called
** - in RT context
** - after Traffic_Calc()
*/

int
User_Traffic_Calc (double dt)
{
    if (SimCore.State != SCState_Simulate)
	return 0;

    return 0;
}



/*
** User_Calc ()
**
** called in RT context
*/

int
User_Calc (double dt)
{
    /* Starting with CM 6.0 User_Calc() will be invoked in EVERY simulation
       state. Uncomment the following line in order to restore the behaviour
       of CM 5.1 and earlier. */
    /*if (!UserCalcCalledByAppTestRunCalc) return 0;*/



    if (SimCore.State != SCState_Simulate)
	return 0;

    //TRAFFIC_OBJ_DEV_ANGLE

	tTrafficObj* t_objs;
	int i;

    double RouteDir[3];

	//if(SimCore.CycleNo%100==0)
	//{
	//Log("\nsRoad.t1:%f sRoad.t1:%f",t_objs[0]->sRoad,t_objs[1]->sRoad);

	for(i=0;i<NO_TRAFFIC_OBJS;i++){


	t_objs = Traffic_GetByTrfId(i);
	if(t_objs!=NULL){
	rin.st[0] = t_objs->sRoad;
	rin.st[1] = t_objs->tRoad;

	lin.xyz[0]=t_objs->t_0[0];
	lin.xyz[1]=t_objs->t_0[1];
	lin.xyz[2]=t_objs->t_0[2];

	lain.xyz[0]=t_objs->t_0[0];
	lain.xyz[1]=t_objs->t_0[1];
	lain.xyz[2]=t_objs->t_0[2];

	rgin.xyz[0]= Car.ConBdy1.t_0[0];

	rgin.xyz[1]= Car.ConBdy1.t_0[1];

	rgin.xyz[2]= Car.ConBdy1.t_0[2];

	if(RoadGeoEval(re,NULL,&rgin,&rgout)==ROAD_Ok){

		Log ("OnJunction: %d", rgout.onJunction);
		}else{
			   LogErrF(EC_Sim,"\nRoad route evaluation failed");
	    }



	if(re!=NULL){
	RoadEvalSetRouteByObjId (re,t_objs->Cfg.RouteObjId, 1);
	}else{
		LogErrF(EC_Init,"\nRoute Eval initialization failed");
	}

    if(RoadLaneEval(re,NULL,&lain,&laout)==ROAD_Ok){
	//RoadLinkGetAllMarkerByObjId(Env.Road,laout.lObjId,0,&*Mout);
	//Log ("ObjID: %d", laout.lObjId);
	}else{
		   LogErrF(EC_Sim,"\nRoad route evaluation failed");
    }

	//int rv=RoadLinkGeoEval(re,NULL,&lin,&lout);
	//Log ("Outputvalue: %d\n", rv);

	if( RoadLinkGeoEval(re,NULL,&lin,&lout)==ROAD_Ok){
	//Log("\ns.t0:%f u.t0:%f v.t0:%f",lout.suv[0],lout.suv[1],lout.suv[2]);
	}else{
			LogErrF(EC_Sim,"\nRoad route evaluation failed");
	}



    if(RoadRouteEval(re,NULL,RIT_ST,&rin,&rout)==ROAD_Ok){
	//Log("\ns.t0:%f u.t0:%f v.t0:%f",rout.suv[0],rout.suv[1],rout.suv[2]);

    	VEC_Normalize2D(RouteDir,rout.suv);
    	DirAngle[i] = atan2(RouteDir[1], RouteDir[0]);
    	//Log("\ns.t0:%f u.t0:%f v.t0:%f",RouteDir[0],RouteDir[1],DirAngle);
    	/* Vehicle-Yaw beschränken auf [-PI ... +PI] */
    	    trf_obj_yaw[i] = fmod(t_objs->r_zyx[2], 2*M_PI);
    		if (trf_obj_yaw[i] > M_PI)
    			trf_obj_yaw[i] -= 2*M_PI;
    		else if (trf_obj_yaw[i] < -M_PI)
    			trf_obj_yaw[i] += 2*M_PI;

    		/* Deviation Angle: Winkel zwischen Fahrzeug-Gierwinkel undprojiziertem
    		Straßen-Richtungsvektor */
    		DevAng[i] = trf_obj_yaw[i] - DirAngle[i];

    		/* DeviationAngle beschränken auf [-PI ... +PI] */
    		if (DevAng[i] > M_PI)
    			DevAng[i] -= 2*M_PI;
    		else if (DevAng[i] < -M_PI)
    			DevAng[i] += 2*M_PI;

    		//Log("\n Traffic Object:%d VhclDir Angle:%f rad Yaw Angle:%f rad Dev Angle:%f rad",i,DirAngle[i],trf_obj_yaw[i],DirAngle[i]);

	}else{
		LogErrF(EC_Sim,"\nRoad route evaluation failed");
	}


	}else{
		LogErrF(EC_Init,"\nTraffic Object %d initialization failed",i);
	}
	}
	Log("\n");
	//}



	return 0;
}



/*
** User_Check_IsIdle ()
**
** Checking, if the simulation model is in idle conditions (stand still,
** steeringwheel angle zero, cluch pedal pressed, ...).
** If reached idle state, the calculation of vehicle model and driving
** manoevers is stopped.
** Ready for start new simulation.
**
** Return:
** 1  idle state reached
** 0  else
**
** Call:
** - in main task, in the main loop
** - pay attention to realtime condition
** - while SimCore.State==SCState_EndIdleGet
*/

int
User_Check_IsIdle (int IsIdle)
{
    double val;

    /*** ECU / carmodel signals */

    /* vehicle and wheels: stand still */
    val = 0.5*kmh2ms;
    if (Vehicle.v > val
     || fabs(Vehicle.Wheel[0]->vBelt) > val || fabs(Vehicle.Wheel[1]->vBelt) > val
     || fabs(Vehicle.Wheel[2]->vBelt) > val || fabs(Vehicle.Wheel[3]->vBelt) > val) {
	IsIdle = 0;
    }

    /* SteerAngle: drive  straight forward position */
    val = 1.0*deg2rad;
    if (Vehicle.Steering.Ang > val || Vehicle.Steering.Ang < -val)
	IsIdle = 0;

    return IsIdle;
}



/*
** User_Out ()
**
** Assigns model quantities to variables of the i/o vector
**
** call:
** - in the main loop
** - pay attention to realtime condition
** - just before IO_Out();
*/

void
User_Out (const unsigned CycleNo)
{
#if !defined(LABCAR)
    RBS_OutMap(CycleNo);
#endif

    if (SimCore.State != SCState_Simulate)
	return;
}



/*
** User_ApoMsg_Eval ()
**
** Communication between the application and connected GUIs.
** Evaluate messages from GUIs
**
** Call:
** - in the main loop
** - pay attention to realtime condition
** - near the end of the main loop, if the function SimCore_ApoMsg_Eval()
**    skips the message
**
** Return:
**   0 : message evaluated
**  -1 : message not handled
*/

int
User_ApoMsg_Eval (int Ch, char *Msg, int len, int who)
{
#if defined(CM_HIL)
    /*** FailSafeTester */
    if (Ch == ApoCh_CarMaker) {
	if (FST_ApoMsgEval(Ch, Msg, len) <= 0)
	    return 0;
    }

#endif
    return -1;
}



/*
** User_ApoMsg_Send ()
**
** Communication between the application and connected GUIs.
** Sends messages to GUIs
**
** Call:
** - near the end of the main loop, in MainThread_FinishCycle()
** - pay attention to realtime condition
*/

void
User_ApoMsg_Send (double T, const unsigned CycleNo)
{
}



/*
** User_ShutDown ()
**
** Prepare application for shut down
**
** Call:
** - at end of program
** - no realtime conditions
*/

int
User_ShutDown (int ShutDownForced)
{
    int IsDown = 0;

    /* Prepare application for shutdown and return that
       shutdown conditions are reached */
    if (1) {
	IsDown = 1;
    }

    return IsDown;
}



/*
** User_End ()
**
** End all models of the user module
**
** Call:
** - one times at end of program
** - no realtime conditions
*/

int
User_End (void)
{
    return 0;
}



/*
** User_Cleanup ()
**
** Cleanup function of the User module
**
** Call:
** - one times at end of program, just before exit
** - no realtime conditions
*/

void
User_Cleanup (void)
{
}
