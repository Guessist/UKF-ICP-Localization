#ifndef _PARAMETER_H_
#define _PARAMETER_H_


//// 고산터널
#define START_FRAME 10 
#define END_FRAME 1480 

//// 운증터널
//#define START_FRAME 6803
//#define END_FRAME 7103 

#define STEP 1
#define KDTREE_SEARCH_NORM 3
#define LEAF_SIZE 0.15


#define REF_NUMBER 10
#define dt 0.1*(STEP)


//const char GLOBAL_PATH[] = "D:\\SNU\\VI lab\\Reserch\\Velodynedata\\Headquater\\";
//const char GLOBAL_PATH[] = "D:\\SNU\\VI lab\\Reserch\\Velodynedata\\[WSHur]150914_DragonWest\\";
const char GLOBAL_PATH[] = "D:/SNU/VI lab/Reserch/Velodynedata/[SHKim] 160418_Teheran avenue/3/";
/*----- SYSTEM CONSTANT -----*/
const double k0 = 0.9996;
const double drad = 0.01745329251994329576923690768489;

// WGS84 Model
const double eqRad = 6378137.0;
const double flat = 298.2572236;
const double EARTH_RADIUS_MAJOR = 6378137.0;
const double EARTH_RADIUS_MINOR = 6356752.3142;

// ICP parameters
const double ICP_TRANSFORM_EPSILON = 1e-7;
const double ICP_INIT_OFFSET = 2.006;
const double ICP_DIST_FROM = 3.0;
const double ICP_DIST_UNIT = 0.15;
const double ICP_DIST_TO = 0.06;
const double ICP_ITR_FROM = 10;
const double ICP_ITR_UNIT = 10;
const double ICP_EPS_FROM = 0.0000001;
const double ICP_EPS_UNIT = 0.00000001;
const double ICP_DEQUE_SIZE = 100;
const double ICP_SAMP_DIST = 120;


//////////////////////////////////////// IVN_Parsing Parameter////////////////////////////////////////
//const char* const DELIMITER2 = "	";
//const int resolution = 255;
////double distance_RL;
//const double distanceBetweenWheel = 1.633;//m
//const double wheel_radius = (24.26 + 24.5*0.4) / 100;//m
//const double PI = 3.14159265;
//const double encoder_resolution = 0.01769; // cm/pulse 1.7679
//const double TWO_PI = 2 * PI;
//
//const double degree_to_radian = 0.0174532925199432957692369076849;



#endif _PARAMETER_H_