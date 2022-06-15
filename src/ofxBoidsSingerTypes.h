#pragma once 
// ofxBoidsSingerTypes.h

#include "ofMain.h"

// Max + C Header files, already included in ofMain.h
// #include	<ext.h>
// #include	<ext_common.h>
// #include	<stdlib.h>
// #include	<math.h>

// constants
#define			kAssistInlet	1
#define			kAssistOutlet	2
#define			kMaxLong		0xFFFFFFFF
#define			kMaxNeighbors	4

// initial flight parameters
const short			kNumBoids		= 12;	// number of boids
const short			kNumNeighbors	= 2;	// must be <= kMaxNeighbors
const double 		kMinSpeed		= 0.15;	// boids' minimum speed
const double		kMaxSpeed		= 0.25;	// boids' maximum speed
const double		kCenterWeight	= 0.25;	// bflock centering
const double		kAttractWeight	= 0.000;// attraction point seeking
const double		kMatchWeight	= 0.100;// neighbors velocity matching
const double		kAvoidWeight	= 0.10;	// neighbors avoidance
const double		kWallsWeight	= 0.500;// wall avoidance [210]
const double		kEdgeDist		= 0.5;	// vision distance to avoid wall edges [5]
const double		kSpeedupFactor	= 0.100;// alter animation speed
const double		kInertiaFactor	= 0.20;	// willingness to change speed & direction
const double		kAccelFactor	= 0.100;// neighbor avoidance accelerate or decelerate rate
const double		kPrefDist		= 0.25;	// preferred distance from neighbors
const double		kFlyRectTop		= 1.0;	// fly rect boundaries
const double		kFlyRectLeft	= -1.0;
const double		kFlyRectBottom	= -1.0;
const double		kFlyRectRight	= 1.0;
const double		kFlyRectFront	= 1.0;
const double		kFlyRectBack	= -1.0;

// typedefs
typedef struct Velocity {
	double		x;
	double		y;
//	double		z;
} Velocity;

typedef struct Point2d {
	double		x;
	double		y;
//	double		z;
} Point2d;

// typedef struct Box3D {
typedef struct Box2d {
	double		left, right;
	double		top, bottom;
//	double		front, back;
};
// Box3D;

// typedef struct Boid {
typedef struct Boid2d {
	// Point3d		oldPos;
	// Point3d		newPos;
	Point2d		oldPos;
	Point2d		newPos;
	Velocity	oldDir;
	Velocity	newDir;
	double		speed;
	short		neighbor[kMaxNeighbors];
	double		neighborDistSqr[kMaxNeighbors];
} Boid2d, *Boid2dPtr;

typedef struct FlockObject2d {
	// Object		theObject;
	void		*out1 = nullptr, *out2 = new char[0]; // 2 outlets
	short		mode;
	long		numBoids;
	// short		numBoids;
	long		numNeighbors;
	// short		numNeighbors;
	Box2d		flyRect;
	double 		minSpeed;
	double		maxSpeed;
	double		centerWeight;
	double		attractWeight;
	double		matchWeight;
	double		avoidWeight;
	double		wallsWeight;
	double		edgeDist;
	double		speedupFactor;
	double		inertiaFactor;
	double		accelFactor;
	double		prefDist;
	double		prefDistSqr;
	Point2d		centerPt;
	Point2d		attractPt;
	Boid2dPtr	boid;
	double 		d2r, r2d;
} FlockObject2d, *Flock2dPtr;



// typedefs
typedef struct Velocity3d {
	double		x;
	double		y;
	double		z;
} Velocity3d;

typedef struct Point3d {
	double		x;
	double		y;
	double		z;
} Point3d;

typedef struct Box3D {
// typedef struct Box2d {
	double		left, right;
	double		top, bottom;
	double		front, back;
} Box3d;

// typedef struct Boid {
typedef struct Boid3d {
	Point3d		oldPos;
	Point3d		newPos;
	// Point2d		oldPos;
	// Point2d		newPos;
	Velocity3d	oldDir;
	Velocity3d	newDir;
	double		speed;
	short		neighbor[kMaxNeighbors];
	double		neighborDistSqr[kMaxNeighbors];
} Boid3d, *Boid3dPtr;

typedef struct FlockObject3d {
	// Object		theObject;
	void		*out1 = nullptr, *out2 = new char[0]; // 2 outlets
	short		mode;
	long		numBoids;
	// short		numBoids;
	long		numNeighbors;
	// short		numNeighbors;
	Box3d		flyRect;
	double 		minSpeed;
	double		maxSpeed;
	double		centerWeight;
	double		attractWeight;
	double		matchWeight;
	double		avoidWeight;
	double		wallsWeight;
	double		edgeDist;
	double		speedupFactor;
	double		inertiaFactor;
	double		accelFactor;
	double		prefDist;
	double		prefDistSqr;
	Point3d		centerPt;
	Point3d		attractPt;
	Boid3dPtr	boid;
	double 		d2r, r2d;
} FlockObject3d, *Flock3dPtr;


static double RandomInt(double minRange, double maxRange)
	{
		unsigned short	qdRdm;
		double			t, result;

		// qdRdm = Random();
		qdRdm = rand() % 65536;
		t = (double)qdRdm / 65536.0; 	// now 0 <= t <= 1
		result = (t * (maxRange - minRange)) + minRange;
		return(result);
	}
