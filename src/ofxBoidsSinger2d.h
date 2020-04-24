#pragma once
#include "ofMain.h"
/*
	boids2d 08/2005 a.sier / jasch adapted from boids by eric singer � 1995-2003 eric l. singer
	free for non-commercial use
*/
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
const double		kAttractWeight	= 0.300;// attraction point seeking
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
// const double		kFlyRectFront	= 1.0;
// const double		kFlyRectBack	= -1.0;


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
// Point3d; // ok, you can play with both dimensions

// typedef struct Box3D {
typedef struct Box2d {
	double		left, right;
	double		top, bottom;
//	double		front, back;
}
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
	void		*out1, *out2; // 2 outlets
	short		mode;
	// long		numBoids;
	short		numBoids;
	// long		numNeighbors;
	short		numNeighbors;
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
	// Boid2dPtr	boid;
	Boid2dPtr	boids;
	double 		d2r, r2d;
} FlockObject2d, *Flock2dPtr;

// // variables
// void*	bflock;
// t_symbol *ps_nothing;



// prototypes
// void main();
// void* Flock_new(long numBoids, long mode);
// void Flock_free(FlockPtr bflockPtr);
// void Flock_assist(FlockPtr bflockPtr, void* temp, long letType, long letNum, char *assistStr);
// void Flock_bang(FlockPtr bflockPtr);
// void Flock_dump(FlockPtr bflockPtr);
// void Flock_mode(FlockPtr bflockPtr, long arg);
// void Flock_numNeighbors(FlockPtr bflockPtr, long arg);
// void Flock_numBoids(FlockPtr bflockPtr, long arg);
// void Flock_donumBoids(FlockPtr bflockPtr, Symbol *msg, short argc, Atom *argv);
// void Flock_minSpeed(FlockPtr bflockPtr, double arg);
// void Flock_maxSpeed(FlockPtr bflockPtr, double arg);
// void Flock_centerWeight(FlockPtr bflockPtr, double arg);
// void Flock_attractWeight(FlockPtr bflockPtr, double arg);
// void Flock_matchWeight(FlockPtr bflockPtr, double arg);
// void Flock_avoidWeight(FlockPtr bflockPtr, double arg);
// void Flock_wallsWeight(FlockPtr bflockPtr, double arg);
// void Flock_edgeDist(FlockPtr bflockPtr, double arg);
// void Flock_speedupFactor(FlockPtr bflockPtr, double arg);
// void Flock_inertiaFactor(FlockPtr bflockPtr, double arg);
// void Flock_accelFactor(FlockPtr bflockPtr, double arg);
// void Flock_prefDist(FlockPtr bflockPtr, double arg);
// void Flock_flyRect(FlockPtr bflockPtr, Symbol *msg, short argc, Atom *argv);
// void Flock_attractPt(FlockPtr bflockPtr, Symbol *msg, short argc, Atom *argv);
// void Flock_reset(FlockPtr bflockPtr);
// void Flock_resetBoids(FlockPtr bflockPtr);
// void InitFlock(FlockPtr bflockPtr);
// void FlightStep(FlockPtr bflockPtr);
// Point3d FindFlockCenter(FlockPtr bflockPtr);
// float MatchAndAvoidNeighbors(FlockPtr bflockPtr, short theBoid, Velocity *matchNeighborVel, Velocity *avoidNeighborVel);
// Velocity SeekPoint(FlockPtr bflockPtr, short theBoid, Point3d seekPt);
// Velocity AvoidWalls(FlockPtr bflockPtr, short theBoid);
// Boolean InFront(BoidPtr theBoid, BoidPtr neighbor);
// void NormalizeVelocity(Velocity *direction);
// double RandomInt(double minRange, double maxRange);
// double DistSqrToPt(Point3d firstPoint, Point3d secondPoint);
//
// void Flock_set_pos(FlockPtr bflockPtr, Symbol *msg, short argc, Atom *argv);
// void Flock_set_dir(FlockPtr bflockPtr, Symbol *msg, short argc, Atom *argv);
// void Flock_set_speed(FlockPtr bflockPtr, Symbol *msg, short argc, Atom *argv);
// void Flock_set_speedinv(FlockPtr bflockPtr, Symbol *msg, short argc, Atom *argv);




class ofxBoidsSinger2D {
protected:
// variables
	void*	bflock;
	// t_symbol *ps_nothing;
	char *ps_nothing = nullptr;

	FlockPtr flock2d;

public:

	void setup(int nboids, int mode){

	}

	void update( ) {

	}

};
