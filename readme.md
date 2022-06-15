## ofxBoidsSinger

By s373.net/x art codex studios. Ported from boids package for max/msp/jitter 051219 by a.sier, jasch, wesley smith, originally extended from Eric Singer's Max Boids external. This boids algorithm is based on Simon Fraser's implementation of Craig Reynolds' Boids algorithm. Boids is free for non-commercial use. Licensed under Gnu GPL License. Please refer to the accompanying COPYING file.

```
// ofxBoidsSinger algorithm api

post("Boids 1.7 (OSX) (c) 1995-2003 Eric L. Singer (eric@ericsinger.com)");
post("    Based on Simon Fraser's implementation");
post("    of Craig Reynolds' Boids algorithm");
post("    Boids is free for non-commercial use");


// example usage program

    include/class ofxBoidsSinger2d,3d.h

    ofxBoidsSinger2D flock2d;

    flock2d.setup(100);
    flock2d.set_minSpeed(0.100);	
    flock2d.set_maxSpeed(0.720);

    flock2d.update();

    Flock2dPtr flock  = flock2d.GetFlock();
    for(i = 0; i < flock->numBoids; ++i)
        ofRect( flock->boid[i].oldPos/newPos.x,
                flock->boid[i].oldPos/newPos.y, 5, 5 ); 

// methods
	void setNumBoids(int nboids)
	void Flock_resetBoids(Flock2dPtr bflockPtr)
	void InitFlock(Flock2dPtr bflockPtr)
	void reset(Flock2dPtr bflockPtr)
	void set_flyRect(float a, float b, float c, float d)
	void set_minSpeed(float d)
	void set_maxSpeed(float d)
	void set_centerWeight(float d)
	void set_attractWeight(float d)
	void set_matchWeight(float d)
	void set_avoidWeight(float d)
	void set_wallsWeight(float d)
	void set_speedupFactor(float d)
	void set_inertiaFactor(float d)
	void set_accelFactor(float d)
	void set_prefDist(float d)
	void set_pos(int ix, float x, float y)
	void set_dir(int ix, float x, float y)
	void set_speed(int ix, float x)
	void set_speedinv(int ix)
	void attractPt(float x, float y)
	void update() 
	void FlightStep(Flock2dPtr bflockPtr)
	double RandomInt(double minRange, double maxRange)

// vars
    bflockPtr->numNeighbors		= kNumNeighbors;
    bflockPtr->minSpeed			= kMinSpeed;
    bflockPtr->maxSpeed			= kMaxSpeed;
    bflockPtr->centerWeight		= kCenterWeight;
    bflockPtr->attractWeight	= kAttractWeight;
    bflockPtr->matchWeight		= kMatchWeight;
    bflockPtr->avoidWeight		= kAvoidWeight;
    bflockPtr->wallsWeight		= kWallsWeight;
    bflockPtr->edgeDist			= kEdgeDist;
    bflockPtr->speedupFactor	= kSpeedupFactor;
    bflockPtr->inertiaFactor	= kInertiaFactor;
    bflockPtr->accelFactor		= kAccelFactor;
    bflockPtr->prefDist			= kPrefDist;
    bflockPtr->prefDistSqr		= kPrefDist * kPrefDist;
    bflockPtr->flyRect.top		= kFlyRectTop;
    bflockPtr->flyRect.left		= kFlyRectLeft;
    bflockPtr->flyRect.bottom	= kFlyRectBottom;
    bflockPtr->flyRect.right	= kFlyRectRight;
    // bflockPtr->flyRect.front		= kFlyRectFront;
    // bflockPtr->flyRect.back		= kFlyRectBack;
    bflockPtr->attractPt.x		= (kFlyRectLeft + kFlyRectRight) * 0.5;
    bflockPtr->attractPt.y		= (kFlyRectTop + kFlyRectBottom) * 0.5;
    // bflockPtr->attractPt.z		= (kFlyRectFront + kFlyRectBack) * 0.5;

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
    // const double		kFlyRectFront	= 1.0;
    // const double		kFlyRectBack	= -1.0;

```


## boids package for max/msp/jitter 051219

```
/*
	boids package for max/msp/jitter 051219
	(universal binary version 060828)

	
	  Boids............................................. by eric singer
	  boids2d, boids3d.................................. by jasch & andr� sier
	  jit.boids3d, jit.boids2d, xray.jit.boidsrender.... by wesley smith

	� 1995-98 Eric L. Singer (eric@ericsinger.com)
	3d adaptation 08/2005 by a. sier / jasch
	jitter adaptation 12/2005 by w. smith

	boids package released under Gnu GPL license.
	please refer to the accompanying COPYING file.
*/



Based on Simon Fraser's implementation of Craig Reynolds' Boids algorithm. Boids is free for non-commercial use

Boids is a bird flight and animal flock simulator. It is based on the same algorithm which was used in Jurassic Park for the herding dinosaurs. Boids takes an integer argument which is the number of boids. Each time Boids receives a bang, it calculates and outputs the new positions of the boids. The output consists of thew coordiantes for each boid, the number and type depending on the mode.

The flight parameters can be changed with messages. Use the 'dump' message to output a list of the current parameter settings. 

For more information about the Boids algorithm, see Craig Reynolds' Web site at "http://reality.sgi.com/employees/craig/boids.html".


UB notes (version 1.1 / 20070125): 
	- changed flock structure to bflock. conflicts with fcntl.h:398
	- added set messages to externals (set pos, set dir, set speed, set invert speed)

``` 


---

2020.3107671
2022.4509265 
