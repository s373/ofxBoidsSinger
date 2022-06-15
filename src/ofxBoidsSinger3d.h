/*
	boids2d 08/2005 a.sier / jasch 
	adapted from boids by eric singer 
	� 1995-2003 eric l. singer
	free for non-commercial use
*/

#pragma once
#include "ofxBoidsSingerTypes.h"


class ofxBoidsSinger3D {
protected:
	// variables
	void*	bflock;
	// t_symbol *ps_nothing;
	char *ps_nothing = nullptr;

	Flock3dPtr flock3d;

public:

	void setup(int nboids)
	{ 
		flock3d = new FlockObject3d();
		setNumBoids(nboids);
		flock3d->mode = 0;
		flock3d->d2r = 3.141592653589793238462643383279502884197169399375105820974944592307816406286208998628034825342117068/180.0;
		flock3d->r2d = 180.0/3.141592653589793238462643383279502884197169399375105820974944592307816406286208998628034825342117068;
		InitFlock(flock3d);
		// 0 0x7fffee410880 0 0x5645dbf8ef70 ofxBoidsSinger2D May 16 2020 07:37:43
	}

	void FlightStep(Flock2dPtr bflockPtr)
	{
		Velocity3d		goCenterVel;
		Velocity3d		goAttractVel;
		Velocity3d		matchNeighborVel;
		Velocity3d		avoidWallsVel;
		Velocity3d		avoidNeighborVel;
		float			avoidNeighborSpeed;
		const Velocity3d	zeroVel	= {0.0, 0.0, 0.0};
		short i;
		// long i;

		bflockPtr->centerPt = FindFlockCenter(bflockPtr);
		for (i = 0; i <  bflockPtr->numBoids; i++) 
		{	// save position and velocity
			bflockPtr->boid[i].oldPos.x = bflockPtr->boid[i].newPos.x;
			bflockPtr->boid[i].oldPos.y = bflockPtr->boid[i].newPos.y;
			bflockPtr->boid[i].oldPos.z = bflockPtr->boid[i].newPos.z;

			bflockPtr->boid[i].oldDir.x = bflockPtr->boid[i].newDir.x;
			bflockPtr->boid[i].oldDir.y = bflockPtr->boid[i].newDir.y;
			bflockPtr->boid[i].oldDir.z = bflockPtr->boid[i].newDir.z;
		}
		
		for (i = 0; i < bflockPtr->numBoids; i++) {
			if (bflockPtr->numNeighbors > 0) 
			{	// get all velocity components
				avoidNeighborSpeed = MatchAndAvoidNeighbors(bflockPtr, i,&matchNeighborVel, &avoidNeighborVel);
			} else {
				matchNeighborVel = zeroVel;
				avoidNeighborVel = zeroVel;
				avoidNeighborSpeed = 0;
			}
			goCenterVel = SeekPoint(bflockPtr, i, bflockPtr->centerPt);
			goAttractVel = SeekPoint(bflockPtr, i, bflockPtr->attractPt);
			avoidWallsVel = AvoidWalls(bflockPtr, i);

			// compute resultant velocity using weights and inertia
			bflockPtr->boid[i].newDir.x = bflockPtr->inertiaFactor * (bflockPtr->boid[i].oldDir.x) +
								(bflockPtr->centerWeight * goCenterVel.x +
								 bflockPtr->attractWeight * goAttractVel.x +
								 bflockPtr->matchWeight * matchNeighborVel.x +
								 bflockPtr->avoidWeight * avoidNeighborVel.x +
								 bflockPtr->wallsWeight * avoidWallsVel.x) / bflockPtr->inertiaFactor;
			bflockPtr->boid[i].newDir.y = bflockPtr->inertiaFactor * (bflockPtr->boid[i].oldDir.y) +
								(bflockPtr->centerWeight * goCenterVel.y +
								 bflockPtr->attractWeight * goAttractVel.y +
								 bflockPtr->matchWeight * matchNeighborVel.y +
								 bflockPtr->avoidWeight * avoidNeighborVel.y +
								 bflockPtr->wallsWeight * avoidWallsVel.y) / bflockPtr->inertiaFactor;
			bflockPtr->boid[i].newDir.z = bflockPtr->inertiaFactor * (bflockPtr->boid[i].oldDir.z) +
								(bflockPtr->centerWeight * goCenterVel.z +
								 bflockPtr->attractWeight * goAttractVel.z +
								 bflockPtr->matchWeight * matchNeighborVel.z +
								 bflockPtr->avoidWeight * avoidNeighborVel.z +
								 bflockPtr->wallsWeight * avoidWallsVel.z) / bflockPtr->inertiaFactor;

			NormalizeVelocity(&(bflockPtr->boid[i].newDir));	// normalize velocity so its length is unity

			// set to avoidNeighborSpeed bounded by minSpeed and maxSpeed
			if ((avoidNeighborSpeed >= bflockPtr->minSpeed) &&
					(avoidNeighborSpeed <= bflockPtr->maxSpeed))
				bflockPtr->boid[i].speed = avoidNeighborSpeed;
			else if (avoidNeighborSpeed > bflockPtr->maxSpeed)
				bflockPtr->boid[i].speed = bflockPtr->maxSpeed;
			else
				bflockPtr->boid[i].speed = bflockPtr->minSpeed;

			// calculate new position, applying speedupFactor
			bflockPtr->boid[i].newPos.x += bflockPtr->boid[i].newDir.x * bflockPtr->boid[i].speed * (bflockPtr->speedupFactor / 100.0);
			bflockPtr->boid[i].newPos.y += bflockPtr->boid[i].newDir.y * bflockPtr->boid[i].speed * (bflockPtr->speedupFactor / 100.0);
			bflockPtr->boid[i].newPos.z += bflockPtr->boid[i].newDir.z * bflockPtr->boid[i].speed * (bflockPtr->speedupFactor / 100.0);

		}
	}

	Flock3dPtr GetFlock() const
	{
			return flock3d;
	}

	Point3d FindFlockCenter(Flock3dPtr bflockPtr)
	{
		double			totalH = 0, totalV = 0, totalD = 0;
		Point3d			centerPoint;
		short i;
		// long i;

		for (i = 0 ; i <  bflockPtr->numBoids; i++)
		{
			totalH += bflockPtr->boid[i].oldPos.x;
			totalV += bflockPtr->boid[i].oldPos.y;
			totalD += bflockPtr->boid[i].oldPos.z;
		}
		centerPoint.x = (double)	(totalH / bflockPtr->numBoids);
		centerPoint.y = (double)	(totalV / bflockPtr->numBoids);
		centerPoint.z = (double)	(totalD / bflockPtr->numBoids);

		return(centerPoint);
	}

	float MatchAndAvoidNeighbors(Flock3dPtr bflockPtr, short theBoid, Velocity3d *matchNeighborVel, Velocity3d *avoidNeighborVel)
	{
		short i, j, neighbor;
		// long i, j, neighbor;
		double			distSqr;
		double			dist, distH, distV,distD;
		double			tempSpeed;
		short			numClose = 0;
		Velocity3d		totalVel = {0.0,0.0,0.0};

		/**********************/
		/* Find the neighbors */
		/**********************/

		/* special case of one neighbor */
		if (bflockPtr->numNeighbors == 1) 
		{
			bflockPtr->boid[theBoid].neighborDistSqr[0] = kMaxLong;

			for (i = 0; i < bflockPtr->numBoids; i++) {
				if (i != theBoid) {
					distSqr = DistSqrToPt(bflockPtr->boid[theBoid].oldPos, bflockPtr->boid[i].oldPos);

					/* if this one is closer than the closest so far, then remember it */
					if (bflockPtr->boid[theBoid].neighborDistSqr[0] > distSqr) {
						bflockPtr->boid[theBoid].neighborDistSqr[0] = distSqr;
						bflockPtr->boid[theBoid].neighbor[0] = i;
					}
				}
			}
		}
		/* more than one neighbor */
		else 
		{
			for (j = 0; j < bflockPtr->numNeighbors; j++)
				bflockPtr->boid[theBoid].neighborDistSqr[j] = kMaxLong;

			for (i = 0 ; i < bflockPtr->numBoids; i++) {
				/* if this one is not me... */
				if (i != theBoid) {
					distSqr = DistSqrToPt(bflockPtr->boid[theBoid].oldPos, bflockPtr->boid[i].oldPos);

					/* if distSqr is less than the distance at the bottom of the array, sort into array */
					if (distSqr < bflockPtr->boid[theBoid].neighborDistSqr[bflockPtr->numNeighbors-1]) {
						j = bflockPtr->numNeighbors - 1;

						/* sort distSqr in to keep array in size order, smallest first */
						while ((distSqr < bflockPtr->boid[theBoid].neighborDistSqr[j-1]) && (j > 0)) {
							bflockPtr->boid[theBoid].neighborDistSqr[j] = bflockPtr->boid[theBoid].neighborDistSqr[j - 1];
							bflockPtr->boid[theBoid].neighbor[j] = bflockPtr->boid[theBoid].neighbor[j - 1];
							j--;
						}
						bflockPtr->boid[theBoid].neighborDistSqr[j] = distSqr;
						bflockPtr->boid[theBoid].neighbor[j] = i;
					}
				}
			}
		}

		/*********************************/
		/* Match and avoid the neighbors */
		/*********************************/

		matchNeighborVel->x = 0;
		matchNeighborVel->y = 0;
		matchNeighborVel->z = 0;

		// set tempSpeed to old speed
		tempSpeed = bflockPtr->boid[theBoid].speed;

		for (i = 0; i < bflockPtr->numNeighbors; i++) 
		{
			neighbor = bflockPtr->boid[theBoid].neighbor[i];

			// calculate matchNeighborVel by averaging the neighbor velocities
			matchNeighborVel->x += bflockPtr->boid[neighbor].oldDir.x;
			matchNeighborVel->y += bflockPtr->boid[neighbor].oldDir.y;
			matchNeighborVel->z += bflockPtr->boid[neighbor].oldDir.z;

			// if distance is less than preferred distance, then neighbor influences boid
			distSqr = bflockPtr->boid[theBoid].neighborDistSqr[i];
			if (distSqr < bflockPtr->prefDistSqr) 
			{
				dist = sqrt(distSqr);

				distH = bflockPtr->boid[neighbor].oldPos.x - bflockPtr->boid[theBoid].oldPos.x;
				distV = bflockPtr->boid[neighbor].oldPos.y - bflockPtr->boid[theBoid].oldPos.y;
				distD = bflockPtr->boid[neighbor].oldPos.z - bflockPtr->boid[theBoid].oldPos.z;

				if(dist == 0.0) dist = 0.0000001;
				totalVel.x = totalVel.x - distH - (distH * ((float) bflockPtr->prefDist / (dist)));
				totalVel.y = totalVel.y - distV - (distV * ((float) bflockPtr->prefDist / (dist)));
				totalVel.z = totalVel.z - distD - (distV * ((float) bflockPtr->prefDist / (dist)));

				numClose++;
			}
			if (InFront(&(bflockPtr->boid[theBoid]), &(bflockPtr->boid[neighbor]))) 
			{	// adjust speed
				if (distSqr < bflockPtr->prefDistSqr)
					tempSpeed /= (bflockPtr->accelFactor / 100.0);
				else
					tempSpeed *= (bflockPtr->accelFactor / 100.0);
			}
			else 
			{
				if (distSqr < bflockPtr->prefDistSqr)
					tempSpeed *= (bflockPtr->accelFactor / 100.0);
				else
					tempSpeed /= (bflockPtr->accelFactor / 100.0);
			}
		}

		if (numClose) {
			avoidNeighborVel->x = totalVel.x / numClose;
			avoidNeighborVel->y = totalVel.y / numClose;
			avoidNeighborVel->z = totalVel.z / numClose;
			NormalizeVelocity(matchNeighborVel);
		}
		else 
		{
			avoidNeighborVel->x = 0;
			avoidNeighborVel->y = 0;
			avoidNeighborVel->z = 0;
		}
		return(tempSpeed);
	}


	Velocity3d SeekPoint(Flock3dPtr bflockPtr, short theBoid, Point3d seekPt)
	{
		Velocity3d	tempDir;
		tempDir.x = seekPt.x - bflockPtr->boid[theBoid].oldPos.x;
		tempDir.y = seekPt.y - bflockPtr->boid[theBoid].oldPos.y;
		tempDir.z = seekPt.z - bflockPtr->boid[theBoid].oldPos.z;
		NormalizeVelocity(&tempDir);
		return(tempDir);
	}


	Velocity3d AvoidWalls(Flock3dPtr bflockPtr, short theBoid)
	{
		Point3d		testPoint;
		Velocity3d	tempVel = {0.0, 0.0, 0.0};

		/* calculate test point in front of the nose of the boid */
		/* distance depends on the boid's speed and the avoid edge constant */
		testPoint.x = bflockPtr->boid[theBoid].oldPos.x + bflockPtr->boid[theBoid].oldDir.x * bflockPtr->boid[theBoid].speed * bflockPtr->edgeDist;
		testPoint.y = bflockPtr->boid[theBoid].oldPos.y + bflockPtr->boid[theBoid].oldDir.y * bflockPtr->boid[theBoid].speed * bflockPtr->edgeDist;
		testPoint.z = bflockPtr->boid[theBoid].oldPos.z + bflockPtr->boid[theBoid].oldDir.z * bflockPtr->boid[theBoid].speed * bflockPtr->edgeDist;

		/* if test point is out of the left (right) side of bflockPtr->flyRect, */
		/* return a positive (negative) horizontal velocity component */
		if (testPoint.x < bflockPtr->flyRect.left)
			tempVel.x = fabs(bflockPtr->boid[theBoid].oldDir.x);
		else if (testPoint.x > bflockPtr->flyRect.right)
			tempVel.x = - fabs(bflockPtr->boid[theBoid].oldDir.x);

		/* same with top and bottom */
		if (testPoint.y < bflockPtr->flyRect.top)
			tempVel.y = fabs(bflockPtr->boid[theBoid].oldDir.y);
		else if (testPoint.y > bflockPtr->flyRect.bottom)
			tempVel.y = - fabs(bflockPtr->boid[theBoid].oldDir.y);

		/* same with front and back */
		if (testPoint.z < bflockPtr->flyRect.front)
			tempVel.z = fabs(bflockPtr->boid[theBoid].oldDir.z);
		else if (testPoint.z > bflockPtr->flyRect.back)
			tempVel.z = - fabs(bflockPtr->boid[theBoid].oldDir.z);
		

		return(tempVel);
	}


	bool InFront(Boid3dPtr theBoid, Boid3dPtr neighbor)
	{
		float	grad, intercept;
		bool	result;

/*

Find the gradient and y-intercept of a line passing through theBoid's oldPos
perpendicular to its direction of motion.  Another boid is in front of theBoid
if it is to the right or left of this linedepending on whether theBoid is moving
right or left.  However, if theBoid is travelling vertically then just compare
their vertical coordinates.

*/
		// xy plane

		// if theBoid is not travelling vertically...
		if (theBoid->oldDir.x != 0) 
		{ // calculate gradient of a line _perpendicular_ 
		  // to its direction (hence the minus)
			
			grad = -theBoid->oldDir.y / theBoid->oldDir.x;

			// calculate where this line hits the y axis (from y = mx + c)
			intercept = theBoid->oldPos.y - (grad * theBoid->oldPos.x);

			/* compare the horizontal position of the neighbor boid with */
			/* the point on the line that has its vertical coordinate */
			if (neighbor->oldPos.x >= ((neighbor->oldPos.y - intercept) / grad)) {
				/* return true if the first boid's horizontal movement is +ve */
				result = (theBoid->oldDir.x > 0);

				if (result==0) return 0;
				else goto next;

			} 
			else 
			{
				/* return true if the first boid's horizontal movement is +ve */
				result = (theBoid->oldDir.x < 0);
				if (result==0) return 0;
				else goto next;
			}
		}
		/* else theBoid is travelling vertically, so just compare vertical coordinates */
		else if (theBoid->oldDir.y > 0) 
		{
			result = (neighbor->oldPos.y > theBoid->oldPos.y);
			if (result==0){
				return 0;
			}else{
				goto next;
			}
		}
		else
		{
			result = (neighbor->oldPos.y < theBoid->oldPos.y);
			if (result==0){
				return 0;
			} else {
				goto next;
			}
		}
	next:
	
		// yz plane

		// if theBoid is not travelling vertically...
		if (theBoid->oldDir.y != 0) {
			// calculate gradient of a line _perpendicular_ to its direction (hence the minus)
			grad = -theBoid->oldDir.z / theBoid->oldDir.y;

			// calculate where this line hits the y axis (from y = mx + c)
			intercept = theBoid->oldPos.z - (grad * theBoid->oldPos.y);

			// compare the horizontal position of the neighbor boid with
			// the point on the line that has its vertical coordinate
			if (neighbor->oldPos.y >= ((neighbor->oldPos.z - intercept) / grad)) {
				// return true if the first boid's horizontal movement is +ve
				result = (theBoid->oldDir.y > 0);
				if (result==0){
					return 0;
				}else{
					goto next2;
				}
			} else {
				// return true if the first boid's horizontal movement is +ve
				result = (theBoid->oldDir.y < 0);
				if (result==0){
					return 0;
				}else{
					goto next2;
				}
			}
		}
		// else theBoid is travelling vertically, so just compare vertical coordinates
		else if (theBoid->oldDir.z > 0) {
			result = (neighbor->oldPos.z > theBoid->oldPos.z);
			if (result==0){
				return 0;
			}else{
				goto next2;
			}
		}else{
			result = (neighbor->oldPos.z < theBoid->oldPos.z);
			if (result==0){
				return 0;
			}else{
				goto next2;
			}
		}
	next2: 
		return 1;
	}

	void NormalizeVelocity(Velocity3d *direction)
	{
		float	hypot;

		hypot = sqrt(direction->x * direction->x + direction->y * direction->y + direction->z * direction->z );

		if (hypot != 0.0) 
		{
			direction->x = direction->x / hypot;
			direction->y = direction->y / hypot;
			direction->z = direction->z / hypot;
		}
	}

	double DistSqrToPt(Point3d & firstPoint, Point3d & secondPoint)
	{
		double	a, b,c;
		a = firstPoint.x - secondPoint.x;
		b = firstPoint.y - secondPoint.y;
		c = firstPoint.z - secondPoint.z;
		return(a * a + b * b + c * c);
	}



	void setNumBoids(int nboids){
		// reset feature
		if(flock3d&&flock3d->boid)
			delete [] flock3d->boid;
		flock3d->boid = new Boid3d[nboids];
		flock3d->numBoids = nboids;
		Flock_resetBoids(flock3d);
	}

	void Flock_resetBoids(Flock3dPtr bflockPtr)
	{
		// long i, j;
		short i, j;
		double rndAngle;

		for (i = 0; i <  bflockPtr->numBoids; i++) 
		{ // init everything to 0.0
			bflockPtr->boid[i].oldPos.x = 0.0;
			bflockPtr->boid[i].oldPos.y = 0.0;
			bflockPtr->boid[i].oldPos.z = 0.0;

			bflockPtr->boid[i].newPos.x = 0.0;
			bflockPtr->boid[i].newPos.y = 0.0;
			bflockPtr->boid[i].newPos.z = 0.0;

			bflockPtr->boid[i].oldDir.x = 0.0;
			bflockPtr->boid[i].oldDir.y = 0.0;
			bflockPtr->boid[i].oldDir.z = 0.0;

			bflockPtr->boid[i].newDir.x = 0.0;
			bflockPtr->boid[i].newDir.y = 0.0;
			bflockPtr->boid[i].newDir.z = 0.0;

			bflockPtr->boid[i].speed = 0.0;

			for(j=0; j<kMaxNeighbors;j++){
				bflockPtr->boid[i].neighbor[j] = 0;
				bflockPtr->boid[i].neighborDistSqr[j] = 0.0;
			}
		}
		for (i = 0; i <  bflockPtr->numBoids; i++) 
		{	// set the initial locations and velocities of the boids
			bflockPtr->boid[i].newPos.x = bflockPtr->boid[i].oldPos.x = RandomInt(bflockPtr->flyRect.right,bflockPtr->flyRect.left);		// set random location within flyRect
			bflockPtr->boid[i].newPos.y = bflockPtr->boid[i].oldPos.y = RandomInt(bflockPtr->flyRect.bottom, bflockPtr->flyRect.top);
			bflockPtr->boid[i].newPos.z = bflockPtr->boid[i].oldPos.z = RandomInt(bflockPtr->flyRect.back, bflockPtr->flyRect.front);
			rndAngle = RandomInt(0, 360) * bflockPtr->d2r;		// set velocity from random angle
			bflockPtr->boid[i].newDir.x = sin(rndAngle);
			bflockPtr->boid[i].newDir.y = cos(rndAngle);
			bflockPtr->boid[i].newDir.z = (cos(rndAngle) + sin(rndAngle)) * 0.5;
			bflockPtr->boid[i].speed = (kMaxSpeed + kMinSpeed) * 0.5;
		}

	}

	void InitFlock(Flock3dPtr bflockPtr)
	{
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
		bflockPtr->flyRect.front		= kFlyRectFront;
		bflockPtr->flyRect.back		= kFlyRectBack;
		bflockPtr->attractPt.x		= (kFlyRectLeft + kFlyRectRight) * 0.5;
		bflockPtr->attractPt.y		= (kFlyRectTop + kFlyRectBottom) * 0.5;
		bflockPtr->attractPt.z		= (kFlyRectFront + kFlyRectBack) * 0.5;
		Flock_resetBoids(bflockPtr);
	}

	void reset(Flock3dPtr bflockPtr)
	{
		InitFlock(bflockPtr);
	}

	void set_flyRect(float a, float b, float c, float d, float e, float f){
		flock3d->flyRect.left = a;
		flock3d->flyRect.top = b;
		flock3d->flyRect.right = c;
		flock3d->flyRect.bottom = d;
		flock3d->flyRect.back = e;
		flock3d->flyRect.front = f;
	}
	void set_minSpeed(float d){
		flock3d->minSpeed = d;
	}
	void set_maxSpeed(float d){
		flock3d->maxSpeed = d;
	}
	void set_centerWeight(float d){
		flock3d->centerWeight = d;
	}
	void set_attractWeight(float d){
		flock3d->attractWeight = d;
	}
	void set_matchWeight(float d){
		flock3d->matchWeight = d;
	}
	void set_avoidWeight(float d){
		flock3d->prefDist = d;
	}
	void set_wallsWeight(float d){
		flock3d->wallsWeight = d;
	}
	void set_speedupFactor(float d){
		flock3d->speedupFactor = d;
	}
	void set_inertiaFactor(float d){
		flock3d->inertiaFactor = MAX((double)(9.999999999999999547e-07), d);
	}
	void set_accelFactor(float d){
		flock3d->accelFactor = d;
	}
	void set_prefDist(float d){
		flock3d->prefDist = d;
	}
	void set_pos(int ix, float x, float y, float z){
		// set ixth boid to place
		flock3d->boid[ix].oldPos.x = x;
		flock3d->boid[ix].newPos.x = x;
		flock3d->boid[ix].oldPos.y = y;
		flock3d->boid[ix].newPos.y = y;
		flock3d->boid[ix].oldPos.z = z;
		flock3d->boid[ix].newPos.z = z;
	}
	void set_dir(int ix, float x, float y, float z){
		// set ixth boid to place
		flock3d->boid[ix].oldDir.x = x;
		flock3d->boid[ix].newDir.x = x;
		flock3d->boid[ix].oldDir.y = y;
		flock3d->boid[ix].newDir.y = y;
		flock3d->boid[ix].oldDir.z = z;
		flock3d->boid[ix].newDir.z = z;
	}
	void set_speed(int ix, float x){
		// set ixth boid to place
		flock3d->boid[ix].speed = x;
	}
	void set_speedinv(int ix){
		// set ixth boid to place
		flock3d->boid[ix].speed *= -1;
	}
	void attractPt(float x, float y, float z){
		// set ixth boid to place
		flock3d->attractPt.x = x;
		flock3d->attractPt.y = y;
		flock3d->attractPt.z = z;
	}

	void update() {
		FlightStep( flock3d );
	}


};
