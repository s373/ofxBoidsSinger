#include "ofxBoidsSinger2d.h"

class ofApp : public ofBaseApp{

	ofxBoidsSinger2D singerflock2d;

	void setup(){ 
		singerflock2d.setup(15000);
		singerflock2d.set_minSpeed(0.100);	
		singerflock2d.set_maxSpeed(0.720);
	}
	void update(){
		singerflock2d.update();
	}
	void draw(){
		ofBackgroundGradient(240,180); 
		ofSetColor(255,128,0);
		Flock2dPtr flock  = singerflock2d.GetFlock();
		for(long long i = 0; i < flock->numBoids; ++i)
		{
			ofRect( flock->boid[i].oldPos.x * ofGetWidth()/2 + ofGetWidth()/2,
					flock->boid[i].oldPos.y * ofGetHeight()/2 + ofGetHeight()/2 , 
					5, 5);
			ofLine (
				flock->boid[i].oldPos.x * ofGetWidth()/2 + ofGetWidth()/2,
				flock->boid[i].oldPos.y * ofGetHeight()/2 + ofGetHeight()/2,
				flock->boid[i].newPos.x * ofGetWidth()/2 + ofGetWidth()/2,
				flock->boid[i].newPos.y * ofGetHeight()/2 + ofGetHeight()/2
			);
			ofRect( flock->boid[i].newPos.x * ofGetWidth()/2 + ofGetWidth()/2,
					flock->boid[i].newPos.y * ofGetHeight()/2 + ofGetHeight()/2 , 
					5, 5);
		}
	}

};

int main(){
	ofSetupOpenGL(1024,600,OF_WINDOW);
	ofRunApp(new ofApp());
	return 0;
}