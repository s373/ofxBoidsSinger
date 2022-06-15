#include "ofxBoidsSinger2d.h"

class ofApp : public ofBaseApp{

	ofxBoidsSinger2D flock2d;

	void setup(){ 
		flock2d.setup(100);
		flock2d.set_minSpeed(0.100);	
		flock2d.set_maxSpeed(0.720);
	}
	void update(){
		flock2d.update();
	}
	void draw(){
		ofBackgroundGradient(240,180); 
		ofSetColor(255,128,0);
		Flock2dPtr flock  = flock2d.GetFlock();
		for(int i = 0; i < flock->numBoids; ++i)
		{
			ofRect( flock->boid[i].oldPos.x * ofGetWidth()/2 + ofGetWidth()/2,
					flock->boid[i].oldPos.y * ofGetHeight()/2 + ofGetHeight()/2 , 
					5, 5);
			ofLine ( flock->boid[i].oldPos.x * ofGetWidth()/2 + ofGetWidth()/2,
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