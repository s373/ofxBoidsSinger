#include "ofxBoidsSinger.h"

class ofApp : public ofBaseApp{

	ofxBoidsSinger3D flock3d;
    ofEasyCam cam;

	void setup(){ 
		flock3d.setup(100);
		flock3d.set_minSpeed(0.100);	
		flock3d.set_maxSpeed(0.720);
	}
	void update(){
		flock3d.update();
	}
	void draw(){
		ofBackgroundGradient(240,180); 
		ofSetColor(255,128,0);
        cam.begin();
		Flock3dPtr flock  = flock3d.GetFlock();
		for(int i = 0; i < flock->numBoids; ++i)
		{
            ofPoint pos(flock->boid[i].newPos.x, flock->boid[i].newPos.y, flock->boid[i].newPos.z);
            pos *= ofPoint(ofGetWidth(), ofGetWidth(), ofGetWidth());
            glPushMatrix();
            glTranslatef(pos.x, pos.y, pos.z);
			ofRect( -25, -25 , 50, 50 );
            glPopMatrix();
		}
        cam.end();
	}

};

int main(){
	ofSetupOpenGL(1024,600,OF_WINDOW);
	ofRunApp(new ofApp());
	return 0;
}