#include "ofMain.h"
#include "ofxBoidsSinger2d.h"
int main(){
	class ChirpApp : public ofBaseApp{
		ofxBoidsSinger2D singerflock2d;
		void setup(){ singerflock2d.setup(100, 0);}
		void update(){singerflock2d.update();}
		void draw(){
			ofBackground(180); ofSetColor(255,128,0);
			Flock2dPtr flock  = singerflock2d.GetFlock();
			for(long long i = 0; i < flock->numBoids; ++i)
			{
				ofLine (
					flock->boid[i].oldPos.x,flock->boid[i].oldPos.y,
					flock->boid[i].newPos.x,flock->boid[i].newPos.y
				);
			}
		}
	};
	ofSetupOpenGL(1024,600,OF_WINDOW);
	ofRunApp(new ChirpApp());
	return 0;
}
