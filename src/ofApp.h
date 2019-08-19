#pragma once

#include "ofMain.h"
#include "ofxOsc.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"
#include "ofxGui.h"

using namespace ofxCv;
using namespace cv;

// send host (aka ip address)
#define HOST "localhost"

/// send port
#define PORT 8000

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		
    ofxOscSender sender;

    ofVideoGrabber cam;
    ofxCvColorImage cCam;
    ofxCvGrayscaleImage grayCam;
    ofxCv::ContourFinder contourFinder;
    ofxCv::ContourFinder contourFinderTgtColor;

    ofColor targetColor;

    
    ofPixels previous;
    ofImage diff;
    cv::Scalar diffMean;
    
    
    ofxPanel gui;
    ofParameter<float> minArea, maxArea, threshold;
    ofParameter<bool> holes;
    ofParameter<bool> bUseTgtColor;
    ofParameter<bool> trackHs;
    ofParameter<float> tgtColorThreshold;

    
};
