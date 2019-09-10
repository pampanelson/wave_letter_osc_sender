#pragma once

#include "ofMain.h"
#include "ofxOsc.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"
#include "ofxGui.h"
#include "ofxKinect.h"
using namespace ofxCv;
using namespace cv;

// send host (aka ip address)
#define HOST "localhost"
//#define HOST "192.168.0.174"

/// send port
#define PORT 8000

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();
    void exit();
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
		
    float myPosToAngle(float x,float y);
    ofxOscSender sender;

    ofxCv::ContourFinder contourFinder;
    ofxCv::ContourFinder contourFinderTgtColor;

    ofColor targetColor;

    

    
    ofxPanel gui;
    ofParameter<bool> bSendingOSC;
    ofParameter<bool> bTracking;
    ofParameter<float> minArea, maxArea, threshold;
    ofParameter<bool> holes;
    ofParameter<bool> bUseTgtColor;
    ofParameter<bool> trackHs;
    ofParameter<float> tgtColorThreshold;

    
    ofxKinect kinect;
    ofxCvColorImage colorImg;
    
    ofxCvGrayscaleImage grayImage; // grayscale depth image
    ofxCvGrayscaleImage grayImage1; // grayscale depth image
    ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
    ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
    

    
    ofParameter<bool> bThreshWithOpenCV;
    ofParameter<bool> bFlip;

    
    ofParameter<int> nearThreshold;
    ofParameter<int> farThreshold;
    
    ofParameter<int> angle;
    
    ofParameter<int> arcMin;
    ofParameter<int> arcMax;

    vector<float> trackingData;
    int trackingDataSize;
};
