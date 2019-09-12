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

    
    ofxKinect kinect;
    
    ofxCvGrayscaleImage grayImage; // grayscale depth image

    ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
    ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
    
    ofPixels previous;
    ofImage diff;
    cv::Scalar diffMean;
    
    ofParameter<bool> bThreshWithOpenCV;
    ofParameter<bool> bFlip;


    ofParameter<int> nearThreshold;
    ofParameter<int> farThreshold;
    
    ofParameter<int> angle;

    ofParameter<float> minAreaRadius;
    ofParameter<float> maxAreaRadius;
    ofParameter<float> trackingThreshold;
    ofParameter<float> maxDistance;
    ofParameter<float> trackingPersistence;

    ofParameter<float> waveMoveSpeed;


    float preX;
    bool  waveFromLeftToRight;
    bool  waveFromRightToLeft;
    
    bool waveLtRon = false;
    bool waveRtLon = false;

    
    
    int trackingDataSize;
    vector<ofPoint> trackingData;
    
    int oscTrackingDataSize;
    vector<float> oscTrackingData;
};
