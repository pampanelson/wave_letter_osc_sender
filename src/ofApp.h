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
    ofxCvColorImage colorImg;
    
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
    

    
    
    ofParameter<float> track1PosX;
    ofParameter<float> track1PosY;
    ofParameter<float> track1W;
    ofParameter<float> track1H;
    ofParameter<float> track2PosX;
    ofParameter<float> track2PosY;
    ofParameter<float> track2W;
    ofParameter<float> track2H;
    ofParameter<float> track3PosX;
    ofParameter<float> track3PosY;
    ofParameter<float> track3W;
    ofParameter<float> track3H;
    
    ofParameter<float> track4PosX;
    ofParameter<float> track4PosY;
    ofParameter<float> track4W;
    ofParameter<float> track4H;
    ofParameter<float> track5PosX;
    ofParameter<float> track5PosY;
    ofParameter<float> track5W;
    ofParameter<float> track5H;
    ofParameter<float> track6PosX;
    ofParameter<float> track6PosY;
    ofParameter<float> track6W;
    ofParameter<float> track6H;
    
    ofParameter<float> track7PosX;
    ofParameter<float> track7PosY;
    ofParameter<float> track7W;
    ofParameter<float> track7H;
    
    
    
    
    
    
    
    vector<float> trackingData;
    int trackingDataSize = 7;
};
