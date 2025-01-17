#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"
#include "ofxGui.h"
#include "ofxKinect.h"
#include "ofxSyphon.h"
#include "ofxShadertoy.h"
#include "ofxFx.h"


using namespace ofxCv;
using namespace cv;



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
    

    ofxSyphonServer syphonServer;
    ofTexture       syphonTex;
    ofFbo           syphonFbo;
    ofxShadertoy    shadertoy;
    
    int w = 3840;
    int h = 960;
};
