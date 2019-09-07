#include "ofApp.h"


using namespace ofxCv;
using namespace cv;


//--------------------------------------------------------------
void ofApp::setup(){
    ofSetFrameRate(30); // run at 60 fps

    // open an outgoing connection to HOST:PORT
    sender.setup(HOST, PORT);


    
    // enable depth->video image calibration
    kinect.setRegistration(true);
    
    kinect.init();
    //kinect.init(true); // shows infrared instead of RGB video image
    //kinect.init(false, false); // disable video image (faster fps)
    
    kinect.open();        // opens first available kinect
    //kinect.open(1);    // open a kinect by id, starting with 0 (sorted by serial # lexicographically))
    //kinect.open("A00362A08602047A");    // open a kinect using it's unique serial #
    
    cout << kinect.width << "," << kinect.height << endl;
    colorImg.allocate(kinect.width, kinect.height);
    grayImage.allocate(kinect.width, kinect.height);
    grayImage1.allocate(kinect.width, kinect.height);
    grayThreshNear.allocate(kinect.width, kinect.height);
    grayThreshFar.allocate(kinect.width, kinect.height);
    
    nearThreshold = 230;
    farThreshold = 70;
    bThreshWithOpenCV = true;
    
//    ofSetFrameRate(60);
    
    // zero the tilt on startup
    angle = 0;
    kinect.setCameraTiltAngle(angle);
    
    
    
    
    
    // print the intrinsic IR sensor values
    if(kinect.isConnected()) {
        ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
        ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
        ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
        ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
    }
    
    
    
    
    // zero the tilt on startup

    gui.setup();
    gui.add(bSendingOSC.set("Sending osc",false));
    gui.add(bTracking.set("Tracking",false));
    gui.add(minArea.set("Min area", 10, 1, 100));
    gui.add(maxArea.set("Max area", 200, 1, 500));
    gui.add(threshold.set("Threshold", 128, 0, 255));
    gui.add(holes.set("Holes", false));
    gui.add(bUseTgtColor.set("use target color",false));
    gui.add(tgtColorThreshold.set("target color Threshold", 128, 0, 255));
//    gui.add(trackHs.set("Track Hue/Saturation", false));
    
    gui.add(nearThreshold.set("near",230,1,255));
    gui.add(farThreshold.set("far",70,1,255));
    gui.add(bThreshWithOpenCV.set("use opencv", false));
    gui.add(angle.set("angle",1,0,180));
    gui.add(arcMin.set("arc min",10,1,480));
    gui.add(arcMax.set("arc max",50,1,480));

    
    if (!ofFile("settings.xml"))
        gui.saveToFile("settings.xml");
    
    gui.loadFromFile("settings.xml");
    
    
    // init tracking data size;
    trackingDataSize = 10;
    trackingData.resize(trackingDataSize);
}

//--------------------------------------------------------------


float ofApp::myPosToAngle(float x,float y){
    float res;
    // center on (320,480)
    float centerX = 320;
    float centerY = 480;
    res = atan(
                        (centerY - y) // always > 0 for top lefter (0,0)
                        /
                       (centerX - x) // if > 0 means point on the left of center
                                    // if < 0 point on the right of center

                       )/PI;
    
    
    // clock direction of angle from -x axis ///////////////  *********** IMPORTANT **********
    // so , res should always < 1 aka. PI
    if(res < 0){
        res = 1. + res;
    }
    
    return res;
}
//--------------------------------------------------------------
void ofApp::exit() {
    kinect.setCameraTiltAngle(0); // zero the tilt on exit
    kinect.close();
    
}


//--------------------------------------------------------------
void ofApp::update(){

    trackingData.clear();
    
    
    kinect.setCameraTiltAngle(angle);
    
    
    kinect.update();

    // there is a new frame and we are connected
    if(kinect.isFrameNew()) {
        
        // load grayscale depth image from the kinect source
        grayImage.setFromPixels(kinect.getDepthPixels());
        
        // we do two thresholds - one for the far plane and one for the near plane
        // we then do a cvAnd to get the pixels which are a union of the two thresholds
        if(bThreshWithOpenCV) {
            grayThreshNear = grayImage;
            grayThreshFar = grayImage;
            grayThreshNear.threshold(nearThreshold, true);
            grayThreshFar.threshold(farThreshold);
            cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
        } else {
            
            // or we do it ourselves - show people how they can work with the pixels
            ofPixels & pix = grayImage.getPixels();
            int numPixels = pix.size();
            for(int i = 0; i < numPixels; i++) {
                if(pix[i] < nearThreshold && pix[i] > farThreshold) {
                    pix[i] = 255;
                } else {
                    pix[i] = 0;
                }
            }
            
        }
        
        // update the cv images
        grayImage.flagImageChanged();
        
        grayImage1 = grayImage;

        
        // get roi frm gray image
        
        {
            ofPixels & pix = grayImage1.getPixels();
            int numPixels = pix.size();
            for(int i = 0; i < numPixels; i++) {
                
                int h = floor(i/640);
                int w = i % 640;
                
                float arc = sqrt((w-320)*(w-320) + (h-480)*(h-480));
                if(arc > arcMax || arc < arcMin ){
                    
                    pix[i] = 0;
                    
                }
                
            }
        }
        
        
        // get image to track contour
        grayImage1.flagImageChanged();
        
        grayImage1.blur();
        
        
        // tracking
        
        if(bTracking){
            contourFinder.setMinAreaRadius(minArea);
            contourFinder.setMaxAreaRadius(maxArea);
            contourFinder.setThreshold(threshold);
            contourFinder.setFindHoles(holes);
            
            
            contourFinder.findContours(grayImage1);
            
            
        }

        
        if(bUseTgtColor){
            //            contourFinderTgtColor.setTargetColor(targetColor, trackHs ? TRACK_COLOR_HS : TRACK_COLOR_RGB);
            contourFinderTgtColor.setTargetColor(targetColor);
            
            contourFinderTgtColor.setMinAreaRadius(minArea);
            contourFinderTgtColor.setMaxAreaRadius(maxArea);
            contourFinderTgtColor.setFindHoles(holes);
            contourFinderTgtColor.setThreshold(tgtColorThreshold.get());
            contourFinderTgtColor.findContours(grayImage1);
            
        }


    }
    
    
    
    // ----------------------------------------  preapare tracking data
    
    
    for (int i = 0; i<contourFinderTgtColor.size(); i++) {
        
        if(i<trackingDataSize){
            cv::Rect rect = contourFinderTgtColor.getBoundingRect(i);
            float x = rect.x+rect.width * 0.5;
            float y = rect.y + rect.height * 0.5;
            
            float trackingAngle = myPosToAngle(x, y);
            trackingData.push_back(trackingAngle);

        }

    }
    
    
    // =================  debug tracking data
    cout << ofToString(0) << ": " << trackingData[0] << endl;
    cout << ofToString(1) << ": " << trackingData[1] << endl;

    
    if(bSendingOSC){
        // prepare data for osc send ----------------------------------------
        
        ofxOscMessage m;
//        m.setAddress("/composition/selectedclip/video/effects/pwl00/effect/float1");
//        m.addFloatArg(ofMap(ofGetMouseX(), 0, ofGetWidth(), 0.f, 1.f, true));
//        //    m.addFloatArg(ofMap(ofGetMouseY(), 0, ofGetHeight(), 0.f, 1.f, true));
//        sender.sendMessage(m, false);
//        m.clear();
        
        m.setAddress("/composition/selectedclip/video/effects/addsubexample/effect/textdata");
        
        
        m.addStringArg(ofToString((ofGetElapsedTimef()/1000.0)));
        sender.sendMessage(m,false);
        m.clear();
        
        
        
        
//
    }
}

//--------------------------------------------------------------
void ofApp::draw(){
    



    
    
//    kinect.getDepthTexture().draw(0, 0);
    grayImage.draw(0,0);
    grayImage1.draw(640,0);
    
    
    ofSetColor(255,0,0);
    contourFinder.draw();
    if(bUseTgtColor){
        ofTranslate(640, 480);
        contourFinderTgtColor.draw();
        ofTranslate(-640, -480);
        
    }
    
    ofSetColor(255,255,255);

    gui.draw();

}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
    

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
//    targetColor = cam.getPixels().getColor(x, y);

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
