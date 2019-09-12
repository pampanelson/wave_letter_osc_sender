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
    
//    cout << kinect.width << "," << kinect.height << endl;
    grayImage.allocate(kinect.width, kinect.height);
    grayThreshNear.allocate(kinect.width, kinect.height);
    grayThreshFar.allocate(kinect.width, kinect.height);
    
    
    imitate(previous, grayImage);
    imitate(diff, grayImage);
    
    
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
    gui.add(nearThreshold.set("near",230,1,255));
    gui.add(farThreshold.set("far",70,1,255));
    gui.add(bThreshWithOpenCV.set("use opencv", false));
    gui.add(bFlip.set("flip", false));
    gui.add(angle.set("angle",1,0,180));
    

    gui.add(minAreaRadius.set("min area",1,1,300));
    gui.add(maxAreaRadius.set("max area",10,1,800));
    gui.add(trackingThreshold.set("tracking thresh",1,1,100));
    gui.add(maxDistance.set("max dis",10,1,500));
    gui.add(trackingPersistence.set("persistence",15,1,60)); // frames

    gui.add(waveMoveSpeed.set("wave move speed",0.005,0.001,0.1)); //

    
    if (!ofFile("settings.xml"))
        gui.saveToFile("settings.xml");
    
    gui.loadFromFile("settings.xml");
    
    
    // init tracking data size;
    trackingDataSize = 30;// 30 frames for a sencod
    
    for(int i = 0;i<trackingDataSize;i++){
        
        ofPoint p = ofPoint(0.0,0.0,0.0);
        trackingData.push_back(p);
        
    }
    
    oscTrackingDataSize = 8;
    for(int i = 0;i<oscTrackingDataSize;i++){
        
        float angle = -0.1;
        oscTrackingData.push_back(angle);
        
    }
    
    
    
    
    waveFromLeftToRight = false;
    waveFromRightToLeft = false;
    
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

    waveFromLeftToRight = false;
    waveFromRightToLeft = false;
    
    
    kinect.setCameraTiltAngle(angle);
    
    
    kinect.update();

    // there is a new frame and we are connected
    if(kinect.isFrameNew()) {
        
        // load grayscale depth image from the kinect source
        grayImage.setFromPixels(kinect.getDepthPixels());
        
        
        
        if(bFlip){
            
            grayImage.mirror(false, true);
        }
        
        
        
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
        // get diff
//        absdiff(grayImage, previous, diff);
//        diff.update();
//        copy(grayImage, previous);
//        blur(diff,10);
        
        grayImage.blur();
        
        contourFinder.setMinAreaRadius(minAreaRadius);
        contourFinder.setMaxAreaRadius(maxAreaRadius);
        contourFinder.setThreshold(trackingThreshold);
        // wait for half a second before forgetting something
//        contourFinder.getTracker().setPersistence(trackingPersistence);
        // an object can move up to 32 pixels per frame
//        contourFinder.getTracker().setMaximumDistance(maxDistance);
        
        

    
        
        // tracking
        // judege if need to sending signal for make waves
        
        // get all tracking contour centroid average value
       contourFinder.findContours(grayImage);

        
        // clear saving
        centerPoint.x = 0.0;
        centerPoint.y = 0.0;
        int trackingContourSize = contourFinder.size();
        
        if(trackingContourSize > 0){
            for (int i = 0; i < contourFinder.size(); i++) {
                centerPoint.x += contourFinder.getCentroid(i).x;
                centerPoint.y += contourFinder.getCentroid(i).y;
            }
            centerPoint.x /= contourFinder.size();
            centerPoint.y /= contourFinder.size();

        }
        
        
        // push into state stack saving
        for (int i = 0; i < trackingDataSize; i++) {
            if(i == trackingDataSize - 1){
                // last element been replaced;
                trackingData[i] = centerPoint;
                
            }
            else{
                trackingData[i] = trackingData[i+1];
            }
            
            
        }
        
        float deltaX = trackingData[trackingDataSize-1].x - trackingData[0].x;
        
        
        
        if (deltaX > maxDistance) {
            waveFromLeftToRight = true;
        }
        
        
        // right to left is minus value
        if(deltaX < -maxDistance){
            waveFromRightToLeft = true;
        }
        
        
        
    
        // check if wave on  now
        waveLtRon = oscTrackingData[0] > -0.1 && oscTrackingData[0] < 1.1;
        
        waveRtLon = oscTrackingData[1] > -0.1 && oscTrackingData[1] < 1.1;
        
        if(waveLtRon){
            oscTrackingData[0] += waveMoveSpeed;
            oscTrackingData[0] += abs(deltaX)/5000.;// speed based on position change distance,deltaX has signal

        }else{
            oscTrackingData[0] = -0.1; // back to ready pos
        }
        
        if(waveRtLon){
            

            oscTrackingData[1] -= waveMoveSpeed;
            oscTrackingData[1]  -= abs(deltaX)/5000.; // speed based on position change distance,and

            
        }else{
            oscTrackingData[1] = 1.1;// back to ready pos
        }
        
        
        
    // prepare osc data
        if(waveFromLeftToRight && !waveLtRon){
            // add speed value first than chack if finish
            oscTrackingData[0] += waveMoveSpeed;
        
        }
        

        
        if(waveFromRightToLeft && !waveRtLon){
            oscTrackingData[1] -= waveMoveSpeed;
        }
        
        
        
    if(bSendingOSC){
        // prepare data for osc send ----------------------------------------
        
        ofxOscMessage m;
        ofxOscMessage m1;
//        m.setAddress("/composition/selectedclip/video/effects/pwl00/effect/float1");
//        m.addFloatArg(ofMap(ofGetMouseX(), 0, ofGetWidth(), 0.f, 1.f, true));
//        //    m.addFloatArg(ofMap(ofGetMouseY(), 0, ofGetHeight(), 0.f, 1.f, true));
//        sender.sendMessage(m, false);
//        m.clear();
        
        string data;
        
        for(int i = 0;i<oscTrackingDataSize;i++){
//            oscTrackingData[i] = -0.1;
//            oscTrackingData[i] += i * 0.1;
            data += ofToString(oscTrackingData[i]);
            if(i != oscTrackingDataSize - 1){
                data += ",";
            }
            
        }
        

        cout << data << endl;
        
        // debug ================
//        m.setAddress("/composition/selectedclip/video/effects/pwaveword/effect/osctextdata0");
        m.setAddress("/composition/selectedclip/video/effects/pwaveword/effect/osctextdata0");
        m.addStringArg(data);
        sender.sendMessage(m,false);
        
        
        m1.setAddress("/composition/selectedclip/video/effects/pwaveline/effect/oscdataline0");
        m1.addStringArg(data);
        sender.sendMessage(m1,false);
        
        m1.clear();
        
        
        
        
//
        }
    }
}

//--------------------------------------------------------------
void ofApp::draw(){
    



    
    
//    kinect.getDepthTexture().draw(0, 0);
    grayImage.draw(640,0);
//    diff.draw(0, 0);

    
    contourFinder.draw();
    
    // for debug
    ofSetColor(0, 0, 155);
    ofDrawCircle(centerPoint.x, centerPoint.y, 30);
    
    ofSetColor(255);
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
