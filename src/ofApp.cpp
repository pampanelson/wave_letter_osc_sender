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
    
    gui.add(bShowLabels.set("show label", false));
    gui.add(minAreaRadius.set("min area",1,1,300));
    gui.add(maxAreaRadius.set("max area",10,1,800));
    gui.add(trackingThreshold.set("tracking thresh",1,1,100));
    gui.add(maxDistance.set("max dis",10,1,500));
    gui.add(trackingPersistence.set("persistence",15,1,60)); // frames
    
   
    gui.add(topSignThresh.set("top sign",0,0,60)); // frames
    gui.add(leftSignThresh.set("left sign",0,0,60)); // frames
    gui.add(rightSignThresh.set("right sign",0,0,60)); // frames
    gui.add(leftCountLimit.set("left limit",0,0,20)); // frames
    gui.add(rightCountLimit.set("right limit",0,0,20)); // frames

    
    if (!ofFile("settings.xml"))
        gui.saveToFile("settings.xml");
    
    gui.loadFromFile("settings.xml");
    
    
    // init tracking data size;
    trackingDataSize = 8;// 30 frames for a sencod
    
    for(int i = 0;i<trackingDataSize;i++){
        
        ofPoint p = ofPoint(0.0,0.0,0.0);
        trackingData.push_back(p);
        
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
        float topSign;
        float leftSign;
        float rightSign;
        contourFinder.findContours(grayImage);

        if(bTracking && contourFinder.size() > 0){
//            contourFinder.findContours(diff);

            
            float biggest;
            int biggestIndex;
            for(int i = 0; i < contourFinder.size(); i++) {
                
                if(contourFinder.getBoundingRect(i).area() > biggest){
                    biggest = contourFinder.getBoundingRect(i).area();
                    biggestIndex = i;
                }
            }
            
            float x = contourFinder.getBoundingRect(biggestIndex).x;
            float y = contourFinder.getBoundingRect(biggestIndex).y;
            
            float w = contourFinder.getBoundingRect(biggestIndex).width;
            float h = contourFinder.getBoundingRect(biggestIndex).height;
            
            float left = x;
            float right = x + w;

            float deltaTop = y - preTop;
            float deltaLeft = left - preLeft;
            float deltaRight = right - preRigh;// delta is smaller is better , smaller than 0 means keeping move to left
            
            
            preRigh = right;
            preLeft = left;
            preTop = y;
            

            for (int i = 0; i < trackingDataSize; i++) {
                if(i == trackingDataSize - 1){
                    // last element been replaced;
                    trackingData[i] = ofPoint(deltaTop,deltaLeft,deltaRight);

                }
                else{
                    trackingData[i] = trackingData[i+1];
                }
                
                
                topSign += trackingData[i][0];
                leftSign += trackingData[i][1];
                rightSign += trackingData[i][2];
                
            }
            
            
            
            topSign /= trackingDataSize;
            leftSign /= trackingDataSize;
            rightSign /= trackingDataSize;
            
            
            
            
        }

    
        if(topSign > topSignThresh ){
            
            if(leftSign > leftSignThresh){
                // make wave from left to right
                waveLtoRCount += 1;
            }
            
            // smaller right sign means keep moving to left
            if(rightSign < -rightSignThresh){
                // make wave from righ to left
                waveRtoLCount += 1;
            }
        }
    
            
        if(waveLtoRCount > 0){
            waveLtoRCount -= 0.5;
        }
        
        if(waveRtoLCount > 0){
            waveRtoLCount -= 0.5;
        }
    
        
        
        if(waveRtoLCount > rightCountLimit){
            waveFromRightToLeft = true;
        }
        
        if(waveLtoRCount > leftCountLimit){
            waveFromLeftToRight = true;
        }
    
    
        cout << "l to r :" << waveLtoRCount << "," << " r to l :" << waveRtoLCount << endl;
    if(bSendingOSC){
        // prepare data for osc send ----------------------------------------
        
        ofxOscMessage m;
        ofxOscMessage m1;
//        m.setAddress("/composition/selectedclip/video/effects/pwl00/effect/float1");
//        m.addFloatArg(ofMap(ofGetMouseX(), 0, ofGetWidth(), 0.f, 1.f, true));
//        //    m.addFloatArg(ofMap(ofGetMouseY(), 0, ofGetHeight(), 0.f, 1.f, true));
//        sender.sendMessage(m, false);
//        m.clear();
        
        string data = ofToString(waveFromLeftToRight) + ofToString(waveFromRightToLeft);
        
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

    
    ofSetBackgroundAuto(true);
    RectTracker& tracker = contourFinder.getTracker();
    
    if(bShowLabels && contourFinder.size() > 0) {
        ofSetColor(255);
        
//        contourFinder.draw();
        
        float biggest;
        int biggestIndex;
        for(int i = 0; i < contourFinder.size(); i++) {
            
            if(contourFinder.getBoundingRect(i).area() > biggest){
                biggest = contourFinder.getBoundingRect(i).area();
                biggestIndex = i;
            }
        }
        
        float x = contourFinder.getBoundingRect(biggestIndex).x;
        float y = contourFinder.getBoundingRect(biggestIndex).y;
        
        float w = contourFinder.getBoundingRect(biggestIndex).width;
        float h = contourFinder.getBoundingRect(biggestIndex).height;
        
        ofSetColor(255,255,0,50);
        ofDrawRectangle(x, y, w, h);
        
        ofSetColor(255);
        ofPoint center = toOf(contourFinder.getCenter(biggestIndex));
        ofPushMatrix();
        ofTranslate(center.x, center.y);
        int label = contourFinder.getLabel(biggestIndex);
        string msg = ofToString(label) + ":" + ofToString(tracker.getAge(label));
        ofDrawBitmapString(msg, 0, 0);
        ofVec2f velocity = toOf(contourFinder.getVelocity(biggestIndex));
        ofScale(5, 5);
        ofDrawLine(0, 0, velocity.x, velocity.y);
        ofPopMatrix();
        
        
    }
    
    
    float highLight = 0;
    if(waveFromLeftToRight){
        highLight = 50;
    }
    
        ofSetColor(255, 0, 0,waveLtoRCount*10+highLight);
        ofDrawRectangle(0,0, 320, 480);

    highLight = 0;
    
    if(waveFromRightToLeft){
        highLight = 50;
    }

        ofSetColor(0,0,255,waveRtoLCount*10 + highLight);
        ofDrawRectangle(320, 0, 320, 480);
        
    
    
    
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
