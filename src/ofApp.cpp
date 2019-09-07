#include "ofApp.h"


using namespace ofxCv;
using namespace cv;


//--------------------------------------------------------------
void ofApp::setup(){
    ofSetFrameRate(30); // run at 60 fps

    // open an outgoing connection to HOST:PORT
    sender.setup(HOST, PORT);

    cam.listDevices();
    if(cam.listDevices().size() > 1){
        cam.setDeviceID(1);
    }
    
    cam.setup(640, 480);
    
    
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

    
    cCam.allocate(640, 480);
    grayCam.allocate(640, 480);
    //grayCam.allocatePixels(640,480);
    imitate(previous, grayCam);
    imitate(diff, grayCam);
    
    
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

    
    if (!ofFile("settings.xml"))
        gui.saveToFile("settings.xml");
    
    gui.loadFromFile("settings.xml");}

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
        
        // find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
        // also, find holes is set to true so we will get interior contours as well....
//        contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 20, false);
    }
    
    
    
    
    
    cam.update();
    
    bool bTrackingAreaSet = track1H * track2H * track2W * track3H * track3W;
//    cout << bTrackingAreaSet << endl;
    if(cam.isFrameNew() && bTracking.get() && bTrackingAreaSet) {
        

        
        
        
        // prepare tracking --------------------------------
        // take the absolute difference of prev and cam and save it inside diff
        cCam.setFromPixels(cam.getPixels());
//        cCam.setROI(100, 100, 100, 100);

        grayCam = cCam; // convert our color image to a grayscale image

        // set ROI

        absdiff(grayCam, previous, diff);
        diff.update();

        // like ofSetPixels, but more concise and cross-toolkit
        copy(grayCam, previous);


        contourFinder.setMinAreaRadius(minArea);
        contourFinder.setMaxAreaRadius(maxArea);
        contourFinder.setThreshold(threshold);
        contourFinder.setFindHoles(holes);


        cv::Mat diff1 = toCv(diff);
        cv::Rect crop_roi;
        cv::Mat crop;
        
        // tracking area 1 -----------------------------------
        crop_roi = cv::Rect(track1PosX,track1PosY,track1W,track1H);
        crop = diff1(crop_roi).clone();


        // analyse original cam
        //        contourFinder.findContours(cam);

        // analyse diff
        //        contourFinder.findContours(diff);

        // analyse roi
        
        trackers1.clear();
        contourFinder.findContours(crop);

        for (int i = 0; i<contourFinder.size(); i++) {
            cv::Rect rect = contourFinder.getBoundingRect(i);
            float x = track1PosX + rect.x+rect.width * 0.5;
            float y = track1PosY + rect.y + rect.height * 0.5;
            float area = rect.area();
            
            Vec3f t = Vec3f(x,y,area);
            
            trackers1.push_back(t);
        }
        
        
        // tracking area 2 -----------------------------------

        crop_roi = cv::Rect(track2PosX,track2PosY,track2W,track2H);
        crop = diff1(crop_roi).clone();
        
        trackers2.clear();
        contourFinder.findContours(crop);

        for (int i = 0; i<contourFinder.size(); i++) {
            cv::Rect rect = contourFinder.getBoundingRect(i);
            float x = track2PosX + rect.x+rect.width * 0.5;
            float y = track2PosY + rect.y + rect.height * 0.5;
            float area = rect.area();
            
            Vec3f t = Vec3f(x,y,area);
            
            trackers2.push_back(t);
        }
        
        
        
        // tracking area 3 -----------------------------------

        crop_roi = cv::Rect(track3PosX,track3PosY,track3W,track3H);
        crop = diff1(crop_roi).clone();
        
        trackers3.clear();
        contourFinder.findContours(crop);

        for (int i = 0; i<contourFinder.size(); i++) {
            cv::Rect rect = contourFinder.getBoundingRect(i);
            float x = track3PosX + rect.x+rect.width * 0.5;
            float y = track3PosY + rect.y + rect.height * 0.5;
            float area = rect.area();
            
            Vec3f t = Vec3f(x,y,area);
            
            trackers3.push_back(t);
        }
        
        
        
        
        if(bUseTgtColor){
//            contourFinderTgtColor.setTargetColor(targetColor, trackHs ? TRACK_COLOR_HS : TRACK_COLOR_RGB);
            contourFinderTgtColor.setTargetColor(targetColor);
            contourFinderTgtColor.setThreshold(tgtColorThreshold.get());
            contourFinderTgtColor.findContours(grayCam);

        }

        
        
        
//        cout << trackers.size() << endl;
//        cout << trackers[0][0] << "," << trackers[0][1] << "," << trackers[0][2] << endl;
        
    }
    
    
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
    

    
    ofSetColor(255);
    cam.draw(0, 0);
    grayCam.draw(0,480);
    
    
    diff.draw(640, 0);
    

    if(bUseTgtColor){
        ofTranslate(640, 480);
        contourFinderTgtColor.draw();
        ofTranslate(-640, -480);

    }
    

    
    // draw tracking area
    ofSetColor(255, 0, 0,50);
    ofDrawRectangle(track1PosX, track1PosY, track1W, track1H);

    ofSetColor(0, 255, 0,50);
    ofDrawRectangle(track2PosX, track2PosY, track2W, track2H);
    
    ofSetColor(0, 0, 255,50);
    ofDrawRectangle(track3PosX, track3PosY, track3W, track3H);
    

    ofSetColor(0,255,255,100);
    for (int i = 0; i < trackers1.size(); i++) {
        ofDrawRectangle(trackers1[i][0], trackers1[i][1], trackers1[i][2]/1000, trackers1[i][2]/1000);
    }
    
    ofSetColor(255,0,255,100);
    for (int i = 0; i < trackers2.size(); i++) {
        ofDrawRectangle(trackers2[i][0], trackers2[i][1], trackers2[i][2]/1000, trackers2[i][2]/1000);
    }
    
    ofSetColor(255,255,0,100);
    for (int i = 0; i < trackers3.size(); i++) {
        ofDrawRectangle(trackers3[i][0], trackers3[i][1], trackers3[i][2]/1000, trackers3[i][2]/1000);
    }
    
    ofSetColor(255,0,0,255);
    
    ofDrawBitmapString("trackers 1 : " + ofToString(trackers1.size()), 10, 10);
    ofDrawBitmapString("trackers 2 : " + ofToString(trackers2.size()), 10, 20);
    ofDrawBitmapString("trackers 3 : " + ofToString(trackers3.size()), 10, 30);


    
//    kinect.getDepthTexture().draw(0, 0);
    grayImage.draw(0,0);
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
    targetColor = cam.getPixels().getColor(x, y);

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
