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


    gui.add(track1PosX.set("t1 x",0,0,640));
    gui.add(track1PosY.set("t1 y",0,0,480));

    gui.add(track1W.set("t1 w",0,0,640));
    gui.add(track1H.set("t1 h",0,0,480));
    
    gui.add(track2PosX.set("t2 x",0,0,640));
    gui.add(track2PosY.set("t2 y",0,0,480));
    
    gui.add(track2W.set("t2 w",0,0,640));
    gui.add(track2H.set("t2 h",0,0,480));
    
    gui.add(track3PosX.set("t3 x",0,0,640));
    gui.add(track3PosY.set("t3 y",0,0,480));
    
    gui.add(track3W.set("t3 w",0,0,640));
    gui.add(track3H.set("t3 h",0,0,480));
    
    gui.add(track4PosX.set("t4 x",0,0,640));
    gui.add(track4PosY.set("t4 y",0,0,480));
    
    gui.add(track4W.set("t4 w",0,0,640));
    gui.add(track4H.set("t4 h",0,0,480));
    
    gui.add(track5PosX.set("t5 x",0,0,640));
    gui.add(track5PosY.set("t5 y",0,0,480));
    
    gui.add(track5W.set("t5 w",0,0,640));
    gui.add(track5H.set("t5 h",0,0,480));
    
    gui.add(track6PosX.set("t6 x",0,0,640));
    gui.add(track6PosY.set("t6 y",0,0,480));
    
    gui.add(track6W.set("t6 w",0,0,640));
    gui.add(track6H.set("t6 h",0,0,480));
    
    gui.add(track7PosX.set("t7 x",0,0,640));
    gui.add(track7PosY.set("t7 y",0,0,480));
    
    gui.add(track7W.set("t7 w",0,0,640));
    gui.add(track7H.set("t7 h",0,0,480));
    
    

    
    gui.add(nearThreshold.set("near",230,1,255));
    gui.add(farThreshold.set("far",70,1,255));
    gui.add(bThreshWithOpenCV.set("use opencv", false));
    gui.add(bFlip.set("flip", false));
    gui.add(angle.set("angle",1,0,180));


    
    if (!ofFile("settings.xml"))
        gui.saveToFile("settings.xml");
    
    gui.loadFromFile("settings.xml");
    
    
    // init tracking data size;
    trackingDataSize = 12;
    
    for(int i = 0;i<trackingDataSize;i++){
        trackingData.push_back(0.0);
        
    }
    
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

    // clear tracking data with 0.0
    trackingData.clear();
    for(int i = 0;i<trackingDataSize;i++){
        trackingData.push_back(0.0f);
        
    }
    
    
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
        absdiff(grayImage, previous, diff);
        
        
        diff.update();
        
        copy(grayImage, previous);

        cv::	Mat diff1 = toCv(diff);

        
        
        
        // get roi frm gray image
        

        
    
        
        
        // tracking
        
        if(bTracking){
            for (int i = 0; i < diff1.rows; i++) {
                for (int j = 0; j < diff1.cols; j++) {
                    Scalar col = diff1.at<uchar>(i,j);
//                    cout << col[0] << "," << col[1] << "," << col[2] << "," << col[3] << endl;
//                    only col[0] has value 0 or 255
                    float mark = col[0];
                    
                    // handle tracking data ==================================  IMPORTANT ++++++++++++
                    
                }
            }
        }

        
    }
    
    
    
    // ----------------------------------------  preapare tracking data

    
    
    
    
    
    
    
    
    
    
    
    
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
        for(int i = 0;i < trackingData.size();i++){
            data += ofToString(trackingData[i]);
            if(i != trackingData.size() - 1){
                data += ",";
                
            }
        }

        cout << data <<  endl;
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

//--------------------------------------------------------------
void ofApp::draw(){
    



    
    
//    kinect.getDepthTexture().draw(0, 0);
    grayImage.draw(0,0);
    diff.draw(640,0);


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
