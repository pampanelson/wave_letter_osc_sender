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
//    gui.add(ellipseRadius.set("ellipse raidus",0.1,0,1));
//    gui.add(ellipseWideX.set("ellipse wide x",0.05,0,1));
//    gui.add(ellipseWideY.set("ellipse wide y",0.05,0,2));

    gui.add(bTrack1Diff.set("t1 diff",false));
    gui.add(track1PosX.set("t1 x",0,0,640));
    gui.add(track1PosY.set("t1 y",0,0,480));

    gui.add(track1W.set("t1 w",1,1,640));
    gui.add(track1H.set("t1 h",1,1,480));
    
    
    gui.add(bTrack2Diff.set("t2 diff",false));
    gui.add(track2PosX.set("t2 x",1,1,640));
    gui.add(track2PosY.set("t2 y",1,1,480));
    
    gui.add(track2W.set("t2 w",1,1,640));
    gui.add(track2H.set("t2 h",1,1,480));
    
    gui.add(bTrack3Diff.set("t3 diff",false));
    gui.add(track3PosX.set("t3 x",1,1,640));
    gui.add(track3PosY.set("t3 y",1,1,480));
    
    gui.add(track3W.set("t3 w",1,1,640));
    gui.add(track3H.set("t3 h",1,1,480));
    
    
    gui.add(bTrack4Diff.set("t4 diff",false));
    gui.add(track4PosX.set("t4 x",1,1,640));
    gui.add(track4PosY.set("t4 y",1,1,480));
    
    gui.add(track4W.set("t4 w",1,1,640));
    gui.add(track4H.set("t4 h",1,1,480));
    
    
    gui.add(bTrack5Diff.set("t5 diff",false));
    gui.add(track5PosX.set("t5 x",1,1,640));
    gui.add(track5PosY.set("t5 y",1,1,480));
    
    gui.add(track5W.set("t5 w",1,1,640));
    gui.add(track5H.set("t5 h",1,1,480));
    
    
    gui.add(bTrack6Diff.set("t6 diff",false));
    gui.add(track6PosX.set("t6 x",1,1,640));
    gui.add(track6PosY.set("t6 y",1,1,480));
    
    gui.add(track6W.set("t6 w",1,1,640));
    gui.add(track6H.set("t6 h",1,1,480));
    
    
    gui.add(bTrack7Diff.set("t7 diff",false));
    gui.add(track7PosX.set("t7 x",1,1,640));
    gui.add(track7PosY.set("t7 y",1,1,480));
    
    gui.add(track7W.set("t7 w",1,1,640));
    gui.add(track7H.set("t7 h",1,1,480));
    
    

    
    gui.add(nearThreshold.set("near",230,1,255));
    gui.add(farThreshold.set("far",70,1,255));
    gui.add(bThreshWithOpenCV.set("use opencv", false));
    gui.add(bFlip.set("flip", false));
    gui.add(angle.set("angle",1,0,180));


    
    if (!ofFile("settings.xml"))
        gui.saveToFile("settings.xml");
    
    gui.loadFromFile("settings.xml");
    
    
    // init tracking data size;
    trackingDataSize = 7;
    
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
    trackingPointDataX.clear();
    trackingPointDataY.clear();

    for(int i = 0;i<trackingDataSize;i++){
        trackingData.push_back(0.0f);
        trackingPointDataX.push_back(0);
        trackingPointDataY.push_back(0);

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

        cv::Mat diff1 = toCv(diff);

        // try use color in area but change;
        cv::Mat gray1 = toCv(grayImage);
        
        // get roi frm gray image
        
        
//        for (int i = 0; i < gray1.cols; i++) {
//            for (int j = 0; j < gray1.rows; j++) {
//                Scalar col = gray1.at<uchar>(j,i);
//
//                float x = j;
//                float y = i;
//
//
//
//                float length = sqrt((x-480)*ellipseWideX*ellipseWideX*(x-480) + (y-320)*ellipseWideY*ellipseWideY*(y-320));
//
//                if(length < ellipseRadius * 240){
//                    gray1.at<uchar>(j, i, 0) = 255;
//                }
//            }
//        }
//
    
//        grayImage.flagImageChanged();

        
        // tracking
        
        if(bTracking){
            
            vector<int> tracking1VecX;
            vector<int> tracking2VecX;
            vector<int> tracking3VecX;
            vector<int> tracking4VecX;
            vector<int> tracking5VecX;
            vector<int> tracking6VecX;
            vector<int> tracking7VecX;
            
            vector<int> tracking1VecY;
            vector<int> tracking2VecY;
            vector<int> tracking3VecY;
            vector<int> tracking4VecY;
            vector<int> tracking5VecY;
            vector<int> tracking6VecY;
            vector<int> tracking7VecY;
            
            for (int i = 0; i < diff1.cols; i++) {
                for (int j = 0; j < diff1.rows; j++) {
                    Scalar col = diff1.at<uchar>(j,i);
                    //                    cout << col[0] << "," << col[1] << "," << col[2] << "," << col[3] << endl;
                    //                    only col[0] has value 0 or 255
                    float mark = col[0]/255.0;
                    
                    Scalar col1 = gray1.at<uchar>(j,i);
                    float mark1 = col1[0]/255.0;
                    
                    // handle tracking data ==================================  IMPORTANT ++++++++++++
                    if(i < track1PosX + track1W && i > track1PosX && j < track1PosY + track1H && j > track1PosY ){
                        
                        if(bTrack1Diff){
                            trackingData[0] += mark/(track1W*track1H);
                            
                        }else{
                            
                            
                            trackingData[0] += mark1/(track1W*track1H);
                            if(mark1 > 0.0){
                                
                                tracking1VecX.push_back(i);
                                tracking1VecY.push_back(j);
                            }
                            
                        }
                        
                    }
                    
                    
                    
                    
                    if(i < track2PosX + track2W && i > track2PosX && j < track2PosY + track2H && j > track2PosY ){
                        if(bTrack2Diff){
                            trackingData[1] += mark/(track2W*track2H);
                            
                        }else{
                            
                            
                            trackingData[1] += mark1/(track2W*track2H);
                            if(mark1 > 0.0){
                                tracking2VecX.push_back(i);
                                tracking2VecY.push_back(j);
                            }
                        }
                        
                    }
                    
                    
                    
                    
                    
                    if(i < track3PosX + track3W && i > track3PosX && j < track3PosY + track3H && j > track3PosY ){
                        if(bTrack3Diff){
                            trackingData[2] += mark/(track3W*track3H);
                            
                        }else{
                            
                            
                            trackingData[2] += mark1/(track3W*track3H);
                            if(mark1 > 0.0){
                                tracking3VecX.push_back(i);
                                tracking3VecY.push_back(j);
                            }
                        }
                    }
                    
                    
                    
                    
                    
                    if(i < track4PosX + track4W && i > track4PosX && j < track4PosY + track4H && j > track4PosY ){
                        if(bTrack4Diff){
                            trackingData[3] += mark/(track4W*track4H);
                            
                        }else{
                            
                            
                            trackingData[3] += mark1/(track4W*track4H);
                            if(mark1 > 0.0){
                                tracking4VecX.push_back(i);
                                tracking4VecY.push_back(j);
                            }
                        }
                    }
                    
                    
                    
                    
                    
                    
                    if(i < track5PosX + track5W && i > track5PosX && j < track5PosY + track5H && j > track5PosY ){
                        if(bTrack5Diff){
                            trackingData[4] += mark/(track5W*track5H);
                            
                        }else{
                            
                            
                            trackingData[4] += mark1/(track5W*track5H);
                            if(mark1 > 0.0){
                                tracking5VecX.push_back(i);
                                tracking5VecY.push_back(j);
                            }
                        }
                    }
                    
                    
                    
                    
                    
                    
                    if(i < track6PosX + track6W && i > track6PosX && j < track6PosY + track6H && j > track6PosY ){
                        if(bTrack6Diff){
                            trackingData[5] += mark/(track6W*track6H);
                            
                        }else{
                            
                            
                            trackingData[5] += mark1/(track6W*track6H);
                            if(mark1 > 0.0){
                                tracking6VecX.push_back(i);
                                tracking6VecY.push_back(j);
                            }
                        }
                    }
                    
                    
                    
                    
                    
                    if(i < track7PosX + track7W && i > track7PosX && j < track7PosY + track7H && j > track7PosY ){
                        if(bTrack7Diff){
                            trackingData[6] += mark/(track7W*track7H);
                            
                        }else{
                            
                            
                            trackingData[6] += mark1/(track7W*track7H);
                            if(mark1 > 0.0){
                                tracking7VecX.push_back(i);
                                tracking7VecY.push_back(j);
                            }
                        }
                        
                    }
                    
                    
                    
                }
                
                
                
            
                
                
            }
            
            
            // proress tracking point raw data
            
            for (int i = 0; i < tracking1VecX.size(); i++) {
                trackingPointDataX[0] += tracking1VecX[i];
                trackingPointDataY[0] += tracking1VecY[i];
            }
            if(tracking1VecX.size() > 0){
                trackingPointDataX[0] = trackingPointDataX[0] / tracking1VecX.size();
                trackingPointDataY[0] = trackingPointDataY[0] / tracking1VecX.size();
                
            }

            
            
            for (int i = 0; i < tracking2VecX.size(); i++) {
                trackingPointDataX[1] += tracking2VecX[i];
                trackingPointDataY[1] += tracking2VecY[i];
            }
            
            if(tracking2VecX.size()>0){
                trackingPointDataX[1] = trackingPointDataX[1] / tracking2VecX.size();
                trackingPointDataY[1] = trackingPointDataY[1] / tracking2VecX.size();
            }
            
            
            
            
            
            for (int i = 0; i < tracking3VecX.size(); i++) {
                trackingPointDataX[2] += tracking3VecX[i];
                trackingPointDataY[2] += tracking3VecY[i];
            }
            
            if(tracking3VecX.size()>0){
                trackingPointDataX[2] = trackingPointDataX[2] / tracking3VecX.size();
                trackingPointDataY[2] = trackingPointDataY[2] / tracking3VecX.size();
            }
            
            for (int i = 0; i < tracking4VecX.size(); i++) {
                trackingPointDataX[3] += tracking4VecX[i];
                trackingPointDataY[3] += tracking4VecY[i];
            }
            
            if(tracking4VecX.size()>0){
                trackingPointDataX[3] = trackingPointDataX[3] / tracking4VecX.size();
                trackingPointDataY[3] = trackingPointDataY[3] / tracking4VecX.size();
            }
            
            
            
            
            
            
            for (int i = 0; i < tracking5VecX.size(); i++) {
                trackingPointDataX[4] += tracking5VecX[i];
                trackingPointDataY[4] += tracking5VecY[i];
            }
            
            if(tracking5VecX.size()>0){
                trackingPointDataX[4] = trackingPointDataX[4] / tracking5VecX.size();
                trackingPointDataY[4] = trackingPointDataY[4] / tracking5VecX.size();
            }
            
            
            
            for (int i = 0; i < tracking6VecX.size(); i++) {
                trackingPointDataX[5] += tracking6VecX[i];
                trackingPointDataY[5] += tracking6VecY[i];
            }
            
            if(tracking6VecX.size()>0){
                trackingPointDataX[5] = trackingPointDataX[5] / tracking6VecX.size();
                trackingPointDataY[5] = trackingPointDataY[5] / tracking6VecX.size();
            }
            
            
            
            for (int i = 0; i < tracking7VecX.size(); i++) {
                trackingPointDataX[6] += tracking7VecX[i];
                trackingPointDataY[6] += tracking7VecY[i];
            }
            
            if(tracking7VecX.size()>0){
                trackingPointDataX[6] = trackingPointDataX[6] / tracking7VecX.size();
                trackingPointDataY[6] = trackingPointDataY[6] / tracking7VecX.size();
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
    grayImage.draw(640,0);
    diff.draw(0, 0);



//    // draw tracking area
    ofSetColor(255, 0, 0,128 + trackingData[0] * 128);
    ofDrawRectangle(track1PosX, track1PosY, track1W, track1H);

    ofSetColor(0,255,255);
    ofDrawBitmapString(ofToString(bTrack1Diff) + "-" + ofToString(trackingData[0]), track1PosX, track1PosY);

    
    ofSetColor(0, 255, 0,128 + trackingData[1] * 128);
    ofDrawRectangle(track2PosX, track2PosY, track2W, track2H);

    ofSetColor(255,0,255);
    ofDrawBitmapString(ofToString(bTrack2Diff) +  "-" + ofToString(trackingData[1]), track2PosX, track2PosY);

    
    ofSetColor(0, 0, 255,128 + trackingData[2] * 128);
    ofDrawRectangle(track3PosX, track3PosY, track3W, track3H);

    ofSetColor(255,255,0);
    ofDrawBitmapString(ofToString(bTrack3Diff) + "-" + ofToString(trackingData[2]), track3PosX, track3PosY);


    ofSetColor(255, 255, 255,128 + trackingData[3] * 128);
    ofDrawRectangle(track4PosX, track4PosY, track4W, track4H);
    
    ofSetColor(255,255,0);
    ofDrawBitmapString(ofToString(bTrack4Diff) + "-" + ofToString(trackingData[3]), track4PosX, track4PosY);

    

    ofSetColor(255, 0, 0,128 + trackingData[4] * 128);
    ofDrawRectangle(track5PosX, track5PosY, track5W, track5H);

    ofSetColor(0,255,255);
    ofDrawBitmapString(ofToString(bTrack5Diff) + "-" + ofToString(trackingData[4]), track5PosX, track5PosY);

    

    ofSetColor(0, 255, 0,128 + trackingData[5] * 128);
    ofDrawRectangle(track6PosX, track6PosY, track6W, track6H);

    ofSetColor(255,0,255);
    ofDrawBitmapString(ofToString(bTrack6Diff) + "-" + ofToString(trackingData[5]), track6PosX, track6PosY);

    
    ofSetColor(0, 0, 255,128 + trackingData[6] * 128);
    ofDrawRectangle(track7PosX, track7PosY, track7W, track7H);

    ofSetColor(255,255,0);
    ofDrawBitmapString(ofToString(bTrack7Diff) + "-" + ofToString(trackingData[6]), track7PosX, track7PosY);

    ofSetColor(0, 255,0);
    
    for (int i = 0 ; i < trackingPointDataX.size(); i++) {
        ofDrawCircle(trackingPointDataX[i], trackingPointDataY[i], 5.0);
    }
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
