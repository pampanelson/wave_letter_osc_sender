#include "ofApp.h"


using namespace ofxCv;
using namespace cv;


//--------------------------------------------------------------
void ofApp::setup(){
    ofSetFrameRate(60); // run at 60 fps

    // open an outgoing connection to HOST:PORT
    sender.setup(HOST, PORT);

    cam.setup(640, 480);
    
    
    cCam.allocate(640, 480);
    grayCam.allocate(640, 480);
    //grayCam.allocatePixels(640,480);
    imitate(previous, grayCam);
    imitate(diff, grayCam);
    
    
    gui.setup();
    gui.add(minArea.set("Min area", 10, 1, 100));
    gui.add(maxArea.set("Max area", 200, 1, 500));
    gui.add(threshold.set("Threshold", 128, 0, 255));
    gui.add(holes.set("Holes", false));
    gui.add(bUseTgtColor.set("use target color",false));
    gui.add(tgtColorThreshold.set("target color Threshold", 128, 0, 255));
//    gui.add(trackHs.set("Track Hue/Saturation", false));
    
    if (!ofFile("settings.xml"))
        gui.saveToFile("settings.xml");
    
    gui.loadFromFile("settings.xml");}

//--------------------------------------------------------------
void ofApp::update(){
    ofxOscMessage m;
    m.setAddress("/composition/selectedclip/video/effects/pwl00/effect/float1");
    m.addFloatArg(ofMap(ofGetMouseX(), 0, ofGetWidth(), 0.f, 1.f, true));
//    m.addFloatArg(ofMap(ofGetMouseY(), 0, ofGetHeight(), 0.f, 1.f, true));
    sender.sendMessage(m, false);
    
    cam.update();
    if(cam.isFrameNew()) {
        

        
        
        
        // take the absolute difference of prev and cam and save it inside diff
        cCam.setFromPixels(cam.getPixels());
//        cCam.setROI(100, 100, 100, 100);

        grayCam = cCam; // convert our color image to a grayscale image

        // set ROI

        absdiff(grayCam, previous, diff);
        diff.update();

        // like ofSetPixels, but more concise and cross-toolkit
        copy(grayCam, previous);


        cv::Mat diff1 = toCv(diff);
        cv::Rect crop_roi = cv::Rect(100,100,100,100);
        cv::Mat crop = diff1(crop_roi).clone();

        contourFinder.setMinAreaRadius(minArea);
        contourFinder.setMaxAreaRadius(maxArea);
        contourFinder.setThreshold(threshold);
        // analyse original cam
        //        contourFinder.findContours(cam);

        // analyse diff
        //        contourFinder.findContours(diff);

        // analyse roi
        contourFinder.findContours(crop);

        contourFinder.setFindHoles(holes);
        
        if(bUseTgtColor){
//            contourFinderTgtColor.setTargetColor(targetColor, trackHs ? TRACK_COLOR_HS : TRACK_COLOR_RGB);
            contourFinderTgtColor.setTargetColor(targetColor);
            contourFinderTgtColor.setThreshold(tgtColorThreshold.get());
            contourFinderTgtColor.findContours(grayCam);

        }

        
        

        
    }
}

//--------------------------------------------------------------
void ofApp::draw(){
    cam.draw(0, 0);
    grayCam.draw(0,480);
    
    ofTranslate(100, 100);
    contourFinder.draw();
    
    ofTranslate(-100, -100);
    
    diff.draw(640, 0);
    
    gui.draw();

    ofSetColor(255, 0, 0, 100);
    ofDrawRectangle(100, 100, 100, 100);
    
    ofSetColor(255);
    if(bUseTgtColor){
        ofTranslate(640, 480);
        contourFinderTgtColor.draw();
    }
    

    
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
