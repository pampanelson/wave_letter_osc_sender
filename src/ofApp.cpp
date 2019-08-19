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
    gui.add(bTracking.set("Tracking",false));
    gui.add(minArea.set("Min area", 10, 1, 100));
    gui.add(maxArea.set("Max area", 200, 1, 500));
    gui.add(threshold.set("Threshold", 128, 0, 255));
    gui.add(holes.set("Holes", false));
    gui.add(bUseTgtColor.set("use target color",false));
    gui.add(tgtColorThreshold.set("target color Threshold", 128, 0, 255));
//    gui.add(trackHs.set("Track Hue/Saturation", false));
    
    gui.add(track1PosX.set("track 1 r x",0,0,640));
    gui.add(track1PosY.set("track 1 r y",0,0,480));
    gui.add(track1W.set("track 1 r w",0,0,640));
    gui.add(track1H.set("track 1 r h",0,0,480));
    gui.add(track2PosX.set("track 2 g x",0,0,640));
    gui.add(track2PosY.set("track 2 g y",0,0,480));
    gui.add(track2W.set("track 2 g w",0,0,640));
    gui.add(track2H.set("track 2 g h",0,0,480));
    gui.add(track3PosX.set("track 3 b x",0,0,640));
    gui.add(track3PosY.set("track 3 b y",0,0,480));
    gui.add(track3W.set("track 3 b w",0,0,640));
    gui.add(track3H.set("track 3 b h",0,0,480));
    
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
    
    bool bTrackingAreaSet = track1H * track2H * track2W * track3H * track3W;
    cout << bTrackingAreaSet << endl;
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

        trackers.clear();

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
        contourFinder.findContours(crop);

        for (int i = 0; i<contourFinder.size(); i++) {
            cv::Rect rect = contourFinder.getBoundingRect(i);
            float x = track1PosX + rect.x+rect.width * 0.5;
            float y = track1PosY + rect.y + rect.height * 0.5;
            float area = rect.area();
            
            Vec3f t = Vec3f(x,y,area);
            
            trackers.push_back(t);
        }
        
        
        // tracking area 2 -----------------------------------

        crop_roi = cv::Rect(track2PosX,track2PosY,track2W,track2H);
        crop = diff1(crop_roi).clone();
        contourFinder.findContours(crop);

        for (int i = 0; i<contourFinder.size(); i++) {
            cv::Rect rect = contourFinder.getBoundingRect(i);
            float x = track2PosX + rect.x+rect.width * 0.5;
            float y = track2PosY + rect.y + rect.height * 0.5;
            float area = rect.area();
            
            Vec3f t = Vec3f(x,y,area);
            
            trackers.push_back(t);
        }
        
        
        
        // tracking area 3 -----------------------------------

        crop_roi = cv::Rect(track3PosX,track3PosY,track3W,track3H);
        crop = diff1(crop_roi).clone();
        contourFinder.findContours(crop);

        for (int i = 0; i<contourFinder.size(); i++) {
            cv::Rect rect = contourFinder.getBoundingRect(i);
            float x = track3PosX + rect.x+rect.width * 0.5;
            float y = track3PosY + rect.y + rect.height * 0.5;
            float area = rect.area();
            
            Vec3f t = Vec3f(x,y,area);
            
            trackers.push_back(t);
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
}

//--------------------------------------------------------------
void ofApp::draw(){
    

    
    ofSetColor(255);
    cam.draw(0, 0);
    grayCam.draw(0,480);
    
    
    diff.draw(640, 0);
    
    gui.draw();

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
    

    ofSetColor(255,255,0,100);
    for (int i = 0; i < trackers.size(); i++) {
        ofDrawRectangle(trackers[i][0], trackers[i][1], trackers[i][2]/1000, trackers[i][2]/1000);
    }
    
    ofSetColor(0,255,0,255);
    if(trackers.size() > 0){
        ofDrawBitmapString("trackers : " + ofToString(trackers.size()), 10, 10);

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
