#include "midiKinect.h"

void midiKinect::setup() {
    ofSetLogLevel(OF_LOG_VERBOSE);

    // enable depth->video image calibration
    kinect.setRegistration(true);

    if(kinect.init(true) == false) {
        ofLogNotice() << "BAEM" << endl;
        ofLogNotice() << "Aborting..." << endl;
        return;
    }
    //kinect.init(true); // shows infrared instead of RGB video image
    //kinect.init(false, false); // disable video image (faster fps)

    if(kinect.open()==false) {
        ofLogNotice() << "Can not connect to kinect!" << endl;
        ofLogNotice() << "Aborting..." << endl;
        return;
    }	
    ofLogNotice() << "kinect resolution:" << kinect.width << "x" << kinect.height << " " << endl;
    // print the intrinsic IR sensor values
    //if(kinect.isConnected()) {
    //    ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
    //    ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
    //    ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
    //    ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
    //}

    // setup opencv image arrays
    colorImg.allocate(kinect.width, kinect.height);
    grayImage.allocate(kinect.width, kinect.height);
    grayThreshNear.allocate(kinect.width, kinect.height);
    grayThreshFar.allocate(kinect.width, kinect.height);

    // configure my grid
    columns = 5;
    lines = 4;

    noteGridWidth = kinect.width;
    noteGridHeight = kinect.height;

    stepX = noteGridWidth / columns;
    stepY = noteGridHeight / lines;

    // grid arrays
    noteGrid = new int[lines * columns];
    noteGridPrevious = new int[lines * columns];

    // TODO: set me via calibration
    nearThreshold = 255;
    farThreshold = 243;

    // framerate 60Hz
    ofSetFrameRate(60);

    // set kinect tilt on startup
    // TODO: get value from calibration
    angle = 27;
    kinect.setCameraTiltAngle(angle);

    // list available midi ports
    midiOut.listPorts();
    if (midiOut.openPort(0) == false) {
        // we dont have a physical midi out port, so let's open a virtual one
        ofLogNotice() << "we dont have a physical midi out port, so let's open a virtual one" << endl;
        midiOut.openVirtualPort("midiKinectOut"); 
    }

    // midi parameters
    // TODO: make configurable optargs?
    channel = 1;
    note = 0;
    velocity = 64; // 0-127
}

//--------------------------------------------------------------
void midiKinect::update() {

    ofBackground(100, 100, 100);

    kinect.update();

    // there is a new frame and we are connected
    if(kinect.isFrameNew()) {

        // load grayscale depth image from the kinect source
        grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);

        // we do two thresholds - one for the far plane and one for the near plane
        // we then do a cvAnd to get the pixels which are a union of the two thresholds
        grayThreshNear = grayImage;
        grayThreshFar = grayImage;
        grayThreshNear.threshold(nearThreshold, true);
        grayThreshFar.threshold(farThreshold);
        cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);

        // update the cv images
        grayImage.flagImageChanged();

        // find contours which are between the size of 700 and 6000.
        // also, find holes is set to false so we will save cpu cycles
        // parameter tun auf ca 60 cm
        // TODO: fenster an haende anpassen
        // TODO: parameter von entfernung abhaengig machen
        // int ofxCvContourFinder::findContours(ofxCvGrayscaleImage &input, int minArea, 
        // int maxArea, int nConsidered, bool bFindHoles, bool bUseApproximation=true)
        contourFinder.findContours(grayImage, 700, 10000, 2, false, false);

        // reset grid
        memset(noteGrid, 0, sizeof(int) * lines * columns);

        // calculate the position of the hands in the grid
        int blobLastPos = 0;
        for (int i = 0; i < contourFinder.nBlobs; i++){

            // get center of blob
            int blobX = int(contourFinder.blobs[i].centroid.x);
            int blobY = int(contourFinder.blobs[i].centroid.y);

            // which field are we in?
            int blobPosX = int( blobX / stepX);
            int blobPosY = int( blobY / stepY);
            int blobPosInGrid = (blobPosY * columns) + blobPosX; 

            // get distance to blob in millimeters
            float blobDistance = kinect.getDistanceAt(blobX, blobY);
            // map distance to midi CC range
            velocity = 127 - ofMap(blobDistance, 480, 650, 30, 127, true);
            //ofLogNotice() << "blob distance: " << blobDistance << "mm , velocity: " << velocity <<  endl;

            // retrigger?
            if (blobPosInGrid == blobLastPos)
                continue;

            noteGrid[blobPosInGrid] = velocity; 
            blobLastPos = blobPosInGrid;
        }

        // generate MIDI messages
        for(int i = 0; i < lines * columns; i++) {
            // index is the note 
            // TODO: better mapping, support for scales
            note = 64 - i;
            // grid array value is the velocity
            velocity = noteGrid[i];

            // note did change
            if(noteGridPrevious[i] == 0 && noteGrid[i] > 0) {
                midiOut.sendNoteOn(channel, note, 100);
                ofLogNotice() << "note ON for " << i << " v: 100"<< endl;

            } else if (noteGridPrevious[i] > 0 && noteGrid[i] > 0) {
                midiOut.sendControlChange(channel, 74, velocity);
                //ofLogNotice() << "CC for 74 to " << velocity << endl;
                
            } else if (noteGridPrevious[i] > 0 && noteGrid[i] == 0) { 
                midiOut.sendNoteOff(channel, note, 0);
                ofLogNotice() << "note off for " << i << endl;

            }
        }
        memcpy(noteGridPrevious, noteGrid, sizeof(int) * lines * columns);
    }

}

void midiKinect::draw() {

    // camera view offset
    int offset=0;

    // flip image
    //ofPushMatrix(); // save the old coordinate system
    //ofTranslate(ofGetWidth(), 0.0f); // move the origin to the bottom-left hand corner of the window
    //ofScale(-1.0f, 1.0f); // flip the y axis vertically, so that it points upward


    // ofSetColor(255, 0, 0);
    grayImage.draw(offset, offset, 640, 480);

    // draw from the live kinect
    kinect.drawDepth(offset, offset, 640, 480);
    //kinect.draw(offset, offset, 640, 480);

    
    // fill detected grid
    for(int i = 0; i < lines * columns; i++) {
        if (noteGrid[i] > 0) {
            int colorval = ofMap(noteGrid[i], 0, 127, 0, 255);
            ofSetColor(colorval, 160, 0);
            ofFill();
            ofRect(offset + (i % columns) * stepX, int(i / columns) * stepY + offset, stepX, stepY);
        }
    }
    // draw detected blobs
    contourFinder.draw(offset, offset);

    // draw grid
    ofSetColor(255, 255, 255);
    for(int i=1; i < lines; i=i+1) {
        ofLine(offset,(i * stepY) + offset, offset + noteGridWidth, (i * stepY) + offset);
    }
    for(int i=1; i < columns; i=i+1) {
        ofLine((i * stepX) + offset, offset, (i * stepX) + offset, noteGridHeight + offset);
    }
    for(int i=0; i < (lines * columns); i++) {
        // TODO: print actual notes
        ofDrawBitmapString(ofToString(i), offset + ((i % columns) * stepX), offset + 10 + ((i / columns) * stepY));
    }

    // draw instructions and debugging
    ofSetColor(255, 255, 255);
    stringstream reportStream;
    reportStream 
        << "set near threshold " << nearThreshold << " (press: + -)" << endl
        << "set far threshold " << farThreshold << " (press: < >)" << endl 
        << "number of blobs found " << contourFinder.nBlobs << endl
        << "fps: " << ofGetFrameRate() << endl
        << "width: " << grayImage.getWidth() << " height: " << grayImage.getHeight() << " " << endl
        << "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl
        << endl;
    ofDrawBitmapString(reportStream.str(), offset + 5, offset + noteGridHeight + 10);

    // flip image
    //ofPopMatrix();
}

void midiKinect::exit() {
    //kinect.setCameraTiltAngle(0); // zero the tilt on exit
    midiOut.closePort();
    kinect.close();
}

void midiKinect::keyPressed (int key) {
    switch (key) {

        case '>':
        case '.':
            farThreshold ++;
            if (farThreshold > 255) farThreshold = 255;
            break;

        case '<':
        case ',':
            farThreshold --;
            if (farThreshold < 0) farThreshold = 0;
            break;

        case '+':
        case '=':
            nearThreshold ++;
            if (nearThreshold > 255) nearThreshold = 255;
            break;

        case '-':
            nearThreshold --;
            if (nearThreshold < 0) nearThreshold = 0;
            break;

        case OF_KEY_UP:
            angle++;
            if(angle>30) angle=30;
            kinect.setCameraTiltAngle(angle);
            break;

        case OF_KEY_DOWN:
            angle--;
            if(angle<-30) angle=-30;
            kinect.setCameraTiltAngle(angle);
            break;
    }
}

void midiKinect::mouseDragged(int x, int y, int button)
{}

void midiKinect::mousePressed(int x, int y, int button)
{}

void midiKinect::mouseReleased(int x, int y, int button)
{}

void midiKinect::windowResized(int w, int h)
{}
