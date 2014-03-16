#include "midiKinect.h"

void midiKinect::setup() {
    ofSetLogLevel(OF_LOG_VERBOSE);

    // enable depth->video image calibration
    kinect.setRegistration(true);

    if(kinect.init() == false) {
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

    // setup opencv image arrays
    colorImg.allocate(kinect.width, kinect.height);
    grayImage.allocate(kinect.width, kinect.height);
    grayThreshNear.allocate(kinect.width, kinect.height);
    grayThreshFar.allocate(kinect.width, kinect.height);

    // configure my grid
    columns = 5;
    lines = 4;

    gridWidth = kinect.width;
    gridHeight = kinect.height;

    stepX = gridWidth / columns;
    stepY = gridHeight / lines;

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

    blobPrevA.on = false;
    blobPrevB.on = false;
}

int midiKinect::getPosInGrid(Blob blob) {

    // which field are we in?
    int blobPosX = int( blob.x / stepX);
    int blobPosY = int( blob.y / stepY);

    return (blobPosY * columns) + blobPosX; 
}

void midiKinect::sendNoteOn(Blob blob) {
    midiOut.sendNoteOn(channel, blob.note, 100);
    ofLogNotice() << "noteOn! " << blob.pos << endl;
}

void midiKinect::sendNoteOff(Blob blob) {
    midiOut.sendNoteOff(channel, blob.note, 100);
    ofLogNotice() << "noteOff! " << blob.pos << endl;
}

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

        // find contours which are between an area of 700 and 10000 pixel
        contourFinder.findContours(grayImage, 700, 10000, 2, false, false);


        blobA.on = false;
        blobB.on = false;

        // we only allow two blobs for now...
        if (contourFinder.nBlobs >= 1) {

            blobA.on = true;
            blobA.x = int(contourFinder.blobs[0].centroid.x);
            blobA.y = int(contourFinder.blobs[0].centroid.y);
            blobA.pos = getPosInGrid(blobA);
            blobA.offset = ofDist(blobPrevA.x, blobPrevA.y, blobA.x, blobA.y); 

            // get distance to hand in millimeters
            blobA.distance = kinect.getDistanceAt(blobA.x, blobB.y);
            blobA.velocity = 127 - ofMap(blobA.distance, 480, 650, 30, 127, true);
            ofLogNotice() << blobA.distance << " " << blobA.velocity << endl;

            blobA.note = 64 - blobA.pos;
        }

        if (blobA.on) {
            if (blobPrevA.on) {

                // we don't trigger a new note if we didn't enter the field from the front
                if (blobA.pos != blobPrevA.pos) {

                    // control MIDI CC 74 on y-axis
                    midiOut.sendControlChange(channel, 74, blobA.velocity);

                    // control fine grain pitch on y-axis 
                    midiOut.sendPitchBend(channel, (-1) * ofMap(blobA.y, 0, 480, 0, 4000));
                } 

            } else  {
                // trigger note ON
                sendNoteOn(blobA);
                blobPrevA = blobA;
            }

        } else {
            // note off!
            if (blobPrevA.on) {
                sendNoteOff(blobPrevA);
            }
            blobPrevA = blobA;
        }
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


    if (blobA.on) {
        int colorval = ofMap(blobA.velocity, 0, 127, 0, 255);
        ofSetColor(colorval, 160, 0);
        ofFill();
        ofRect(offset + (blobA.pos % columns) * stepX, int(blobA.pos / columns) * stepY + offset, stepX, stepY);
    }
    if (blobB.on) {
        int colorval = ofMap(blobB.velocity, 0, 127, 0, 255);
        ofSetColor(colorval, 80, 0);
        ofFill();
        ofRect(offset + (blobB.pos % columns) * stepX, int(blobB.pos / columns) * stepY + offset, stepX, stepY);
    }

    // draw detected blobs
    contourFinder.draw(offset, offset);

    // draw grid
    ofSetColor(255, 255, 255);
    for(int i=1; i < lines; i=i+1) {
        ofLine(offset,(i * stepY) + offset, offset + gridWidth, (i * stepY) + offset);
    }
    for(int i=1; i < columns; i=i+1) {
        ofLine((i * stepX) + offset, offset, (i * stepX) + offset, gridHeight + offset);
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
        << "fps: " << ofGetFrameRate() << endl
        << "width: " << grayImage.getWidth() << " height: " << grayImage.getHeight() << " " << endl
        << "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl
        << endl;

    if(blobA.on) {
        reportStream 
            << "blob A, x: " << blobA.x << " y: " << blobA.y << " field: " << blobA.pos << " distance: " << blobA.distance << endl;
    }
    ofDrawBitmapString(reportStream.str(), offset + 5, offset + gridHeight + 10);

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
