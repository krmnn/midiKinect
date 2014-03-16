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

    memset(&blobA, 0, sizeof(Blob));
    memset(&blobB, 0, sizeof(Blob));
    blobA.on = false;
    blobB.on = false;
    blobPrevA.on = false;
    blobPrevB.on = false;

    note_offset = 24;

    scale = new int[lines * columns];
    scale[0] = 65;
    scale[1] = 64;
    scale[2] = 62;
    scale[3] = 60;
    scale[4] = 59;
    scale[5] = 57;
    scale[6] = 55;
    scale[7] = 53;
    scale[8] = 52;
    scale[9] = 50;
    scale[10] = 48;
    scale[11] = 47;
    scale[12] = 45;
    scale[13] = 43;
    scale[14] = 41;
    scale[15] = 40;
    scale[16] = 38;
    scale[17] = 36;
    scale[18] = 35;
    scale[19] = 33;

}

void midiKinect::sendNoteOn(Blob *blob) {
    int note = scale[blob->pos] + note_offset;
    midiOut.sendNoteOn(channel, note, 100);
    ofLogNotice() << "noteOn! " << blob->pos << endl;
}

void midiKinect::sendNoteOff(Blob *blob) {
    int note = scale[blob->pos] + note_offset;
    midiOut.sendNoteOff(channel, note, 100);
    ofLogNotice() << "noteOff! " << blob->pos << endl;
}

void midiKinect::analyzeBlob(Blob *current, Blob *previous, ofxCvBlob cvBlob) {

    int hystX = stepX / 4;
    int hystY = stepY / 4;

    current->on = true;
    current->x = int(cvBlob.centroid.x);
    current->y = int(cvBlob.centroid.y);

    int blobPosX = int( current->x / stepX);
    int blobPosY = int( current->y / stepY);

    current->pos = (blobPosY * columns) + blobPosX; 

    // where do we come from?
    if (previous->on) {
        if (previous->pos > current->pos) {
            blobPosX = int((current->x + hystX) / stepX);
        } else if (previous->pos < current->pos) {
            blobPosX = int((current->x - hystX) / stepX);
        }
        if ((previous->pos / columns) > (current->pos / columns)) {
            blobPosY = int((current->y - hystY) / stepY);
        } else if ((previous->pos / columns) < (current->pos /columns)) {
            blobPosY = int((current->y + hystY) / stepY);
        }
        current->pos = (blobPosY * columns) + blobPosX; 
    }

    current->offset = ofDist(previous->x, previous->y, current->x, current->y); 

    // get distance to object in millimeters and map to MIDI CC range
    current->distance = kinect.getDistanceAt(current->x, current->y);
    current->velocity = 127 - ofMap(current->distance, 472, 648, 0, 127, true);

    // TODO: better notes!
    current->note = 64 - current->pos;

}

void midiKinect::triggerMIDI(Blob *current, Blob *previous) {

    //ofLogNotice() << "current " << current->pos << " last " << previous->pos << endl;

    if (previous->on && (previous->pos != current->pos)) {

        // we don't trigger a new note if we didn't enter the field from the front
        // instead we keep the old note
        //current->pos = previous->pos;

        // test
        sendNoteOn(current);

        // control MIDI CC 74 on z-axis
        //midiOut.sendControlChange(channel, 74, current->velocity);

        // control fine grain pitch on y-axis 
        //midiOut.sendPitchBend(channel, (-1) * ofMap(current->y, 0, 480, 0, 4000));
    } 

    if (!previous->on && current->on) {
        // trigger note ON
        sendNoteOn(current);
    }

    if (previous->on && !current->on) {
        // trigger note OFF
        sendNoteOff(previous);
    }

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
        contourFinder.findContours(grayImage, 700, 12000, 2, false, false);

        // reset blobs
        blobA.on = false;
        blobB.on = false;

        // find out the position of the detected objects
        // we only allow two blobs for now...
        if (contourFinder.nBlobs >= 1) {
            analyzeBlob(&blobA, &blobPrevA, contourFinder.blobs[0]);
        } 
        if (contourFinder.nBlobs == 2) {
            analyzeBlob(&blobB, &blobPrevB, contourFinder.blobs[1]);
        } 

        if (blobA.on && blobB.on && (blobB.pos == blobA.pos)) {
            // stupid...
            triggerMIDI(&blobA, &blobPrevA);
        } else {
            triggerMIDI(&blobA, &blobPrevA);
            triggerMIDI(&blobB, &blobPrevB);
        }

        blobPrevA = blobA;
        blobPrevB = blobB;

    }
}

void midiKinect::draw() {

    // camera view offset
    int offset=0;

    // flip image
    ofPushMatrix(); // save the old coordinate system
    ofTranslate(ofGetWidth(), 0.0f); // move the origin to the bottom-left hand corner of the window
    ofScale(-1.0f, 1.0f); // flip the y axis vertically, so that it points upward

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
    if(blobB.on) {
        reportStream 
            << "blob B, x: " << blobB.x << " y: " << blobB.y << " field: " << blobB.pos << " distance: " << blobB.distance << endl;
    }

    ofDrawBitmapString(reportStream.str(), offset + 5, offset + gridHeight + 10);

    // restore matrix
    ofPopMatrix();
}

void midiKinect::exit() {
    //kinect.setCameraTiltAngle(0); // zero the tilt on exit
    //midiOut.closePort();
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

        case 'p':
            for (int i=0; i < 20; i++) {
                midiOut.sendNoteOff(channel, scale[i], 0);
            }
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

        case OF_KEY_RIGHT:
            note_offset = note_offset + 12;
            break;

        case OF_KEY_LEFT:
            note_offset = note_offset - 12;
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
