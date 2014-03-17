#include "midiKinect.h"

void midiKinect::setup() {
    ofSetLogLevel(OF_LOG_VERBOSE);

    ofSetBackgroundColor(0, 0, 0);

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
    // TODO: make midi optargs
    midiChannel = 1;

    memset(&blobA, 0, sizeof(Blob));
    memset(&blobB, 0, sizeof(Blob));
    blobA.on = false;
    blobB.on = false;
    blobPrevA.on = false;
    blobPrevB.on = false;

    // notes get scaled up two octaves
    note_offset = 24;

    // some modes
    Mode *aMinorPentatonic = new Mode("A Minor Pentatonic" , 33, 35, 36, 38, 40, 41, 43, 45, 47, 48, 50, 52, 53, 55, 57, 59, 60, 62, 64, 65);
    Mode *aMixolydian = new Mode("A Mixolydian" , 33, 34, 36, 38, 40, 41, 43, 45, 46, 48, 50, 52, 53, 55, 57, 58, 60, 62, 64, 65);
    Mode *aDorian = new Mode("A Dorian" , 33, 35, 36, 38, 40, 42, 43, 45, 47, 48, 50, 52, 54, 55, 57, 59, 60, 62, 64, 66);

    availModes = new Mode*[3];
    availModes[0] = aMinorPentatonic;
    availModes[1] = aMixolydian;
    availModes[2] = aDorian;

    currentMode = availModes[0];

}

int midiKinect::posToNote(int pos) {
    return currentMode->notes[pos] + note_offset;
}
void midiKinect::sendNoteOn(Blob *blob) {
    midiOut.sendNoteOn(midiChannel, posToNote(blob->pos), 100);
    ofLogNotice() << "MIDI noteOn! #" << int(currentMode->notes[blob->pos]) << endl;
}

void midiKinect::sendNoteOff(Blob *blob) {
    midiOut.sendNoteOff(midiChannel, posToNote(blob->pos), 100);
    ofLogNotice() << "MIDI noteOff! #" << int(currentMode->notes[blob->pos]) << endl;
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


    if (previous->on && (previous->pos != current->pos)) {

        // play new note
        sendNoteOn(current);

        // turn off old one
        sendNoteOff(previous);

        // TODO: idea 
        // we don't trigger a new note if we didn't enter the field from the front
        // instead we keep the old note
        // current->pos = previous->pos;

        // TODO: idea 
        // control fine grain pitch on y-axis 
        // midiOut.sendPitchBend(channel, (-1) * ofMap(current->y, 0, 480, 0, 4000));
        //
    } else {

        // control MIDI CC 74 on z-axis
        //int diff =  max(previous->velocity, current->velocity) - min(previous->velocity, current->velocity);
        //if (diff > 5) {
        //    ofLogNotice() << "set midi CC 74 to val: " << current->velocity << endl;
        //    midiOut.sendControlChange(midiChannel, 74, current->velocity);
        //}

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
    int offset=10;

    // flip image
    ofPushMatrix(); // save the old coordinate system
    ofTranslate(ofGetWidth(), 0.0f); // move the origin to the bottom-left hand corner of the window
    ofScale(-1.0f, 1.0f); // flip the y axis vertically, so that it points upward

    grayImage.draw(offset, offset, 640, 480);

    // draw depth image from the live kinect
    kinect.drawDepth(offset, offset, 640, 480);

    // draw fields which contain detected object
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
        ofDrawBitmapString(ofToString(i), offset + (stepX) + ((i % columns) * stepX), offset + 10 + ((i / columns) * stepY));
    }

    // draw instructions / shortcuts
    ofSetColor(255, 255, 255);
    stringstream reportStream;
    reportStream 
        << "near threshold " << nearThreshold << " (press: + -)" << "\t" 
        << "far threshold " << farThreshold << " (press: < >)" << endl 
        << "fps: " << int(ofGetFrameRate()) << "\t" 
        << "tilt angle: " << angle << " degrees (press UP/DOWN)" << endl
        << endl
        << "base octave #" << (note_offset / 12) << " (press LEFT/RIGHT)" << endl
        << "mode is " << currentMode->name << " (switch with number keys)" << endl
        << endl;

    if(blobA.on) {
        reportStream 
            << "Hand #1 position: (" << blobA.x << ", " << blobA.y << ") field #" << blobA.pos << " distance: " << blobA.distance << "mm" << endl;
    }

    if(blobB.on) {
        reportStream 
            << "Hand #2 position: (" << blobB.x << ", " << blobB.y << ") field #" << blobB.pos << " distance: " << blobB.distance << "mm" << endl;
    }

    ofDrawBitmapString(reportStream.str(), offset + gridWidth, offset + gridHeight + 20);

    // restore matrix
    ofPopMatrix();
}

void midiKinect::exit() {
    //kinect.setCameraTiltAngle(0); // zero the tilt on exit
    midiOut.closePort();
    kinect.close();
}

void midiKinect::keyPressed (int key) {
    switch (key) {
        case '1':
            currentMode = availModes[0];
            break;

        case '2':
            currentMode = availModes[1];
            break;

        case '3':
            currentMode = availModes[2];
            break;

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
        case 'P':
            //for (int i=0; i < 20; i++) {
            //    midiOut.sendNoteOff(midiChannel, scale[i], 0);
            //}
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
