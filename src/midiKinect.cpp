#include "midiKinect.h"

//TODO: fix image
//--------------------------------------------------------------
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

    // print the intrinsic IR sensor values
    if(kinect.isConnected()) {
        ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
        ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
        ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
        ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
    }

    // setup opencv image arrays
    colorImg.allocate(kinect.width, kinect.height);
    grayImage.allocate(kinect.width, kinect.height);
    grayThreshNear.allocate(kinect.width, kinect.height);
    grayThreshFar.allocate(kinect.width, kinect.height);

    // configure my grid
    columns = 5;
    lines = 4;

    grid_width = kinect.width;
    grid_height = kinect.height;

    grid_width_step = grid_width / columns;
    grid_height_step = grid_height / lines;

    // grid arrays
    grid = new int[lines * columns];
    grid_last = new int[lines * columns];

    // TODO: set me via calibration
    nearThreshold = 255;
    farThreshold = 243;
    bThreshWithOpenCV = true;

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
        if(bThreshWithOpenCV) {
            grayThreshNear = grayImage;
            grayThreshFar = grayImage;
            grayThreshNear.threshold(nearThreshold, true);
            grayThreshFar.threshold(farThreshold);
            cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
        } else {

            // or we do it ourselves - show people how they can work with the pixels
            unsigned char * pix = grayImage.getPixels();

            int numPixels = grayImage.getWidth() * grayImage.getHeight();
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

        // find contours which are between the size of 700 and 6000.
        // also, find holes is set to false so we will save cpu cycles

        // parameter tun auf ca 60 cm
        // TODO: fenster an haende anpassen
        // TODO: parameter von entfernung abhaengig machen
        // int ofxCvContourFinder::findContours(ofxCvGrayscaleImage &input, int minArea, 
        // int maxArea, int nConsidered, bool bFindHoles, bool bUseApproximation=true)

        contourFinder.findContours(grayImage, 700, 10000, 2, false, false);

        // reset grid
        memset(grid, 0, sizeof(int) * lines * columns);
        int last_blob_grid = 0;
        for (int i = 0; i < contourFinder.nBlobs; i++){
            int center_x = int(contourFinder.blobs[i].centroid.x);
            int center_y = int(contourFinder.blobs[i].centroid.y);
            float distance = kinect.getDistanceAt(center_x, center_y);


            int blob_pos_x = int( center_x / grid_width_step);
            int blob_pos_y = int( center_y / grid_height_step);
            //int blob_pos_x = (columns -1) - int( center_x / grid_width_step);
            //int blob_pos_y = int( center_y / grid_height_step);

            int grid_num = (blob_pos_y * columns) + blob_pos_x; 
            velocity = ofMap(distance, 480, 650, 30, 127, true);
            ofLogNotice() << "distance " << distance << endl;
            velocity = 127 - velocity;

            if (grid_num == last_blob_grid)
                continue;

            grid[grid_num] = velocity; // TODO: velocity
            last_blob_grid = grid_num;
        }

        for(int i = 0; i < lines * columns; i++) {
            // index is the note 
            note = 64 - i;
            // grid array value is the velocity
            velocity = grid[i];

            // note did change
            if(grid_last[i] == 0 && grid[i] > 0) {
                midiOut.sendNoteOn(channel, note, 100);
                ofLogNotice() << "note ON for " << i << " v: 100"<< endl;
            } else if (grid_last[i] > 0 && grid[i] > 0) {
                midiOut.sendControlChange(channel, 74, velocity);
                ofLogNotice() << "CC for 74 to " << velocity << endl;
            } else if (grid_last[i] > 0 && grid[i] == 0) { 
                midiOut.sendNoteOff(channel, note, 0);
                ofLogNotice() << "note off for " << i << endl;

            }
        }
        memcpy(grid_last, grid, sizeof(int) * lines * columns);
    }

}

//--------------------------------------------------------------
void midiKinect::draw() {

    // camera view offset
    int woff=100;

    //ofPushMatrix(); // save the old coordinate system
    //ofTranslate(ofGetWidth(), 0.0f); // move the origin to the bottom-left hand corner of the window
    //ofScale(-1.0f, 1.0f); // flip the y axis vertically, so that it points upward

    ofSetColor(255, 255, 255);

    // ofSetColor(255, 0, 0);
    grayImage.draw(woff, woff, 640, 480);

    // draw from the live kinect
    //kinect.drawDepth(10, 10, 400, 300);
    //kinect.draw(420, 10, 400, 300);

    //grayImage.mirror(false, true);
    for(int i=1; i < lines; i=i+1) {
        ofLine(woff,(i * grid_height_step) + woff, woff + grid_width, (i * grid_height_step) + woff);
    }
    for(int i=1; i < columns; i=i+1) {
        ofLine((i * grid_width_step) + woff, woff, (i * grid_width_step) + woff, grid_height + woff);
    }

    for(int i = 0; i < lines * columns; i++) {
        if (grid[i] > 0) {
            ofSetColor(0,0,255);
            ofFill();
            ofRect(woff + (i % columns) * grid_width_step, int(i / columns) * grid_height_step + woff, grid_width_step, grid_height_step);
        }

    }
    contourFinder.draw(woff, woff, 640, 480);


    // draw instructions
    ofSetColor(255, 255, 255);
    stringstream reportStream;

    if(kinect.hasAccelControl()) {
        reportStream << "accel is: " << ofToString(kinect.getMksAccel().x, 2) << " / "
            << ofToString(kinect.getMksAccel().y, 2) << " / "
            << ofToString(kinect.getMksAccel().z, 2) << endl;
    } else {
        reportStream << "Note: this is a newer Xbox Kinect or Kinect For Windows device," << endl
            << "motor / led / accel controls are not currently supported" << endl << endl;
    }

    reportStream 
        << "using opencv threshold = " << bThreshWithOpenCV <<" (press spacebar)" << endl
        << "set near threshold " << nearThreshold << " (press: + -)" << endl
        << "set far threshold " << farThreshold << " (press: < >)" << endl 
        << "num blobs found " << contourFinder.nBlobs << endl
        << ", fps: " << ofGetFrameRate() << endl
        << endl;

    if(kinect.hasCamTiltControl()) {
        reportStream << "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl
            << "press 1-5 & 0 to change the led mode" << endl;
    }

    reportStream << "width: " << grayImage.getWidth() << " height: " << grayImage.getHeight() << " " << endl;
    reportStream << "width_step: " << grid_width_step << " height_step: " << grid_height_step << " " << endl;


    ofDrawBitmapString(reportStream.str(), woff + grid_width, 652);

    //ofPopMatrix();
}

//--------------------------------------------------------------
void midiKinect::exit() {
    //kinect.setCameraTiltAngle(0); // zero the tilt on exit
    midiOut.closePort();
    kinect.close();


}

//--------------------------------------------------------------
void midiKinect::keyPressed (int key) {
    switch (key) {
        case ' ':
            bThreshWithOpenCV = !bThreshWithOpenCV;
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

        case '1':
            kinect.setLed(ofxKinect::LED_GREEN);
            break;

        case '2':
            kinect.setLed(ofxKinect::LED_YELLOW);
            break;

        case '3':
            kinect.setLed(ofxKinect::LED_RED);
            break;

        case '4':
            kinect.setLed(ofxKinect::LED_BLINK_GREEN);
            break;

        case '5':
            kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
            break;

        case '0':
            kinect.setLed(ofxKinect::LED_OFF);
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

//--------------------------------------------------------------
void midiKinect::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void midiKinect::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void midiKinect::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void midiKinect::windowResized(int w, int h)
{}
