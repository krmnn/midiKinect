#include "testApp.h"


//--------------------------------------------------------------
void testApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	// enable depth->video image calibration
	kinect.setRegistration(true);
    
	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)
	
	kinect.open();		// opens first available kinect
	//kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
	//kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #
	
	// print the intrinsic IR sensor values
	if(kinect.isConnected()) {
		ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
		ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
		ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
		ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
	}
	
	
	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);


        // grid
        columns = 5;
        lines = 4;

        grid_width = kinect.width;
        grid_height = kinect.height;

        grid_width_step = grid_width / columns;
        grid_height_step = grid_height / lines;

        grid = new int[lines * columns];
        grid_last = new int[lines * columns];

	nearThreshold = 255;
	farThreshold = 235;
	bThreshWithOpenCV = true;
	
	ofSetFrameRate(60);
	
	// zero the tilt on startup
	angle = 27;
	kinect.setCameraTiltAngle(angle);
	
	// start from the front
	bDrawPointCloud = false;


        // midi
        midiOut.listPorts();
        // connect
        //midiOut.openPort(0);    // by number
        midiOut.openVirtualPort("ofxMidiOut");                // open a virtual port

        channel = 1;
        note = 0;
        velocity = 64; // 0-127
}

//--------------------------------------------------------------
void testApp::update() {

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

        // find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
        // also, find holes is set to true so we will get interior contours as well....
        //
        // TODO: fenster an haende anpassen
        // TODO: parameter von entfernung abhaengig machen
        // int ofxCvContourFinder::findContours(ofxCvGrayscaleImage &input, int minArea, int maxArea, int nConsidered, bool bFindHoles, bool bUseApproximation=true)
        contourFinder.findContours(grayImage, 800, 6000, 2, false, true);
        // parameter tun auf ca 60 cm
        //



        // reset grid
        memset(grid, 0, sizeof(int) * lines * columns);

        for (int i = 0; i < contourFinder.nBlobs; i++){
            int center_x = int(contourFinder.blobs[i].centroid.x);
            int center_y = int(contourFinder.blobs[i].centroid.y);
            float distance = kinect.getDistanceAt(center_x, center_y);


            int blob_pos_x = int( center_x / grid_width_step);
            int blob_pos_y = int( center_y / grid_height_step);
            //int blob_pos_x = (columns -1) - int( center_x / grid_width_step);
            //int blob_pos_y = int( center_y / grid_height_step);

            int grid_num = (blob_pos_y * columns) + blob_pos_x; 
            velocity = ofMap(distance, 500, 750, 30, 127, true);
            velocity = 127 - velocity;

            grid[grid_num] = velocity; // TODO: velocity
        }

        for(int i = 0; i < lines * columns; i++) {
            // index is the note 
            note = 64 - i;
            // grid array value is the velocity
            velocity = grid[i];

            // note did change
            if(grid_last[i] == 0 && grid[i] > 0) {
                midiOut.sendNoteOn(channel, note, velocity);
                ofLogNotice() << "note ON for " << i << " v: " << velocity << endl;
            } else if (grid_last[i] > 0 && grid[i] > 0) {
                // not yet
            } else if (grid_last[i] > 0 && grid[i] == 0) { 
                midiOut.sendNoteOff(channel, note, 0);
                ofLogNotice() << "note off for " << i << endl;

            }
        }
        memcpy(grid_last, grid, sizeof(int) * lines * columns);
    }

}

//--------------------------------------------------------------
void testApp::draw() {


ofPushMatrix(); // save the old coordinate system
ofTranslate(ofGetWidth(), 0.0f); // move the origin to the bottom-left hand corner of the window
ofScale(-1.0f, 1.0f); // flip the y axis vertically, so that it points upward

    ofSetColor(255, 255, 255);

    //    ofSetColor(255, 0, 0);
    grayImage.draw(10, 10, 640, 480);

    // draw from the live kinect
    //kinect.drawDepth(10, 10, 400, 300);
    //kinect.draw(420, 10, 400, 300);

    //grayImage.mirror(false, true);
    for(int i=1; i < lines; i=i+1) {
        ofLine(10,(i * grid_height_step) + 10, 10 + grid_width, (i * grid_height_step) + 10);
    }
    for(int i=1; i < columns; i=i+1) {
        ofLine((i * grid_width_step) + 10, 10, (i * grid_width_step) + 10, grid_height + 10);
    }

    for(int i = 0; i < lines * columns; i++) {
        if (grid[i] > 0) {
            ofSetColor(0,0,255);
            ofFill();
            ofRect(10 + (i % columns) * grid_width_step, int(i / columns) * grid_height_step + 10, grid_width_step, grid_height_step);
        }

    }
    contourFinder.draw(10, 10, 640, 480);




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

    reportStream << "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
        << "using opencv threshold = " << bThreshWithOpenCV <<" (press spacebar)" << endl
        << "set near threshold " << nearThreshold << " (press: + -)" << endl
        << "set far threshold " << farThreshold << " (press: < >) num blobs found " << contourFinder.nBlobs
        << ", fps: " << ofGetFrameRate() << endl
        << "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl;

    if(kinect.hasCamTiltControl()) {
        reportStream << "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl
            << "press 1-5 & 0 to change the led mode" << endl;
    }


    reportStream << "width: " << grayImage.getWidth() << " height: " << grayImage.getHeight() << " " << endl;
    reportStream << "width_step: " << grid_width_step << " height_step: " << grid_height_step << " " << endl;


    ofDrawBitmapString(reportStream.str(), 20, 652);

ofPopMatrix();
}

//--------------------------------------------------------------
void testApp::exit() {
    //kinect.setCameraTiltAngle(0); // zero the tilt on exit
    midiOut.closePort();
    kinect.close();


}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
    switch (key) {
        case ' ':
            bThreshWithOpenCV = !bThreshWithOpenCV;
            break;

        case'p':
            bDrawPointCloud = !bDrawPointCloud;
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

        case 'w':
            kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
            break;

        case 'o':
            kinect.setCameraTiltAngle(angle); // go back to prev tilt
            kinect.open();
            break;

        case 'c':
            kinect.setCameraTiltAngle(0); // zero the tilt
            kinect.close();
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
void testApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{}
