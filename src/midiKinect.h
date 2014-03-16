#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxMidi.h"
#include <vector>

// uncomment this to read from two kinects simultaneously
//#define USE_TWO_KINECTS

class midiKinect : public ofBaseApp {
public:
	
	void setup();
	void update();
	void draw();
	void exit();
	
	void drawPointCloud();

	void keyPressed(int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	
	ofxKinect kinect;
	
	ofxCvColorImage colorImg;
	
	ofxCvGrayscaleImage grayImage; // grayscale depth image
	ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
	ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
	
	ofxCvContourFinder contourFinder;
	
	bool bThreshWithOpenCV;
	bool bDrawPointCloud;
	
	int nearThreshold;
	int farThreshold;
	
	int angle;

        // grid
        int columns;
        int lines;
        int grid_width;
        int grid_height;
        int grid_width_step;
        int grid_height_step;
	
	// used for viewing the point cloud
	ofEasyCam easyCam;

        int *grid;
        int *grid_last;

        // midi
        ofxMidiOut midiOut;
        int channel;
        int note;
        int velocity;
};
