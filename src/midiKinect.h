#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxMidi.h"
#include <vector>

class midiKinect : public ofBaseApp {
public:
	struct Blob {
            Bool on;
            int x;
            int y;
            int pos;
            float distance;
            int note;
            int velocity;
            Bool playing;
            int offset;
        };

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
	
        int getPosInGrid(Blob blob);
        void sendNoteOn(Blob blob);
        void sendNoteOff(Blob blob);

	ofxKinect kinect;
	
	ofxCvColorImage colorImg;
	
	ofxCvGrayscaleImage grayImage; // grayscale depth image
	ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
	ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
	
	ofxCvContourFinder contourFinder;
	
	int nearThreshold;
	int farThreshold;
	
	int angle;

        // grid
        int columns;
        int lines;

        int gridWidth;
        int gridHeight;
        int stepX;
        int stepY;
	
        Blob blobA;
        Blob blobPrevA;

        Blob blobB;
        Blob blobPrevB;

        // midi
        ofxMidiOut midiOut;
        int channel;
        int note;
};
