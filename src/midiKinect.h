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
            int offset;
        };

        struct Mode {
            int notes[20];
            char *name;

            Mode(char *strname, int n1, int n2, int n3, int n4, int n5,         \
                                int n6, int n7, int n8, int n9, int n10,        \
                                int n11, int n12, int n13, int n14, int n15,    \
                                int n16, int n17, int n18, int n19, int n20)
            {
                name = strname;
                notes[19] = n1; notes[18] =  n2; notes[17] = n3;  notes[16] = n4; notes[15] =  n5;
                notes[14] = n6; notes[13] =  n7; notes[12] = n8;  notes[11] = n9; notes[10] = n10;
                notes[9] = n11; notes[8] =  n12; notes[7] = n13;  notes[6] = n14; notes[5] =  n15;
                notes[4] = n16; notes[3] =  n17; notes[2] = n18;  notes[1] = n19; notes[0] =  n20;
            }
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

        void sendNoteOn(Blob *blob);
        void sendNoteOff(Blob *blob);

        void analyzeBlob(Blob *current, Blob *previous, ofxCvBlob cvBlob);
        void triggerMIDI(Blob *current, Blob *previous);
        int posToNote(int pos);

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
        int midiChannel;

        Mode **availModes;
        Mode *currentMode;

        int note_offset;
};
