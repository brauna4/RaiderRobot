/*
 * person_follower.h
 *
 *  Created on: Jul 25, 2012
 *      Author: bnair002
 */

#ifndef PERSON_FOLLOWER_H_
#define PERSON_FOLLOWER_H_

#include <iostream>
#include <fstream>
#include <iomanip>
#include "face_detector.h"
#include "body_detector.h"
#include <fcntl.h>    // Used to set up and use nonblocking stdin
#include <termios.h>  // Used to put stdin into raw mode
#include <signal.h>   // Handle fatal signals so the above doesn't hose the shell
#include <sys/time.h> // For ms resolution wall clock

#include "clearpath.h"

#define TURN_THRESHOLD 20
#define SCALE_THRESHOLD 0.4 //percentage
#define PERSON_ID 1

using namespace std;
using namespace clearpath;

class PersonFollower
{
public:
	//declaring the cascades for detections
	FaceDetector* face_detector;
	BodyDetector* body_detector;
	//declaring variables for keyboard control
	enum _special_chars
	{
		BACKSPACE=0x7f,
	    ESCAPE=   0x1b,
	    BRACKET=  0x5b,
	    /* Arrow keys follow a \x1b\x5b escape sequence: */
	    LEFT=     0x44,
	    RIGHT=    0x43,
	    UP=       0x41,
	    DOWN=     0x42,

	};

	enum _input_states
	{
		BASE,          // initial state.  Waiting for 'q', or an escape
	    HAVE_ESCAPE,   // an escape char has been received.  Looking for an opening bracket
	    HAVE_BRACKET   // have the escape and bracket.  Now we're looking for the particular special char
	}              m_inputState;
	int            m_stdinOrigFlags;
	struct termios m_originalTermios;

	double m_fwdVel;
	double m_turnVel;
	double m_accel;

	bool vision_flag; //flag to turn on vision based follower
	int lastSend;

	//defining the contructor/destructor
	PersonFollower();
	~PersonFollower();

	long timeMillis();
	void pollStdin();

	//control the robot from tracking information from image
	void pollVision(Mat& frame);

	//control the robot from the directions in a text file
	void controlUsingTextFile(const char* filename);

	void spin(Mat& frame);
};

#endif /* PERSON_FOLLOWER_H_ */
