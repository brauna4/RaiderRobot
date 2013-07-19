/*
 * person_follower.cpp
 *
 *  Created on: Jul 25, 2012
 *      Author: bnair002
 */

#include "person_follower.h"
#include <limits.h>
#include <time.h>
#include <sys/time.h>
#include <cstdlib>
#include <algorithm>
#include <opencv/cv.h>
#include <opencv/highgui.h>

PersonFollower *controller = 0;
#define SOURCE_WIN "Input Stream"
#define FRAMES_PATH "/home/bnair002/Frames/"

void teardown()
{
    if( controller ) delete controller;
}

/* Used to turn fatal signals into normal exits, which ensures that
 * KeyboardController gets destroyed properly */
void termSignalHandler(int sig)
{
    sig = 0;  // Go away, warning.  Param is required by spec.
    exit(0);
}

inline string find_FPS()
{
	static struct timespec cur, prev;
	static bool first_run = true;
	stringstream ss_fps;
	string str_fps;

	if(first_run)
	{
		clock_gettime(CLOCK_REALTIME, &cur);					// Store current time for FPS calculaton
		prev = cur;
		first_run = false;
	}

	ss_fps.str("");
	clock_gettime(CLOCK_REALTIME, &cur);
	ss_fps << fixed << setprecision(2) << 1/((1e9*(cur.tv_sec - prev.tv_sec) + (cur.tv_nsec - prev.tv_nsec))/ 1e9) << " FPS";
	prev = cur;
	return ss_fps.str();
}

inline string get_timestamp()
{
	struct tm *local_time;
	time_t cur_time;
	char time_stamp[255] = "";

	cur_time = time(NULL);
	local_time = localtime(&cur_time);
	strftime(time_stamp, sizeof(time_stamp), "LPR %H:%M:%S %m-%d-%y", local_time);

	return string(time_stamp);
}


int main(int argc, char *argv[])
{
	VideoWriter writer;

	/* Parameters for vision processing */
	CvFont font;
	namedWindow("Input");
	cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX,.4,.4,0.0,1,CV_AA);
	char keypress;
	VideoCapture capture;
	Mat frame;

    /* Configure the serial port */
    const char* port = (argc == 2) ? argv[1] : "/dev/tty8100";
    clearpath::Transport::instance().configure(port, 3 /* max retries*/);

    /* Print a nice welcome & some robot info */
    cout << "\n=== Clearpath Robot Keyboard Controller ===" << endl;
    DataPlatformName *robotName = DataPlatformName::getUpdate();
    cout << "Robot    : " << robotName->getName() << endl;
    DataPlatformInfo *robotInfo = DataPlatformInfo::getUpdate();
    cout << "Model    : " << robotInfo->getModel() << endl;
    DataMaxSpeed *maxVel = DataMaxSpeed::getUpdate();
    cout << "Max Speed: +" << maxVel->getForwardMax() << "/-"
         << maxVel->getReverseMax() << " [m/s]" << endl;
    cout << "\n=== Controls ===" << endl;
    cout << "up       : increase forward speed" << endl;
    cout << "down     : decrease forward speed" << endl;
    cout << "left     : increase CCW turn rate" << endl;
    cout << "right    : decrease CCW turn rate"  << endl;
    cout << "backspace: STOP" << endl;
    cout << "q,^C     : Stop robot and exit." << endl;

    cout << "\n=== Status ===" << endl;

    /* Ensure we exit cleanly on fatal signals. */
    signal(SIGINT, termSignalHandler);
    signal(SIGTERM, termSignalHandler);

    /* NB: It is very important that controller is actually destroyed before
     * program exit, since it does some voodoo with stdin that really should
     * be cleaned up before handing control back to bash */
    controller = new PersonFollower();
    controller->lastSend = controller->timeMillis();

    // establishing the IP Camera feed
    //const string videoStreamAddress = "http://192.168.2.8/axis-cgi/mjpg/video.cgi";
    const string videoStreamAddress = "http://192.168.2.10/mjpeg?res=half&quality=15";
    if(!capture.open(videoStreamAddress))
    {
    	cout << "Error opening video stream or file" <<endl;
    	return -1;
    }

	namedWindow(SOURCE_WIN);
	int count = 0;
	char outimg_name[400];
	//writer.open("Raider_Tracker_Demo.avi",CV_FOURCC('P','I','M','1'),25,Size(capture.get(CV_CAP_PROP_FRAME_HEIGHT),capture.get(CV_CAP_PROP_FRAME_WIDTH)),1);
	while(true)
	{
		capture >> frame;
		if(frame.empty())
		{
			cout << " Frame not present " <<endl;
			continue;
		}

		
		controller->spin(frame);
		putText(frame,find_FPS(),Point(5, frame.rows-10),FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,250));
		//putText(frame,get_timestamp(),Point(100, frame.rows-10),FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,250));

		imshow(SOURCE_WIN,frame);


		sprintf(outimg_name,"%sframe-%d.jpg",FRAMES_PATH,count);
		//imwrite(outimg_name,frame);
		//writer << frame;

		char c = cvWaitKey(8);
		if(c == 27)
			break;
		count++;
	}

    atexit(teardown);
}

//defining the class methods
PersonFollower::PersonFollower()
{
	body_detector = new BodyDetector(true);
	face_detector = new FaceDetector();

	m_inputState = BASE;
	m_fwdVel = 0.0;
	m_turnVel = 0.0;
	m_accel = 0.2;

	vision_flag = false;

    if(!isatty(0)) {
        cerr << "Stdin must be a tty." << endl;
        exit(1);
    }

    /* Record original state of stdin, so we can restore
     * NB: We DO NOT want to run the destructor before we successfuly
     * record stdin state, hence the abort. */
    m_stdinOrigFlags = fcntl(0, F_GETFL);
    if(tcgetattr(0, &m_originalTermios)) {
        perror("tcgetattr");
        abort();
    }

    /* Put stdin into non-block mode */
    if( fcntl(0, F_SETFL, m_stdinOrigFlags | O_NONBLOCK ) ) {
        perror("fcntl");
        exit(1);
    }

    /* Disable stdin echoing and linebuffering */
    struct termios rawTermios = m_originalTermios;
    rawTermios.c_lflag &= ~(ECHO | ECHONL | ICANON);
    if(tcsetattr(0, TCSANOW, &rawTermios)) {
        perror("tcsetattr");
        exit(1);
    }

}

PersonFollower::~PersonFollower()
{
    SetVelocity(0, 0, 1.0).send();

    if( fcntl(0, F_SETFL, m_stdinOrigFlags) ) {
        perror("fcntl");
    }

    if(tcsetattr(0, TCSANOW, &m_originalTermios)) {
        perror("tcsetattr");
    }

    cout << "\nExited" << endl;
}

//methods
void PersonFollower::spin(Mat& A)
{
    cout.precision(3);

        // Change state according to keyboard input:
        pollStdin();
        if(!A.empty() && vision_flag)
        	pollVision(A);

        /* Update state printout */
        cout << "\rForward Vel: " << fixed << setw(6) << m_fwdVel << "[m/s]  ";
        cout << "Turn Vel: " << fixed << setw(6) << m_turnVel << "[rad/s] ";
        cout << flush;

        /* Transmit current velocity at 10Hz */
        if( timeMillis() - lastSend > 100 )
        {
            SetVelocity(m_fwdVel, m_turnVel, m_accel).send();
            lastSend = timeMillis();
        }

        // Don't loop too tightly
        usleep(1000);

}

void PersonFollower::pollVision(Mat& A)
{
/*	//DETECT FACES
	//start detecting the face regions
	vector<FaceProfile> faces = face_detector->detectFaces(A);

	//for the time being track the first rectangle region
	for(int k = 0 ; k < faces.size(); k++)
	{
		FaceProfile face_p = faces[k];
		Rect face = face_p.face_image;

		//draw the face image on frame
		rectangle(A,face,Scalar(0,37,204),1.5);

		//identify the location of face in image
		int pos_x = face.x + face.width/2;
		int pos_y = face.y + face.height/2;

		//get the reference point in the image
		int ref_x = A.cols/2;
		int ref_y = A.rows/2;

		//set the turn velocity
		int diff_x = abs(pos_x - ref_x);
		if( diff_x > TURN_THRESHOLD )
		{
			//need to turn left or right
			if(ref_x > pos_x)
			{			
				m_turnVel = m_turnVel = (m_turnVel>= 0.6)?0.6:m_turnVel+0.05; // TURN LEFT
				m_fwdVel = 0.1;
			}
			else
			{
				m_turnVel = (m_turnVel<= -0.6)?-0.6:m_turnVel-0.05; //TURN RIGHT
				m_fwdVel = 0.1;	
			}
		}
		else
			m_turnVel = 0.0;


		//setting the forward direction
		if( (face.width < 0.15 * A.rows) && (face.height < 0.15 * A.cols) ) //move forward
			m_fwdVel = +0.2;
		else
			m_fwdVel = 0.0;
	}
*/
//	//DETECT PEOPLE

	//vector<Rect> bodies = body_detector->DetectAndDrawHumanBody(A);
	vector<PersonProfile> bodies = body_detector->DetectAndDrawHumanBody(A);
	
	if(!body_detector->detected) //check if there were any detections in the current frame
	{
		//SetVelocity(0, 0, 2.0).send();
		m_turnVel = 0.0;
		m_fwdVel = 0.0;
		return;
	}
	
	//for the time being track the first rectangle region
	for(int k = 0 ; k < bodies.size(); k++)
	{
		PersonProfile pf = bodies[k];
		
		//check if that individual is detected
		if(!pf.found)
			continue;
		Rect body_region = pf.location;
		
		//get the ID of the person
		int iD = pf.id;
		
		//check if the ID belongs to the person to be followed
		if(iD != PERSON_ID)
			continue;

		//identify the location of face in image
		int pos_x = body_region.x + body_region.width/2;
		int pos_y = body_region.y + body_region.height/2;

		//get the reference point in the image
		int ref_x = A.cols/2;
		int ref_y = A.rows/2;

		//set the turn velocity
		int diff_x = abs(pos_x - ref_x);
		if( diff_x > TURN_THRESHOLD )
		{
			//need to turn left or right
			if(ref_x > pos_x)
				//m_turnVel = (m_turnVel>= 0.4)?0.4:m_turnVel+0.05; // TURN LEFT
				m_turnVel = 0.2;
			else
				//m_turnVel = (m_turnVel<= -0.4)?-0.4:m_turnVel-0.05; //TURN RIGHT
				m_turnVel = -0.2;
		}
		else
			m_turnVel = 0.0;


		//setting the forward velocity
		if( (body_region.width < 0.5 * A.rows) && (body_region.height < 0.5 * A.cols) ) //move forward
			m_fwdVel = (m_fwdVel>=0.75)?0.75:(m_fwdVel + 0.05);
		else
			m_fwdVel = (m_fwdVel==0.0)?0.0:(m_fwdVel - 0.05);
			
	}

	//if((bodies.size() == 0) && (faces.size() == 0))




 }

void PersonFollower::controlUsingTextFile(const char* filename)
{
	cout.precision(3);
	ifstream inFile;
	int lastSend = timeMillis();
	inFile.open(filename);
	if(!inFile.is_open())
	{
		cerr << " No File present" <<endl;
		exit(1);
	}
	while(!inFile.eof())
	{
		inFile >> m_fwdVel >> m_turnVel >> m_accel;
		/* Update state printout */
		cout << "\rForward Vel: " << fixed << setw(6) << m_fwdVel << "[m/s]  ";
		cout << "Turn Vel: " << fixed << setw(6) << m_turnVel << "[rad/s] ";
		cout << flush;

        /* Transmit current velocity at 10Hz */
        if( timeMillis() - lastSend > 100 )
        {
        	cout << "I am here" <<endl;
            SetVelocity(m_fwdVel, m_turnVel, m_accel).send();
            lastSend = timeMillis();
        }

        // Don't loop too tightly
        usleep(300000); // 0.3 secs delay between the states for smooth movement
	}
}

void PersonFollower::pollStdin()
{
    unsigned char curCh;

    /* State machine which accepts characters from stdin */
    while( read(0, &curCh, 1) == 1 ) {
        switch( m_inputState ) {
            /* Looking either for one of the single-byte character commands
             * ('q'=exit or 'bkspc'=stop), or for the beginning of an arrow
             * key escape sequence */
            case BASE:
                if( curCh == ESCAPE ) {
                    m_inputState = HAVE_ESCAPE;
                } else if( curCh == 'q' ) {
                    exit(0); //need to come back to the main loop from here . Another way of exiting this function and exiting the program
                } else if( curCh == BACKSPACE ) {
                    // Stop immediately
                    SetVelocity(0, 0, 2.0).send();
                    m_fwdVel = 0.0;
                    m_turnVel = 0.0;
                    vision_flag = false; //stop using vision
                } else if(curCh == 'K') {
                	if(vision_flag == false)
                		vision_flag = true;
                	else
                		vision_flag = false;
                } else if(curCh == 'C') {
                	vision_flag = false;
                	body_detector->ClearUniqueDetections();
		}
                break;

            /* The last char we received was escape, which is (probably) the
             * beginning of an arrow key escape.  The next char in the sequence
             * should be a bracket */
            case HAVE_ESCAPE:
                if( curCh == BRACKET ) {
                    m_inputState = HAVE_BRACKET;
                } else {
                    m_inputState = BASE;
                }
                break;

            /* We've received the <esc><bracket> chars in the arrow key escape
             * sequence.  This char should identify which arrow key was pressed */
            case HAVE_BRACKET:
                if( curCh == LEFT ) {
                    m_turnVel += 0.1;
                } else if( curCh == RIGHT ) {
                    m_turnVel -= 0.1;
                } else if( curCh == UP ) {
                    m_fwdVel += 0.1;
                } else if( curCh == DOWN ) {
                    m_fwdVel -= 0.1;
                }
                m_inputState = BASE;
                break;

            default:
                cerr << "Invalid state!" << endl;
                exit(1);
        } // switch( m_inputState )
    } // while( read char successfully )
}

long PersonFollower::timeMillis()
{
    static bool firstRun = true;
    static struct timeval startTime;
    struct timeval tv;
    struct timezone tz;

    if(firstRun) {
        gettimeofday(&startTime, &tz);
        firstRun = false;
    }

    gettimeofday(&tv, &tz);

    return (tv.tv_sec - startTime.tv_sec)*1000
           + (tv.tv_usec - startTime.tv_usec)/1000;
}
