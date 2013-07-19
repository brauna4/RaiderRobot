/**
 *      _____
 *     /  _  \
 *    / _/ \  \
 *   / / \_/   \
 *  /  \_/  _   \  ___  _    ___   ___   ____   ____   ___   _____  _   _
 *  \  / \_/ \  / /  _\| |  | __| / _ \ | ++ \ | ++ \ / _ \ |_   _|| | | |
 *   \ \_/ \_/ /  | |  | |  | ++ | |_| || ++ / | ++_/| |_| |  | |  | +-+ |
 *    \  \_/  /   | |_ | |_ | ++ |  _  || |\ \ | |   |  _  |  | |  | +-+ |
 *     \_____/    \___/|___||___||_| |_||_| \_\|_|   |_| |_|  |_|  |_| |_|
 *             ROBOTICS™ 
 *
 *  File: keyboard_ctrl.cpp
 *  Desc: Simple demonstration program allowing a robot to be controlled
 *        with a keyboard.
 *  Auth: Iain Peet
 *
 *  Copyright (c) 2010, Clearpath Robotics, Inc. 
 *  All Rights Reserved
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * Please send comments, questions, or patches to skynet@clearpathrobotics.com 
 *
 */

#include <iostream>
#include <iomanip>
#include <stdio.h>    
#include <stdlib.h>    
#include <fcntl.h>    // Used to set up and use nonblocking stdin
#include <termios.h>  // Used to put stdin into raw mode
#include <signal.h>   // Handle fatal signals so the above doesn't hose the shell
#include <sys/time.h> // For ms resolution wall clock

#include "clearpath.h"
#include "keyboard_ctrl.h"

using namespace std;
using namespace clearpath;

KeyboardController *controller = 0;

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

int main(int argc, char *argv[])
{
    /* Configure the serial port */
    const char* port = (argc == 2) ? argv[1] : "/dev/ttyUSB0";
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
    controller = new KeyboardController();
    atexit(teardown);

    /* Enter main control loop */
    controller->spin();
}

KeyboardController::KeyboardController() :
    m_inputState(BASE),
    m_fwdVel(0.0),
    m_turnVel(0.0),
    m_accel(0.2)
{
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

KeyboardController::~KeyboardController() 
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

/* Main controller loop.  Watches stdin for user input and
 * issues velocity commands to the robot. */
void KeyboardController::spin()
{
    cout.precision(3);
    int lastSend = timeMillis();
    while(1) {
        // Change state according to keyboard input:
        pollStdin();

        /* Update state printout */
        cout << "\rForward Vel: " << fixed << setw(6) << m_fwdVel << "[m/s]  ";
        cout << "Turn Vel: " << fixed << setw(6) << m_turnVel << "[rad/s] ";
        cout << flush; 

        /* Transmit current velocity at 10Hz */
        if( timeMillis() - lastSend > 100 ) {
            SetVelocity(m_fwdVel, m_turnVel, m_accel).send();
        }

        // Don't loop too tightly
        usleep(1000);
    }
}

/* Fetch any data available from stdin, and handle it */
void KeyboardController::pollStdin() 
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
                    exit(0);
                } else if( curCh == BACKSPACE ) {
                    // Stop immediately
                    SetVelocity(0, 0, 2.0).send();
                    m_fwdVel = 0.0;
                    m_turnVel = 0.0;
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

/* Get wall millis since program start */
long KeyboardController::timeMillis()
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



