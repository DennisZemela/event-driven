// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco
 * email:   francesco.rea@iit.it
 * website: www.robotcub.org 
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

/**
 * @file efExtractorThread.h
 * @brief Definition of a thread that receives events and extracts features
 * (see efExtractorModule.h).
 */

#ifndef _EF_EXTRACTOR_THREAD_H_
#define _EF_EXTRACTOR_THREAD_H_

#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <iostream>
#include <iCub/emorph/eventConversion.h>
#include <iCub/emorph/eventBuffer.h>

//within project includes
#include <iCub/cartesianFrameConverter.h>


class efExtractorThread : public yarp::os::RateThread {
private:
    bool idle;                          // flag that exclude code from the execution loop
    int count;                          // loop counter of the thread
    int width, height;                  // dimension of the extended input image (extending)
    int height_orig, width_orig;        // original dimension of the input and output images
    unsigned long lastTimestampLeft;    // last timestamp received for left camera
    yarp::os::BufferedPort<yarp::sig::ImageOf <yarp::sig::PixelMono> > inLeftPort;       // port where the left event image is received
    yarp::os::BufferedPort<yarp::sig::ImageOf <yarp::sig::PixelMono> > inRightPort;      // port where the right event image is received
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > outLeftPort;       // port whre the output edge (left) is sent
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > outRightPort;      // port whre the output edge (right) is sent
    yarp::os::BufferedPort<eventBuffer> outEventPort;                                    // port sending events
    yarp::sig::ImageOf <yarp::sig::PixelMono>* leftInputImage;                           // image input left 
    yarp::sig::ImageOf <yarp::sig::PixelMono>* rightInputImage;                          // image input right 
    yarp::sig::ImageOf <yarp::sig::PixelMono>* leftOutputImage;                           // image output left 
    
    std::string name;                     // rootname of all the ports opened by this thread
    std::string mapURL;                   // mode name and name of the map
    bool resized;                         // flag to check if the variables have been already resized
    int shiftValue;                       // value of the shift between dragonfly (this is vergence related)
    FILE *pFile;                          // file that contains the rules for the LUT
    FILE *fout;                           // file where the extracted LUT is saved
    FILE *fdebug;                         // file for debug
    int* lut;                             // lut that route the event in a different location 
    int monBufSize_b;                     // dimension of the bufferFEA in bytes
    int countEvent;                       // counter of event that are going to be sent
    int countMap;                         // counter of the mapped events

    cFrameConverter* cfConverter;         // cartesian frame converter
    unmask unmask_events;                 // object that unmasks the event
    char* bufferCopy;                     // local copy of the events read
    char* bufferCopy2;                    // second copy where only correct events are saved
    char* flagCopy;                       // copy of the unreadBuffer
    char* resultCopy;                     // buffer resulting out of the selection
    char* buffer;                         // buffer where the events to send are stored
    AER_struct* eventFeaBuffer;           // list of unmasked events
    aer* bufferFEA;                       // buffer storing events as aer struct
public:
    /**
    * default constructor
    */
    efExtractorThread();

    /**
     * destructor
     */
    ~efExtractorThread();

    /**
    * function that initialise the thread
    */
    bool threadInit();

    /**
    * function called when the thread is stopped
    */
    void threadRelease();

    /**
    * function called every time constant defined by rateThread
    */
    void run(); 

    /**
    * function called when the module is poked with an interrupt command
    */
    void interrupt(); 

    /**
    * function that set operating mode
    * @param str name of the mode
    */
    void setMapURL(std::string str) { mapURL = str; };

    /**
    * function that set the rootname for the ports that will be opened
    * @param str rootname as a string
    */
    void setName(std::string str);
    
    /**
    * function that returns the original root name and appends another string iff passed as parameter
    * @param p pointer to the string that has to be added
    * @return rootname 
    */
    std::string getName(const char* p);

    /**
    * function that sets the width and the height of the images based on the dimension of the input image
    * @param width width of the input image
    * @return height height of the input image
    */
    void resize(int width, int height);

    /**
    * shift one image with respect to the other 
    * @param shift number of pixel of shifts
    * @param outImage reference to the output image
    */
    void shift(int shift, yarp::sig::ImageOf<yarp::sig::PixelRgb>& outImage);

};

#endif  //_EF_EXTRACTOR_THREAD_H_
//----- end-of-file --- ( next line intentionally left blank ) ------------------
