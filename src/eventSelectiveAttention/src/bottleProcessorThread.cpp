// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea
  * email: francesco.rea@iit.it
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
 * @file bottleProcessorThread.cpp
 * @brief Implementation of the thread (see header bottleProcessorThread.h)
 */

#include <iCub/bottleProcessorThread.h>
#include <cstring>
#include <cassert>
#include <cstdlib>
#include <cmath>
#include <time.h>

using namespace emorph::ecodec;

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define INTERVFACTOR 1
#define COUNTERRATIO 1 //1.25       //1.25 is the ratio 0.160/0.128
#define MAXVALUE 0xFFFFFF //4294967295
#define THRATE 5
#define STAMPINFRAME  // 10 ms of period times the us in 1 millisecond + time for computing
//#define retinalSize 128
#define CHUNKSIZE 32768 //65536 //8192
#define dim_window 10
#define synch_time 1

#define VERBOSE
#define INCR_RESPONSE 50
#define DECR_RESPONSE 50

bottleProcessorThread::bottleProcessorThread() : RateThread(THRATE) {
    responseGradient = 127;
    retinalSize      = 128;  //default value before setting 
    saliencySize     = 128;  //default dimension of the saliency map
    lasttimestamp    = 0;
    bottleHandler = true;
    synchronised = false;
    greaterHalf  = false;
    firstRun     = true;
    count        = 0;
    minCount     = 0; //initialisation of the timestamp limits of the first frame
    idle = false;
    bufferCopy = (char*) malloc(CHUNKSIZE);
    countStop = 0;
    verb = false;
    string i_fileName("bottleProcessorThread.eventBottle.txt");
    raw = fopen(i_fileName.c_str(), "wb");
    lc = rc = 0;	
    minCount      = 0;
    minCountRight = 0;
}

bottleProcessorThread::~bottleProcessorThread() {
    printf("freeing memory in collector");
    delete bufferCopy;
}

bool bottleProcessorThread::threadInit() {
    printf(" \nstarting the threads.... \n");
    //outPort.open(getName("/left:o").c_str());
    //outPortRight.open(getName("/right:o").c_str());

    fout = fopen("./eventSelectiveAttention.dump.txt","w+");

    resize(retinalSize, retinalSize);

    printf("starting the first collector!!!.... \n");    
    cfConverter = new eventCartesianCollector();
    cfConverter->useCallback();
    cfConverter->setRetinalSize(retinalSize);
    cfConverter->open(getName("/retina:i").c_str());
    
    printf("\n opening retina\n");
    printf("starting the plotter \n");
    
    //pThread = new plotterThread();
    //pThread->setName(getName("").c_str());
    //pThread->setStereo(stereo);
    //pThread->setRetinalSize(retinalSize);
    //pThread->start();

    ebHandler = new eventBottleHandler();
    ebHandler->useCallback();
    ebHandler->setRetinalSize(128);
    ebHandler->open(getName("/retinaBottle:i").c_str());

    //map1Handler = new eventBottleHandler();
    //map1Handler->useCallback();
    //map1Handler->setRetinalSize(32);
    //map1Handler->open(getName("/map1Bottle:i").c_str());

    receivedBottle = new Bottle();
    
    unmask_events = new unmask(32);
    //unmask_events->setRetinalSize(32);
    
    //unmask_events->setResponseGradient(responseGradient);
    //unmask_events->setASVMode(asvFlag);
    //unmask_events->setDVSMode(dvsFlag);
    //unmask_events->start();

    //minCount = cfConverter->getEldestTimeStamp();
    startTimer = Time::now();
    //clock(); //startTime ;
    //T1 = times(&start_time);
    //microseconds = 0;
    //microsecondsPrev = 0;
    gettimeofday(&tvend, NULL);

    count = 0;
    microsecondsPrev = 0;
    minCount = 0;
    minCountRight= 0;
    
    //saliencyMapLeft  = (double*) malloc(retinalSize * retinalSize * sizeof(double));
    //memset(saliencyMapLeft,0, retinalSize * retinalSize * sizeof(double));
    //saliencyMapRight = (double*) malloc(retinalSize * retinalSize * sizeof(double));
    //memset(saliencyMapRight,0, retinalSize * retinalSize * sizeof(double));
    
    featureMapLeft = (double*) malloc(retinalSize * retinalSize * sizeof(double));
    featureMapRight = (double*) malloc(retinalSize * retinalSize * sizeof(double));
    memset(featureMapLeft ,0, retinalSize * retinalSize * sizeof(double));
    memset(featureMapRight,0, retinalSize * retinalSize * sizeof(double));
    
    //timestampMap = (unsigned long*) malloc(retinalSize * retinalSize * sizeof(unsigned long) );
    //memset(timestampMap,0, retinalSize * retinalSize * sizeof(unsigned long));
    //timestampMapLeft = (unsigned long*) malloc(retinalSize * retinalSize * sizeof(unsigned long) );
    //memset(timestampMapLeft,0, retinalSize * retinalSize * sizeof(unsigned long));
    //timestampMapRight = (unsigned long*) malloc(retinalSize * retinalSize * sizeof(unsigned long) );
    //memset(timestampMapRight,0, retinalSize * retinalSize * sizeof(unsigned long));

    unmaskedEvents = (AER_struct*) malloc (CHUNKSIZE * sizeof(int));
    memset(unmaskedEvents, 0, CHUNKSIZE * sizeof(int));

    rxQueue = new eEventQueue();

    printf("Initialisation in selector thread correctly ended \n");
    return true;
}

void bottleProcessorThread::interrupt() {
    outPort.interrupt();
    outPortRight.interrupt();
}

void bottleProcessorThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string bottleProcessorThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void bottleProcessorThread::resize(int widthp, int heightp) {
    imageLeft = new ImageOf<PixelMono>;
    imageLeft->resize(retinalSize,retinalSize);
    imageRight = new ImageOf<PixelMono>;
    imageRight->resize(retinalSize,retinalSize);
    //imageRight->resize(widthp,heightp);
}

/*
void bottleProcessorThread::getMonoImage(ImageOf<yarp::sig::PixelMono>* image, unsigned long minCount,unsigned long maxCount, bool camera){
    assert(image!=0);
    //printf("retinalSize in getMonoImage %d \n", retinalSize);
    image->resize(retinalSize,retinalSize);
    unsigned char* pImage = image->getRawImage();
    int imagePadding = image->getPadding();
    int imageRowSize = image->getRowSize();
    
    
    // unsigned long int lasttimestamp = getLastTimeStamp();
    // if (lasttimestamp == previousTimeStamp) {   //condition where there were not event between this call and the previous
    //     for(int r = 0 ; r < retinalSize ; r++){
    //         for(int c = 0 ; c < retinalSize ; c++) {
    //             *pImage++ = (unsigned char) 127;
    //         }
    //         pImage+=imagePadding;
    //     }
    //     return;
    // }
    // previousTimeStamp = lasttimestamp;
   

    // determining whether the camera is left or right
    //int* pBuffer = unmask_events->getEventBuffer(camera);
    //unsigned long* pTime   = unmask_events->getTimeBuffer(camera);
    double* pBuffer        = featureMap;
    unsigned long* pTime   = timestampMap;
    
    //printf("timestamp: min %d    max %d  \n", minCount, maxCount);
    //pBuffer += retinalSize * retinalSize - 1;
    double maxLeft = -100, maxRight = -100;
    double minLeft = 100, minRight = 100;
    double maxResponseLeft = 0, maxResponseRight = 0;
    int maxLeftR, maxLeftC, maxRightR, maxRightC;
    for(int r = 0 ; r < retinalSize ; r++){
        for(int c = 0 ; c < retinalSize ; c++) {
            //drawing the retina and the rest of the image separately
            int value = *pBuffer;

            double left_double  = saliencyMapLeft [r * retinalSize + c];
            double right_double = saliencyMapRight[r * retinalSize + c];
            if(left_double < minLeft)   minLeft = left_double;
            if(left_double > maxLeft)   maxLeft = left_double;
            if(right_double > maxRight) maxRight = right_double;
            if(right_double < minRight) minRight = right_double;
            if(maxResponseLeft < abs(minLeft)) {
                maxResponseLeft = abs(minLeft);
                maxLeftR = r; maxLeftC = c;
            }
            if(maxResponseLeft < abs(maxLeft)) {
                maxResponseLeft = abs(maxLeft);
                maxLeftR = r; maxLeftC = c;
            }
            if(maxResponseRight < abs(minRight)) {
                maxResponseRight = abs(minRight);
                maxRightR = r; maxRightC = c;
            }
            if(maxResponseRight < abs(maxRight)) {
                maxResponseRight = abs(maxRight);
                maxRightR = r; maxRightC = c;
            }
                        
            unsigned long timestampactual = *pTime;
            //bool tristateView = false;
            
            if(tristate) {
                // decreasing the spatial response without removing it
                if(count % 10 == 0) {
                    if(*pBuffer > 20){
                        *pBuffer = *pBuffer - 1;                                       
                    }
                    else if(*pBuffer < -20){
                        *pBuffer = *pBuffer + 1;                                       
                    }
                }

                //if(minCount>0 && maxCount > 0 && timestampactual>0)
                //printf("actualTS%ld val%ld max%ld min%ld  are\n",timestampactual,timestampactual * COUNTERRATIO,minCount,maxCount);
                if (((timestampactual * COUNTERRATIO) > minCount)&&((timestampactual * COUNTERRATIO) < maxCount)) {   //(timestampactual != lasttimestamp)
                    *pImage = (unsigned char) (127 + value);
                    //if(value>0)printf("event%d val%d buf%d\n",*pImage,value,*pBuffer);
                    pImage++;
                    //if ((stereo) && (r < 7) && (r >= 16) && (c < 7) && (c >= 16)) {
                    //  *pImage = (unsigned char) (127 + value);
                    //  pImage += imageRowSize;
                    //  *pImage = (unsigned char) (127 + value);
                    //pImage--;
                    //  *pImage = (unsigned char) (127 + value);
                    //  pImage -= (imageRowSize + 1);
                    //}
                    
                }
                else {
                    *pImage = (unsigned char) 127 ;
                    //printf("NOT event%d \n",*pImage);
                    pImage++;
                    
                    //if ((stereo) && (r < 7) && (r >= 16) && (c < 7) && (c >= 16)) {
                    //  *pImage = (unsigned char) 127;
                    //  pImage += imageRowSize;
                    //  *pImage = (unsigned char) 127 + value;
                    //  pImage--;
                    //  *pImage = (unsigned char) 127 + value;
                    //  pImage -= (imageRowSize + 1);
                    //}
                    
                }
                pBuffer++;
                pTime++;
            }
            // branch !tristateView
            else {
                // decreasing the spatial response without removing 
                if((*pBuffer > 20) && (count % 10 == 0)){
                    *pBuffer = *pBuffer - 1;                                       
                }
                //if(minCount>0 && maxCount > 0 && timestampactual>0)
                //printf("actualTS%ld val%ld max%ld min%ld  are\n",timestampactual,timestampactual * COUNTERRATIO,minCount,maxCount);
                if (((timestampactual * COUNTERRATIO) > minCount)&&((timestampactual * COUNTERRATIO) < maxCount)) {   //(timestampactual != lasttimestamp)
                    *pImage = (unsigned char) abs(value * 2);
                    //if(value>0)printf("event%d val%d buf%d\n",*pImage,value,*pBuffer);
                    pImage++;
                    //if ((stereo) && (r < 7) && (r >= 16) && (c < 7) && (c >= 16)) {
                    //  *pImage = (unsigned char) (127 + value);
                    //  pImage += imageRowSize;
                    //  *pImage = (unsigned char) (127 + value);
                    //pImage--;
                    //  *pImage = (unsigned char) (127 + value);
                    //  pImage -= (imageRowSize + 1);
                    //}
                    
                }
                else {
                    *pImage = (unsigned char) 0 ;
                    //printf("NOT event%d \n",*pImage);
                    pImage++;
                    
                    //if ((stereo) && (r < 7) && (r >= 16) && (c < 7) && (c >= 16)) {
                    //  *pImage = (unsigned char) 127;
                    //  pImage += imageRowSize;
                    //  *pImage = (unsigned char) 127 + value;
                    //  pImage--;
                    //  *pImage = (unsigned char) 127 + value;
                    //  pImage -= (imageRowSize + 1);
                    //}
                    
                }
                pBuffer++;
                pTime++;
            } // end !tristate            
        } // end inner loop
        pImage+=imagePadding;
    }//end outer loop
    //printf("end of the function get in mono \n");
    //unmask_events->setLastTimestamp(0);

    // cycle after normalisation
    //printf("cycle after the normalisation LEFT:(%f, %f)  RIGHT:(%f,%f) \n", minLeft, maxLeft, minRight, maxRight);
    double* pSalLeft      = saliencyMapLeft;
    double* pSalRight     = saliencyMapRight;
    unsigned long* pTimeLeft     = timestampMapLeft;
    unsigned long* pTimeRight    = timestampMapRight;
    double rangeLeft      = abs(maxLeft  - minLeft);
    double rangeRight     = abs(maxRight - minRight);
    unsigned char* pLeft  = imageLeft->getRawImage();
    unsigned char* pRight = imageRight->getRawImage();
    int padding           = imageLeft->getPadding();
    unsigned long timestampactual ;
    for(int r = 0 ; r < retinalSize ; r++){
        for(int c = 0 ; c < retinalSize ; c++) {
            timestampactual = *pTimeLeft; 
            if (((timestampactual * COUNTERRATIO) > minCount)&&((timestampactual * COUNTERRATIO) < maxCount)) { 
                if((r == maxLeftR) && (c == maxLeftC)) {
                    *pLeft = 255; // maximum response
                }
                else {
                    double d = ((abs(*pSalLeft  - minLeft) )  / rangeLeft ) * 180;
                    //printf(" d=%f %08X < %08X < %08X\n", d, minCount, timestampactual, maxCount);
                    *pLeft  = (unsigned char) d;
                }
            }
            else {
                *pLeft = 0;
            }
            pLeft++;
            pSalLeft++;
            pTimeLeft++;

            // ------------------------------------------------------------------
            timestampactual = *pTimeRight;
            if (((timestampactual * COUNTERRATIO) > minCount)&&((timestampactual * COUNTERRATIO) < maxCount)) { 
                if((r == maxRightR) && (c == maxRightC)) {
                    *pRight = 255;
                }
                else {
                    double d = ((abs(*pSalRight - minRight) )/ rangeRight) * 180;
                    *pRight = (unsigned char) d;
                }
            }
            else {
                *pRight = 0;
            }
            pRight++;
            pSalRight++;
            pTimeRight++;
        }
        pLeft += padding;
        pRight += padding;
    }
}
*/



void bottleProcessorThread::forgettingMemory() {
    double* pLeft = saliencyMapLeft;
    double* pRight = saliencyMapRight;
    for(int r = 0 ; r < retinalSize ; r++){
        for(int c = 0 ; c < retinalSize ; c++) {            
            if(*pLeft > 0) {
                *pLeft -= DECR_RESPONSE;               
            }
            else {
                *pLeft += DECR_RESPONSE;
            }
            pLeft++;
            if(*pRight > 0) {
                
                *pRight -= DECR_RESPONSE;
            }
            else {
                *pRight += DECR_RESPONSE;
            }
            pRight++;
        }
    }
}


void bottleProcessorThread::spatialSelection(eEventQueue *q) {
    int dequeSize = q->size();
    //printf("bottleProcessorThread::spatialSelection: %d events inQueue \n", dequeSize);
    unsigned long ts;
    int countLeftPos  = 0 , countLeftNeg  = 0;
    int countRightPos = 0 , countRightNeg = 0;
    int scaleFactor   = saliencySize / retinalSize; 
    for (int evt = 0; evt < dequeSize; evt++) {
        //printf("analyzing event %d \n", evt);
        if((*q)[evt] != 0) {                    
            //********** extracting the event information **********************
            // to identify the type of the packet
            // user can rely on the getType() method
            // -------------------------- AE  ----------------------------------
            if ((*q)[evt]->getType()=="AE") {
                // identified an  address event
                AddressEvent* ptr=dynamic_cast<AddressEvent*>((*q)[evt]);
                if(ptr->isValid()) { 
                    //ts  = iterEvent->ts;
                    //pol = iterEvent->pol;
                    //cam = iterEvent->cam;
                    int cartX     = retinalSize - ptr->getX();
                    int cartY     = ptr->getY();
                    int camera    = ptr->getChannel();
                    int polarity  = ptr->getPolarity();
                    
                    int xpos      = cartX * scaleFactor;
                    int ypos      = cartY * scaleFactor;

#ifdef VERBOSE
                    //printf(" address event %s \n",ptr->getContent().toString().c_str());
                    //printf("scale factor %d \n", scaleFactor);
                    fprintf(fout,"ae : %08X \n",ptr->encode().get(0).asInt());
                    //fprintf(fout,"%s \n",ptr->getContent().toString().c_str());
                    fprintf(fout,"cartx %d  carty %d \n", cartX, cartY);
#endif
                    
                    xpos = 10;
                    ypos = 10;
                    camera = 0;
                    polarity = 1;

                    if(camera == 0) {
                        
                        //printf("saliencyMapLeft in %d %d changed (pol:%d)  \n", cartX, cartY, polarity);
                        if(polarity == 0) {
                            for ( int xi = 0; xi < scaleFactor; xi++) {
                                for (int yi = 0; yi < scaleFactor; yi++) {
                                    //saliencyMapLeft [(xpos * scaleFactor + xi) * 3   + (ypos + yi) * saliencySize * 3] +=  INCR_RESPONSE;
                                    featureMapLeft [(xpos * scaleFactor + xi)    + (ypos + yi) * saliencySize ] ==  100.0;
                                    countLeftPos++;
                                }
                            }
                        }
                        else {
                            for ( int xi = 0; xi < scaleFactor; xi++) {
                                for (int yi = 0; yi < scaleFactor; yi++) {
                                    //saliencyMapLeft [(xpos * scaleFactor + xi) * 3   + (ypos + yi) * saliencySize * 3 ] -=  INCR_RESPONSE;
                                    featureMapLeft [(xpos * scaleFactor + xi)    + (ypos + yi) * saliencySize  ] = 100;
                                    countLeftNeg++;
                                }
                            }
                        }
                        //---------------
                        for ( int xi = 0; xi < scaleFactor; xi++) {
                            for (int yi = 0; yi < scaleFactor; yi++) {
                                timestampMapLeft[ypos * saliencySize + xpos] = ts;
                            }
                        }
                        
                    }
                    else {
                        //printf("saliencyMapRight in %d %d changed (pol:%d) \n", cartX, cartY, polarity);
                        
                        if(polarity == 0) {
                            for ( int xi = 0; xi < scaleFactor; xi++) {
                                for (int yi = 0; yi < scaleFactor; yi++) {
                                    featureMapRight [xpos * saliencySize + ypos] += INCR_RESPONSE; 
                                    countRightPos++;
                                }
                            }
                        }
                        else {
                            for ( int xi = 0; xi < scaleFactor; xi++) {
                                for (int yi = 0; yi < scaleFactor; yi++) {
                                    featureMapRight [xpos * saliencySize + ypos] -= INCR_RESPONSE; 
                                    countRightNeg++;
                                }
                            }
                        }
                        //----------
                        for ( int xi = 0; xi < scaleFactor; xi++) {
                            for (int yi = 0; yi < scaleFactor; yi++) {
                                timestampMapRight[(xpos + xi) * saliencySize + (ypos + yi)] = ts;
                            }
                        }
                    }                                     
                } //end if (ptr->isValid()) 
            } //end if ((*q)[evt]->getType()=="AE")
            // -------------------------- TS ----------------------------------
            else if((*q)[evt]->getType()=="TS") {
                
                TimeStamp* ptr=dynamic_cast<TimeStamp*>((*q)[evt]);

#ifdef VERBOSE
                fprintf(fout,"ts : %08X  \n",ptr->encode().get(0).asInt());
#endif
                
                //identified an time stamp event
                ts = (unsigned int) ptr->getStamp();
                //printf("timestamp %lu %08X \n", ts, lasttimestamp);
                mutexLastTimestamp.wait();
                *lasttimestamp = ts;
                mutexLastTimestamp.post();
               
                //forgettingMemory();
                //if(VERBOSE) {
                //    fprintf(fdebug, " %08x  \n", (unsigned int) ts);
                //}                
                //countEventToSend++;
            }
            // -------------------------- NULL ----------------------------------
            else {
                printf("not recognized \n");
                return;
            } 
        } //end if((*q)[evt] != 0)
    } //end for


#ifdef VERBOSE
                fprintf(fout,"-------------------------------- \n");
#endif
                //printf("number for left %d %d \n", countLeftPos, countLeftNeg);
                //printf("number for right %d %d \n", countRightPos, countRightNeg);
}

void bottleProcessorThread::spatialSelection(AER_struct* buffer,int numberOfEvents, double w, unsigned long minCount, unsigned long maxCount) {
    // in the list of unmasked event updates the saliency map and the timestamp map 
    AER_struct* iter = buffer; // generated the iterator of the buffer
    int inputRetinaSize = 32;
    
    for (int i = 0; i < numberOfEvents; i++) {        
        if((iter->ts > minCount) && (iter->ts < maxCount)) {
            if((iter->x != 128) && (iter->y != 1)){
                //printf("%02d %02d  ", iter->x, iter->y);    
                
                int pos = (inputRetinaSize - 1 - iter->x) + (inputRetinaSize - 1 - iter->y) * inputRetinaSize;
                if(pos >= 32*32) {
                    printf("Error %d %d \n", iter->x, iter->y);
                }
                if(iter->pol > 0) {
                    featureMapLeft[pos]   = featureMapLeft[pos] + 127;
                    if(featureMapLeft[pos] > 127) {
                        featureMapLeft[pos] = 127;
                    }
                }
                else {
                    featureMapLeft[pos]   = featureMapLeft[pos] - 127;
                    if(featureMapLeft[pos] < -127) {
                        featureMapLeft[pos] = -127;
                    }
                }
                
                //printf (" %08X \n", iter->ts);
                timestampMap[pos] = iter->ts;            
            } 
        }
        iter++;
    }    
}

void bottleProcessorThread::run() {
  count++;
  bottleHandler = true;
  if(!idle) {
    interTimer = Time::now();
    double interval2 = (interTimer - startTimer) * 1000000;
    // reads the buffer received
    //bufferRead = cfConverter->getBuffer();    
    // saves it into a working buffer
    //printf("checking the bottleHandler \n");
    //printf("============================================================= \n");
    if(!bottleHandler) {
        cfConverter->copyChunk(bufferCopy);//memcpy(bufferCopy, bufferRead, 8192);
    }
    else {
        //printf("bottleProcessorThread::run : extracting Bottle! \n");
        receivedBottle->clear();
        ebHandler->extractBottle(receivedBottle); 
        //if(receivedBottle->size() != 0) {
        //    printf("received bottle %d \n", receivedBottle->size());
        //}
        //printf("%s \n", receivedBottle->toString().c_str());
    }
    
    /*
    //======================== temporal synchronization pre-unmasking  =================================
    //printf("after extracting the bottle \n");
    // saving the buffer into the file
    int num_events = CHUNKSIZE / 8 ;
    uint32_t* buf2 = (uint32_t*)bufferCopy;

    // getting the time statistics
    endTimer = Time::now();
    double interval  = (endTimer - startTimer) * 1000000; //interval in us
    //double procInter = interval -interval2;
    //printf("procInter %f \n", procInter);
    startTimer = Time::now();

    //check for wrapping of the left and right timestamp
    if(maxCount >= 4294967268  ) {
      verb = true;
      unmask_events->resetTimestampLeft();
      unmask_events->resetTimestampRight();
      printf("wrapping left %lu %lu \n",lc,unmask_events->getLastTimestamp());
      printf("wrapping left %lu %lu\n",lc,unmask_events->getLastTimestamp());
      printf("wrapping left %lu %lu\n",lc,unmask_events->getLastTimestamp());
      printf("wrapping left %lu %lu\n",lc,unmask_events->getLastTimestamp());
      printf("wrapping left %lu %lu\n",lc,unmask_events->getLastTimestamp());
      printf("wrapping left %lu %lu\n",lc,unmask_events->getLastTimestamp());
      printf("wrapping left %lu %lu\n",lc,unmask_events->getLastTimestamp());
      minCount = 0;
      maxCount      =  minCount      + interval * INTERVFACTOR* (dim_window);
   
    }
    if(maxCountRight >= 4294967268) {
      verb = true;
      unmask_events->resetTimestampRight();
      unmask_events->resetTimestampLeft();
      printf("wrapping right %lu %lu\n",rc,unmask_events->getLastTimestampRight());
      printf("wrapping right %lu %lu\n",rc,unmask_events->getLastTimestampRight());
      printf("wrapping right %lu %lu\n",rc,unmask_events->getLastTimestampRight());
      printf("wrapping right %lu %lu\n",rc,unmask_events->getLastTimestampRight());
      printf("wrapping right %lu %lu\n",rc,unmask_events->getLastTimestampRight());
      printf("wrapping right %lu %lu\n",rc,unmask_events->getLastTimestampRight());

      minCountRight = 0;
      maxCountRight = minCountRight + interval * INTERVFACTOR* (dim_window);
    }
    //printf("end of pre-unmasking synchronization \n ");
    */
    
    // =============================== Unmasking ====================================
    // extract a chunk/unmask the chunk
    // printf("verb %d \n",verb);
    if(!bottleHandler) {
        //printf("unmasking without bottleHandler \n");
        unmask_events->unmaskData(bufferCopy,CHUNKSIZE, unmaskedEvents);
    }
    else {
        //printf("unmasking with bottleHandler \n");
        if(receivedBottle->size() != 0) {     
            //printf("TODO : deleting rxQueue \n");
            //delete rxQueue;   // freeing memory for the new queue of events
            //rxQueue = new eEventQueue(); // preparing the new queue
            rxQueue->clear();
            //printf("unmasking received bottle and creating the queue of event to send \n");            
            unmask_events->unmaskData(receivedBottle, rxQueue); // saving the queue
            //printf("bottle of events to send \n");
        }
    }
    //printf("end of the unmasking \n");
        
    //==================== temporal synchronization post unmasking =======================
    

    /*
        
#ifdef VERBOSE
    int num_events2 = CHUNKSIZE>>3;
    AER_struct* iter = unmaskedEvents;
    for (int evt = 0; evt < num_events2; evt++) {
        //unsigned long blob      = buf2[2 * evt];
        //unsigned long t         = buf2[2 * evt + 1];
        if(iter->ts!=0) {
            fprintf(fout," %08x %d %d \n", iter->ts, iter->x, iter->y);
        }
        //if((iter->x >=32) || (iter->y >= 32)) {
        //    printf("Error after unmasking %d %d \n", iter->x, iter->y);
        //}
        iter++;
    }
#endif

    //gettin the time between two threads
    gettimeofday(&tvend, NULL);
    //Tnow = ((u64)tvend.tv_sec) * 1000000 + ((u64)tvstart.tv_usec);
    Tnow = ((tvend.tv_sec * 1000000 + tvend.tv_usec)
	    - (tvstart.tv_sec * 1000000 + tvstart.tv_usec));
    //printf("timeofday>%ld\n",Tnow );
    gettimeofday(&tvstart, NULL);       
    
    //synchronising the threads at the connection time
    unsigned long int lastleft, lastright;
    if ((cfConverter->isValid())&&(!synchronised)) {
        printf("Sychronising ");
        if(!bottleHandler) {
            lastleft  = unmask_events->getLastTimestamp();
            lastright = unmask_events->getLastTimestampRight();
        }
        else {
            lastleft = lastright = *lasttimestamp;
        }
        lc = lastleft  * COUNTERRATIO; 
        rc = lastright * COUNTERRATIO;
        
        //TODO : Check for negative values of minCount not allowed!!!!!!
        
        minCount = lc - interval * INTERVFACTOR* dim_window; 
        //cfConverter->getEldestTimeStamp();                                                                   
        minCountRight = rc - interval * INTERVFACTOR* dim_window;
        //printf("synchronised %1f! %d,%d,%d||%d,%d,%d \n",interval, minCount, lc, maxCount, minCountRight, rc, maxCountRight);
        startTimer = Time::now();
        synchronised = true;
        //minCount = unmask_events->getLastTimestamp();
        //printf("minCount %d \n", minCount);
        //minCountRight = unmask_events->getLastTimestamp();
        count = synch_time - 200;
    }
    else if ((count % synch_time == 0) && (minCount < 4294500000)) {
        if(!bottleHandler) {
            lastleft  = unmask_events->getLastTimestamp();
            lastright = unmask_events->getLastTimestampRight();
        }
        else {
            lastleft = lastright = *lasttimestamp;  
        }
        lc = lastleft  * COUNTERRATIO; 
        rc = lastright * COUNTERRATIO;
        
        //if (count == synch_time) {
        //printf("Sychronised Sychronised Sychronised Sychronised ");
        if( lc > interval* INTERVFACTOR * dim_window)
            minCount      = lc - interval* INTERVFACTOR * dim_window; //cfConverter->getEldestTimeStamp();        
        else
            minCount = 0;
        
        if( rc > interval* INTERVFACTOR * dim_window)
            minCountRight = rc - interval* INTERVFACTOR * dim_window;
        else
            minCountRight = 0;
        //maxCount = lc; 
        //maxCountRight = rc;
        
        //printf("synchronised %1f! %llu,%llu,%llu||%llu,%llu,%llu \n",interval, minCount, lc, maxCount, minCountRight, rc, maxCountRight);
        startTimer = Time::now();
        synchronised = true; 
    }
    else {
      // this value is simply the ration between the timestamp reported by the aexGrabber (6.25Mhz) 
      //and the correct timestamp counter clock of FPGA (50 Mhz)
      microsecondsPrev = interval;
      interval = Tnow;
      minCount      = minCount + interval * INTERVFACTOR; // * (50.0 MHz FPGA Counter Clock / 6.25 Mhz ;
      minCountRight = minCount + interval * INTERVFACTOR;  // minCountRight + interval;
    } 
    //printf("minCount %d interval %f \n", minCount, interval);
    maxCount      =  minCount      + interval * INTERVFACTOR* (dim_window);
    maxCountRight =  minCountRight + interval * INTERVFACTOR* (dim_window);
    
    
    //---- preventer for fixed  addresses ----//
    if(count % 100 == 0) { 
        if(!bottleHandler) {
            lastleft = unmask_events->getLastTimestamp();
            lastright = unmask_events->getLastTimestampRight();
        }
        else {
            lastleft = lastright = lasttimestamp;
        }
        
        lc = lastleft * COUNTERRATIO; 
        //unsigned long lastright = unmask_events->getLastTimestampRight();
        rc = lastright * COUNTERRATIO;
        //printf("countStop %d lcprev %d lc %d \n",countStop, lcprev,lc);
        if (stereo) {
            if ((lcprev == lc)||(rcprev == rc)) {
                //if (lcprev == lc) {
                countStop++;
                printf("countStop %d %lu %lu %lu %lu \n", countStop, lc, lcprev, rc, rcprev);
            }            
            else {
                countStop--;
                //printf("countStop %d \n", countStop);
                if(countStop<= 0) {
                    countStop = 0;
                }
            }
        }
        else {
            if (lcprev == lc) {
                //if (lcprev == lc) {
                countStop++;
                printf("countStop %d %lu %lu %lu %lu \n", countStop, lc, lcprev, rc, rcprev);
            }            
            else {
                countStop--;
                //printf("countStop %d \n", countStop);
                if(countStop<= 0) {
                    countStop = 0;
                }
            }
        }
        lcprev = lc;
        rcprev = rc;
    }
    
         
    //resetting time stamps at overflow
    if (countStop == 10) {
      //printf("resetting time stamps!!!!!!!!!!!!! %d %d   \n ", minCount, minCountRight);
      unmask_events->resetTimestampLeft(); 
      unmask_events->resetTimestampRight();
      cfConverter->reset();
      unsigned long lastleft = unmask_events->getLastTimestamp();
      lc = lastleft * COUNTERRATIO; 
      unsigned long lastright = unmask_events->getLastTimestampRight();
      rc = lastright * COUNTERRATIO;
      minCount      = 0;
      minCountRight = 0;
      maxCount      =  minCount      + interval * INTERVFACTOR* (dim_window);
      maxCountRight =  minCountRight + interval * INTERVFACTOR* (dim_window);
      
      //maxCount      = 4294967268;
      //maxCountRight = 4294967268;
      countStop = 0;
      verb = true;
      printf("countStop resetting %llu %llu %llu \n",unmask_events->getLastTimestamp(), lc, rc );
      count = synch_time - 200;
      
    }    
    */

    // =======================  spatial processing===========================
    double w1 = 0, w2 = 0, w3 = 0, w4 = 0;
    //spatialSelection(unmaskedEvents,CHUNKSIZE>>3,w1);    
    
    // spatial processing
    if(!bottleHandler) {
        //printf("spatial selection no bottle Handler \n");
        spatialSelection(unmaskedEvents,CHUNKSIZE>>3,w1, minCount, maxCount);
    }
    else {        
        if((rxQueue != 0)&&(receivedBottle->size() != 0)) {
            //printf("spatial selection  bottle Handler\n");
            spatialSelection(rxQueue);
            //printf("after spatial selection bottleHandler \n");
        }
    }
    
    
    /*
    // the getMonoImage gets as default input image the saliency map
    if(imageLeft != 0) {
        //printf("getting the left image \n");
        getMonoImage(imageLeft,minCount,maxCount,1);
        //printf("copying the right \n");
        pThread->copyLeft(imageLeft);
    }
    
    if(stereo) {
        if(imageRight != 0) {
            //printf("getting the right image \n");
            getMonoImage(imageRight,minCountRight,maxCountRight,0);
            //printf("copying the right image \n");
            pThread->copyRight(imageRight);
        }
    }
    */
  }
}

void bottleProcessorThread::threadRelease() {
    idle = false;
    fclose(fout);
    printf("bottleProcessorThread release:freeing bufferCopy \n");

    //free(saliencyMap);
    free(featureMapLeft);
    free(featureMapRight);
    
    free(timestampMap); 
    free(unmaskedEvents); 
    printf("bottleProcessorThread release:closing ports \n");
    outPort.close();
    outPortRight.close();
    //delete imageLeft;
    //delete imageRight;
    delete receivedBottle;
    delete map1Handler;
    delete ebHandler;
    printf("bottleProcessorThread release         stopping plotterThread \n");
    pThread->stop();
    printf("bottleProcessorThread release         deleting converter \n");
    delete cfConverter;
    printf("correctly freed memory from the cfCollector \n");
}


//----- end-of-file --- ( next line intentionally left blank ) ------------------

