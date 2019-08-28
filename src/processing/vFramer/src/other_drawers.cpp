/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
 *           valentina.vasco@iit.it
 *           chiara.bartolozzi@iit.it
 *           massimiliano.iacono@iit.it
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "vDraw.h"

using namespace ev;

// BLOB DRAW //
// ========= //

const std::string blobDraw::drawtype = "BLOB";

std::string blobDraw::getDrawType()
{
    return blobDraw::drawtype;
}

std::string blobDraw::getEventType()
{
    return AE::tag;
}

void blobDraw::draw(cv::Mat &image, const ev::vQueue &eSet, int vTime)
{

    if(eSet.empty()) return;
    if(vTime < 0) vTime = eSet.back()->stamp;

    ev::vQueue::const_reverse_iterator qi;
    for(qi = eSet.rbegin(); qi != eSet.rend(); qi++) {


        int dt = vTime - (*qi)->stamp;
        if(dt < 0) dt += ev::vtsHelper::max_stamp;
        if((unsigned int)dt > display_window) break;

        auto aep = as_event<AE>(*qi);
        if(!aep) continue;

        int y = aep->y;
        int x = aep->x;

        if(flip) {
            y = Ylimit - 1 - y;
            x = Xlimit - 1 - x;
        }

        if(!aep->polarity)
            image.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
    }

    cv::medianBlur(image, image, 5);
    cv::blur(image, image, cv::Size(5, 5));
}

// CIRCLE DRAW //
// =========== //

const std::string circleDraw::drawtype = "CIRC";

std::string circleDraw::getDrawType()
{
    return circleDraw::drawtype;
}

std::string circleDraw::getEventType()
{
    return GaussianAE::tag;
}

void circleDraw::draw(cv::Mat &image, const vQueue &eSet, int vTime)
{
    cv::Scalar blue = CV_RGB(0, 0, 255);
    cv::Scalar red = CV_RGB(255, 0, 0);

    //update the 'persistence' the current state of each of the cluster ID's
    for(vQueue::const_iterator qi = eSet.begin(); qi != eSet.end(); qi++) {
        auto vp = is_event<GaussianAE>(*qi);
        if(vp) {
            persistance[vp->ID] = vp;
        }
    }

    std::map<int, event<GaussianAE> >::iterator ci;
    for(ci = persistance.begin(); ci != persistance.end(); ci++) {

        auto v = ci->second;
        if(v->polarity) continue;

        if(v->x < 0 || v->x >= Xlimit || v->y < 0 || v->y >= Ylimit) continue;
        if(v->sigxy >= v->sigx) continue;

        cv::Point centr(v->x, v->y);
        if(flip) {
            centr.x = Xlimit - 1 - centr.x;
            centr.y = Ylimit - 1 - centr.y;
        }

        cv::circle(image, centr, v->sigx - v->sigxy, red, 1.0);
        cv::circle(image, centr, v->sigx + v->sigxy, red, 1.0);

        continue;
    }

    for(ci = persistance.begin(); ci != persistance.end(); ci++) {

        auto v = ci->second;
        if(!v->polarity) continue;

        if(v->x < 0 || v->x >= Xlimit || v->y < 0 || v->y >= Ylimit) continue;
        if(v->sigxy >= v->sigx) continue;

        cv::Point centr(v->x, v->y);
        if(flip) {
            centr.x = Xlimit - 1 - centr.x;
            centr.y = Ylimit - 1 - centr.y;
        }

        cv::circle(image, centr, v->sigx - v->sigxy, blue, 1.0);
        cv::circle(image, centr, v->sigx + v->sigxy, blue, 1.0);

        continue;
    }

}

// GRAY DRAW //
// =========== //

const std::string grayDraw::drawtype = "GRAY";

std::string grayDraw::getDrawType()
{
    return grayDraw::drawtype;
}

std::string grayDraw::getEventType()
{
    return AddressEvent::tag;
}

void grayDraw::draw(cv::Mat &image, const ev::vQueue &eSet, int vTime)
{
    image = cv::Scalar(127, 127, 127);
    if(eSet.empty()) return;
    if(vTime < 0) vTime = eSet.back()->stamp;
    ev::vQueue::const_reverse_iterator qi;
    for(qi = eSet.rbegin(); qi != eSet.rend(); qi++) {

        int dt = vTime - (*qi)->stamp;
        if(dt < 0) dt += ev::vtsHelper::max_stamp;
        if((unsigned int)dt > display_window) break;


        auto aep = is_event<AddressEvent>(*qi);
        int y = aep->y;
        int x = aep->x;
        if(flip) {
            y = Ylimit - 1 - y;
            x = Xlimit - 1 - x;
        }

        cv::Vec3b &cpc = image.at<cv::Vec3b>(y, x);

        if(!aep->polarity)
        {
            cpc[0] = 0;
            cpc[1] = 0;
            cpc[2] = 0;
        }
        else
        {
            cpc[0] = 255;
            cpc[1] = 255;
            cpc[2] = 255;

        }
    }
}

// STEREO OVERLAY DRAW //
// =================== //

const std::string overlayStereoDraw::drawtype = "OVERLAY";

std::string overlayStereoDraw::getDrawType()
{
    return overlayStereoDraw::drawtype;
}

std::string overlayStereoDraw::getEventType()
{
    return AddressEvent::tag;
}

void overlayStereoDraw::draw(cv::Mat &image, const ev::vQueue &eSet, int vTime)
{
    if(eSet.empty()) return;
    if(vTime < 0) vTime = eSet.back()->stamp;
    ev::vQueue::const_reverse_iterator qi;
    for(qi = eSet.rbegin(); qi != eSet.rend(); qi++) {

        int dt = vTime - (*qi)->stamp;
        if(dt < 0) dt += ev::vtsHelper::max_stamp;
        if((unsigned int)dt > display_window) break;


        auto aep = is_event<AddressEvent>(*qi);
        int y = aep->y;
        int x = aep->x;
        if(flip) {
            y = Ylimit - 1 - y;
            x = Xlimit - 1 - x;
        }


        cv::Vec3b &cpc = image.at<cv::Vec3b>(y, x);

        if(cpc[0]==0 && cpc[1]==255 && cpc[2]==255) //skip marking already overlapping pixels
            continue;
        if(!aep->channel)
        {
            if(cpc[0]==0 && cpc[1]==0 && cpc[2]==255) //both left and right channel, mark YELLOW
            {
                cpc[0]=0;
                cpc[1]=255;
                cpc[2]=255;
            }
            else   //only left channel, mark BLUE
            {
                cpc[0]=255;
                cpc[1]=0;
                cpc[2]=0;
            }
        }
        else
        {
            if(cpc[0]==255 && cpc[1]==0 && cpc[2]==0)   //both left and right channel, mark YELLOW
            {
                cpc[0]=0;
                cpc[1]=255;
                cpc[2]=255;
            }
            else    //only right channel, mark RED
            {
                cpc[0]=0;
                cpc[1]=0;
                cpc[2]=255;
            }
        }
    }
}

//Rasterplot Draw//
//=============//

const std::string rasterDraw::drawtype = "RASTER";

std::string rasterDraw::getDrawType()
{
    return rasterDraw::drawtype;
}

std::string rasterDraw::getEventType()
{
    return AddressEvent::tag;
}

void rasterDraw::draw(cv::Mat &image, const ev::vQueue &eSet, int vTime)
{
    //check if eSet contains elements
    if(eSet.empty()){
        return;
    }
    //get latest timestamp of eSet
    if(vTime < 0){
        vTime = eSet.back()->stamp;
    }
    //Event-to-Pixel Iterator:
    for(int y=(neuronID-1); y>=0; y--){
        for(int x=(timeElements-1); x>=0; x--){

            //if there is an Eventpixel in the storage
            if (eventStorage[y][x] > 0){

                //SCALING timeElements to Xlimit of vFramer [optional]
                if(scaling){
                    //reset if last timeElement is reached
                    if(x >= int(timeElements-1)){
                        eventStorage[y][x] = 0;
                    }
                    xPixel = round(x*xScaler);

                    //when neuronID is bigger than screenheight, yPixel=y:
                    if(neuronID >= Ylimit){
                        image.at<cv::Vec3b>(y, xPixel) = cv::Vec3b(255, 0, 0);
                    }
                    //when neuronID is smaller than screenheight, yPixel is inside storage
                    else{
                        image.at<cv::Vec3b>(eventStorage[y][x], xPixel) = cv::Vec3b(255, 0, 0);
                    }
                }
                //NO SCALING so we stay in x-Limit of vFramer:
                else if ((Xlimit-1) > x){
                    image.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 0, 0);
                    //reset storage if xLimit of vFramer is reached
                    if(x >= (Xlimit-1.0)){
                        eventStorage[y][x] = 0;
                    }
                }
                if(eventStorage[y][x] > 0){
                    //Increase timeElement by the period (dist between two elements)
                    eventStorage[y][x+1] = eventStorage[y][x];
                    //reset the old field value
                    eventStorage[y][x] = 0;
                    }
            }
        }
    }
    //go through the eSet-q and start with the latest event
    ev::vQueue::const_reverse_iterator qi;
    for(qi = eSet.rbegin(); qi != eSet.rend(); qi++) {

        //calculate the time difference between latest and actual event in eSet
        int dt = vTime - (*qi)->stamp;

        // if diff is negative,timestamp was over max_stamp (=85ms) and resetted to 0ms
        if(dt < 0)
            dt += ev::vtsHelper::max_stamp;

        //eSet-q accumulates events, therefore ignore events that are older than 1ms
        if((unsigned int)dt > display_window){
            break;
        }

        //Safety: Make whatevers inside the q-element to AE
        auto aep = as_event<AE>(*qi);

        if(!aep){
            continue;
        }
        //Safe it as an accessible AE type
        AE v = *(aep);

        //measure neuronID (total 32bit address as integer)
        unsigned int y = v._coded_data;

        //flip the y-Axis [optional]
        if(flip){
            y = (neuronID-1) - y;
        }

        //SCALING eventStorage [neuronID][...] to YLimit of vFramer [optional]
        if(scaling){

            yPixel = round(y * yScaler);

            //when neuronID bigger than screenheight, store yPixel in yEvent
            if(neuronID >= Ylimit){
                eventStorage[yPixel][0] = 1;
            }
            //when neuronID's smaller than screenheight, but bigger than measured neuronID, store yPixel in eventStore
            else if(neuronID >= y){
                eventStorage[y][0] = yPixel;
            }
        }
        //NO SCALING so we stay in y-Limit of vFramer
        else if((neuronID-1)>= y){
            eventStorage[y][0] = 1;
        }
    }
}
