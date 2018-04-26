/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
 *           chiara.bartolozzi@iit.it
 *           marino.laterza@iit.it
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

const std::string skinDraw::drawtype = "SKIN";

std::string skinDraw::getDrawType()
{
    return skinDraw::drawtype;
}

std::string skinDraw::getEventType()
{
    return AddressEvent::tag;
}

void skinDraw::draw(cv::Mat &image, const ev::vQueue &eSet, int vTime)
{
    cv::Scalar pos = CV_RGB(160, 0, 160);
    cv::Scalar neg = CV_RGB(0, 60, 1);

    int radius = 4;

    if(image.empty()) {
        image = cv::Mat(Ylimit, Xlimit, CV_8UC3);
        image.setTo(255);
    }

    if(eSet.empty()) return;

    ev::vQueue::const_reverse_iterator qi;
    for(qi = eSet.rbegin(); qi != eSet.rend(); qi++) {


        int dt = eSet.back()->stamp - (*qi)->stamp; // start with newest event
        if(dt < 0) dt += ev::vtsHelper::maxStamp();
        if(dt > twindow) break;


        auto aep = is_event<AddressEvent>(*qi);
        int y = aep->y;
        int x = aep->x;

        if(aep->type == 0)
            y = Ylimit - radius;
        else {
            y = radius + y * 200.0 / 255.0;
            //x = x;
        }

        // decode the event here: i.e. do the mapping from the x value to x,y location on the image

        // get the pixel: substitute with code to draw a circle from circleDrawer
        y = Ylimit - y - 1;
        cv::Point centr(x, y);

        if(!aep->polarity)
        {
            cv::circle(image, centr, radius, pos, CV_FILLED);
        }
        else
        {
            cv::circle(image, centr, radius, neg, CV_FILLED);
        }
    }
}



