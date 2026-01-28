/**
Copyright 2017 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
*/

#include "aruco.h"
#include "cvdrawingutils.h"
#include "posetracker.h"
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>
#include <stdexcept>

using namespace std;
using namespace cv;
using namespace aruco;
/************************************
 *
 *
 *
 *
 ************************************/
int main(int argc, char** argv)
{
    try
    {
        if (argc < 4)
        {
            cerr << "Usage:  in-markerSetConfig.yml out-markerSetConfig.yml  markerId"
                 << endl;
            exit(0);
        }
        // open video


        // read marker map
        MarkerMap TheMarkerMapConfig;  // configuration of the map
        TheMarkerMapConfig.readFromFile(argv[1]);

        //find the one that will be the center
        int id=stoi(argv[3]);
        int thepos=-1;
        for(size_t pos=0;pos<TheMarkerMapConfig.size() && thepos==-1 ;pos++){
            if(TheMarkerMapConfig[pos].id==id){
                thepos=pos;
            }
        }
        if(thepos==-1)throw std::runtime_error("Invalid id specified. Not in the set");

        //calculate center
        cv::Point3f center(0,0,0);
        for(int i=0;i<TheMarkerMapConfig[thepos].points.size();i++)
            center+=TheMarkerMapConfig[thepos].points [i];
        center/=float(TheMarkerMapConfig[thepos].points .size());


        //translate all
        for(auto &m:TheMarkerMapConfig){
            for(auto &p:m.points)
                p-=center;
        }
        TheMarkerMapConfig.saveToFile(argv[2]);


    }
    catch (std::exception& ex)

    {
        cout << "Exception :" << ex.what() << endl;
    }
}
