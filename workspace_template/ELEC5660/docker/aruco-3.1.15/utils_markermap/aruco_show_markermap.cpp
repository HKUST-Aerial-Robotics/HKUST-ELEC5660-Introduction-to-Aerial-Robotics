/**
Copyright 2020 Rafael Muñoz Salinas. All rights reserved.

  This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation version 3 of the License.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
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
#include "markermap.h"
#include "sglviewer.h"

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
        if (argc !=2)throw std::runtime_error("Usage:   markerSetConfig.yml");

        aruco::MarkerMap TheMarkerMapConfig;
        TheMarkerMapConfig.readFromFile(argv[1]);
        sgl_OpenCV_Viewer Viewer;
        Viewer.setParams(1.5,1280,960,"map_viewer");
        cv::Mat TheInputImageCopy( 1280,960,CV_8UC3);
        char key = 0;
        // capture until press ESC or until the end of the video
        cout << "Press 's' to start/stop video" << endl;
        int waitTime=0;
        do
        {
            key =     Viewer.show(TheMarkerMapConfig,cv::Mat(),TheInputImageCopy,waitTime);
            if (key=='s') waitTime=waitTime?0:10;
        } while (key != 27  && key!='q');

    }
    catch (std::exception& ex)

    {
        cout << "Exception :" << ex.what() << endl;
    }
}
