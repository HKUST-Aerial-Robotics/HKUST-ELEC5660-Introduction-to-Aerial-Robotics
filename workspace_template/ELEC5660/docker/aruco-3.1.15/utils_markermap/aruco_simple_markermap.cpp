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
        if (argc < 3)
        {
            cerr << "Usage: (in_image|video.avi)  markerSetConfig.yml [cameraParams.yml] [markerSize]  [outImage]"
                 << endl;
            exit(0);
        }
        // open video
        cv::Mat InImage;
        VideoCapture vreader(argv[1]);
        if (!vreader.isOpened())
            throw std::runtime_error("Could not open input");
        // read input image(or first image from video)
        vreader >> InImage;

        // read marker map
        MarkerMap TheMarkerMapConfig;  // configuration of the map
        TheMarkerMapConfig.readFromFile(argv[2]);

        // read camera params if indicated
        aruco::CameraParameters CamParam;
        if (argc >= 4)
        {
            CamParam.readFromXMLFile(argv[3]);
            // resizes the parameters to fit the size of the input image
            CamParam.resize(InImage.size());
        }
        // read marker size if indicated
        float MarkerSize = -1;
        if (argc >= 5)
            MarkerSize = static_cast<float>(atof(argv[4]));
        // transform the markersetconfig to meter if is in pixels and the markersize indicated
        if (TheMarkerMapConfig.isExpressedInPixels() && MarkerSize > 0)
            TheMarkerMapConfig = TheMarkerMapConfig.convertToMeters(MarkerSize);

        // Let go
        MarkerDetector MDetector;
        // set the appropiate dictionary type so that the detector knows it
        MDetector.setDictionary(TheMarkerMapConfig.getDictionary());
        // detect markers without computing R and T information
        vector<Marker> Markers = MDetector.detect(InImage);

        // print the markers detected that belongs to the markerset
        vector<int> markers_from_set = TheMarkerMapConfig.getIndices(Markers);
        for (auto idx : markers_from_set)
            Markers[idx].draw(InImage, Scalar(0, 0, 255), 2);

        // detect the 3d camera location wrt the markerset (if possible)
        if (TheMarkerMapConfig.isExpressedInMeters() && CamParam.isValid())
        {
            MarkerMapPoseTracker MSPoseTracker;  // tracks the pose of the marker map
            MSPoseTracker.setParams(CamParam, TheMarkerMapConfig);
            if (MSPoseTracker.estimatePose(Markers)) { // if pose correctly computed, print the reference system
                aruco::CvDrawingUtils::draw3dAxis(InImage, CamParam, MSPoseTracker.getRvec(), MSPoseTracker.getTvec(),
                                                  TheMarkerMapConfig[0].getMarkerSize() * 2);
                cout<<"rvec="<<MSPoseTracker.getRvec()<<" tvec="<<MSPoseTracker.getTvec()<<endl;
            }
        }
        // show input with augmented information
        cv::imshow("in", InImage);
        while (char(cv::waitKey(0)) != 27)
            ;  // wait for esc to be pressed
        // save output if indicated
        if (argc >= 6)
            cv::imwrite(argv[5], InImage);
    }
    catch (std::exception& ex)

    {
        cout << "Exception :" << ex.what() << endl;
    }
}
