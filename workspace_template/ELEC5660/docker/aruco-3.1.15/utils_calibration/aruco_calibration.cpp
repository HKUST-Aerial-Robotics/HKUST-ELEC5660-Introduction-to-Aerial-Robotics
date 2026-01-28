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
#include "calibrator.h"
#include <fstream>
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>


#if  CV_MAJOR_VERSION >= 4
#define CV_FOURCC(a,b,c,d) VideoWriter::fourcc(a,b,c,d)
#define CV_CAP_PROP_FOURCC cv::CAP_PROP_FOURCC
#define CV_CAP_PROP_AUTOFOCUS cv::CAP_PROP_AUTOFOCUS
#endif
using namespace std;
using namespace cv;
using namespace aruco;
#ifndef CV_CAP_PROP_AUTOFOCUS
#define CV_CAP_PROP_AUTOFOCUS -1
#endif
float TheMarkerSize = -1;
VideoCapture TheVideoCapturer;
Mat TheInputImage, TheInputImageCopy;
CameraParameters TheCameraParameters;
MarkerDetector TheMarkerDetector;
vector<vector<aruco::Marker> >  allMarkers;
string TheOutCameraParams;
aruco::CameraParameters camp;  // camera parameters estimated
Calibrator calibrator;

aruco::CameraParameters cameraCalibrate(std::vector<std::vector<aruco::Marker> >  &allMarkers, int imageWidth,int imageHeight,float markerSize,float *currRepjErr=0, aruco::MarkerMap *inmmap=0);

class CmdLineParser
{  int argc;char** argv;public:
    CmdLineParser(int _argc, char** _argv): argc(_argc), argv(_argv){}
    bool operator[](string param)    {int idx = -1;   for (int i = 0; i < argc && idx == -1; i++)if (string(argv[i]) == param) idx = i;return (idx != -1); }
    string operator()(string param, string defvalue = "-1"){int idx = -1;for (int i = 0; i < argc && idx == -1; i++)if (string(argv[i]) == param) idx = i;if (idx == -1) return defvalue;else return (argv[idx + 1]);}
};
/************************************
 *
 *
 ************************************/
int main(int argc, char** argv)
{
    try
    {
        bool isLive=false;
         CmdLineParser cml(argc, argv);
        if (argc < 3 || cml["-h"])
        {
            cerr << "Usage: (in.avi|live[:camera_index(e.g 0 or 1)])) out_camera_calibration.yml  [-m markermapConfig.yml (configuration of the "
                    "board. If use default one (in utils), no need to set this)]    [-size <float> :(value in meters "
                    "of a marker. If you provide a board that contains that information, this is ommited) ] "
                    "[-save: if enabled, saves the calibration images used] "
                 << endl;
            return -1;
        }
         // read from camera or from  file
        string TheInputVideo=string(argv[1]);
        if ( TheInputVideo.find( "live")!=std::string::npos)
        {
            int vIdx = 0;
            // check if the :idx is here
            char cad[100];
            if (TheInputVideo.find(":") != string::npos)
            {
                std::replace(TheInputVideo.begin(), TheInputVideo.end(), ':', ' ');
                sscanf(TheInputVideo.c_str(), "%s %d", cad, &vIdx);
            }
            cout << "Opening camera index " << vIdx << endl;
            TheVideoCapturer.open(vIdx);
            TheVideoCapturer.set(CV_CAP_PROP_AUTOFOCUS, 0);

            isLive=true;
         }
        else
            TheVideoCapturer.open(argv[1]);
        // check video is open
        if (!TheVideoCapturer.isOpened())
        {
            cerr << "Could not open video" << endl;
            return -1;
        }
        TheVideoCapturer.set(CV_CAP_PROP_FOURCC ,CV_FOURCC('M', 'J', 'P', 'G') );
        TheVideoCapturer.set(CV_CAP_PROP_AUTOFOCUS,0);
//        TheVideoCapturer.set(CV_CAP_PROP_FRAME_WIDTH,1280);
//        TheVideoCapturer.set(CV_CAP_PROP_FRAME_HEIGHT,720);
        bool saveImages=cml["-save"];
        // read first image to get the dimensions
        TheVideoCapturer >> TheInputImage;

        //configure the calibrator
        calibrator.setParams(TheInputImage.size(),stof(cml("-size","1")),cml("-m",""));


        // set specific parameters for this configuration
        TheMarkerDetector.setDictionary( "ARUCO_MIP_36h12");
        TheMarkerDetector.setDetectionMode(aruco::DM_NORMAL);
         cv::namedWindow("in", cv::WINDOW_NORMAL);
        cv::resizeWindow("in",800,600);

        char key = 0 ;
        int waitKeyTime= isLive?10:-10;
        // capture until press ESC or until the end of the video
        do
        {
            TheVideoCapturer.retrieve(TheInputImage);  // get image
            // detect and print
            vector<aruco::Marker> detected_markers = TheMarkerDetector.detect(TheInputImage);
            // print markers from the board
             for (auto m: detected_markers)
                m.draw(TheInputImage, Scalar(0, 0, 255), 1);

            // draw help
            cv::putText(TheInputImage,"'a' add current image for calibration",cv::Point(10,20),FONT_HERSHEY_SIMPLEX, 0.5f,cv::Scalar(125,255,255),1);
            cv::putText(TheInputImage,"'s' start/stop capture",cv::Point(10,40),FONT_HERSHEY_SIMPLEX, 0.5f,cv::Scalar(125,255,255),1);
            cv::putText(TheInputImage,calibrator.getInfo(),cv::Point(10,60),FONT_HERSHEY_SIMPLEX, 0.5f,cv::Scalar(125,255,255),1);
            cv::imshow("in", TheInputImage);

             key = cv::waitKey(waitKeyTime);   // wait for key to be pressed
            if (key == 'a'){
                calibrator.addView(detected_markers);
                if (saveImages){
                    string number=std::to_string(calibrator.getNumberOfViews()-1);
                    while(number.size()!=5) number="0"+number;
                    cv::imwrite("calib-"+number+".png",TheInputImage);
                }
            }
            // set waitTime in start/stop mode
            if (key == 's') waitKeyTime*=-1;

        } while (key != 27 && TheVideoCapturer.grab());





         aruco::CameraParameters camp;
        if ( calibrator.getCalibrationResults(camp)){
             camp.saveToFile(argv[2]);
            cout<<"results saved to "<<argv[2]<<endl;
        }
        else cerr<<"Could not obtain calibration"<<endl;
    cout<<"Final error="<<calibrator.getReprjError()<<endl;
    }
    catch (std::exception& ex)

    {
        cout << "Exception :" << ex.what() << endl;
    }
}

