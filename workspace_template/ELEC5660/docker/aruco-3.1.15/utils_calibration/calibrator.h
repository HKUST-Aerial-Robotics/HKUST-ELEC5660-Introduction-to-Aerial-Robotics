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

#ifndef ArucoCalibrator_H
#define ArucoCalibrator_H
#include <string>
#include <vector>
#include "marker.h"
#include "markermap.h"
#include "cameraparameters.h"
#include <thread>
#include <mutex>
namespace  aruco {
/**
 * @brief The Calibrator class implements a calibration thread
 */
class Calibrator{

    std::vector<std::vector<aruco::Marker> >  vmarkers,vmarkerstmp;
    float _msize;
    aruco::MarkerMap _mmap;
    cv::Size _imageSize;
    std::thread _calibThread;
    std::mutex markersMutex,markerstmpMutex,infoMutex,calibrationMutex;
    std::string strinfo;
    bool keepCalibrating;
    aruco::CameraParameters cameraParams;
    float calibError=-1;
public:

    Calibrator();
    ~Calibrator();
    void setParams(cv::Size imageSize, float markerSize=1, std::string markerMap="");
    //add a detection of the calibration markerset and do calibration
    //returns the current number of views
    void addView(const std::vector<Marker> &markers);

    //returns a string with info about the status
    std::string getInfo();

    //obtain the camera calibration results. It is blocking function
    //returns true if calibration was ok
    bool getCalibrationResults( aruco::CameraParameters &camp);
    int getNumberOfViews();

    float getReprjError()const{return calibError;}
private:
    void calibration_function();
    void setInfoString(const std::string &str);
    void stopThread();
    void startThread();

   float  cameraCalibrate(std::vector<std::vector<aruco::Marker> >  &allMarkers, int imageWidth,int imageHeight,float markerSize,  aruco::MarkerMap &inmmap, aruco::CameraParameters&io);
    aruco::MarkerMap getDefaultCalibrationBoard();

};
}

#endif
