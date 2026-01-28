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

// saves to file an image of the  indicated marker from the dictionary indicated

#include "aruco.h"
#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
using namespace std;

// convinience command line parser
class CmdLineParser
{
    int argc;
    char** argv;
public:
    CmdLineParser(int _argc, char** _argv)
          : argc(_argc)
          , argv(_argv)
    {
    }
    bool operator[](string param)
    {
        int idx = -1;
        for (int i = 0; i < argc && idx == -1; i++)
            if (string(argv[i]) == param)
                idx = i;
        return (idx != -1);
    }
    string operator()(string param, string defvalue = "-1")
    {
        int idx = -1;
        for (int i = 0; i < argc && idx == -1; i++)
            if (string(argv[i]) == param)
                idx = i;
        if (idx == -1)
            return defvalue;
        else
            return (argv[idx + 1]);
    }
};

int main(int argc, char** argv)
{
    try
    {        CmdLineParser cml(argc, argv);

        if (argc < 2)
        {
            // You can also use ids 2000-2007 but it is not safe since there are a lot of false positives.
            cerr << "Usage: <makerid> outfile.(jpg|png|ppm|bmp)  [options] \n\t[-e use enclsing corners]\n\t[-bs <size>:bit size in pixels. 50 by "
                    "default ] \n\t[-d <dictionary>: ARUCO_MIP_36h12 default] \n\t[-border: adds the white border around]"
                    "\n\t [-center:  highlights the center] "
                 << endl;
            auto dict_names = aruco::Dictionary::getDicTypes();
            cerr << "\t\tDictionaries: ";
            for (auto dict : dict_names)
                cerr << dict << " ";
            cerr << endl;
            cerr << "\t Instead of these, you can directly indicate the path to a file with your own generated "
                    "dictionary"
                 << endl;

            return -1;
        }
        int pixSize = std::stoi(cml("-bs", "75"));  // pixel size each each bit
        bool enclosingCorners = cml["-e"];
        bool waterMark = true;
        // loads the desired dictionary
        aruco::Dictionary dic = aruco::Dictionary::load(cml("-d", "ARUCO_MIP_36h12"));

        cv::imwrite(argv[2], dic.getMarkerImage_id(stoi(argv[1]), pixSize, waterMark, enclosingCorners,cml["-border"],cml["-center"]));
    }
    catch (std::exception& ex)
    {
        cout << ex.what() << endl;
    }
}
