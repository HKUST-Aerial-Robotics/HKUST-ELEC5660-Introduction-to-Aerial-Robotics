// saves all the images of the dictionary indicated to a dicrectory
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


#include "dictionary.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
using namespace std;
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
    {
        CmdLineParser cml(argc, argv);
        if (argc == 1)
        {
            cerr << "Usage:  outdir   dictionary  [ -s bit_image_size: 75 default]  " << endl;
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
        aruco::Dictionary dict = aruco::Dictionary::load(argv[2]);
        int pixSize = stoi(cml("-s", "75"));

        string dict_name = dict.getName();
        std::transform(dict_name.begin(), dict_name.end(), dict_name.begin(), ::tolower);
        //
        for (auto m : dict.getMapCode())
        {
            string number = std::to_string(m.second);
            while (number.size() != 5)
                number = "0" + number;
            stringstream name;
            name << argv[1] << "/" + dict_name + "_" << number << ".png";
            cout << name.str() << endl;
            cv::imwrite(name.str(), dict.getMarkerImage_id(m.second, pixSize,true,false,true));
        }
    }
    catch (std::exception& ex)
    {
        cerr << ex.what() << endl;
    }
}
