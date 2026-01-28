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

// Creation of MarkerMap to be printed in a piece of paper. This is the old boards in Aruco 1.x.x

#include "dictionary.h"
#include "markermap.h"
#include <cstdio>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <stdexcept>

using namespace std;

class CmdLineParser
{
    int argc;char** argv;public:
    CmdLineParser(int _argc, char** _argv): argc(_argc), argv(_argv){}
    bool operator[](string param)
    {int idx = -1;   for (int i = 0; i < argc && idx == -1; i++)if (string(argv[i]) == param) idx = i;return (idx != -1); }
    string operator()(string param, string defvalue = ""){int idx = -1;for (int i = 0; i < argc && idx == -1; i++)if (string(argv[i]) == param) idx = i;if (idx == -1) return defvalue;else return (argv[idx + 1]);}
};

std::vector<int> getIdList(string ids){
    if (ids.empty())return {};
    for(auto &c:ids) if (c==':') c=' ';
    stringstream sstr(ids);
    int val;
    std::vector<int> vids;
    while(!sstr.eof()){
        if (sstr>>val)
            vids.push_back(val);
    }
    return vids;
}

using namespace std;
using namespace cv;
int main(int argc, char** argv)
{
    try
    {
        CmdLineParser cml(argc, argv);
        if (argc < 4 || cml["-h"])
        {
            cerr << "Usage: X:Y Image.png Configuration.yml [-d dictionary_name (ARUCO_MIP_36h12 default)] [-s "
                    "<pixSize>] [-t <Type>(0: panel,1: chessboard )] [-r rand_seed] [-i interMarkerDistance(0,1)] [-ids n1:n2:... List the ids to be employed]"
                 << endl;
            cerr << "\tDictionaries: ";
            for (auto dict : aruco::Dictionary::getDicTypes())
                cerr << dict << " ";
            cerr << endl;
            return -1;
        }
        int XSize, YSize;
        if (sscanf(argv[1], "%d:%d", &XSize, &YSize) != 2)
        {
            cerr << "Incorrect X:Y specification" << endl;
            return -1;
        }

        auto Dict = aruco::Dictionary::loadPredefined(cml("-d", "ARUCO_MIP_36h12"));
        int pixSize = stoi(cml("-s", "500"));
        int typeMarkerMap = stoi(cml("-t", "0"));
        int rand_seed = stoi(cml("-r", "0"));
        float interMarkerDistance = stof(cml("-i", "0.2"));
        if ((interMarkerDistance > 1.f) || (interMarkerDistance < 0.f))
        {
            cerr << "Incorrect interMarkerDistance '" << interMarkerDistance << "' -- needs to be [0,1]" << endl;
            return -1;
        }

        // get the list of ids


        std::vector<int> ids=getIdList(cml("-ids"));
        if ( ids.size()==0){//no one specified, use random ones
            for (auto d : Dict.getMapCode())
                ids.push_back(d.second);
            // random shuffle
            srand(rand_seed);
            std::random_shuffle(ids.begin(), ids.end());
            cout<<ids.size()<<endl;
            ids.resize(XSize* YSize);
        }
        cout<<ids.size()<<endl;
        if ( size_t(XSize* YSize)!=ids.size())throw std::runtime_error("The number of ids is different from the one that are required");

        // take the required ones
        ids.resize(XSize * YSize);
        aruco::MarkerMap BInfo =
            Dict.createMarkerMap(Size(XSize, YSize), pixSize, static_cast<int>( float(pixSize) * interMarkerDistance), ids, typeMarkerMap == 1);
        // create a printable image to save
        cv::Mat MarkerMapImage = BInfo.getImage();

        // save
        BInfo.saveToFile(argv[3]);
        imwrite(argv[2], MarkerMapImage);
    }
    catch (std::exception& ex)
    {
        cout << ex.what() << endl;
    }
}
