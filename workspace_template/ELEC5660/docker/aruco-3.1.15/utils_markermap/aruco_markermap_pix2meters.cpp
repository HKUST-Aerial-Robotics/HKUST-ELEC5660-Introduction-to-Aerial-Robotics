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

// This program converts a boardconfiguration file expressed in pixel to another one expressed in meters
#include "markermap.h"
#include <iostream>
using namespace std;
using namespace aruco;
int main(int argc, char** argv)
{
    try
    {
        if (argc < 4)
        {
            cerr << "Usage:  in_boardConfiguration.yml markerSize_meters out_boardConfiguration.yml" << endl;
            return -1;
        }
        aruco::MarkerMap BInfo;
        BInfo.readFromFile(argv[1]);
        BInfo.convertToMeters(static_cast<float>(atof(argv[2]))).saveToFile(argv[3]);
    }
    catch (std::exception& ex)
    {
        cout << ex.what() << endl;
    }
}
