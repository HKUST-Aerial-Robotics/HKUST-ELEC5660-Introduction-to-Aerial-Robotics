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



#ifndef __OPENARUCO_CORE_TYPES_H__
#define __OPENARUCO_CORE_TYPES_H__

#if !defined _CRT_SECURE_NO_DEPRECATE && _MSC_VER > 1300
#define _CRT_SECURE_NO_DEPRECATE /* to avoid multiple Visual Studio 2005 warnings */
#endif


#if (defined WIN32 || defined _WIN32 || defined WINCE) && defined ARUCO_DSO_EXPORTS
#define ARUCO_EXPORT __declspec(dllexport)
#else
#define ARUCO_EXPORT
#endif


#endif
