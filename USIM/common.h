#pragma once
#ifndef __COMMON__
#define __COMMON__

#define CORNER_SIZE_X 9
#define CORNER_SIZE_Y 6
#define SUB_PIXEL_X 5
#define SUB_PIXEL_Y 5
#define BLOCK_SIZE 10.0
#define PRINT if(DISPLAY)cout
#define DISPLAY false

#include <string>
#include <iostream>
#include <fstream>
#include <assert.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
using namespace std;

#endif // !__COMMON__
