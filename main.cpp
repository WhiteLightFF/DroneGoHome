#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <vector>
#include <iostream>
#include "GeoSnapLib.h"

//#include <gdal.h>
//#include <gdal_priv.h>




int main()
{

	
	int angle = START_ANGLE;
	double scale = START_SCALE;
	double fps, startFps;
	const int dAngle = 1;
	const double dScale = 0.01;
	cv::Point maxLoc;
	cv::Mat frame, frameProc, frameRender, mapPrepared, mapRender, map;
	std::vector<std::vector <double>> correlate(5, std::vector<double>(9, 0));
	cv::VideoCapture cap("C:/C++/Matching/Matching/src/frame.mp4");
	map = cv::imread("C:/C++/Matching/Matching/src/example_substrate.tif", 0);
	fps = cap.get(cv::CAP_PROP_FPS);
	// Подготовка картинки
	mapPrepared = MapPrepare(map);


	while (true)
	{
		startFps = cv::getTickCount();
		cap >> frame;
		double curFrame = cap.get(cv::CAP_PROP_POS_FRAMES);
		frameRender = frame.clone();
		
		frame = VideoPrepare(frame); //подготовка видео
		
		int numMatching = 0;
		for (int ang = angle - dAngle; ang <= angle + dAngle; ang += dAngle)
		{
			for (double scl = scale - dScale; scl <= scale + dScale; scl += dScale)
			{
				frameProc = ResizeImg(frame, scl);
				frameProc = RotScalePos(frameProc, 1, ang);
				correlate = SearchMatchParams(mapPrepared, frameProc, numMatching, correlate, ang, scl);
				numMatching++;
			}
		}

		int maxIndex = FindIdxMaxValCorr(correlate);
		angle = correlate.at(1).at(maxIndex);
		scale = correlate.at(2).at(maxIndex);
		maxLoc.x = correlate.at(3).at(maxIndex);
		maxLoc.y = correlate.at(4).at(maxIndex);
		
		Rendering(map, frameRender, scale, angle, maxLoc, startFps, curFrame);
		
		if (cv::waitKey(1) == 'q')
			break;
	}


	cap.release();
	cv::destroyAllWindows();

	return 0;
}


