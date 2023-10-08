#pragma once
#include "opencv2/opencv.hpp"



#define START_ANGLE 42					//Degrees
#define START_SCALE 0.35				// scale / 100%


#define M_PI				3.141592653589793238
#define HEIGHT_MAP			400			//Meters
#define RENDER_SCALE		0.2			//  scale / 100%
#define TOP_LEFT_LAT		59.70832	//Degrees
#define TOP_LEFT_LON		30.08929	//Degrees

#define BOTTOM_RIGHT_LAT	59.70475	//Degrees
#define BOTTOM_RIGHT_LON	30.10209	//Degrees

cv::Mat MapPrepare(cv::Mat mapFrame);
cv::Mat VideoPrepare(cv::Mat videoFrame);
cv::Mat ResizeImg(cv::Mat img, double coeff);
cv::Mat RotScalePos(cv::Mat& source, double resize_scale, int ang);
std::vector<std::string>GetTelemetry(cv::Point maxLoc, cv::Mat outFrame, cv::Mat source, float renderScale, double scale);
void showInfo(std::vector<std::string> info, cv::Mat& img);
cv::Mat BlendModeAlpha(cv::Mat& imgReciever, cv::Mat& alphaChImage, cv::Point maxLoc);
std::vector<std::vector <double>> SearchMatchParams(cv::Mat& img, cv::Mat& templ, int count, std::vector<std::vector <double>> params, int ang, double scl);
void Rendering(cv::Mat map, cv::Mat frameRender, double scale, int angle, cv::Point maxLoc, double startFps, int curFrame);
int FindIdxMaxValCorr(std::vector<std::vector <double>> params);
void FindBestCorrParams(std::vector<std::vector <double>> params, int* angle, double* scale, cv::Point* maxLoc, int maxIndex);
