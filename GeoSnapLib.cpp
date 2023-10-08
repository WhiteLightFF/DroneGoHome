#include "GeoSnapLib.h"
#include "opencv2/opencv.hpp"




cv::Mat MapPrepare(cv::Mat mapFrame)
{
	mapFrame = ResizeImg(mapFrame, RENDER_SCALE);
	Canny(mapFrame, mapFrame, 100, 200);
	//GaussianBlur(mapFrame, mapFrame, Size(5, 5), 0);
	return mapFrame;
}

cv::Mat VideoPrepare(cv::Mat videoFrame)
{
	cvtColor(videoFrame, videoFrame, cv::COLOR_BGR2GRAY);
	videoFrame = ResizeImg(videoFrame, RENDER_SCALE);
	GaussianBlur(videoFrame, videoFrame, cv::Size(5, 5), 0);
	Canny(videoFrame, videoFrame, 100, 200);
	GaussianBlur(videoFrame, videoFrame, cv::Size(5, 5), 0);
	return videoFrame;
}

cv::Mat ResizeImg(cv::Mat img, double coeff)
{
	int w = int(img.cols * coeff);
	int h = int(img.rows * coeff);
	resize(img, img, cv::Size(w, h), cv::INTER_LINEAR);
	return img;
}

cv::Mat RotScalePos(cv::Mat& source, double resize_scale, int ang)
{
	double angle = ang * (M_PI / 180.0);
	int height = source.rows;
	int width = source.cols;

	int new_width = int(height * abs(sin(angle)) + width * abs(cos(angle)));
	int new_height = int(height * abs(cos(angle)) + width * abs(sin(angle)));

	cv::Point2f center(width / 2, height / 2);
	cv::Mat rot_mat_source = getRotationMatrix2D(center, ang, resize_scale);
	rot_mat_source.at<double>(0, 2) += (new_width - width) / 2;
	rot_mat_source.at<double>(1, 2) += (new_height - height) / 2;
	cv::Mat affine;
	warpAffine(source, affine, rot_mat_source, cv::Size(new_width, new_height), cv::INTER_CUBIC);
	return affine;
}
std::vector<std::string>GetTelemetry(cv::Point maxLoc, cv::Mat outFrame, cv::Mat source, float renderScale, double scale)
{
	std::vector<std::string> telemetry;
	int xPos = int((maxLoc.x + (outFrame.cols / 4)) / renderScale);
	int yPos = int((maxLoc.y + (outFrame.rows / 4)) / renderScale);

	double deltaX = abs(TOP_LEFT_LON - BOTTOM_RIGHT_LON);
	double deltaXLon = (deltaX * xPos) / (source.cols / renderScale);

	double deltaY = abs(TOP_LEFT_LAT - BOTTOM_RIGHT_LAT);
	double deltaYLat = (deltaY * yPos) / (source.rows / renderScale);



	int	   lat_deg = floor(TOP_LEFT_LAT - deltaYLat);
	double lat_min = (TOP_LEFT_LAT - lat_deg - deltaYLat) * 60;
	double lat_sec = (lat_min - floor(lat_min)) * 60;

	int    lon_deg = floor(TOP_LEFT_LON + deltaXLon);
	double lon_min = (TOP_LEFT_LON - lon_deg + deltaXLon) * 60;
	double lon_sec = (lon_min - floor(lon_min)) * 60;

	int alt = HEIGHT_MAP * scale;

	std::string  lat_str = "N " + std::to_string(lat_deg) + " " + std::to_string(int(lat_min)) + "' " + std::to_string(int(lat_sec)) + "''";
	std::string  lon_str = "E " + std::to_string(lon_deg) + " " + std::to_string(int(lon_min)) + "' " + std::to_string(int(lon_sec)) + "''";
	std::string  xPos_str = "xPos: " + std::to_string(xPos) + " px";
	std::string  yPos_str = "yPos: " + std::to_string(yPos) + " px";
	std::string  alt_str = "Alt: " + std::to_string(alt) + " m";
	telemetry.push_back(lat_str);
	telemetry.push_back(lon_str);
	telemetry.push_back(xPos_str);
	telemetry.push_back(yPos_str);
	telemetry.push_back(alt_str);


	return telemetry;
}

void showInfo(std::vector<std::string> info, cv::Mat& img)
{
	for (int i = 0; i < info.capacity(); i++)
		putText(img, info[i], cv::Point(10, 50 + 20 * i), cv::QT_FONT_NORMAL, 0.5, cv::Scalar(255), 1);
}


cv::Mat BlendModeAlpha(cv::Mat& imgReciever, cv::Mat& alphaChImage, cv::Point maxLoc)
{

	for (int y = 0; y < alphaChImage.rows; y++)
	{
		for (int x = 0; x < alphaChImage.cols; x++)
		{
			if (alphaChImage.at<uchar>(y, x))
				imgReciever.at<uchar>(maxLoc.y + y, maxLoc.x + x) = alphaChImage.at<uchar>(y, x);
		}
	}

	return imgReciever;
}



std::vector<std::vector <double>> SearchMatchParams(cv::Mat& img, cv::Mat& templ, int count, std::vector<std::vector <double>> params, int ang, double scl)
{
	cv::Mat matching;
	double min_val, max_val;
	cv::Point min_loc, maxLoc;
	cv::matchTemplate(img, templ, matching, cv::TM_CCOEFF_NORMED);
	minMaxLoc(matching, &min_val, &max_val, &min_loc, &maxLoc);
	params.at(0).at(count) = max_val;
	params.at(1).at(count) = ang;
	params.at(2).at(count) = scl;
	params.at(3).at(count) = maxLoc.x;
	params.at(4).at(count) = maxLoc.y;

	return params;
}

int FindIdxMaxValCorr(std::vector<std::vector <double>> params)
{
	auto max_iterator = max_element(params[0].begin(), params[0].end());
	int max_index;
	if (max_iterator != params[0].end())
		return distance(params[0].begin(), max_iterator);

}

void Rendering(cv::Mat map, cv::Mat frameRender, double scale, int angle, cv::Point maxLoc, double startFps, int curFrame)
{
	map = map.clone();
	map = ResizeImg(map, RENDER_SCALE);
	cvtColor(frameRender, frameRender, cv::COLOR_BGR2GRAY);
	frameRender = ResizeImg(frameRender, RENDER_SCALE);
	frameRender = ResizeImg(frameRender, scale);
	frameRender = RotScalePos(frameRender, 1, angle);
	frameRender = BlendModeAlpha(map, frameRender, maxLoc);
	double end = cv::getTickCount();
	double elapsed_time = (end - startFps) / cv::getTickFrequency();
	double current_fps = 1.0 / elapsed_time;
	std::vector<std::string> telemetry = GetTelemetry(maxLoc, frameRender, map, RENDER_SCALE, scale);
	putText(map, "FPS: " + std::to_string(int(current_fps)), cv::Point(10, 30), cv::QT_FONT_NORMAL, 0.5, cv::Scalar(255), 1);
	showInfo(telemetry, map);
	imshow("Render", map);
	std::string fileName = "C:/C++/Matching/Matching/out_" + std::to_string(curFrame) + ".jpg";
	imwrite(fileName, map);
	
}

