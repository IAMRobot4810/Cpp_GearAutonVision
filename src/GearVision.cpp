/*
 * GearVision.cpp
 *
 *  Created on: Apr 13, 2017
 *      Author: 1750800446
 */

#ifndef SRC_GEARVISION_CPP_
#define SRC_GEARVISION_CPP_

#include <iostream>
#include <memory>
#include <string>
#include <thread>

//#include "vision/VisionRunner.h"
#include <WPILib.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
//using namespace cv;
//using namespace cs;

class GearVision {
private:
    static const int WIDTH = 640;
    static const int HEIGHT = 480;
    static const int DISTANCE_CONSTANT = unKnown;
    static const int DROP_DISTANCE_CONSTANT = unKnown;

    static const float ROTATE_NUDGE_SPEED = -0.25;
    static const float ROTATE_NUDGE_TIME = 0.2;
    static const float STRAIGHT_NUDGE_SPEED = -.025;
    static const float STRAIGHT_NUDGE_TIME = 0.2;
    static const float BACKUP_SPEED = 0.25;
    static const float BACKUP_TIME = 0.35;
    static const float GEAR_DROP_DELAY = 0.5;

    static constexpr int lowerGreen[3] = {75, 30, 150};
    static constexpr int upperGreen[3] = {95, 240, 255};

    static bool visionDone = false;

    struct ContourPair {
    	vector<cv::Point> biggestContour, secondBiggestContour;
    };

    static bool aboutTheSame(int val1,int val2, float diffPercent = .10) {
        return ( (val1 * (1 - diffPercent)) <= val2 ) && ( val2 <= (val1 * (1 + diffPercent)) );
    }

    static bool compareContourAreas (vector<cv::Point> contour1, vector<cv::Point> contour2) {
        double i = fabs(contourArea(cv::Mat(contour1)));
        double j = fabs(contourArea(cv::Mat(contour2)));
        return (i > j);
    }

    static cv::Mat processImg(cv::Mat source) {
		cv::Mat blur, hsv, mask, match, gray, grayBlur;

		// Double blurring to ensure there are not disconnected areas

		// Blur the image so imperfections in field tape are not noticeable
		bilateralFilter(source, blur, 9, 150, 150);
		// Convert the color-system to a more computer friendly one
		cvtColor(source, hsv, cv::COLOR_BGR2HSV);
		// filter out all colors that aren't in the lowerGreen to upperGreen range
		inRange(hsv, cv::Scalar(lowerGreen[0], lowerGreen[1], lowerGreen[2]), cv::Scalar(upperGreen[0], upperGreen[1], upperGreen[2]), mask);
		// apply this filter to the image
		bitwise_and(source, source, match, mask);
		// strip out colors as they are no longer needed
		cvtColor(match, gray, cv::COLOR_BGR2GRAY);
		// blur again so that imperfections are minimized
		bilateralFilter(gray, grayBlur, 9, 150, 150);

		return grayBlur;
    }
    static ContourPair getTwoBiggestContours(vector<vector<cv::Point>> contours) {
    	if(contours.size() < 2)
    		throw std::invalid_argument( "NOT ENOUGH CONTOURS FOUND" );
    	// sorts from biggest to smallest
    	std::sort(contours.begin(), contours.end(), compareContourAreas);

    	ContourPair twoBiggestContours;
    	twoBiggestContours.biggestContour = contours[0];
    	twoBiggestContours.secondBiggestContour = contours[1];

    	return twoBiggestContours;
    }
    // returns in form Point(minX, maxX)
    static cv::Point getMinMaxX(vector<cv::Point> contour) {
    	int minX = contour[0].x, maxX = contour[0].x;
    	for(cv::Point pt : contour) {
    		if(pt.x < minX)
    			minX = pt.x;
    		else if(pt.x > maxX)
    			maxX = pt.x;
    	}
    	return cv::Point(minX, maxX);
    }
    /** "S: #" means straight # seconds
     * "L" means Left
     * "R" means Right
     * "D" means Drop
    */
    static void decideWhereToGo(DriveSystem* dS, GearSystem* gS, cv::Point minMaxXs[3]) {
    	int leftWidth, rightWidth, centerWidth;

    	leftWidth = abs(minMaxXs[0].y - minMaxXs[0].x);
    	rightWidth = abs(minMaxXs[1].y - minMaxXs[1].x);
    	centerWidth = ( abs(minMaxXs[1].x - minMaxXs[0].x) + abs(minMaxXs[1].y - minMaxXs[0].y) ) / 2;

    	// S
    	if(aboutTheSame(leftWidth, rightWidth)) {
    		double distance = DISTANCE_CONSTANT / centerWidth;

    		// D
    		if(distance <= DROP_DISTANCE_CONSTANT){
    			gS->openClaw();
    			Wait(GEAR_DROP_DELAY);
    			gS->lowerClaw();

    			dS->TimeStraightDrive(BACKUP_SPEED, BACKUP_TIME);
    			visionDone = true;
    			return;
    		}
    		else {
    			dS->TimeStraightDrive(STRAIGHT_NUDGE_SPEED, STRAIGHT_NUDGE_TIME);
    		}
    	} // L
    	else if (leftWidth > rightWidth) {
    		dS->TankControllerDrive(ROTATE_NUDGE_SPEED, 0.0);
    		Wait(ROTATE_NUDGE_TIME);
    	} // R
    	else {
    		dS->TankControllerDrive(0.0, ROTATE_NUDGE_SPEED);
    		Wait(ROTATE_NUDGE_TIME);
    	}
    	dS->TankControllerDrive(0.0, 0.0);
    }
public:
	GearVision(){

	}
	virtual ~GearVision(){

	}

    //enum Position {LEFT = 1 , CENTER = 2 , RIGHT = 3};

    // Do threads have to call a static function?
    static void VisionThread(DriveSystem* dS, GearSystem* gS, cs::UsbCamera camera/*, Position pos*/) {


		camera.SetResolution(WIDTH, HEIGHT);
		cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();
		cv::Mat source;
		vector<vector<cv::Point>> contours;
		vector<cv::Vec4i> hierarchy; // placeholder
		while(!visionDone) {
			// process the source frame
			cv::Mat output = processImg(source), canny;

			// get the edges of these areas that are the correct color
			Canny(output, canny, 50.0, 150.0, 3, false);
			findContours(canny, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

			ContourPair twoBiggestContours = getTwoBiggestContours(contours);

			// 'x' == minX, 'y'== maxX
			cv::Point minMaxXs[3];
			minMaxXs[0] = getMinMaxX(twoBiggestContours.biggestContour);
			minMaxXs[1] = getMinMaxX(twoBiggestContours.secondBiggestContour);


			// ensure that 'left' actually IS 'left'
			if(minMaxXs[1].x < minMaxXs[0].x) {
				minMaxXs[2] = minMaxXs[0];
				minMaxXs[0] = minMaxXs[1];
				minMaxXs[1] = minMaxXs[2];
			}
			decideWhereToGo(dS, gS, minMaxXs);

		}
	}

//}
};

#endif /* SRC_GEARVISION_CPP_ */
