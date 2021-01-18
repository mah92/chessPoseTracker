//ALLAH
//Inputs: ./calib.xml(copied from other program)
//	      images: line 200
#define IMAGES_ADDR "../CamSim/log/177"

enum {BOARD_UNKNOWN, BOARD_CHESSBOARD, BOARD_CIRCLES_GRID, BOARD_ASYMMETRIC_CIRCLES_GRID};
int board_type = BOARD_ASYMMETRIC_CIRCLES_GRID;
int board_w = 4; // Board width
int board_h = 11; // Board height
float squareSize = 350/2.;
//Important Note: In assymetric circular pattern, 
//square size is horizontal-vertical distance between circles in diagons,
//half the distance of circles in rows-cols

//Outputs: ./path.csv

// Unlike docs and experiment, javad said needs 3d locations, not works with 2d grids

#include <pugixml/pugixml.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "opencv2/opencv.hpp"
//#include "cv.h"
#include <iostream>
#include <fstream>
#include "MyUtility-Win.hpp"

#define TAG "ChessPoseTracker:"
using namespace cv;
using namespace std;

#define equals(variable, value, thresh)	\
	((((variable) - (value)) <= thresh && ((value) - (variable)) <= thresh)?1:0)

#define saturate(variable, minimum, maximum) \
	((variable < minimum)? minimum:((variable > maximum)? maximum: variable))

//also check min & max
#define saturate2(variable, minimum, maximum) \
		saturate(variable, nsrMin(minimum, maximum), nsrMax(minimum, maximum))

#define rescale(var, min, max, newmin, newmax) \
	((((newmax) - (newmin))/((max) - (min)))*((var) - (min)) + (newmin))
	//y=((y1-y0)/(x1-x0))*(x-x0)+y0;

cv::Point2f backproject3DPoint(const cv::Mat &R_matrix, const cv::Mat &t_matrix, cv::Point3f &point3d);

double normalize_angle(double angle){
	//return fmod( angle +M_PI, 2*M_PI ) - M_PI; //wrong
	while(angle > M_PI) angle -= 2*M_PI;
	while(angle <= -M_PI) angle += 2*M_PI;
	return angle;
}

//! [compute_errors]
//! [board_corners]
//in asymetric pattern, most negative point is the point opposite of text, x in the shortest direction towards text
//0,0,0 is center of picture(circles, and also image, but not image printed on A4)
static void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners,
									 int patternType = BOARD_CHESSBOARD)
{
    corners.clear();

    switch(patternType) {
			
	//original, switched x, y so that x, y become normal as other images and z becomes inward
	/*case BOARD_CHESSBOARD:
    case BOARD_CIRCLES_GRID:
		for( int i = 0; i < boardSize.height; i++ )
        	for( int j = 0; j < boardSize.width; j++ )
                corners.push_back(Point3f(float(j*squareSize),
                                          float(i*squareSize), 0));
        break;

    case BOARD_ASYMMETRIC_CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; i++ )
            for( int j = 0; j < boardSize.width; j++ )
                corners.push_back(Point3f(float((i % 2 + 2*j)*squareSize),
                                          float(i*squareSize), 0));
		break;
	*/
			
	//moved origin to corner-center of image
	case BOARD_CHESSBOARD:
    case BOARD_CIRCLES_GRID:
		for( int i = 0; i < boardSize.height; i++ )
        	for( int j = 0; j < boardSize.width; j++ )
                corners.push_back(Point3f(float(j*squareSize),
                                          float(i*squareSize), 0));
        break;

    case BOARD_ASYMMETRIC_CIRCLES_GRID: 
	//Important Note: In assymetric circular pattern, 
	//square size is horizontal-vertical distance between circles in diagons,
	//half the distance of circles in rows-cols
        for( int i = 0; i < boardSize.height; i++ )
            for( int j = 0; j < boardSize.width; j++ )
                //corners.push_back(Point3f(float((i % 2 + 2*j)*squareSize), float(i*squareSize), 0)); //corner(one of circles)
                //corners.push_back(Point3f(float((2*1.21875 + i % 2 + 2*j)*squareSize), float((2*1.58482 + i)*squareSize), 0)); //corner(opposite of text, image corner)
				corners.push_back(Point3f(float((-3.5 + i % 2 + 2*j)*squareSize), float((-5 + i)*squareSize), 0)); //center
		break;
			
    default:
		break;
    }
}

float param_max_lens_iterations, param_max_pix_error, param_resolution_scale, param_width, param_height, param_f, param_ox, param_oy, param_k1, param_k2, param_t1, param_t2, param_k3, param_k4, param_k5, param_k6;

int main(int argc, char** argv)
{
	Mat frame, frame2;
	Point QRpt;
	VideoCapture QRCap;
	Scalar color;
	vector<Point2f> image_points;
	vector<Point3f> objectPoints;

	char addr_str[100];
	FILE *pathFile;
	int i, j;
	bool found;
	int board_n;
	Mat cameraMatrix, distCoeffs;
	Mat rvec, tvec, tvec2;
	Mat CP2C(3, 3, CV_64FC1); //pattern(inertia) to camera = CI2C = CI2B
	Mat CC2P(3, 3, CV_64FC1); //camera to pattern(inertia) = CC2I = CB2I
	double s, phi, theta, psi;

	//converting to real trackable points
	if(board_type == BOARD_CHESSBOARD) {
		board_w--; board_h--;
	}

	board_n = board_w * board_h;
	CvSize board_sz = cvSize( board_w, board_h);

	LOGOPEN(".");

	LOGI(TAG, "In the name of ALLAH\n");

	namedWindow("im1",1);

	calcBoardCornerPositions(board_sz, squareSize, objectPoints, BOARD_ASYMMETRIC_CIRCLES_GRID);

	////////////////////////////////////////
	pugi::xml_parse_result result;
	pugi::xml_document calibDoc;

	result = calibDoc.load_file(std::string("./calib.xml").c_str());
	if(result) {
		LOGI(TAG, " calib.xml loaded successfully!(%s)\n", result.description());

		pugi::xml_node simParams = calibDoc.child("simParams");

		if(simParams.child("idealParams")) {
			double param_fov_x, param_fov_y;
			pugi::xml_node idealParams = simParams.child("idealParams");
			if(idealParams.attribute("resolutionScale")) param_resolution_scale = idealParams.attribute("resolutionScale").as_double();
			if(idealParams.attribute("width")) param_width = idealParams.attribute("width").as_int();
			if(idealParams.attribute("height")) param_height = idealParams.attribute("height").as_int();

			if(idealParams.attribute("oxOffset")) {
				param_ox = idealParams.attribute("oxOffset").as_double();
				param_ox+= (param_width-1.)/2.;
			}
			if(idealParams.attribute("oyOffset")) {
				param_oy = idealParams.attribute("oyOffset").as_double();
				param_oy+= (param_height-1.)/2.;
			}

			if(idealParams.attribute("ox")) {
				param_ox = idealParams.attribute("ox").as_double();
			}
			if(idealParams.attribute("oy")) {
				param_oy = idealParams.attribute("oy").as_double();
			}

			if(idealParams.attribute("fovX")) {
				param_fov_x = idealParams.attribute("fovX").as_double();
				param_f = ((param_width-1.)/2.)/tan(param_fov_x/2.*M_PI/180.);
			}
			if(idealParams.attribute("fovY")) {
				param_fov_y = idealParams.attribute("fovY").as_double();
				param_f = ((param_height-1.)/2.)/tan(param_fov_y/2.*M_PI/180.);
			}
			if(idealParams.attribute("f"))
				param_f = idealParams.attribute("f").as_double();

			param_width*=param_resolution_scale;
			param_height*=param_resolution_scale;
			param_ox*=param_resolution_scale;
			param_oy*=param_resolution_scale;
			param_f*=param_resolution_scale;

			LOGI(TAG, "idealParams: %f, %f, %f, %f, %f\n", param_width, param_height, param_ox, param_oy, param_f);
		}

		if(simParams.child("staticParams")) {
			pugi::xml_node staticParams = simParams.child("staticParams");
			if(staticParams.attribute("maxLensIterations")) param_max_lens_iterations = staticParams.attribute("maxLensIterations").as_int();
			if(staticParams.attribute("maxPixError")) param_max_pix_error = staticParams.attribute("maxPixError").as_double();
			if(staticParams.attribute("k1")) param_k1 = staticParams.attribute("k1").as_double();
			if(staticParams.attribute("k2")) param_k2 = staticParams.attribute("k2").as_double();
			if(staticParams.attribute("t1")) param_t1 = staticParams.attribute("t1").as_double();
			if(staticParams.attribute("t2")) param_t2 = staticParams.attribute("t2").as_double();
			if(staticParams.attribute("k3")) param_k3 = staticParams.attribute("k3").as_double();
			if(staticParams.attribute("k4")) param_k4 = staticParams.attribute("k4").as_double();
			if(staticParams.attribute("k5")) param_k5 = staticParams.attribute("k5").as_double();
			if(staticParams.attribute("k6")) param_k6 = staticParams.attribute("k6").as_double();
			LOGI(TAG, "staticParams: %i, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", param_max_lens_iterations, param_max_pix_error, param_k1, param_k2, param_t1, param_t2, param_k3, param_k4, param_k5, param_k6);
		}


	} else {
		LOGE(TAG, " Parameters.xml loaded with error: %s, using defaults...\n", result.description());
	}


	cameraMatrix = Mat::eye(3, 3, CV_64F);
	cameraMatrix.at<double>(0,0) = param_f;
	cameraMatrix.at<double>(1,1) = param_f;
	cameraMatrix.at<double>(0,2) = param_ox;
	cameraMatrix.at<double>(1,2) = param_oy;
	cameraMatrix.at<double>(2,2) = 1.0;

	distCoeffs = Mat::zeros(8, 1, CV_64F);
	distCoeffs.at<double>(0,0) = param_k1;
	distCoeffs.at<double>(0,1) = param_k2;
	distCoeffs.at<double>(0,2) = param_t1;
	distCoeffs.at<double>(0,3) = param_t2;
	distCoeffs.at<double>(0,4) = param_k3;
	distCoeffs.at<double>(0,5) = param_k4;
	distCoeffs.at<double>(0,6) = param_k5;
	distCoeffs.at<double>(0,7) = param_k6;

	//clean file
	pathFile = fopen("./path.csv", "w");
	fclose(pathFile);
	
	for(i=0; ;i++ ) {
		//im1//////////////////////////////////////////////
		image_points.clear();
		sprintf(addr_str, IMAGES_ADDR"/screenshot%05d.bmp", i);
		frame = imread(addr_str);
		if (!frame.data) {printf(" before:%i!\n", i); break;}
		//cvtColor(frame, frame2, CV_RGB2GRAY);

		switch(board_type) {
		case BOARD_CHESSBOARD:
			//found = findChessboardCorners(QRColorFrame, board_sz, QRMatchLoc, CALIB_CB_FAST_CHECK);
			found = findChessboardCorners( frame, board_sz, image_points,
										   CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
			break;
		case BOARD_CIRCLES_GRID:
			found = findCirclesGrid( frame, board_sz, image_points );
			break;
		case BOARD_ASYMMETRIC_CIRCLES_GRID:
			found = findCirclesGrid( frame, board_sz, image_points, CALIB_CB_ASYMMETRIC_GRID );
			break;
		}

		if(!found || image_points.size()!=board_n) {//not found correct number of points, really needed
			printf("%i->chess not found!\n", i);
			
			pathFile = fopen("./path.csv", "a");
			fprintf(pathFile, "%f, , , , , , \n", i*1.);
			fclose(pathFile);
			continue;
		}

		for(j = 0; j < board_n; j++){
			color=Scalar(255-255*j/board_n, 255*j/board_n, 0);
			QRpt = Point(image_points[j].x, image_points[j].y);
			circle(frame, QRpt, 3, color, CV_FILLED, 8, 0);
		}

		//PnP
		solvePnP(objectPoints, image_points,
				 cameraMatrix, distCoeffs,
				 rvec, tvec);

		//printf("(%f, %f, %f), (%f, %f, %f)\n", tvec.at<double>(0,0), tvec.at<double>(0,1), tvec.at<double>(0,2),
		//	rvec.at<double>(0,0), rvec.at<double>(0,1), rvec.at<double>(0,2));

		/////////////////////////////////////////////////////////////
		//rvec & tvec are parts of 4x4 homgenuous matrix transferring from world to cam coordinates
		//so rvec:=CP2C, tvec:=(Ptrn_origin - Cam_origin)| in cam coords (cam origin at center of lens)
		//Verified by experiment
		//see: https://docs.opencv.org/3.1.0/dc/d2c/tutorial_real_time_pose.html
		//     https://github.com/opencv/opencv/blob/master/samples/cpp/tutorial_code/calib3d/real_time_pose_estimation/src/PnPProblem.cpp
		//note: "Learning OpenCV3" by Bradsky is misleading!!!			
		
		Rodrigues(rvec, CP2C);

		//convert CI2B to ZYX euler angles, verified
		// gamma:phi:eu[0], beta: theta:eu[1], alpha:psi:eu[2],
		s = sqrt(pow(CP2C.at<double>(0,0), 2) + pow(CP2C.at<double>(0, 1), 2));
		if(s < 1e-3) {
			phi = atan2(CP2C.at<double>(1, 0), CP2C.at<double>(1, 1));
			theta = M_PI/2;
			psi = 0;
		} else {
			phi = atan2(CP2C.at<double>(1, 2), CP2C.at<double>(2, 2));
			theta = atan2(-CP2C.at<double>(0, 2), s);
			psi = atan2(CP2C.at<double>(0, 1), CP2C.at<double>(0, 0));
		}

		phi = normalize_angle(phi);
		theta = normalize_angle(theta);
		psi = normalize_angle(psi);
		//printf("%f, %f, %f\n", phi*180./M_PI, theta*180./M_PI, psi*180./M_PI);

		CC2P = CP2C.t();
		
		//////////////////////////////////////////////////////
		tvec2 = (CC2P*(-tvec)); //convert to cam in pattern
		
		pathFile = fopen("./path.csv", "a");
		fprintf(pathFile, "%f, %f, %f, %f, %f, %f, %f\n",
				i*1., tvec2.at<double>(0,0), tvec2.at<double>(0,1), tvec2.at<double>(0,2),
					phi*180./M_PI, theta*180./M_PI, psi*180./M_PI);
		printf("%f, %f, %f, %f, %f, %f, %f\n",
				i*1., tvec2.at<double>(0,0), tvec2.at<double>(0,1), tvec2.at<double>(0,2),
					phi*180./M_PI, theta*180./M_PI, psi*180./M_PI);
		fclose(pathFile);
		
		//verify//////////////////////////////////////////////
		cv::Point3f pt_in_pattern;
		cv::Point2f pt;
		int k,i,j;
		int steps = 10;
		for(k=0;k<=4*steps;k++) {
			if     (k<=10) { i = 0;         j = k;         }
			else if(k<=20) { i = k-10;      j = 10;        }
			else if(k<=30) { i = 10;        j = 10-(k-20); }
			else if(k<=40) { i = 10-(k-30); j = 0;         }
			else break;
				
			pt_in_pattern.x = rescale(i,0,10, -3.5*squareSize, 3.5*squareSize);
			pt_in_pattern.y = rescale(j,0,10, -5*squareSize, 5*squareSize);
			//pt_in_pattern.x = 3.5*squareSize;
			//pt_in_pattern.y = 5*squareSize;
			pt_in_pattern.z = 0;
			pt = backproject3DPoint(CP2C, tvec, pt_in_pattern);
			circle(frame, pt, 5, Scalar(0,255,0), -1);
		}
		//////////////////////////////////////////////////////
		imshow("im1",frame);
		if(waitKey(250) >= 0) break;
	}

	LOGDUMP();
	LOGCLOSE();
	return 0;
}

cv::Point2f backproject3DPoint(const cv::Mat &R_matrix, const cv::Mat &t_matrix, cv::Point3f &point3d)
{
	// Rotation-Translation Matrix Definition  
	cv::Mat _P_matrix = cv::Mat::zeros(3, 4, CV_64FC1);   // rotation-translation matrix
	_P_matrix.at<double>(0,0) = R_matrix.at<double>(0,0);
	_P_matrix.at<double>(0,1) = R_matrix.at<double>(0,1);
	_P_matrix.at<double>(0,2) = R_matrix.at<double>(0,2);
	_P_matrix.at<double>(1,0) = R_matrix.at<double>(1,0);
	_P_matrix.at<double>(1,1) = R_matrix.at<double>(1,1);
	_P_matrix.at<double>(1,2) = R_matrix.at<double>(1,2);
	_P_matrix.at<double>(2,0) = R_matrix.at<double>(2,0);
	_P_matrix.at<double>(2,1) = R_matrix.at<double>(2,1);
	_P_matrix.at<double>(2,2) = R_matrix.at<double>(2,2);
	_P_matrix.at<double>(0,3) = t_matrix.at<double>(0);
	_P_matrix.at<double>(1,3) = t_matrix.at<double>(1);
	_P_matrix.at<double>(2,3) = t_matrix.at<double>(2);
	
	//cv::Point3f point3d;
	//point3d.x = 0; point3d.y = 0; point3d.z = 0;
	//point3d.x = 3.5*squareSize; point3d.y = 5*squareSize; point3d.z = 0;
	
	// 3D point vector [x y z 1]'
	cv::Mat point3d_vec = cv::Mat(4, 1, CV_64FC1);
	point3d_vec.at<double>(0) = point3d.x;
	point3d_vec.at<double>(1) = point3d.y;
	point3d_vec.at<double>(2) = point3d.z;
	point3d_vec.at<double>(3) = 1;
	
	//cv::Mat point2d_vec = cv::Mat(3, 1, CV_64FC1);
	//point2d_vec = _A_matrix * _P_matrix * point3d_vec;
	cv::Mat uv = cv::Mat(3, 1, CV_64FC1);
	uv = _P_matrix * point3d_vec;
	
	float u = uv.at<double>(0)/uv.at<double>(2);
	float v = uv.at<double>(1)/uv.at<double>(2);
	float u2 = u*u;
	float v2 = v*v;
	float r2 = u2 + v2;
	float r4 = r2*r2;
	float r6 = r4*r2;
	float dr = 1 + param_k1*r2 + param_k2*r4 + param_k3*r6;
	
	float dt_x = 2*u*v*param_t1 + (r2+2*u2)*param_t2;
	float dt_y = 2*u*v*param_t2 + (r2+2*v2)*param_t1;

	cv::Mat uv_distort = cv::Mat(3, 1, CV_64FC1);
	uv_distort.at<double>(0) = dr*u + dt_x;
	uv_distort.at<double>(1) = dr*v + dt_y;
	uv_distort.at<double>(2) = 1;
	
	//printf("dr:%f, dt:(%f,%f)\n", dr, dt_x, dt_y);
	//cv::Mat uv_distort = uv;
	///////////////////
	cv::Mat _A_matrix = cv::Mat::zeros(3, 3, CV_64FC1);	 // intrinsic camera parameters
	_A_matrix.at<double>(0, 0) = param_f; //params[0];       //      [ fx   0  cx ]
	_A_matrix.at<double>(1, 1) = param_f; //params[1];       //      [  0  fy  cy ]
	_A_matrix.at<double>(0, 2) = param_ox;//params[2];       //      [  0   0   1 ]
	_A_matrix.at<double>(1, 2) = param_oy;//params[3];
	_A_matrix.at<double>(2, 2) = 1;

	cv::Mat point2d_vec = cv::Mat(3, 1, CV_64FC1);
	point2d_vec = _A_matrix * uv_distort;

	// Normalization of [u v]'
	cv::Point2f point2d;
	point2d.x = (float)(point2d_vec.at<double>(0) / point2d_vec.at<double>(2));
	point2d.y = (float)(point2d_vec.at<double>(1) / point2d_vec.at<double>(2));	
	return point2d;
}
