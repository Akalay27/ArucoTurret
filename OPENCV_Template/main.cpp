
#include <stdio.h>
#include <string.h>
#include <errno.h>

#if defined(_WIN64)
#include "opencv2\core.hpp"
#include "opencv2\imgcodecs.hpp"
#include "opencv2\imgproc.hpp"
#include "opencv2\highgui.hpp"
#include "opencv2\aruco.hpp"
#include "opencv2\calib3d.hpp"
bool pi = 0;
#elif defined(__linux__) || defined(__unix__)
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/calib3d.hpp"
#include <wiringPi.h>
#include <wiringSerial.h>
bool pi = 1;
#endif

#include <sstream>
#include <iostream>
#include <fstream>
#include <vector>
using namespace std;
using namespace cv;

const float calibrationSquareDimension = 0.02553f;  // Need to test this
const float arucoSquareDimension = 0.076f; // need to test this
const Size  chessboardDimensions = Size(6, 9);

const int cupMarkerId = 5;
const float gravitationalConstant = 9.80665f;
const float speed = 5;  // need to determine

const Vec3d cameraOffset = { 0,0,0 };

const float zRotAdjust = 0.5;
void createArucoMarkers() {

	Mat outputMarker;

	Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);

	for (int i = 0; i < 50; i++) {
		aruco::drawMarker(markerDictionary, i, 500, outputMarker, 1);
		ostringstream convert;
		string imageName = "4x4Marker_";
		convert << "markers/" << imageName << i << ".jpg";
		imwrite(convert.str(), outputMarker);
	}


}

void createKnownBoardPosition(Size boardSize, float squareEdgeLength, vector<Point3f>& corners) {
	for (int i = 0; i < boardSize.height; i++)
	{
		for (int j = 0; j < boardSize.width; j++)
		{
			corners.push_back(Point3f(j*squareEdgeLength, i * squareEdgeLength, 0.0f));

		}
	}
}

void getChessboardCorners(vector<Mat> images, vector<vector<Point2f>>& allFoundCorners, bool showResults = false)
{
	for (vector<Mat>::iterator iter = images.begin(); iter != images.end(); iter++)
	{
		vector<Point2f> pointBuf; // point buffer
		bool found = findChessboardCorners(*iter, Size(9, 6), pointBuf, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

		if (found)
		{
			allFoundCorners.push_back(pointBuf);
		}
		if (showResults)
		{
			drawChessboardCorners(*iter, Size(9, 6), pointBuf, found);
			imshow("Looking for Corners", *iter);
			waitKey();
		}
	}
}

void cameraCalibration(vector<Mat> calibrationImages, Size boardSize, float squareEdgeLength, Mat& cameraMatrix, Mat& distanceCoefficient)
{
	vector<vector<Point2f>> checkerboardImageSpacePoints;
	getChessboardCorners(calibrationImages, checkerboardImageSpacePoints, false);

	vector<vector<Point3f>> worldSpaceCornerPoints(1);

	createKnownBoardPosition(boardSize, squareEdgeLength, worldSpaceCornerPoints[0]);

	worldSpaceCornerPoints.resize(checkerboardImageSpacePoints.size(), worldSpaceCornerPoints[0]);

	vector<Mat> rVectors, tVectors;
	distanceCoefficient = Mat::zeros(8, 1, CV_64F);

	calibrateCamera(worldSpaceCornerPoints, checkerboardImageSpacePoints, boardSize, cameraMatrix, distanceCoefficient, rVectors, tVectors);

}

double determineZRot(Vec3d MarkerPos) {  // Find nema17 rotation pretty much

	return atan2(MarkerPos[0], MarkerPos[2])/3.14156265359*180*-1; // Marker Z and X

}

double determineTrajectoryAngle(Vec3d MarkerPos, float Grav, float Speed) // calculate angle to hit target WITHOUT DRAG!!
{
	double x = sqrt(pow(MarkerPos[2], 2) + pow(MarkerPos[0], 2));
	double y = MarkerPos[1];
	double angle1 = atan((pow(Speed, 2) + sqrt(pow(Speed, 4) - Grav * (Grav*pow(x, 2) + 2 * pow(Speed, 2)*y))) / Grav * x); // From https://blog.forrestthewoods.com/solving-ballistic-trajectories-b0165523348c

	double angle2 = atan((pow(Speed, 2) - sqrt(pow(Speed, 4) - Grav * (Grav*pow(x, 2) + 2 * pow(Speed, 2)*y))) / Grav * x);
	angle1 = angle1 / 3.14159265359 * 180;
	angle2 = angle2 / 3.14159265359 * 180;


	if (!isnan(angle2)) {
		return angle2;
	}
	else {
		return 0;
	}




}

int startWebcamMonitoring(const Mat& cameraMatrix, const Mat& distanceCoefficient, float arucoSquareDimensions, Vec3d& cupPos)      
{
	Mat frame;
	int fd;
	string serialPort;
	cin >> serialPort;

	cout << "initializing Arduino..." << endl;
#if defined(__linux__) || defined(__unix__)
	fd = serialOpen(serialPort.c_str(), 9600);

#endif
	bool begun = false;
	while (begun == false) {
		vector<char> arduinoPacket;
		if (serialDataAvail(fd) > 0) {
			begun = true;
		}
		
	}
	vector<int> markerIds;
	vector<vector<Point2f>> markerCorners, rejectedCanidates;
	aruco::DetectorParameters parameters;

	Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);

	VideoCapture vid(0);

	if (!vid.isOpened()) {
		cout << "failed to initiate camera" << endl;
		return -1;
	}


	namedWindow("Webcam", WINDOW_AUTOSIZE);

	vector<Vec3d> rotationVectors, translationVectors;

	float prevZRot = 0;
	while (true) {
		if (!vid.read(frame)) {
			cout << "failed to initiate camera" << endl;
			break;
		}
		

		aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds);
		aruco::estimatePoseSingleMarkers(markerCorners, arucoSquareDimension, cameraMatrix, distanceCoefficient, rotationVectors, translationVectors);
		// single object with one aruco marker
		for (int m = 0; m < markerIds.size(); m++) {
			aruco::drawAxis(frame, cameraMatrix, distanceCoefficient, rotationVectors[m], translationVectors[m], 0.1f);

			//cout << translationVectors[m] << endl;
			if (markerIds[m] == cupMarkerId) {
				//cout << "found the cup!";
				cupPos = translationVectors[m];
				cupPos[0] -= 0.36;
				cupPos[1] -= 0.19;
				cupPos[1] *= -1;
			}
		}
		
		aruco::drawDetectedMarkers(frame, markerCorners, markerIds, 0.1f);
		
		cout << cupPos[0] << "/" << cupPos[1] << "/" << cupPos[2] << endl;
		
		//cout << "Cup is at " << cupPos << " relative to the camera." << endl;
		//cout << "Rotation of motors: " << determineZRot(cupPos) << " in y and " << determineTrajectoryAngle(cupPos,gravitationalConstant,speed) << " in x." << endl;
		string message;
		message = "90/90e";
		imshow("Webcam", frame);
		//	cout << serialDataAvail(fd) << endl;
		vector<char> arduinoPacket;
		while (serialDataAvail(fd) > 0) {
			arduinoPacket.push_back(serialGetchar(fd));
		}
		string packet(arduinoPacket.begin(), arduinoPacket.end());
		cout << packet << endl;

		serialPrintf(fd, "%f/%fe", determineTrajectoryAngle(cupPos, gravitationalConstant, speed), determineZRot(cupPos));
		//serialPuts(fd, message.c_str());

		cout << printf("%f/%f\n", determineTrajectoryAngle(cupPos, gravitationalConstant, speed), determineZRot(cupPos)) << endl;

		//Recieving handshake from Arduino:
		
		if (waitKey(30) >= 0) {

			serialClose(fd);

			break;
		}





	}
	return 1;
}

bool saveCameraCalibration(string name, Mat cameraMatrix, Mat distanceCoefficients)
{
	ofstream outStream(name);
	if (outStream)
	{
		uint16_t rows = cameraMatrix.rows;
		uint16_t columns = cameraMatrix.cols;

		outStream << rows << endl;
		outStream << columns << endl;

		for (int r = 0; r < rows; r++)
		{
			for (int c = 0; c < columns; c++)
			{
				double value = cameraMatrix.at<double>(r, c);
				outStream << value << endl;
			}
		}

		rows = distanceCoefficients.rows;
		columns = distanceCoefficients.cols;

		outStream << rows << endl;
		outStream << columns << endl;

		for (int r = 0; r < rows; r++)
		{
			for (int c = 0; c < columns; c++)
			{
				double value = distanceCoefficients.at<double>(r, c);
				outStream << value << endl;
			}
		}

		outStream.close();
		return true;
	}
	return false;
}

bool loadCameraCalibration(string name, Mat& cameraMatrix, Mat distanceCoefficients)
{
	ifstream inStream(name);
	if (inStream)
	{
		uint16_t rows;
		uint16_t columns;

		inStream >> rows;
		inStream >> columns;

		cameraMatrix = Mat(Size(columns, rows), CV_64F);

		for (int r = 0; r < rows; r++)
		{
			for (int c = 0; c < columns; c++) {
				double read = 0.0f;

				inStream >> read;
				cameraMatrix.at<double>(r, c) = read;
				cout << cameraMatrix.at<double>(r, c) << "\n";
			}
		}
		//distance coefficients

		inStream >> rows;
		inStream >> columns;

		distanceCoefficients = Mat::zeros(rows, columns, CV_64F);
		for (int r = 0; r < rows; r++)
		{
			for (int c = 0; c < columns; c++) {
				double read = 0.0f;
				inStream >> read;
				distanceCoefficients.at<double>(r, c) = read;
				cout << distanceCoefficients.at<double>(r, c) << "\n";
			}
		}
		inStream.close();
		return true;
	}
	return false;
}

void cameraCalibrationProcess(Mat& cameraMatrix, Mat& distanceCoefficients) {
	Mat frame;
	Mat drawToFrame;

	vector<Mat> savedImages;

	vector<vector<Point2f>> markerCorners, rejectedCanidates; // all the points on checkerboard

	VideoCapture vid(0);

	if (!vid.isOpened())
	{
		return;
	}

	int framesPerSecond = 20;

	namedWindow("Webcam", WINDOW_AUTOSIZE);

	while (true)
	{
		if (!vid.read(frame))
			break;
		

		vector<Vec2f> foundPoints;
		bool found = false;

		found = findChessboardCorners(frame, chessboardDimensions, foundPoints, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FAST_CHECK);
		frame.copyTo(drawToFrame);
		drawChessboardCorners(drawToFrame, chessboardDimensions, foundPoints, found);
		if (found) {
			imshow("Webcam", drawToFrame);
		}
		else {
			imshow("Webcam", frame);
		}
		char character = waitKey(1000 / framesPerSecond);

		switch (character)
		{
		case 32:
			//saving image
			if (found)
			{
				Mat temp;
				cout << "taken";
				frame.copyTo(temp);
				savedImages.push_back(temp);
			}
			break;

		case 13:
			//start calibration
			if (savedImages.size() > 15) {
				destroyAllWindows();
				cameraCalibration(savedImages, chessboardDimensions, calibrationSquareDimension, cameraMatrix, distanceCoefficients);
				saveCameraCalibration("Okaythen", cameraMatrix, distanceCoefficients);
				return;
			}
			break;

		case 27:
			//exit
			return;
			break;
		}
	}

	return;

}

void testSerial() {

	string port;
	cin >> port;

	int fd = serialOpen(port.c_str(), 9600);
	while (true) {
		string message;

		cin >> message;

		serialPuts(fd, message.c_str());
	}
}
void testTurret() {
	string port;
	cin >> port;

	int fd = serialOpen(port.c_str(), 9600);
	cout << "Turret Test" << endl << "type X, Y, and Z and magnitude. Press enter for each." << endl;
	cout << "X (left-right): ";
	float xPos;
	cin >> xPos;
	cout << "Y (up-down): ";
	float yPos;
	cin >> yPos;
	cout << "Z (forward-back): ";
	float zPos;
	cin >> zPos;
	cout << "Magnitude: ";
	float magnitude;
	cin >> magnitude;
	float yAngle = determineZRot(Vec3d(xPos, yPos, zPos));
	float xAngle = determineTrajectoryAngle(Vec3d(xPos, yPos, zPos), gravitationalConstant, magnitude);
	cout << "calculated " << yAngle << " degrees X and " << xAngle << " degrees Y" << endl;
	cout << "adjusting motors";

	serialPrintf(fd, "%f/%fe", xAngle,yAngle);
	string motorPos;
	while (motorPos != "0.00/0.00\n") {
		vector<char> arduinoPacket;
		while (serialDataAvail(fd) > 0) {
			arduinoPacket.push_back(serialGetchar(fd));
		}
		string packet(arduinoPacket.begin(), arduinoPacket.end());
		cout << packet << endl;

		serialPuts(fd, "de");
		delay(500);
	}
	cout << "Firing..." << endl;
}

int main(int argv, char** argc)
{
	Mat cameraMatrix = Mat::eye(3, 3, CV_64F);

	Mat distanceCoefficients;

	Vec3d cupPos;

	string input;
	cin >> input;

	if (input == "c") {
		cameraCalibrationProcess(cameraMatrix, distanceCoefficients);
		return 1;
	}
	if (input == "p") {
		createArucoMarkers();
	}
	if (input == "test") {
		testTurret();
	}
	else {
		wiringPiSetup();
		loadCameraCalibration("Okaythen", cameraMatrix, distanceCoefficients);
		startWebcamMonitoring(cameraMatrix, distanceCoefficients, arucoSquareDimension, cupPos);
	}






	return 0;
}