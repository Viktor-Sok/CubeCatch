#include <fstream>
#include <iostream>
#include <vector>
#include <algorithm>
#include <sstream>
#include <exception>
#include <nlohmann/json.hpp>
#include "CubeRecognition.h"
#include <cmath>
#include <iostream>
#include <iomanip>
using json = nlohmann::json; // synonim for data type nlohmann::json

const double PI = 3.141592653589793238463;
const int FPS = 60;
const double EXTR_SQUARE_SIZE = 44.5; // mm

CubeDetector::CubeDetector(const std::string& intrinsics_file_name, const std::string& extrinsics_file_name) {

    // read the intrinsics and ectrinsics from JSON 
    std::ifstream file(intrinsics_file_name);
    json intrinsics;
    if (file.is_open()) {
        intrinsics = json::parse(file)["intrinsics"];
    } else {
        std::cerr << "Intrinsics' file opening error" << std::endl;
    }
    file.close();
    
    json extrinsics;
    std::ifstream file1(extrinsics_file_name);
    if (file1.is_open()) {
        extrinsics = json::parse(file1)["extrinsics"];
    } else {
        std::cerr << "Extrinsics' file opening error" << std::endl;
    }
    file1.close();
    
    // Camera matrix
    std::vector<double> temp = intrinsics["K"];
    K = cv::Matx<double,1,9>(temp.data()).reshape<3,3>();
    // Inverse camera matrix
    Kinv = K.inv();
    // Distortion Coefficients
    std::vector<double> temp1 = intrinsics["distortion"];
    dist = cv::Vec<double,5>(temp1.data());
    // rvecs, tvecs
    std::vector<double> temp2 = extrinsics["rvecs"];
    rvecs = cv::Vec<double,3>(temp2.data());
    std::vector<double> temp3 = extrinsics["tvecs"];
    tvecs = cv::Vec<double,3>(temp3.data());
    // Inverse(transpose) rotation matrix
    cv::Matx<double,3,3> R;
    cv::Rodrigues(rvecs, R);
    Rinv = R.t();
}



std::tuple<cv::Vec2d, double> CubeDetector::getPositionAndOrientation(const cv::Vec3d& rw1, const cv::Vec3d& rw2) {
    cv::Vec3d diff = rw2 - rw1;
    double dist = std::abs(diff(0) - diff(1)) * EXTR_SQUARE_SIZE;
    if ( dist > 30 * std::sqrt(2) || dist < 30) {
        std::stringstream ss;
        ss << "Computer vision failed: " << "wrong distance between corners = " << dist;
        throw std::runtime_error(ss.str());
    }
    cv::Vec3d temp = (rw2 - diff/2);
    cv::Vec2d center(temp(0), temp(1)); // cartesian coordinates of the cube center
    double angle = std::atan(diff(1)/diff(0)) * 180 / PI;
    if (angle < 0.0) {
        return std::make_tuple(center, angle + 45);
    } else {
        return std::make_tuple(center, angle - 45);
    }
}

cv::Vec3d CubeDetector::getWorldCoordinates(cv::Vec2d px, double z) {
    // Mat intrinsics, distCoeffs; // K, dist
    // Transforming pixel coordinates to homogeneous coordinates
    cv::Vec<double,3> uvPoint; //u,v,1
    uvPoint(0) = px(0);
    uvPoint(1) = px(1);
    uvPoint(2) = 1.0;
    // Finding World Coordinates in the units of a checkerpattern square which was used for finding extrinsics
    cv::Vec<double,3> tempMat, tempMat2;
    double s;
    tempMat = Rinv * Kinv * uvPoint;
    tempMat2 = Rinv * tvecs;
    s = (z + tempMat2(2))/tempMat(2); // coefficient of proportionality
    //cv::Matx<double,1,3> tvecs1(tvecs(0), tvecs(1), tvecs(2));
    cv::Vec<double,3> rw = Rinv * (s * Kinv * uvPoint - tvecs); // cartesian vector of the world coordinates
    return rw; 
}




std::vector<cv::Vec2f> CubeDetector::getPixelCoordinates(cv::Mat& img) {
    // Loading and Preprocessing the image
    cv::Mat img_gray;
    cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);
    // Applying Harris Corrner Detector
    cv::Mat rscore; // result of cornerHarrisDetector
    cv::cornerHarris(img_gray, rscore, 3, 3, 0.04);
    cv::dilate(rscore, rscore, cv::Mat());
    // Thresholding R_score to find blobs of pixels where corners are located
    double minVal, maxVal;          
    cv::minMaxLoc(rscore, &minVal, &maxVal);
    cv::threshold(rscore, rscore, 0.1*maxVal, 255, cv::THRESH_BINARY); 
    rscore = cv::max(rscore, 0); // returns all teh values in rscere greater than 0
    rscore.convertTo(rscore, CV_8U); 
    // Finding centroids (pixel coordinates of the blob centers)
    cv::Mat labels, stats, centroids;
    int num_blobs = cv::connectedComponentsWithStats(rscore, labels, stats, centroids); // background also counts as a blob
    if (num_blobs != 3) {
        std::stringstream ss;
        ss << "Computer vision failed: " << "wrong number of recognized corners = " << num_blobs << " Expected: 2";
        throw std::runtime_error(ss.str());
    } 
    //std::cout << cv::typeToString(centroids.row(1).type()) << std::endl;
    //{cv::Point2f(710.5, 460.1), cv::Point2f(708.8, 537.3)};
    //std::vector<cv::Point2f> centers = {cv::Point2f(centroids.row(1)), cv::Point2f(centroids.row(2))}; // 0th row is the background
    std::vector<cv::Vec2f> centers = {cv::Vec2f(centroids.row(1)), cv::Vec2f(centroids.row(2))}; // 0th row is the background

    //cv::Mat centers = centroids.rowRange(1,3); // leave out the 0th background component 
    // Refining pixel coordinates of the corners
    cv::TermCriteria termCrit(cv::TermCriteria::MAX_ITER|cv::TermCriteria::EPS,100,0.01); // 100 iterations max and epsilon = 0.01
    cv::Size subPixWinSize(5,5); // size of window which looks for corners
    cv::cornerSubPix(img_gray, centers, subPixWinSize, cv::Size(-1,-1), termCrit); // centers should be of a loat type (e.g. Point2f, Vec2f))
    return centers;
}

std::tuple<cv::Vec2d, double> CubeDetector::detectCube(const bool Manual) {
    
    std::string cameraSource ="http://192.168.137.242:8080/video";
    cv::VideoCapture vid(cameraSource);
    
    // if (!vid.isOpened())
    //{
      //  std::cerr << "Error capturing the video" << std::endl;
        
    //} 
    /*
    cv::VideoCapture vid(1, cv::CAP_DSHOW); // 0-Webcam; 1-USB-connected camera
    vid.set(cv::CAP_PROP_FRAME_WIDTH, 1280); // set resolution
    vid.set(cv::CAP_PROP_FRAME_HEIGHT, 1024);
    */
    cv::namedWindow("Cube Detection", cv::WINDOW_AUTOSIZE);
    cv::Mat frame, undist_frame;
    std::tuple<cv::Vec2d, double> posAndOrient;
    cv::Vec2d center;
    double angle;
    while (Manual) {
        bool readable = vid.read(frame);
        if(!readable)
            break; // or error handling
        std::cout << "Start of while loop" << std::endl;
        cv::undistort(frame, undist_frame, K ,dist);
        std::vector<cv::Vec2f> px_coord = getPixelCoordinates(undist_frame); 
        cv::Point2i px1(px_coord.front()); // double implicetly converted to integer
        cv::Point2i px2(px_coord.back());
        cv::circle(undist_frame, px1, 5, cv::Scalar(0, 0, 255), 2);
        cv::circle(undist_frame, px2, 5, cv::Scalar(0, 0, 255), 2);
        cv::line(undist_frame, px1, px2, cv::Scalar(0, 0, 255), 2);
        // Getting world coordinates (z is assumed to be known, default z = 0)
        // 0.6666 is the height of the cube ib the units of extrinsics checkerboard square
        cv::Vec3d rw1 = getWorldCoordinates(px_coord.front(), - 0.6666); 
        cv::Vec3d rw2 = getWorldCoordinates(px_coord.back(), - 0.6666);
        // Finding center and the angle of the gripper in the world coordinates 
        posAndOrient = getPositionAndOrientation(rw1, rw2);
        // Drawing center on the frame and reprojection error of the corners
        
        
        std::tie(center, angle) = posAndOrient;
        cv::Vec3d center3D(center(0), center(1), -0.6666);
        // Object Points to be projected on the image and output array for Image Points
        std::vector<cv::Point3d> objPoints = {cv::Point3d(center3D), cv::Point3d(rw1), cv::Point3d(rw2)};
        std::vector<cv::Point2d> imgPoints(3); // vector of 3 pixel points
        // Preapring arrays for projectPoints function
        //cv::Mat center3D(3, 1, CV_32FC1, cv::Vec3d(center(0), center(1), -0.6666));// adding z=-0.66 coordinate in the units of the extrinsics checkerboard pattern square size
        //cv::Mat rw13D(3, 1, CV_32FC1, rw1);
        //cv::Mat rw23D(3, 1, CV_32FC1, rw2);
        //cv::Mat centerpx, rw1px, rw2px;
        cv::Vec<double,5> dist_empty(0,0,0,0,0); // projecting on undistorted frame so dist coef = 0
        // Projecting object poits onto the image
        cv::projectPoints(objPoints, rvecs, tvecs, K, dist_empty, imgPoints);

        //cv::projectPoints(rw13D, rvecs, tvecs, K, dist_empty, rw1px);
        //cv::projectPoints(rw23D, rvecs, tvecs, K, dist_empty, rw2px);
        // Converting Image Points from double to int
        std::for_each(imgPoints.begin(), imgPoints.end(), [](cv::Point2d &p){p = static_cast<cv::Point2i>(p);});
        // Drawing
        cv::circle(undist_frame, imgPoints[0], 3, cv::Scalar(255, 255, 0), 2);
        cv::circle(undist_frame, imgPoints[1], 3, cv::Scalar(0, 255, 0), 2);
        cv::circle(undist_frame, imgPoints[2], 3, cv::Scalar(0, 255, 0), 2);
        cv::line(undist_frame, imgPoints[1], imgPoints[2], cv::Scalar(0, 255, 0), 1);
        // Printing data to the console
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "x = " << (center(0) + 1)*EXTR_SQUARE_SIZE  << " mm, "<< "y = " << (center(1)+1)*EXTR_SQUARE_SIZE << " mm, " << "angle = " << angle << " degrees" << std::endl;
        std::cout << "imshowfun" << std::endl;
        cv::imshow("Cube Detection", undist_frame);
        std::cout << "Press 'Enter' to use current Cube Coordinates for EGM motion" << "\n" << "Change position of the cube and press 'Esc' to find the new position and orientation" << std::endl;
        //char character = cv::waitKey(-1); // waits infinetly for user to press the specific key
        if (cv::waitKey(-1) == 13) {
            vid.release();
            cv::destroyWindow("Cube Detection");
            std::cout<<"Video stream has stopped and Cube Coordinates passed to the EGM module"<<std::endl;
            break;
        }
        /*
        switch(character) {
            case 13:
                // start EGM Motion (press Enter)
                vid.release();
                cv::destroyWindow("Cube Detection");
                std::cout<<"Video stream has stopped and Cube Coordinates passed to the EGM module"<<std::endl;
                return std::make_tuple(cv::Vec2d((center(0) + 1)*EXTR_SQUARE_SIZE, (center(1)+1)*EXTR_SQUARE_SIZE),angle);
                break;

            case 27:
                // cnahge position of  the cube (press Esc)
                break;
            default:
                std::cout<<" Wrong key, try again..."<< std::endl;
                goto press_key_again;

        }
        */
        std::cout << "End of while loop" << std::endl;
        
    } 
    return std::make_tuple(cv::Vec2d((center(0) + 1)*EXTR_SQUARE_SIZE, (center(1)+1)*EXTR_SQUARE_SIZE),angle);
}




