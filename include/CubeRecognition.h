#ifndef CUBE_RECOGNITION_H
#define CUBE_RECOGNITION_H

#include "opencv2/opencv.hpp"
#include <string>
#include <tuple>

extern const double PI;
extern const int FPS;
extern const double EXTR_SQUARE_SIZE; //mm

class CubeDetector {
    private:
    cv::Matx<double,3,3> K;
    cv::Matx<double,3,3> Kinv;
    cv::Vec<double,5> dist;
    cv::Vec<double,3> rvecs;
    cv::Matx<double,3,3> Rinv;
    cv::Vec<double,3> tvecs;
    //cv::Matx<double,1,3> tvecs;
    std::tuple<cv::Vec2d, double> getPositionAndOrientation(const cv::Vec3d& rw1, const cv::Vec3d& rw2);
    std::vector<cv::Vec2f> getPixelCoordinates(cv::Mat& img);
    public:
    CubeDetector() = delete; // delete default (empty) constructor
    explicit CubeDetector(const std::string& intrinsics_file_name, const std::string& extrinsics_file_name);
 

    // preventing copying amd moving the object which contains video stream
    CubeDetector(const CubeDetector&) = delete; // copy constructor
    CubeDetector(CubeDetector&&) = delete; // move constructor
    CubeDetector& operator = (const CubeDetector&) = delete; // copy assignemt operator
    CubeDetector& operator = (CubeDetector&&) = delete; // move assignment operato

    cv::Vec3d getWorldCoordinates(cv::Vec2d px, double z = 0.0);
    std::tuple<cv::Vec2d, double> detectCube(const bool Manual);
};
#endif /* CUBE_RECOGNITION_H */