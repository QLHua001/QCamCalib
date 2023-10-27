#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

class QCamCalib{
public:
    enum CamType{
        CAM_GENERAL,
        CAM_FISHEYE
    };

    struct Config{
        CamType camType{CamType::CAM_GENERAL};
        cv::Size patternSize;
        cv::Size squareSize;
        cv::Size srcImgSize;

        Config& operator=(const Config& config){
            this->camType = config.camType;
            this->patternSize = config.patternSize;
            this->squareSize = config.squareSize;
            this->srcImgSize = config.srcImgSize;
            return *this;
        }
    };

public:
    bool init(const Config& config);
    bool findChessboardCorners(cv::Mat srcImg, std::vector<cv::Point2f>& corners);
    bool calibrateCamera(const std::vector<std::vector<cv::Point2f>>& imagePoints, cv::Mat& cameraMatrix, cv::Mat& distCoeffs);
    void undistortImage(cv::Mat srcImg, cv::Mat& dstImg, cv::Mat cameraMatrix=cv::Mat(), cv::Mat distCoeffs=cv::Mat());

private:
    Config _config;
    cv::Mat _cameraMatrix;
    cv::Mat _distCoeffs;
};