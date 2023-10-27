#include "QCamCalib.h"

bool QCamCalib::init(const Config& config){

    this->_config = config;

    return true;
}

bool QCamCalib::findChessboardCorners(cv::Mat srcImg, std::vector<cv::Point2f>& corners){
    if(srcImg.empty()){
        printf("srcImg is None!\n");
        return false;
    }
    
    cv::Mat grayImg;
    cv::cvtColor(srcImg, grayImg, cv::COLOR_BGR2GRAY);

    cv::Size patternSize = this->_config.patternSize;
    //CALIB_CB_FAST_CHECK saves a lot of time on images
    //that do not contain any chessboard corners
    bool patternfound = cv::findChessboardCorners(grayImg, patternSize, corners,
                                                cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);
    if(patternfound){
        cv::cornerSubPix(grayImg, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
    }

    // cv::Mat showImg = srcImg.clone();
    // cv::drawChessboardCorners(showImg, patternSize, corners, patternfound);
    // cv::imwrite("./temp/showImg.jpg", showImg);

    return patternfound;
}

bool QCamCalib::calibrateCamera(const std::vector<std::vector<cv::Point2f>>& imagePoints, cv::Mat& cameraMatrix, cv::Mat& distCoeffs){

    std::vector<std::vector<cv::Point3f>> objectPoints;
    for(int i = 0; i < imagePoints.size(); i++){
        std::vector<cv::Point3f> corners;
        for(int row = 0; row < this->_config.patternSize.height; row++){
            for(int col = 0; col < this->_config.patternSize.width; col++){
                cv::Point3f pt;
                pt.x = col * this->_config.squareSize.width;
                pt.y = row * this->_config.squareSize.height;
                pt.z = 0;
                corners.push_back(pt);
            }
        }
        objectPoints.push_back(corners);
    }
    
    cv::Size imageSize = this->_config.srcImgSize;
    // cv::Mat cameraMatrix;
    // cv::Mat distCoeffs;
    std::vector<cv::Vec3d> rvecs;
    std::vector<cv::Vec3d> tvecs;

    switch(this->_config.camType){
    case CamType::CAM_GENERAL:{
        cv::calibrateCamera(objectPoints, imagePoints, imageSize, this->_cameraMatrix, this->_distCoeffs, rvecs, tvecs);
    }
    case CamType::CAM_FISHEYE:{
        int flags = cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC | cv::fisheye::CALIB_CHECK_COND | cv::fisheye::CALIB_FIX_SKEW;
        cv::fisheye::calibrate(objectPoints, imagePoints, imageSize, this->_cameraMatrix, this->_distCoeffs, rvecs, tvecs, flags, cv::TermCriteria(3, 20, 1e-6));
    }
    }

    cameraMatrix = this->_cameraMatrix;
    distCoeffs = this->_distCoeffs;

    return true;
}

void QCamCalib::undistortImage(cv::Mat srcImg, cv::Mat& dstImg, cv::Mat cameraMatrix, cv::Mat distCoeffs){
    if(cameraMatrix.empty() || distCoeffs.empty()){
        cameraMatrix = this->_cameraMatrix;
        distCoeffs = this->_distCoeffs;
    }

    switch(this->_config.camType){
    case CamType::CAM_GENERAL:{
        cv::undistort(srcImg, dstImg, cameraMatrix, distCoeffs, cameraMatrix);
    }
    case CamType::CAM_FISHEYE:{
        cv::fisheye::undistortImage(srcImg, dstImg, cameraMatrix, distCoeffs, cameraMatrix);
    }
    }

}

