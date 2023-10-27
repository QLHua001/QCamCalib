#include <iostream>
#include <unistd.h>
#include <string>
#include <vector>
#include <dirent.h>
#include <opencv2/opencv.hpp>

#include "QCamCalib/QCamCalib.h"

bool searchSpecifiedFiles(std::string folder, std::string extension, std::vector<std::string>& filePaths){

    if(folder.empty() || extension.empty()) return false; 

    if(folder.back() != '/'){
        folder += "/";
    }

    DIR* pDir;
    dirent* pCur;

    if((pDir = opendir(folder.c_str())) == NULL){
        printf("opendir %s fail!\n", folder.c_str());
        return false;
    }

    while((pCur = readdir(pDir)) != NULL){
        if(DT_REG == pCur->d_type){
            const char* selfExtension = strrchr(pCur->d_name, '.');
            if(selfExtension != NULL && strcmp(selfExtension, extension.c_str()) == 0){
                std::string filePath = folder + pCur->d_name;
                filePaths.push_back(filePath);
            }
        }
    }

    return true;
}

void test(){

    std::string imgPath{"./temp/chessboard.jpg"};

    cv::Mat srcImg = cv::imread(imgPath);
    if(srcImg.empty()){
        printf("imread %s fail!\n", imgPath.c_str());
        return;
    }

    cv::Mat grayImg;
    cv::cvtColor(srcImg, grayImg, cv::COLOR_BGR2GRAY);

    cv::Size patternSize(9, 6);
    std::vector<cv::Point2f> corners;
    //CALIB_CB_FAST_CHECK saves a lot of time on images
    //that do not contain any chessboard corners
    bool patternfound = cv::findChessboardCorners(grayImg, patternSize, corners,
                                                cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);
    if(patternfound){
        cv::cornerSubPix(grayImg, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
    }

    cv::Mat showImg = srcImg.clone();
    cv::drawChessboardCorners(showImg, patternSize, corners, patternfound);
    cv::imwrite("./temp/showImg.jpg", showImg);

}

void test_QCamCalib(){

    std::string imgRoot{"./example/calib_img_fisheye/"};    // calib_img
    cv::Size patternSize(9, 6);
    cv::Size squareSize(20, 20); // mm
    cv::Size srcImgSize(1280, 960);

    std::string testImgPath{"./example/test_fisheye.jpg"};

    std::vector<std::string> filePaths;
    if(!searchSpecifiedFiles(imgRoot, ".jpg", filePaths)){
        printf("searchSpecifiedFiles fail!(%s)\n", imgRoot.c_str());
        return;
    }
    printf("Found %ld imgs.\n", filePaths.size());

    // init QCamCalib
    QCamCalib camCalib;
    QCamCalib::Config config;
    config.camType = QCamCalib::CamType::CAM_FISHEYE;
    config.patternSize = patternSize;
    config.squareSize = squareSize;
    config.srcImgSize = srcImgSize;
    if(!camCalib.init(config)){
        printf("QCamCalib init fail!\n");
        return;
    }

    std::vector<std::vector<cv::Point2f>> imagePoints;
    for(int i = 0; i < filePaths.size(); i++){
        std::string filePath = filePaths[i];

        cv::Mat srcImg = cv::imread(filePath);
        if(srcImg.empty()){
            printf("imread %s fail!\n", filePath.c_str());
            continue;
        }

        std::vector<cv::Point2f> corners;
        if(!camCalib.findChessboardCorners(srcImg, corners)){
            printf("findChessboardCorners fail!(%s)\n", filePath.c_str());
            continue;
        }
        // cv::Mat showImg = srcImg.clone();
        // cv::drawChessboardCorners(showImg, patternSize, corners, true);
        // cv::imwrite("./temp/showImg.jpg", showImg);
        // usleep(3*1000*1000);
        imagePoints.push_back(corners);
    } 

    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    camCalib.calibrateCamera(imagePoints, cameraMatrix, distCoeffs);
    std::cout << "cameraMatrix: \n" << cameraMatrix << std::endl;
    std::cout << "distCoeffs: \n" << distCoeffs << std::endl;

    cv::Mat testImg = cv::imread(testImgPath);
    if(testImg.empty()){
        printf("imread %s fail!\n", testImgPath.c_str());
        return;
    }
    cv::Mat undistortedImg;
    camCalib.undistortImage(testImg, undistortedImg);
    cv::imwrite("./temp/undistortedImg.jpg", undistortedImg);
}

int main(int, char**){
    std::cout << "Hello, from QCamCalib!\n";

    // test();

    test_QCamCalib();

    return 0;
}
