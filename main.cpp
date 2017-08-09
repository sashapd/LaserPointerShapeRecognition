#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc.hpp"
#include <iostream>

cv::Mat getLaserMask(const cv::Mat& baseFrame, const cv::Mat& image) {
    cv::Mat diff = image - baseFrame;

    cv::Mat redDiffMask;
    cv::inRange(diff, cv::Scalar(0, 0, 50), cv::Scalar(255, 255, 255), redDiffMask);
    cv::Mat redMask;
    cv::inRange(image, cv::Scalar(0, 0, 150), cv::Scalar(150, 150, 255), redMask);
    cv::Mat whiteMask;
    cv::inRange(image, cv::Scalar(210, 210, 230), cv::Scalar(255, 255, 255), whiteMask);
    cv::Mat mask = (redMask | whiteMask) & redDiffMask;

    return mask;
}

bool getPointerLocation(const cv::Mat& maskImage, cv::Point& location) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(maskImage, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    if(contours.empty()) {
        std::cout << "no lasers found" << std::endl;
        return false;
    }

    std::vector<cv::Point> maxCountour;
    double maxArea = 0;

    for(int i = 0; i < contours.size(); i++) {
        std::vector<cv::Point>& countour = contours[i];
        double area = cv::contourArea(countour);
        if(area > maxArea) {
            maxCountour = countour;
            maxArea = area;
        }
    }
    if(maxArea == 0) {
        return false;
    }
    std::cout << maxArea << " contour size " <<maxCountour.size() << std::endl;
    location = maxCountour[0];

    return true;
}

int main() {
    cv::VideoCapture cap(0);

    cv::waitKey(1000);

    cv::Mat baseFrame;
    cap.read(baseFrame);

    std::vector<cv::Point> contour;
    std::vector<std::vector<cv::Point>> contours;

    int framesNotFound = 0;

    while((cv::waitKey(1) & 0xFF) != 27) {
        cv::Mat frame;
        cap.read(frame);

        cv::Mat mask = getLaserMask(baseFrame, frame);
        cv::Point pointerLocation;
        bool found = getPointerLocation(mask, pointerLocation);

        if(found) {
            contours.back().push_back(pointerLocation);
            framesNotFound = 0;
        } else {
            framesNotFound++;
        }
        if(framesNotFound > 25) {

        }
        cv::drawContours(frame, contours, 0, cv::Scalar(0, 255, 0));
        cv::imshow("mask", frame);
    }
    return 0;
}