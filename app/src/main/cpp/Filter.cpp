//
// Created by Vanya on 07.11.2018.
//
#include <jni.h>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
using namespace std;
using namespace cv;

struct Filter {
  //  string name;
    int code;
    int corners;
    vector<cv::KeyPoint> keypoints;
    Mat descriptors;

    Filter(int code, int corners, const vector<KeyPoint> &keypoints, const Mat &descriptors) : code(
            code), corners(corners), keypoints(keypoints), descriptors(descriptors) {}
};
