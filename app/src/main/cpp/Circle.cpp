//
// Created by Vanya on 08.11.2018.
//
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
using namespace cv;

struct Circle {
    Circle() {

    }

    int radius;
    CvPoint center;

    Circle(int radius, const CvPoint &center) : radius(radius), center(center) {}
};
