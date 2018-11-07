//
// Created by Vanya on 07.11.2018.
//
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
using namespace cv;
struct Triangle {
    Point p1, p2, p3;

    Triangle(const Point &p1, const Point &p2, const Point &p3) : p1(p1), p2(p2), p3(p3) {}
};
