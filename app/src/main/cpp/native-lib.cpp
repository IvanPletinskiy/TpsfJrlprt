#include <jni.h>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "Filter.cpp"
#include "Triangle.cpp"
#include "Circle.cpp"
#include "string"
#include "bits/stdc++.h"

using namespace std;

using namespace cv;

#define _GLIBCXX_HAVE_WCSTOF 1
#define _GLIBCXX_USE_C99 1

void find_shapes(Mat mat);

Mat retMat;
std::vector<Rect> detected_rects;
vector<Triangle> detected_triangles;
vector<Circle> detected_circles;
Point P1, P2, P3, P4;
CvPoint CENTER;
long RADIUS;
vector<Filter> filters;
Ptr<ORB> orbDetector = ORB::create();
BFMatcher matcher(NORM_HAMMING);
bool orb(Mat mat, int corners);

int detected_sign_id = 0;

void draw_detected_shapes();

extern "C" jint JNICALL
Java_com_handen_roadhelper_MainActivity_nativeOnFrame(JNIEnv *env, jobject instance,
                                                      jlong matAddr) {
    retMat = *(Mat *) matAddr;
    detected_sign_id = 0;
    blur(retMat, retMat, Size(5, 5));
    find_shapes(retMat);
    draw_detected_shapes();
    return detected_sign_id;
}

void draw_detected_shapes() {
    if (detected_rects.size() != 0) {
        Rect minRect;
        int minArea = -1;
        for (Rect r : detected_rects) {
            if (r.area() < minArea || minArea == -1) {
                minArea = r.area();
                minRect = r;
            }
        }
        rectangle(retMat, minRect, Scalar(0, 255, 0), 6);
    }
    if (detected_triangles.size() != 0) {
        Triangle maxTriangle;
        double maxSum = -1;
        for (Triangle triangle : detected_triangles) {
            double a1 =
                    (pow((double) (triangle.p2.x - triangle.p1.x), 2) +
                     (pow((double) (triangle.p2.y - triangle.p1.y), 2)));
            double a2 =
                    pow((double) (triangle.p3.x - triangle.p2.x), 2) +
                    pow((double) (triangle.p3.y - triangle.p2.y), 2);
            double a3 =
                    pow((double) (triangle.p3.x - triangle.p1.x), 2) +
                    pow((double) (triangle.p3.y - triangle.p1.y), 2);
            if (maxSum == -1 || (a1 + a2 + a3) > maxSum) {
                maxSum = a1 + a2 + a3;
                maxTriangle = triangle;
            }
        }
        line(retMat, maxTriangle.p1, maxTriangle.p2, Scalar(0, 255, 0), 6);
        line(retMat, maxTriangle.p2, maxTriangle.p3, Scalar(0, 255, 0), 6);
        line(retMat, maxTriangle.p3, maxTriangle.p1, Scalar(0, 255, 0), 6);
    }
    if (detected_circles.size() != 0) {
        Circle minCircle;
        int minRadius = -1;
        for (Circle c : detected_circles) {
            if (c.radius == -1 || c.radius < minRadius) {
                minRadius = c.radius;
                minCircle = c;
            }
        }
        circle(retMat, minCircle.center, minCircle.radius, Scalar(0, 255, 0), 4);
    }

    detected_triangles.clear();
    detected_rects.clear();
    detected_circles.clear();
}

void find_shapes(Mat mat) {
    IplImage *img = NULL;
    img = cvCreateImage(cvSize(mat.cols, mat.rows), 8, 3);
    IplImage ipltemp = mat;
    img = &ipltemp;

    IplImage *imgGrayScale = cvCreateImage(cvGetSize(img), 8, 1);
    cvCvtColor(img, imgGrayScale, CV_RGBA2GRAY);

    cvThreshold(imgGrayScale, imgGrayScale, 128, 255, CV_THRESH_BINARY);

    CvSeq *contours = NULL;  //hold the pointer to a contour in the memory block
    CvSeq *result = NULL;   //hold sequence of points of a contour
    CvMemStorage *storage = cvCreateMemStorage(0); //storage area for all contours

    cvFindContours(imgGrayScale, storage, &contours, sizeof(CvContour), CV_RETR_LIST,
                   CV_CHAIN_APPROX_SIMPLE, cvPoint(0, 0));

    while (contours) {
        result = cvApproxPoly(contours, sizeof(CvContour), storage, CV_POLY_APPROX_DP,
                              cvContourPerimeter(contours) * 0.05, 0);
        if (cvContourArea(result) < 100) {
            contours = contours->h_next;
            continue;
        }
        //if circle
        /*
        if(cvCheckContourConvexity(result)) {
            CvPoint *pt[result->total];
            for(int i = 0; i < result->total; ++i) {
                pt[i] = (CvPoint *) cvGetSeqElem(result, i);
            }
            CvRect rect  = cvBoundingRect(result);
            CvPoint2D32f p0;
            CvPoint2D32f *p = &p0;
            float f = 0;
            float *r = &f;
            cvMinEnclosingCircle(result, p, r);
            if(p == nullptr || r == nullptr) {
                contours = contours->h_next;
            }

            CENTER = CvPoint(p->x, p->y);
            RADIUS = (long) &r;
   //         Rect rect1(rect.x, rect.y, rect.width, rect.height);
    //        rectangle(retMat, rect1, Scalar(255, 0, 0), 4);
            cvSetImageROI(img, cvRect(rect.x, rect.y, rect.width, rect.height));
            retMat.adjustROI(rect.x, rect.y, rect.width, rect.height);
            cv::Mat mat123 = cv::cvarrToMat(img);
            orb(mat123, 0);
        }
         */
        //if there are 3  vertices  in the contour(It should be a triangle)
        if (result->total == 3) {
            //iterating through each point
            CvPoint *pt[3];
            for (int i = 0; i < 3; i++) {
                pt[i] = (CvPoint *) cvGetSeqElem(result, i);
            }
            int min_x = 10000;
            int min_y = 10000;
            int max_x = 0;
            int max_y = 0;

            for (int i = 0; i < 3; i++) {
                CvPoint *p = pt[i];
                if (p->x > max_x)
                    max_x = p->x;
                if (p->x < min_x)
                    min_x = p->x;
                if (p->y > max_y)
                    max_y = p->y;
                if (p->y < min_y)
                    min_y = p->y;
            }
            P1 = Point(pt[0]->x, pt[0]->y);
            P2 = Point(pt[1]->x, pt[1]->y);
            P3 = Point(pt[2]->x, pt[2]->y);

            cvSetImageROI(img, cvRect(min_x, min_y, max_x - min_x, max_y - min_y));
            retMat.adjustROI(min_x, min_y, max_x - min_x, max_y - min_y);
            cv::Mat mat123 = cv::cvarrToMat(img);
            orb(mat123, 3);
        } else
            if (result->total == 4) {
                CvPoint *pt[4];
                for (int i = 0; i < 4; i++) {
                    pt[i] = (CvPoint *) cvGetSeqElem(result, i);
                }

                int min_x = 10000;
                int min_y = 10000;
                int max_x = 0;
                int max_y = 0;

                for (int i = 0; i < 4; i++) {
                    CvPoint *p = pt[i];
                    if (p->x > max_x)
                        max_x = p->x;
                    if (p->x < min_x)
                        min_x = p->x;
                    if (p->y > max_y)
                        max_y = p->y;
                    if (p->y < min_y)
                        min_y = p->y;
                }

                CvPoint p1(min_x, min_y);
                CvPoint p2(max_x, min_y);
                CvPoint p3(max_x, max_y);
                CvPoint p4(min_x, max_y);
                P1 = p1;
                P2 = p2;
                P3 = p3;
                P4 = p4;

                cvSetImageROI(img, cvRect(min_x, min_y, max_x - min_x, max_y - min_y));
                retMat.adjustROI(min_x, min_y, max_x - min_x, max_y - min_y);
                cv::Mat mat123 = cv::cvarrToMat(img);
                //Rect rect = Rect(min_x, min_y, max_x - min_x, max_y - min_y);
                orb(mat123, 4);
            }

        contours = contours->h_next;
    }
    if (storage)
        cvReleaseMemStorage(&storage);
    if (imgGrayScale)
        cvReleaseImage(&imgGrayScale);
}

bool orb(Mat mat, int corners) {
    resize(mat, mat, Size(200, 200));
    for (Filter filter : filters) {
        if (filter.corners != corners)
            continue;
        std::vector<Point2f> targetCorners(4);
        std::vector<cv::KeyPoint> targetKeypoints;
        cv::Mat targetDescriptors;
        orbDetector->detectAndCompute(mat, noArray(), targetKeypoints, targetDescriptors);

        std::vector<DMatch> matches;
        matcher.match(targetDescriptors, filter.descriptors,
                      matches, noArray());

        if (matches.size() < 4) {
            continue;
        }

        double maxDist = 0.0;
        double minDist = 100000;

        for (DMatch match : matches) {
            double dist = match.distance;
            if (dist < minDist) {
                minDist = dist;
            }
            if (dist > maxDist) {
                maxDist = dist;
            }
        }

        if (minDist > 50.0) {
            continue;
        } else {
            if (minDist > 25.0) {
                continue;
            }
        }

        std::vector<cv::Point> goodTargetPoints;
        std::vector<cv::Point> goodReferencePoints;

        double maxGoodMatchDist = 1.75 * minDist;
        for (DMatch match : matches) {
            if (match.distance < maxGoodMatchDist) {
                goodReferencePoints.push_back(filter.keypoints[match.trainIdx].pt);
                goodTargetPoints.push_back(targetKeypoints[match.queryIdx].pt);
            }
        }

        if (goodTargetPoints.size() < 4 ||
            goodReferencePoints.size() < 4) {
            return false;
        }

        Mat homography = findHomography(goodReferencePoints, goodTargetPoints, RANSAC, 5);

        if (!homography.empty()) {
            detected_sign_id = filter.code;
            if (corners == 4)
                detected_rects.push_back(Rect(P1.x, P1.y, P2.x - P1.x, P4.y - P1.y));
            else
                if (corners == 3)
                    detected_triangles.push_back(
                            Triangle(Point(P1.x, P1.y), Point(P2.x, P2.y), Point(P3.x, P3.y)));
                else
                    if (corners == 0) {
                        detected_circles.push_back(Circle(RADIUS, CENTER));
                    }
            return true;
        }
    }
    return false;
}

extern "C" void JNICALL
Java_com_handen_roadhelper_MainActivity_addFilter(JNIEnv *env, jobject instance, jlong matAddr,
                                                  jint code, jint corners) {
    Ptr<ORB> orbDetector = ORB::create();
    Mat mat = *(Mat *) matAddr;
    if (mat.empty())
        return;
    resize(mat, mat, Size(200, 200));
    if (mat.channels() == 3)cv::cvtColor(mat, mat, CV_BGR2GRAY);
    else
        if (mat.channels() == 4)
            cv::cvtColor(mat, mat, CV_BGRA2GRAY);
    threshold(mat, mat, 128, 255, CV_THRESH_BINARY);
    std::vector<cv::KeyPoint> referenceKeypoints;
    cv::Mat referenceDescriptors;
    orbDetector -> detectAndCompute(mat, noArray(), referenceKeypoints, referenceDescriptors);
    filters.push_back(Filter(code, corners, referenceKeypoints, referenceDescriptors));
}