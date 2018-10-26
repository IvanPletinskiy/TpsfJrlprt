#include <jni.h>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
//#include "Java_com_handen_roadhelper_MainActivity_nativeOnFrame.h"
using namespace std;

using namespace cv;

void find_shapes(Mat mat);

void find_squares(Mat mat);

void convert_to_gray(Mat mat);

void detect_square_sign();

Rect createRect();

Mat pedastrian;

BFMatcher matcher(NORM_HAMMING);


Mat retMat;
std::vector<Rect>  detected_rects;
Point2f P1,P2,P3,P4;
vector<pair<vector<cv::KeyPoint>, cv::Mat> > filters;

void orb(IplImage *pImage);

extern "C" void JNICALL
Java_com_handen_roadhelper_MainActivity_nativeOnFrame(JNIEnv *env, jobject instance,
                                                      jlong matAddr,
                                                      jint nbrElem) {
    //TODO warpPerspective() позволяет выровнять матрицу на плоскости
    int64 e1 = cv::getTickCount();

    retMat = *(Mat *) matAddr;

    blur(retMat, retMat, Size(5, 5));

    find_shapes(retMat);


    char cbuff[20];
    int64 e2 = cv::getTickCount();
    float time = (e2 - e1) / cv::getTickFrequency();
    sprintf(cbuff, "%f sec", time);
    putText(retMat, cbuff, CvPoint(30, 30), FONT_HERSHEY_COMPLEX, 1.0, cvScalar(255, 255, 255));

    Rect minRect;
    int minArea = -1;
    for(Rect r : detected_rects) {
        if(r.area() < minArea || minArea == -1) {
            minArea = r.area();
            minRect = r;
        }
    }

    rectangle(retMat, minRect, Scalar(0, 255, 0), 4);

    detected_rects.clear();
}

void find_shapes(Mat mat) {
    IplImage *img = NULL;
    img = cvCreateImage(cvSize(mat.cols, mat.rows), 8, 3);
    IplImage ipltemp = mat;
    img = &ipltemp;

    IplImage *imgGrayScale = cvCreateImage(cvGetSize(img), 8, 1);
    cvCvtColor(img, imgGrayScale, CV_RGBA2GRAY);

    //thresholding the grayscale image to get better results
    cvThreshold(imgGrayScale, imgGrayScale, 128, 255, CV_THRESH_BINARY);

    CvSeq *contours = NULL;  //hold the pointer to a contour in the memory block
    CvSeq *result = NULL;   //hold sequence of points of a contour
    CvMemStorage *storage = cvCreateMemStorage(0); //storage area for all contours

    //finding all contours in the image
    cvFindContours(imgGrayScale, storage, &contours, sizeof(CvContour), CV_RETR_LIST,
                   CV_CHAIN_APPROX_SIMPLE, cvPoint(0, 0));

    //iterating through each contour
    while (contours) {
        //obtain a sequence of points of contour, pointed by the variable 'contour'
        result = cvApproxPoly(contours, sizeof(CvContour), storage, CV_POLY_APPROX_DP,
                              cvContourPerimeter(contours) * 0.05, 0);
        if (cvContourArea(result) < 100) {
            contours = contours->h_next;
            continue;
        }
        //if there are 3  vertices  in the contour(It should be a triangle)
        if (result->total == 3) {
            /*
            //iterating through each point
            CvPoint *pt[3];
            for (int i = 0; i < 3; i++) {
                pt[i] = (CvPoint *) cvGetSeqElem(result, i);
            }

            //drawing lines around the triangle
            cvLine(img, *pt[0], *pt[1], cvScalar(255, 0, 0), 4);
            cvLine(img, *pt[1], *pt[2], cvScalar(255, 0, 0), 4);
            cvLine(img, *pt[2], *pt[0], cvScalar(255, 0, 0), 4);
            */
        }
            //if there are 4 vertices in the contour(It should be a quadrilateral)
        else if (result->total == 4) {
            //iterating through each point
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
/*

            cvLine(img, p1, p2, cvScalar(0, 255, 0), 4);
            cvLine(img, p2, p3, cvScalar(0, 255, 0), 4);
            cvLine(img, p3, p4, cvScalar(0, 255, 0), 4);
            cvLine(img, p4, p1, cvScalar(0, 255, 0), 4);
*/

            cvSetImageROI(img, cvRect(min_x, min_y, max_x - min_x, max_y - min_y));
            retMat.adjustROI(min_x, min_y, max_x - min_x, max_y - min_y);
            cv::Mat mat123 = cv::cvarrToMat(img);
            Rect rect = Rect(min_x, min_y, max_x - min_x, max_y - min_y);
         //   Mat mat1 = Mat(mat123, rect);
            orb(img);
            //       cvAddS(img, cvScalar(0, 255, 0), img);

            /*
            //drawing lines around the quadrilateral
            cvLine(img, *pt[0], *pt[1], cvScalar(0, 255, 0), 4);
            cvLine(img, *pt[1], *pt[2], cvScalar(0, 255, 0), 4);
            cvLine(img, *pt[2], *pt[3], cvScalar(0, 255, 0), 4);
            cvLine(img, *pt[3], *pt[0], cvScalar(0, 255, 0), 4);
            */
            //      detect_square_sign();
        }
            //if there are 7  vertices  in the contour(It should be a heptagon)
        else if (result->total == 7) {
            /*
            //iterating through each point
            CvPoint *pt[7];
            for (int i = 0; i < 7; i++) {
                pt[i] = (CvPoint *) cvGetSeqElem(result, i);
            }

            //drawing lines around the heptagon
            cvLine(img, *pt[0], *pt[1], cvScalar(0, 0, 255), 4);
            cvLine(img, *pt[1], *pt[2], cvScalar(0, 0, 255), 4);
            cvLine(img, *pt[2], *pt[3], cvScalar(0, 0, 255), 4);
            cvLine(img, *pt[3], *pt[4], cvScalar(0, 0, 255), 4);
            cvLine(img, *pt[4], *pt[5], cvScalar(0, 0, 255), 4);
            cvLine(img, *pt[5], *pt[6], cvScalar(0, 0, 255), 4);
            cvLine(img, *pt[6], *pt[0], cvScalar(0, 0, 255), 4);
             */
        }
  //      retMat.()
        //obtain the next contour
        contours = contours->h_next;
    }
    if (storage)
        cvReleaseMemStorage(&storage);
    if (imgGrayScale)
        cvReleaseImage(&imgGrayScale);
}


extern "C" void JNICALL
Java_com_handen_roadhelper_MainActivity_addFilter(JNIEnv *env, jobject instance,
                                                      jlong matAddr) {
    Ptr<ORB> orbDetector = ORB::create();
    Mat mat = *(Mat *) matAddr;
    resize(mat, mat, Size(100, 100));
    if(mat.channels() == 3)
        cv::cvtColor(mat, mat, CV_BGR2GRAY);
    else
        if(mat.channels() == 4)
            cv::cvtColor(mat, mat, CV_BGRA2GRAY);

    threshold(mat, mat, 128, 255, CV_THRESH_BINARY);
    int a = mat.channels();
    std::vector<cv::KeyPoint> referenceKeypoints;
    cv::Mat referenceDescriptors;
    orbDetector->detectAndCompute(mat, noArray(), referenceKeypoints, referenceDescriptors);

    filters.push_back(pair<vector<cv::KeyPoint>, cv::Mat>(referenceKeypoints, referenceDescriptors));
}

void orb(IplImage *pImage) {
    //  Mat(const IplImage* pImage, bool copyData=false);
    Ptr<ORB> orbDetector = ORB::create();

    cv::Mat mat = cv::cvarrToMat(pImage);
    resize(mat, mat, Size(100, 100));
    for(pair<vector<cv::KeyPoint>, cv::Mat> p : filters) {
        std::vector<Point2f> targetCorners(4);

        std::vector<cv::KeyPoint> targetKeypoints;

        cv::Mat targetDescriptors;

        //  DescriptorMatcher matcher = DescriptorMatcher();

        orbDetector->detectAndCompute(mat, noArray(), targetKeypoints, targetDescriptors);
     //   orbDetector->detectAndCompute(pedastrian, noArray(), p.first, p.second);

        //Match images based on k nearest neighbour
        //std::vector<std::vector<cv::DMatch> > matches;
        std::vector<DMatch> matches;
        matcher.match(targetDescriptors, p.second,
                      matches, noArray());

        if (matches.size() < 4) {
            //There are too few matches to find the homogrhaphy
            return;
        }

        //Calculate the max and min distances between keypoints
        double maxDist = 0.0;
        double minDist = 100000;

        for (DMatch match : matches) {
            double dist = match.distance;
            if(dist < minDist) {
                minDist = dist;
            }
            if(dist > maxDist) {
                maxDist = dist;
            }
        }

        if(minDist > 50.0) {
            targetCorners = Mat(0, 0, CV_32FC2);
            return;
        }
        else {
            if(minDist > 25.0) {
                return;
            }
        }

        std::vector<cv::Point> goodTargetPoints;
        std::vector<cv::Point> goodReferencePoints;

        double maxGoodMatchDist = 1.75 * minDist;
        for(DMatch match : matches) {
            if(match.distance < maxGoodMatchDist) {
                goodReferencePoints.push_back(p.first[match.trainIdx].pt);
                goodTargetPoints.push_back(targetKeypoints[match.queryIdx].pt);
            }
        }

        if(goodTargetPoints.size() < 4 ||
           goodReferencePoints.size() < 4) {
            // There are too few good points to find the homography.
            return;
        }

        Mat homography = findHomography(goodReferencePoints, goodTargetPoints, RANSAC, 5);

        if(!homography.empty()) {
            detected_rects.push_back(Rect(P1.x, P1.y, P2.x - P1.x, P4.y - P1.y));
        }
    }

    /*
    std::vector<Point2f> referenceCorners;
    referenceCorners.push_back(Point(0, 0));
    referenceCorners.push_back(Point(pedastrian.cols, 0));
    referenceCorners.push_back(Point(pedastrian.cols, pedastrian.rows));
    referenceCorners.push_back(Point(0, pedastrian.rows));
    */
    /*
    std::vector<Point2f> targetCorners(4);

    std::vector<cv::KeyPoint> targetKeypoints;
    std::vector<cv::KeyPoint> referenceKeypoints;

    cv::Mat targetDescriptors;
    cv::Mat referenceDescriptors;

    //  DescriptorMatcher matcher = DescriptorMatcher();

    orbDetector->detectAndCompute(mat, noArray(), targetKeypoints, targetDescriptors);
    orbDetector->detectAndCompute(pedastrian, noArray(), referenceKeypoints, referenceDescriptors);

    //Match images based on k nearest neighbour
    //std::vector<std::vector<cv::DMatch> > matches;
    std::vector<DMatch> matches;
    matcher.match(targetDescriptors, referenceDescriptors,
                  matches, noArray());

    if (matches.size() < 4) {
        //There are too few matches to find the homogrhaphy
        return;
    }

    //Calculate the max and min distances between keypoints
    double maxDist = 0.0;
    double minDist = 100000;

    for (DMatch match : matches) {
        double dist = match.distance;
        if(dist < minDist) {
            minDist = dist;
        }
        if(dist > maxDist) {
            maxDist = dist;
        }
    }

    if(minDist > 50.0) {
        targetCorners = Mat(0, 0, CV_32FC2);
        return;
    }
    else {
        if(minDist > 25.0) {
            return;
        }
    }

    std::vector<cv::Point> goodTargetPoints;
    std::vector<cv::Point> goodReferencePoints;

    double maxGoodMatchDist = 1.75 * minDist;
    for(DMatch match : matches) {
        if(match.distance < maxGoodMatchDist) {
            goodReferencePoints.push_back(referenceKeypoints[match.trainIdx].pt);
            goodTargetPoints.push_back(targetKeypoints[match.queryIdx].pt);
        }
    }

    if(goodTargetPoints.size() < 4 ||
            goodReferencePoints.size() < 4) {
        // There are too few good points to find the homography.
        return;
    }

    //Mat homography;
    Mat homography = findHomography(goodReferencePoints, goodTargetPoints, RANSAC, 5);

    if(!homography.empty()) {
   //     perspectiveTransform(referenceCorners, targetCorners, homography);
    //    targetCorners[0] += P1;
  //      targetCorners[1] += P1;
   //     targetCorners[2] += P1;
  //      targetCorners[3] += P1;

        detected_rects.push_back(Rect(P1.x, P1.y, P2.x - P1.x, P4.y - P1.y));
        */
/*
        line(retMat, P1, P2 , Scalar(0, 255, 0), 4);
        line(retMat, P2, P3, Scalar(0, 255, 0), 4);
        line(retMat, P3, P4, Scalar(0, 255, 0), 4);
        line(retMat, P4, P1, Scalar(0, 255, 0), 4);
*/
    }



/// Show the image
//   CannyThreshold(0, 0);

/// Reduce noise with a kernel 3x3
//   blur(mGr, detected_edges, Size(50, 50));

/// Canny detector
//  Canny(detected_edges, detected_edges, lowThreshold, lowThreshold * 3, kernel_size);

/// Using Canny's output as a mask, we display our result
//  dst = Scalar::all(0);
//   mGr = detected_edges;

//   mGr.copyTo(mGr, detected_edges);

//       for (int k = 0; k < nbrElem; k++) {
//           int i = rand() % dst.cols;
//           int j = rand() % dst.rows;
//           dst.at<uchar>(j, i) = 255;
//       }
