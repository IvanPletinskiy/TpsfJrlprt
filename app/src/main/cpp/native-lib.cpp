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

void orb(IplImage *pImage);

extern "C" void JNICALL
Java_com_handen_roadhelper_MainActivity_nativeOnFrame(JNIEnv *env, jobject instance,
                                                      jlong matAddr,
                                                      jint nbrElem) {
    //TODO warpPerspective() позволяет выровнять матрицу на плоскости
    int64 e1 = cv::getTickCount();

    retMat = *(Mat *) matAddr;
    //  convert_to_gray(mat); //не работает

    blur(retMat, retMat, Size(10, 10));

    // inRange(secordMat, Scalar(110,50,50), Scalar(130, 255, 255), mat); //не работает

    find_shapes(retMat);

    char cbuff[20];
    int64 e2 = cv::getTickCount();
    float time = (e2 - e1) / cv::getTickFrequency();
    sprintf(cbuff, "%f sec", time);
    putText(retMat, cbuff, CvPoint(30, 30), FONT_HERSHEY_COMPLEX, 1.0, cvScalar(255, 255, 255));
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

            /*

            cvLine(img, p1, p2, cvScalar(0, 255, 0), 4);
            cvLine(img, p2, p3, cvScalar(0, 255, 0), 4);
            cvLine(img, p3, p4, cvScalar(0, 255, 0), 4);
            cvLine(img, p4, p1, cvScalar(0, 255, 0), 4);

            */

            cvSetImageROI(img, cvRect(min_x, min_y, max_x - min_x, max_y - min_y));
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
        //obtain the next contour
        contours = contours->h_next;
    }
    if (storage)
        cvReleaseMemStorage(&storage);
    if (imgGrayScale)
        cvReleaseImage(&imgGrayScale);
}


extern "C" void JNICALL
Java_com_handen_roadhelper_MainActivity_setPedastrian(JNIEnv *env, jobject instance,
                                                      jlong matAddr
) {
    pedastrian = *(Mat *) matAddr;
}

void orb(IplImage *pImage) {
    //  Mat(const IplImage* pImage, bool copyData=false);
    cv::Mat mat = cv::cvarrToMat(pImage);

    line(retMat,Point(0, 0), Point (100, 100), Scalar(0, 255, 0), 4);

  //  cv::Mat referenceCorners(4, 1, CV_32FC2);
  //  referenceCorners.at()
    std::vector<Point2f> referenceCorners;
    referenceCorners.push_back(Point(0, 0));
    referenceCorners.push_back(Point(pedastrian.cols, 0));
    referenceCorners.push_back(Point(pedastrian.cols, pedastrian.rows));
    referenceCorners.push_back(Point(0, pedastrian.rows));


   // cv::Mat_<cv::Point> targetCorners(4, 1, cv::Point(0, 0));
    std::vector<Point2f> targetCorners(4);

    std::vector<cv::KeyPoint> targetKeypoints;
    std::vector<cv::KeyPoint> referenceKeypoints;

    cv::Mat targetDescriptors;
    cv::Mat referenceDescriptors;


    //  DescriptorMatcher matcher = DescriptorMatcher();
    BFMatcher matcher(NORM_HAMMING);
    Ptr<ORB> orb = ORB::create();

    orb->detectAndCompute(mat, noArray(), targetKeypoints, targetDescriptors);
    orb->detectAndCompute(pedastrian, noArray(), referenceKeypoints, referenceDescriptors);

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

    // The thresholds for minDist are chosen subjectively
    // based on testing. The unit is not related to pixel
    // distances; it is related to the number of failed tests
    // for similarity between the matches descriptors
    if(minDist > 50.0) {
        // The target is completely lost.
        // Discard any previously found corners.
        //mSceneCorners.create(0, 0, mSceneCorners.type());
        targetCorners = Mat(0, 0, CV_32FC2);
        return;
    }
    else {
        if(minDist > 25.0) {
            // The target is post but maybe it is still close.
            // Keep any previously found corners.
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

    Mat homography, m = findHomography(goodReferencePoints, goodTargetPoints, RANSAC, 5.0);

    if(!homography.empty()) {
        perspectiveTransform(referenceCorners, targetCorners, homography);

        line(retMat, targetCorners[0], targetCorners[1], Scalar(0, 255, 0), 4);
        line(retMat, targetCorners[1], targetCorners[2], Scalar(0, 255, 0), 4);
        line(retMat, targetCorners[2], targetCorners[3], Scalar(0, 255, 0), 4);
        line(retMat, targetCorners[3], targetCorners[0], Scalar(0, 255, 0), 4);
    }
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
