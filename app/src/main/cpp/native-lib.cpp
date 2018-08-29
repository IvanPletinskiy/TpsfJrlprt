#include <jni.h>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
//#include "Java_com_handen_roadhelper_MainActivity_nativeOnFrame.h"
using namespace std;

using namespace cv;

extern "C"
{
void find_shapes(Mat mat);
void convert_to_gray(Mat mat);

void JNICALL Java_com_handen_roadhelper_MainActivity_nativeOnFrame(JNIEnv *env, jobject instance,
                                                                   jlong matAddr,
                                                                   jint nbrElem) {
    //bitwise_add позволяет выделить кусок матрицы res = cv2.bitwise_and(frame,frame, mask= mask)
    cv::Mat mat = *(Mat *) matAddr;
  //  convert_to_gray(mat); //не работает

    blur(mat, mat, Size(10, 10));
   // Mat secordMat = *(Mat *) matAddr;
   // cvtColor(secordMat, secordMat, COLOR_RGBA2RGB);
   // cvtColor(secordMat, secordMat, COLOR_RGB2HSV);
   // inRange(secordMat, Scalar(110,50,50), Scalar(130, 255, 255), mat); //не работает

    find_shapes(mat);
}
}
void convert_to_gray(Mat mat) {
    cvtColor(mat, mat, COLOR_RGBA2GRAY);
}

void find_shapes(Mat mat) {
    int64 e1 = cv::getTickCount();
    IplImage *img = NULL;
    img = cvCreateImage(cvSize(mat.cols, mat.rows), 8, 3);
    IplImage ipltemp = mat;
    // cvCopy(&ipltemp, img);
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
                              cvContourPerimeter(contours) * 0.02, 0);

        //if there are 3  vertices  in the contour(It should be a triangle)
        if (result->total == 3) {
            //iterating through each point
            CvPoint *pt[3];
            for (int i = 0; i < 3; i++) {
                pt[i] = (CvPoint *) cvGetSeqElem(result, i);
            }

            //drawing lines around the triangle
            cvLine(img, *pt[0], *pt[1], cvScalar(255, 0, 0), 4);
            cvLine(img, *pt[1], *pt[2], cvScalar(255, 0, 0), 4);
            cvLine(img, *pt[2], *pt[0], cvScalar(255, 0, 0), 4);

        }

            //if there are 4 vertices in the contour(It should be a quadrilateral)
        else if (result->total == 4) {
            //iterating through each point
            CvPoint *pt[4];
            for (int i = 0; i < 4; i++) {
                pt[i] = (CvPoint *) cvGetSeqElem(result, i);
            }

            //drawing lines around the quadrilateral
            cvLine(img, *pt[0], *pt[1], cvScalar(0, 255, 0), 4);
            cvLine(img, *pt[1], *pt[2], cvScalar(0, 255, 0), 4);
            cvLine(img, *pt[2], *pt[3], cvScalar(0, 255, 0), 4);
            cvLine(img, *pt[3], *pt[0], cvScalar(0, 255, 0), 4);
        }

            //if there are 7  vertices  in the contour(It should be a heptagon)
        else if (result->total == 7) {
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
        }

        //obtain the next contour
        contours = contours->h_next;
    }
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 1, 1);

    char cbuff[20];
    int64 e2 = cv::getTickCount();
    float time = (e2 - e1)/ cv::getTickFrequency ();
    sprintf (cbuff, "%f sec", time);
    cvPutText (img, cbuff, CvPoint (30, 30), &font, cvScalar (255, 255, 255));

//    img = nullptr;
   // delete ipltemp;
//    imgGrayScale = nullptr;
//    contours = nullptr;
//    result = nullptr;
//    cvReleaseMemStorage(&storage);
//    storage = nullptr;

    ///cleaning up
//    cvDestroyAllWindows();
    if (storage)
      cvReleaseMemStorage(&storage);
//    if (img)
//      cvReleaseImage(&img);
    if (imgGrayScale)
      cvReleaseImage(&imgGrayScale);
}















































/*
 //  Mat &mGr = *(Mat *) matAddrGray;
   Mat matRgba = *(Mat *) matAddrGray;
   Mat matRgb;
   cvtColor(matRgba, matRgb, COLOR_RGBA2RGB);
   //Mat3b hsv_inv;
   Mat matHsv;
   cvtColor(matRgb, matHsv, COLOR_RGB2HSV);

   Mat mask;
   inRange(matHsv, Scalar(90 - 10, 70, 50), Scalar(90 + 10, 255, 255), mask); // Cyan is 90

   matRgba = mask;
    */

/// Create a matrix of the same type and size as src (for dst)
//  dst.create(mGr.size(), mGr.type());

/// Convert the image to grayscale
//   cvtColor(mGr, gray, CV_BGR2GRAY);

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
//   mGr.copyTo (mGr, dst);

//  GaussianBlur(&mGr, &mGr, )

//extern "C" JNIEXPORT jstring
/*
JNICALL
Java_com_handen_roadhelper_MainActivity_stringFromJNI(JNIEnv *env, jobject ) {
    std::string hello = "Ky";
    return env->NewStringUTF(hello.c_str());
}
*/
