#include <jni.h>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "stats.h" // Stats structure definition
#include "utils.h" // Drawing and printing functions
//#include "Java_com_handen_roadhelper_MainActivity_nativeOnFrame.h"
using namespace std;

using namespace cv;

void find_shapes(Mat mat);

void find_squares(Mat mat);

void convert_to_gray(Mat mat);

void detect_square_sign();

Rect createRect();

Mat pedastrian;

void orb(IplImage *pImage);

extern "C" void JNICALL
Java_com_handen_roadhelper_MainActivity_nativeOnFrame(JNIEnv *env, jobject instance,
                                                      jlong matAddr,
                                                      jint nbrElem) {
    //TODO warpPerspective() позволяет выровнять матрицу на плоскости
    int64 e1 = cv::getTickCount();

    cv::Mat mat = *(Mat *) matAddr;
    //  convert_to_gray(mat); //не работает

    blur(mat, mat, Size(10, 10));

    // inRange(secordMat, Scalar(110,50,50), Scalar(130, 255, 255), mat); //не работает

    find_shapes(mat);

    char cbuff[20];
    int64 e2 = cv::getTickCount();
    float time = (e2 - e1) / cv::getTickFrequency();
    sprintf(cbuff, "%f sec", time);
    putText(mat, cbuff, CvPoint(30, 30), FONT_HERSHEY_COMPLEX, 1.0, cvScalar(255, 255, 255));
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
                if(p->x < min_x )
                    min_x = p->x;
                if(p->y > max_y)
                    max_y = p->y;
                if(p->y < min_y)
                    min_y = p->y;
            }

            CvPoint p1 (min_x, min_y);
            CvPoint p2 (max_x, min_y);
            CvPoint p3 (max_x, max_y);
            CvPoint p4 (min_x, max_y);
            cvLine(img, p1, p2, cvScalar(0, 255, 0), 4);
            cvLine(img, p2, p3, cvScalar(0, 255, 0), 4);
            cvLine(img, p3, p4, cvScalar(0, 255, 0), 4);
            cvLine(img, p4, p1, cvScalar(0, 255, 0), 4);

            cvSetImageROI(img, cvRect(min_x, min_y, max_x - min_x, max_y - min_y));
            orb(img);
     //       cvAddS(img, cvScalar(0, 255, 0), img);

            /*
            //drawing lines around the quadrilateral
            cvLine(img, *pt[0], *pt[1], cvScalar(0, 255, 0), 4);
            cvLine(img, *pt[1], *pt[2], cvScalar(0, 255, 0), 4);
            cvLine(img, *pt[2], *pt[3], cvScalar(0, 255, 0), 4);
            cvLine(img, *pt[3], *pt[0], cvScalar(0, 255, 0), 4);
            */
            detect_square_sign();
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
                                                      jlong matAddr,
                                                      ) {
    pedastrian = *(Mat * ) matAddr;
}

void orb(IplImage *pImage) {

    vector<Point2f> bb;
    FileStorage fs(argv[3], FileStorage::READ);
    if(fs["bounding_box"].empty()) {
        cerr << "Couldn't read bounding_box from " << argv[3] << endl;
        return 1;
    }
    fs["bounding_box"] >> bb;


    Ptr<ORB> orb = ORB::create();
    orb->setMaxFeatures(stats.keypoints);
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
    Tracker akaze_tracker(akaze, matcher);
    Tracker orb_tracker(orb, matcher);

    Mat frame;
    video_in >> frame;
    akaze_tracker.setFirstFrame(frame, bb, "AKAZE", stats);
    orb_tracker.setFirstFrame(frame, bb, "ORB", stats);

    Stats akaze_draw_stats, orb_draw_stats;
    int frame_count = (int)video_in.get(CAP_PROP_FRAME_COUNT);
    Mat akaze_res, orb_res, res_frame;
    for(int i = 1; i < frame_count; i++) {
        bool update_stats = (i % stats_update_period == 0);
        video_in >> frame;

        akaze_res = akaze_tracker.process(frame, stats);
        akaze_stats += stats;
        if(update_stats) {
            akaze_draw_stats = stats;
        }

        orb_tracker.getDetector()->set("nFeatures", stats.keypoints);
        orb_res = orb_tracker.process(frame, stats);
        orb_stats += stats;
        if(update_stats) {
            orb_draw_stats = stats;
        }

        drawStatistics(akaze_res, akaze_draw_stats);
        drawStatistics(orb_res, orb_draw_stats);
        vconcat(akaze_res, orb_res, res_frame);
        video_out << res_frame;
        cout << i << "/" << frame_count - 1 << endl;
    }
    akaze_stats /= frame_count - 1;
    orb_stats /= frame_count - 1;
    printStatistics("AKAZE", akaze_stats);
    printStatistics("ORB", orb_stats);
    return 0;
}

class Tracker
{
public:
    Tracker(Ptr<Feature2D> _detector, Ptr<DescriptorMatcher> _matcher) :
            detector(_detector),
            matcher(_matcher)
    {}

    void setFirstFrame(const Mat frame, vector<Point2f> bb, string title, Stats& stats);
    Mat process(const Mat frame, Stats& stats);
    Ptr<Feature2D> getDetector() {
        return detector;
    }
protected:
    Ptr<Feature2D> detector;
    Ptr<DescriptorMatcher> matcher;
    Mat first_frame, first_desc;
    vector<KeyPoint> first_kp;
    vector<Point2f> object_bb;
};

void Tracker::setFirstFrame(const Mat frame, vector<Point2f> bb, string title, Stats& stats)
{
    first_frame = frame.clone();
    detector->detectAndCompute(first_frame, noArray(), first_kp, first_desc);
    stats.keypoints = (int)first_kp.size();
    drawBoundingBox(first_frame, bb);
    putText(first_frame, title, Point(0, 60), FONT_HERSHEY_PLAIN, 5, Scalar::all(0), 4);
    object_bb = bb;
}

Mat Tracker::process(const Mat frame, Stats& stats)
{
    vector<KeyPoint> kp;
    Mat desc;
    detector->detectAndCompute(frame, noArray(), kp, desc);
    stats.keypoints = (int)kp.size();

    vector< vector<DMatch> > matches;
    vector<KeyPoint> matched1, matched2;
    matcher->knnMatch(first_desc, desc, matches, 2);
    for(unsigned i = 0; i < matches.size(); i++) {
        if(matches[i][0].distance < nn_match_ratio * matches[i][1].distance) {
            matched1.push_back(first_kp[matches[i][0].queryIdx]);
            matched2.push_back(      kp[matches[i][0].trainIdx]);
        }
    }
    stats.matches = (int)matched1.size();

    Mat inlier_mask, homography;
    vector<KeyPoint> inliers1, inliers2;
    vector<DMatch> inlier_matches;
    if(matched1.size() >= 4) {
        homography = findHomography(Points(matched1), Points(matched2),
                                    RANSAC, ransac_thresh, inlier_mask);
    }

    if(matched1.size() < 4 || homography.empty()) {
        Mat res;
        hconcat(first_frame, frame, res);
        stats.inliers = 0;
        stats.ratio = 0;
        return res;
    }
    for(unsigned i = 0; i < matched1.size(); i++) {
        if(inlier_mask.at<uchar>(i)) {
            int new_i = static_cast<int>(inliers1.size());
            inliers1.push_back(matched1[i]);
            inliers2.push_back(matched2[i]);
            inlier_matches.push_back(DMatch(new_i, new_i, 0));
        }
    }
    stats.inliers = (int)inliers1.size();
    stats.ratio = stats.inliers * 1.0 / stats.matches;

    vector<Point2f> new_bb;
    perspectiveTransform(object_bb, new_bb, homography);
    Mat frame_with_bb = frame.clone();
    if(stats.inliers >= bb_min_inliers) {
        drawBoundingBox(frame_with_bb, new_bb);
    }
    Mat res;
    drawMatches(first_frame, inliers1, frame_with_bb, inliers2,
                inlier_matches, res,
                Scalar(255, 0, 0), Scalar(255, 0, 0));
    return res;
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
