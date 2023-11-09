// triangulate.cpp
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types_c.h>

extern "C" void TriangulatePoints(
    double* projMatr1, double* projMatr2, double* projPoints1, double* projPoints2, int n, double* points4D) {
    cv::Mat P1(3, 4, CV_64F, projMatr1);
    cv::Mat P2(3, 4, CV_64F, projMatr2);
    cv::Mat points1(2, n, CV_64F, projPoints1);
    cv::Mat points2(2, n, CV_64F, projPoints2);
    cv::Mat points4Dmat(4, n, CV_64F, points4D);

    cv::triangulatePoints(P1, P2, points1, points2, points4Dmat);
}
