#ifndef PLANARSEGMENT_H
#define PLANARSEGMENT_H
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/cv.h>
//#include <opencv2/highgui.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;

void extractLines(cv::Mat &depthMat, cv::Mat &edgeMat, float slopeThresh, cv::Mat &lineMat);
#endif // PLANARSEGMENT_H
