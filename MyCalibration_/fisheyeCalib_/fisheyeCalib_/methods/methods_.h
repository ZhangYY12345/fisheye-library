#pragma once
#include <opencv2/opencv.hpp>

void calcOpticalFlow(cv::Mat img1, cv::Mat img2);

void processAllImages(std::string imgFilePath, std::string cameraParaPath);
cv::Mat detectEdges(cv::Mat& image, cv::Mat& mask);
std::vector<std::vector<cv::Point2i> > extractEdges(cv::Mat& image);
std::vector<std::vector<cv::Point2i> > clusteringEdges(std::vector<cv::Point2i> points);
std::vector<std::vector<cv::Point2i> > detectValley(cv::Mat &img1, cv::Mat &img2);
std::vector<std::vector<cv::Point2i> > detectLines(cv::Mat &img1, cv::Mat &img2);

