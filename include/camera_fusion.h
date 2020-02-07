#ifndef CAMERA_FUSION_CAMERA_FUSION_H
#define CAMERA_FUSION_CAMERA_FUSION_H

#include <algorithm>
#include <cstdio>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

#include "data_frame.h"

void ClusterLidarWithROI(std::vector<BoundingBox> &bounding_boxes, std::vector<LidarPoint> &lidar_points, float shrink_factor,
                         cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT);
void ClusterKptMatchesWithROI(BoundingBox &bounding_box, std::vector<cv::KeyPoint> &keypoints_prev,
                              std::vector<cv::KeyPoint> &keypoints_current, std::vector<cv::DMatch> &keypoints_matches);
void MatchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bounding_box_best_matches, DataFrame &prev_frame,
                        DataFrame &current_frame);

void Show3DObjects(std::vector<BoundingBox> &bounding_boxes, const cv::Size &world_size, const cv::Size &image_size, bool wait = true);

void ComputeTTCCamera(std::vector<cv::KeyPoint> &keypoints_prev, std::vector<cv::KeyPoint> &keypoints_current,
                      const std::vector<cv::DMatch> &keypoints_matches, double frame_rate, double &TTC, cv::Mat *visible_image = nullptr);

void ComputeTTCLidar(std::vector<LidarPoint> &lidar_points_prev, std::vector<LidarPoint> &lidar_points_current, double frame_rate,
                     double &TTC);

#endif
