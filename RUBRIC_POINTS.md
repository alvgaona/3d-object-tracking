# Rubric points

This document gathers information about all the rubric points met for this project.

## FP.1 Match 3D Objects

**CRITERIA**

Implement the method `MatchBoundingBoxes`, which takes as input both the previous and the current data frames and provides as output the ids of the matched regions of interest (i.e. the `id` property).
Matches must be the ones with the highest number of keypoint correspondences.

Implementation can be found [here][MatchBoundingBoxes].

## FP.2 Compute Lidar-based TTC

**CRITERIA**

Compute the time-to-collision in seconds for all matched 3D objects using only Lidar measurements from the matched bounding boxes between current and previous frame.

Implementation can be found [here][ComputeTTCLidar].

## FP.3 Associate Keypoint Correspondences with Bounding Boxes

**CRITERIA**

Prepare the TTC computation based on camera measurements by associating keypoint correspondences to the bounding boxes which enclose them.
All matches which satisfy this condition must be added to a vector in the respective bounding box.

Implementation can be found [here][ClusterKptsMatchesWithROI].
   
## FP.4 Compute Camera-based TTC

**CRITERIA**

Compute the time-to-collision in second for all matched 3D objects using only keypoint correspondences from the matched bounding boxes between current and previous frame.
   
Implementation can be found [here][ComputeTTCCamera].
   
## FP.5 Performance Evaluation 1
   
## FP.6 Performance Evaluation 2

[MatchBoundingBoxes]: src/camera_fusion.cpp#L221
[ComputeTTCLidar]: src/camera_fusion.cpp#L208
[ClusterKptsMatchesWithROI]: src/camera_fusion.cpp#L115
[ComputeTTCCamera]: src/camera_fusion.cpp#L137

