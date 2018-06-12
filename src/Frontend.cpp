/*
 * Frontend.cpp
 *
 *  Created on: 9 Dec 2016
 *      Author: sleutene
 */

#include <arp/Frontend.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>

namespace arp {

void Frontend::setCameraParameters(int imageWidth, int imageHeight,
                                   double focalLengthU, double focalLengthV,
                                   double imageCenterU, double imageCenterV,
                                   double k1, double k2, double p1, double p2)
{
  camera_.reset(
      new arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion>(
          imageWidth, imageHeight, focalLengthU, focalLengthV, imageCenterU,
          imageCenterV,
          arp::cameras::RadialTangentialDistortion(k1, k2, p1, p2)));
  camera_->initialiseUndistortMaps();
}

// the undistorted camera model used for the estimator (later)
arp::cameras::PinholeCamera<arp::cameras::NoDistortion>
    Frontend::undistortedCameraModel() const {
  assert(camera_);
  return camera_->undistortedPinholeCamera();
}

int Frontend::detect(const cv::Mat& image, DetectionVec & detections)
{
  // undistort image
  cv::Mat undistortedImg; cv::Mat grayImg;
  cv::cvtColor(image, grayImg, CV_BGR2GRAY);
  camera_->undistortImage(grayImg, undistortedImg);
  //std::cout << grayImg << std::endl;
  // extract AprilTags
  std::vector<AprilTags::TagDetection> aprilTags = tagDetector_.extractTags(undistortedImg);
  int noDectection = 0;
  
  for(int i = 0; i != aprilTags.size(); i++) {

      AprilTags::TagDetection currTag = aprilTags[i];
      int id = currTag.id;

      if(idToSize_.find(id) != idToSize_.end()){

        Detection newDectection;
        newDectection.id = id;
        newDectection.T_CT = kinematics::Transformation(currTag.getRelativeTransform(idToSize_[id],
          camera_->undistortedPinholeCamera().focalLengthU(), camera_->undistortedPinholeCamera().focalLengthV(), camera_->undistortedPinholeCamera().imageCenterU(),
        camera_->undistortedPinholeCamera().imageCenterV()));

        newDectection.points << currTag.p[0].first, currTag.p[1].first, currTag.p[2].first, currTag.p[3].first,
                                currTag.p[0].second, currTag.p[1].second, currTag.p[2].second, currTag.p[3].second;
        detections.push_back(newDectection);
        noDectection ++;
    }
  }
  return noDectection;
}

bool Frontend::setTarget(unsigned int id, double targetSizeMeters) {
  idToSize_[id] = targetSizeMeters;
  return true;
}

}  // namespace arp
