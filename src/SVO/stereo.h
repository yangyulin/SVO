//
// Created by lindeyang on 11/30/16.
//

#ifndef SVO_STEREO_H
#define SVO_STEREO_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <libviso2/matcher.h>
#include <libviso2/viso_stereo.h>
#include "libviso2/matrix.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/StereoCamera.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/slam/StereoFactor.h>



using namespace libviso2;
using namespace std;

namespace SVO {
    void cvtMatrix2Eigen(const Matrix &Min, Eigen::MatrixXd &Mout);
    void cvtMatrix2RT(const Matrix &Min, gtsam::Rot3 &R, gtsam::Point3 &T);
    void cvtgtPose2RT(const gtsam::Pose3 &pose, libviso2::Matrix &ptrM);

    void localOptimization(const std::vector<libviso2::Matcher::p_match>& pM,
                           const std::vector<int32_t> &inliers,
                           const libviso2::Matrix& pose,
                           const libviso2::VisualOdometryStereo::parameters param,
                           const double sigmaPixel,
                           const gtsam::Cal3_S2Stereo::shared_ptr K,
                           const gtsam::noiseModel::Isotropic::shared_ptr model,
                           libviso2::Matrix &poseOpt);//,
                           //Eigen::Matrix<double, 6,6> &cov);

    // From the matched feature pair to previous and current sterepoints
    void matchedpairs(const libviso2::Matcher::p_match &pM,
                      gtsam::StereoPoint2 &p1,
                      gtsam::StereoPoint2 &p2);


}
#endif //SVO_STEREO_H
