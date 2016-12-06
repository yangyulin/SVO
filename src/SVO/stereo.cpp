//
// Created by lindeyang on 11/30/16.
//

#include <gtsam/nonlinear/Marginals.h>
#include "SVO/stereo.h"

using namespace SVO;

// learning coding: must be carefull that when define the function, I must have the namespace included.
void SVO::cvtMatrix2Eigen(const Matrix &Min, Eigen::MatrixXd &Mout) {
    long m = Min.m;
    long n = Min.n;

    Mout.resize(m,n);
    for (int i=0;i<m; i++){
        for (int j=0;j<n;j++){
            Mout(i,j) = Min.val[i][j];
        }
    }
}


void ::SVO::cvtMatrix2RT(const Matrix &Min, gtsam::Rot3 &R, gtsam::Point3 &T) {
    long m = Min.m;
    long n = Min.n;
    if(m!=4 || n!=4){
        std::cerr<<"Not a Transformation Matrix!!"<<endl;
    }

    R = gtsam::Rot3(Min.val[0][0], Min.val[0][1],Min.val[0][2],
                    Min.val[1][0], Min.val[1][1],Min.val[1][2],
                    Min.val[2][0], Min.val[2][1],Min.val[2][2]);
    //T = gtsam::Point3(gtsam::Vector3(Min.val[0][3], Min.val[1][3], Min.val[2][3]));
    T = gtsam::Point3(Min.val[0][3], Min.val[1][3], Min.val[2][3]);

}


void matchedpairs(const libviso2::Matcher::p_match &pM,
                  gtsam::StereoPoint2 &p1,
                  gtsam::StereoPoint2 &p2){
    p1 = gtsam::StereoPoint2(pM.u1p,pM.u2p,pM.v1p);
    p2 = gtsam::StereoPoint2(pM.u1c,pM.u2c,pM.v1c);
}


void SVO::localOptimization(const std::vector<libviso2::Matcher::p_match> &pM,
                            const std::vector<int32_t> &inliers,
                            const libviso2::Matrix &pose,
                            const libviso2::VisualOdometryStereo::parameters param,
                            const double sigmaPixel,
                            const gtsam::Cal3_S2Stereo::shared_ptr K,
                            const gtsam::noiseModel::Isotropic::shared_ptr model,
                            libviso2::Matrix &poseOpt){//},
                            //Eigen::Matrix<double, 6, 6> &cov) {

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initialEstimate;
    //const gtsam::noiseModel::Isotropic::shared_ptr model = gtsam::noiseModel::Isotropic::Sigma(3,sigmaPixel);

    /// debug
    //std::cout<<endl<< "OK flag1" <<endl;

    //setup the first pose
    //assume the first pose is the origin
    gtsam::Pose3 firstPose;
    graph.push_back(gtsam::NonlinearEquality<gtsam::Pose3>(gtsam::Symbol('x', 1),firstPose));
    initialEstimate.insert(gtsam::Symbol('x',1), firstPose);

    //get the second camera pose and initialize the second pose
    gtsam::Rot3 R_2to1;
    gtsam::Point3 T_2in1;
    cvtMatrix2RT(pose,R_2to1,T_2in1);
    initialEstimate.insert(gtsam::Symbol('x',2),gtsam::Pose3(R_2to1,T_2in1));

    //setup the stereo camera
    gtsam::StereoCamera stereoCam(firstPose,K);

    /// debug
    //cout<<"OK flag2" <<endl;


    size_t counter = 0;
    //construct the points
    for(size_t kk=0; kk<inliers.size()-1; kk++){
        gtsam::StereoPoint2 spt1 = gtsam::StereoPoint2(pM[inliers[kk]].u1p,pM[inliers[kk]].u2p,pM[inliers[kk]].v1p);
        gtsam::StereoPoint2 spt2 = gtsam::StereoPoint2(pM[inliers[kk]].u1c,pM[inliers[kk]].u2c,pM[inliers[kk]].v1c);

        gtsam::Point3 pt = stereoCam.backproject(spt1);
        if(pt.z() <= 40*(K->baseline())){

            counter++;
            initialEstimate.insert(gtsam::Symbol('f',counter),pt);
            graph.push_back(gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>(spt1,model,gtsam::Symbol('x',1), gtsam::Symbol('f',counter),K));
            graph.push_back(gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>(spt2,model,gtsam::Symbol('x',2), gtsam::Symbol('f',counter),K));

        }else{
//            std::cout<<pt.z()<<endl;
        }


    }
    //std::cout<< "OK flag3" <<endl;
    //std::cout<< "size of inliers "<<inliers.size()<<endl;

    //finish the graph and begin to optimization
    gtsam::LevenbergMarquardtParams lmParams;
    lmParams.maxIterations = 10;
    //gtsam::LevenbergMarquardtOptimizer lmOptimizer = gtsam::LevenbergMarquardtOptimizer(graph,initialEstimate,lmParams);
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initialEstimate, lmParams);
    std::cout<<"OK flag31"<<endl;


    //gtsam::Values result = optimizer.optimize();
    gtsam::Values result = optimizer.optimize();
    std::cout<<"OK flag32"<<endl;

    //gtsam::Values result = lmOptimizer.optimize();
    gtsam::Pose3 tempPose = result.at<gtsam::Pose3>(gtsam::Symbol('x',2));

    std::cout<<"OK flag4" <<endl;

    //get the new relative pose pointer
    //poseOpt = new libviso2::Matrix(4,4);
    cvtgtPose2RT(tempPose,poseOpt);
    //std::cout<<endl<<"OK flag5"<<endl;
    //get the covariances
    //gtsam::Marginals marginals(graph,result);
    //std::cout<<endl<<"OK flag51"<<endl;
    //cov = marginals.marginalCovariance(gtsam::Symbol('x',2));
    std::cout<< "OK flag6" <<endl;
    //cout<<"covarianes"<<endl<<cov<<endl;
    //cout<<"covarianes"<<endl<<marginals.marginalCovariance(gtsam::Symbol('x',2))<<endl;
    //delete K;

    std::cout<< "Final result sample: "<<endl;
    gtsam::Values pose_values = result.filter<gtsam::Pose3> ();
    pose_values.print("Final camera poses:\n");

}

void ::SVO::cvtgtPose2RT(const gtsam::Pose3 &pose, libviso2::Matrix &ptrM) {

    ptrM.val[0][0] = pose.matrix()(0,0);
    ptrM.val[1][0] = pose.matrix()(1,0);
    ptrM.val[2][0] = pose.matrix()(2,0);
    ptrM.val[3][0] = pose.matrix()(3,0);
    ptrM.val[0][1] = pose.matrix()(0,1);
    ptrM.val[1][1] = pose.matrix()(1,1);
    ptrM.val[2][1] = pose.matrix()(2,1);
    ptrM.val[3][1] = pose.matrix()(3,1);
    ptrM.val[0][2] = pose.matrix()(0,2);
    ptrM.val[1][2] = pose.matrix()(1,2);
    ptrM.val[2][2] = pose.matrix()(2,2);
    ptrM.val[3][2] = pose.matrix()(3,2);
    ptrM.val[0][3] = pose.matrix()(0,3);
    ptrM.val[1][3] = pose.matrix()(1,3);
    ptrM.val[2][3] = pose.matrix()(2,3);
    ptrM.val[3][3] = pose.matrix()(3,3);

}


