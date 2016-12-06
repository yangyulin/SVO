/*
  Usage: ./main_SVO_gt path/to/sequence/00
*/

#include <iostream>
#include <vector>

#include "libviso2/viso_stereo.h"
#include <png++/png.hpp>


#include "SVO/stereo.h"
#include "SVO/drawer.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <boost/filesystem.hpp>

using namespace std;
using namespace cv;


int main (int argc, char** argv) {

    // we need the path name to odometry sequence 00 as input argument
    if (argc<2) {
        cerr << "Usage: ./main_SVO_gt path/to/sequence/00" << endl;
        return 1;
    }

    // sequence directory
    string dir = argv[1];

    string poseDir = "/home/lindeyang/workspace/odometry_data/poses/";
    string poseFileName = poseDir + "00.txt";
    std::cout<<poseFileName<<endl;


    // set most important visual odometry parameters
    // for a full parameter list, look at: viso_stereo.h
    libviso2::VisualOdometryStereo::parameters param;

    // calibration parameters for odometery sequence 00
    param.calib.f  = 718.85; // focal length in pixels
    param.calib.cu = 607.19; // principal point (u-coordinate) in pixels
    param.calib.cv = 185.21; // principal point (v-coordinate) in pixels
    param.base     = 0.5371; // baseline in meters

    //setup the camera calibration
    double fx, fy, s, px, py, b;
    fx = param.calib.f;
    fy = param.calib.f;
    s = 0;
    px = param.calib.cu;
    py = param.calib.cv;
    b = param.base;
    // construct the stereo calibration shared pointer
    const gtsam::Cal3_S2Stereo::shared_ptr K(new gtsam::Cal3_S2Stereo(fx,fy,s,px,py,b));


    // std for the image noises
    double sigmaPixel =2;
    const gtsam::noiseModel::Isotropic::shared_ptr model = gtsam::noiseModel::Isotropic::Sigma(3,sigmaPixel);

    // init visual odometry
    libviso2::VisualOdometryStereo* viso = new libviso2::VisualOdometryStereo(param);
    std::vector<int32_t> inliers;
    std::vector<libviso2::Matcher::p_match> pM;

    //// For display the pose, begin the thread
    // init the drawer
    SVO::drawer poseDrawer(poseFileName.c_str());
    // Start the drawer thread
    boost::thread threadDrawer(boost::bind(&SVO::drawer::start, poseDrawer));


    // current pose (this matrix transforms a point from the current
    // frame's camera coordinates to the first frame's camera coordinates)
    libviso2::Matrix pose = libviso2::Matrix::eye(4);
    libviso2::Matrix poselib = libviso2::Matrix::eye(4); //libviso2 results

    ////// the third pointer
    std::vector<libviso2::Matrix>* vptrPose = new std::vector<libviso2::Matrix>();
    std::vector<libviso2::Matrix>* vptrPoselib = new std::vector<libviso2::Matrix>();
    vptrPose->push_back(pose);
    vptrPoselib->push_back(poselib);
    // Update pose drawer
    poseDrawer.updatePoses(vptrPose);
    poseDrawer.updatePoseslib(vptrPoselib);


    // loop through all frames i=0:372
    // TODO: also consider to read in the ground truth
    for (int32_t i=0; i<4500; i++) {

        // input file names
        char base_name[256]; sprintf(base_name,"%06d.png",i);
        string left_img_file_name  = dir + "/image_0/" + base_name;
        string right_img_file_name = dir + "/image_1/" + base_name;
        std::cout<<"OK flag7 the files are successfully read"<<endl;

        // catch image read/write errors here
        try {

            // load left and right input image
            png::image< png::gray_pixel > left_img(left_img_file_name);
            png::image< png::gray_pixel > right_img(right_img_file_name);
            std::cout<<"Images are loaded"<<endl;

            // image dimensions
            int32_t width  = left_img.get_width();
            int32_t height = left_img.get_height();

            // convert input images to uint8_t buffer
            //////// the fourth and fifth order
            uint8_t* left_img_data  = (uint8_t*)malloc(width*height*sizeof(uint8_t));
            uint8_t* right_img_data = (uint8_t*)malloc(width*height*sizeof(uint8_t));
            std::cout<<"image memory are successfully allocated!"<<endl;
            int32_t k=0;
            for (int32_t v=0; v<height; v++) {
                for (int32_t u=0; u<width; u++) {
                    left_img_data[k]  = left_img.get_pixel(u,v);
                    right_img_data[k] = right_img.get_pixel(u,v);
                    k++;
                }
            }

            //std::cout<<"OK flag8"<<endl;

            // status
            cout << "Processing: Frame: " << i;

            // compute visual odometry
            int32_t dims[] = {width,height,width};
            if (viso->process(left_img_data,right_img_data,dims)) {

                std::cout<<endl<<"OK flag9: the frames are successfully processed"<<endl;

                // on success, update current pose
                //if(i==0)  continue;
                // get the matches
                pM.clear();
                inliers.clear();
                pM = viso->getMatches();
                inliers = viso->getInlierIndices();
                //libviso2::Matrix poseInit = libviso2::Matrix::inv(viso->getMotion());
                libviso2::Matrix poseInit = libviso2::Matrix::eye(4);

                std::cout<<"The initial values:"<<endl;
                std::cout<<poseInit<<endl;

                libviso2::Matrix poseOpt(4,4);
                //Eigen::Matrix<double, 6,6> cov;

                std::cout<<endl<<"OK flag10"<<endl;

                //TODO: Local Optimization to get an optimized relative pose and feature point
                SVO::localOptimization(pM,
                                       inliers,
                                       poseInit,
                                       param,
                                       sigmaPixel,
                                       K,
                                       model,
                                       poseOpt);//,
                                       //cov);

                //pose = pose * libviso2::Matrix::inv(*poseOpt);
                pose = pose * (poseOpt);
                vptrPose->push_back(pose);

                poselib = poselib * libviso2::Matrix::inv(viso->getMotion());
                vptrPoselib->push_back(poselib);

                //delete poseOpt;
                //TODO: Local marginalization of feature points ( 2 ways)


                //TODO: Global Optimization or sliding window optimization


                // output some statistics
                double num_matches = viso->getNumberOfMatches();
                double num_inliers = viso->getNumberOfInliers();
                cout << ", Matches: " << num_matches;
                cout << ", Inliers: " << 100.0*num_inliers/num_matches << " %" << ", Current pose: " << endl;
                //SVO::cvtMatrix2Eigen(pose,poseEigen);
                //cout << poseEigen << endl << endl;
                //cout << pose << endl << endl;


                //TODO: Display the pose and the feature points
                // use pangolin and learn from Orb SLAM2 to display the camera poses
                poseDrawer.updatePoses(vptrPose);
                poseDrawer.updatePoseslib(vptrPoselib);//aws

                std::cout<<"OK flag11"<<endl;

                // Display nice image
                //cv::Mat img_C1 = cv::Mat(height, width, CV_8UC1, *left_img_data);
                //cv::Mat img_C2 = cv::Mat(height, width, CV_8UC1, *right_img_data);
                //cv::Mat img_C1 = Mat(height, width, CV_8UC1, left_img_data);
                //cv::Mat img_C2 = Mat(height, width, CV_8UC1, right_img_data);
                cv::Mat img_C1 =imread(left_img_file_name,IMREAD_GRAYSCALE);
                cv::Mat img_C2 =imread(right_img_file_name,IMREAD_GRAYSCALE);
                cv::Size sz1 = img_C1.size();
                cv::Size sz2 = img_C2.size();

                // Create combined matrix
                cv::Mat im3(sz1.height+sz2.height+5, sz1.width, CV_8UC1);
                cv::Mat left(im3, cv::Rect(0, 0, sz1.width, sz1.height));
                img_C1.copyTo(left);
                cv::Mat right(im3, cv::Rect(0, sz1.height+5, sz2.width, sz2.height));
                img_C2.copyTo(right);

                // Convert to color type
                cv::cvtColor(im3, im3, CV_GRAY2RGB);

                // ==============================
                // Add extracted features to mat
                // ==============================
                for(size_t ki=0; ki<pM.size(); ki++) {
                    // Get match data
                    libviso2::Matcher::p_match match;
                    match = pM.at(ki);
                    // Point color object
                    cv::Scalar color;
                    // Check to see if inlier, if so color it as one
                    // The outliers are red
                    if(std::find(inliers.begin(), inliers.end(), ki) != inliers.end()) {
                        color = CV_RGB(0,255,255);
                    } else {
                        color = CV_RGB(255,0,0);
                    }
                    // Add to image
                    cv::Point2f pt_left(match.u1c, match.v1c);
                    cv::circle(im3, pt_left, 2, color);
                    cv::Point2f pt_right(match.u2c, match.v2c + (sz1.height+5));
                    cv::circle(im3, pt_right, 2, color);
                }


                // Display it
                cv::imshow("Stereo Gray Image", im3);
                cv::waitKey(10);

                // Free the data once done
                //im3.release();

                // Done delete them
                // Memory will grow unbounded otherwise
                img_C1.release();
                img_C2.release();


            } else {
                cout << " ... failed!" << endl;
                free(left_img_data);
                free(right_img_data);
            }

            // release uint8_t buffers
            //free(left_img_data);
            //free(right_img_data);

            // catch image read errors here
        } catch (...) {
            cerr << "ERROR: Couldn't read input files!" << endl;
            return 1;
        }
    }

    // Stop the visualization thread if it was running
    // https://www.quantnet.com/threads/c-multithreading-in-boost.10028/

    while (!threadDrawer.timed_join(boost::posix_time::seconds(1))) {
        threadDrawer.interrupt();
    }


    // output
    cout << "main_libviso2 complete! Exiting ..." << endl;

    // exit
    return 0;
}
