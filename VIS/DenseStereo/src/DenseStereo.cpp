#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include <iostream>


cv::Mat calculateDisparityMap( cv::Mat imageLeft, cv::Mat imageRight, int numDisparities, int blockSize )
{
    /*
     *  Calculate disparity map using Block Matching
     */
    cv::Mat disparity;
    cv::Ptr<cv::StereoBM> sbm = cv::StereoBM::create( numDisparities, blockSize );
    sbm->compute(imageLeft, imageRight, disparity);
    return disparity;
}

void visualizeDisparity( cv::Mat disparity )
{
    cv::Mat normalizedDisparity;
    cv::normalize(disparity, normalizedDisparity, 0, 255, cv::NORM_MINMAX, CV_8UC1);    // normalize to [0, 255] to visualize
    cv::imshow("Disparity", normalizedDisparity);
    cv::waitKey(0);
}

int main(int argc, char** argv)
{
    std::cout << "-- Begin --" << std::endl;
    if ( argc != 3 ){
        std::cout << "Usage:" << std::endl;
        std::cout << "leftImgPath.png rightImgPath.png" << std::endl;
        return -1;
    }

    /*******************************************************************
     *      Load images
     *******************************************************************/

    cv::Mat leftImage   = cv::imread(argv[1]);
    cv::Mat rightImage  = cv::imread(argv[2]);
    if ( leftImage.empty() || rightImage.empty() ){
        std::cout << "Error loading images" << std::endl;
        return -1;
    }

    /*******************************************************************
     *  Save colors and cast to gray scale
    *******************************************************************/
    cv::Mat colors = leftImage;                         // Save colors from left image
    cv::cvtColor(leftImage, leftImage, CV_BGR2GRAY);
    cv::cvtColor(rightImage, rightImage, CV_BGR2GRAY);

    /*******************************************************************
     *  Calculate disparity and display
    *******************************************************************/
    int numDisparity = 16, blockSize = 11;
    cv::Mat disparityMap = calculateDisparityMap(leftImage, rightImage, numDisparity, blockSize);
    visualizeDisparity( disparityMap );

    /*******************************************************************
     *  If it is a rectified system:
     *  - Images are from same image plane.
     *  - Depth can be computed from disparity.
    *******************************************************************/
    // Q is recieved from rectification by openCV

//    cv::Mat Q;      // Camera calibrations is needed to define Q.

//    cv::Mat depthMat;
//    cv::reprojectImageTo3D(disparityMap, depthMat, Q);

    // get pointcloud from depthMat


    std::cout << "This is a test" << std::endl;
    return 0;
}
