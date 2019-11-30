#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>


/*
 * Contains methods for evaluating the performance of the sparse stereo methods.
 *
 *
 */





// 1) 30 gange evalueret for hver gaussian versus euclidian distance hen til rigtige for 30 positioner.

cv::Mat add_gaussian_noise(float mean, float sigma, cv::Mat pic){
    cv::Mat result = pic.clone();
    cv::Mat noise(pic.size(), CV_32SC3);
    cv::randn(noise, mean, sigma); //mean and variance

    for(unsigned i = 0; i < pic.rows; i++)
    {
        for(unsigned j = 0; j < pic.cols; j++)
        {
            if ( result.at<cv::Vec3b>(i,j)[0] + noise.at<cv::Vec3i>(i, j)[0] > 255 )
                result.at<cv::Vec3b>(i,j)[0] = 255;
            else if ( result.at<cv::Vec3b>(i,j)[0] + noise.at<cv::Vec3i>(i, j)[0] < 0 )
                result.at<cv::Vec3b>(i,j)[0] = 0;
            else
                result.at<cv::Vec3b>(i,j)[0] += noise.at<cv::Vec3i>(i, j)[0];

            if ( result.at<cv::Vec3b>(i,j)[1] + noise.at<cv::Vec3i>(i, j)[1] > 255 )
                result.at<cv::Vec3b>(i,j)[1] = 255;
            else if ( result.at<cv::Vec3b>(i,j)[1] + noise.at<cv::Vec3i>(i, j)[1] < 0 )
                result.at<cv::Vec3b>(i,j)[1] = 0;
            else
               result.at<cv::Vec3b>(i,j)[1] + noise.at<cv::Vec3i>(i, j)[1];

            if ( result.at<cv::Vec3b>(i,j)[2] + noise.at<cv::Vec3i>(i, j)[2] > 255 )
                result.at<cv::Vec3b>(i,j)[2] = 255;
            else if ( result.at<cv::Vec3b>(i,j)[2] + noise.at<cv::Vec3i>(i, j)[2] < 0 )
                result.at<cv::Vec3b>(i,j)[2] = 0;
            else
                result.at<cv::Vec3b>(i,j)[2] + noise.at<cv::Vec3i>(i, j)[2];
        }
    }

    return result;
}
