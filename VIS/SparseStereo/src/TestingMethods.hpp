#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <fstream>

#include "SparseStereoMethods.hpp"


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

void evaluate_ball_performance(float mean, std::vector<float> std_dev, const std::string test_pic_path, int num_test_pic, const std::string save_file_name, cv::Mat proj_mat_left, cv::Mat proj_mat_right){
    // Loading images
    std::vector<cv::Mat> test_left_ball_pics;
    std::vector<cv::Mat> test_right_ball_pics;

    for(unsigned int i = 0; i < num_test_pic; i++)
    {
        test_left_ball_pics.push_back(cv::imread(test_pic_path + "Camera_Left" + std::to_string(i) + ".png"));
        test_right_ball_pics.push_back(cv::imread(test_pic_path + "Camera_Right" + std::to_string(i) + ".png"));
    }

    // Evaluating performance
    cv::Mat left_eval_pic;
    cv::Mat right_eval_pic;
    cv::Mat ball_pose;
    float cur_std_dev;

    std::ofstream myFile;
    myFile.open(save_file_name);
    // Transform from world to table
    cv::Mat TF_TABLE = (cv::Mat_<double>(4, 4) << 1, 0, 0, 0,
                                                  0, 1, 0, 0,
                                                  0, 0, 1, 0.1,
                                                  0, 0, 0, 1);
    for(unsigned int i = 0; i < std_dev.size(); i++)
    {
        cur_std_dev = std_dev[i];
        for(unsigned int j = 0; j < num_test_pic; j++)
        {
            if(j % 5 == 0)
            {
                std::cout << "Testing standard deviation " << cur_std_dev << " Picture " << j << " out of " << num_test_pic << std::endl;
            }
            left_eval_pic = add_gaussian_noise(0, cur_std_dev, test_left_ball_pics[j]);
            right_eval_pic = add_gaussian_noise(0, cur_std_dev, test_right_ball_pics[j]);

            ball_pose = find_ball_pose(left_eval_pic, right_eval_pic, proj_mat_left, proj_mat_right, false);
            ball_pose = TF_TABLE * ball_pose;
            myFile << cur_std_dev << " " << ball_pose.at<double>(0, 0) << " " << ball_pose.at<double>(0, 1) << " " << ball_pose.at<double>(0, 2) << std::endl;
        }
    }

    myFile.close();
}

cv::Mat colorFiltering(const cv::Mat &input) {
    cv::Mat result, img = input.clone(), hsv, mask;
    //Create trackbars in "Control" window
    cv::namedWindow("Control", cv::WINDOW_AUTOSIZE); //create a window called "Control"
    int lowH = 0, highH = 179, lowS = 0, highS = 255, lowV = 0, highV = 255;
    cv::createTrackbar("LowH", "Control", &lowH, 179); //Hue (0 - 179)
    cv::createTrackbar("HighH", "Control", &highH, 179);
    cv::createTrackbar("LowS", "Control", &lowS, 255); //Saturation (0 - 255)
    cv::createTrackbar("HighS", "Control", &highS, 255);
    cv::createTrackbar("LowV", "Control", &lowV, 255); //Value (0 - 255)
    cv::createTrackbar("HighV", "Control", &highV, 255);
    while (true) {
        cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
        cv::inRange(hsv, cv::Scalar(lowH, lowS, lowV), cv::Scalar(highH, highS, highV), mask); //Threshold the image
        //morphological opening (remove small objects from the foreground)
        cv::erode(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
        cv::dilate(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
        //morphological closing (fill small holes in the foreground)
        cv::dilate(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
        cv::erode(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
        cv::bitwise_and(img, img, result, mask=mask);
        cv::imshow("Thresholded Image", mask); //show the thresholded image
        cv::imshow("Original", img); //show the original image
        cv::imshow("Output", result);
        //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
        if (cv::waitKey(0) == 27) { break; }
    }
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(lowH, lowS, lowV), cv::Scalar(highH, highS, highV), mask);
    cv::erode(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::dilate(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::dilate(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::erode(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::bitwise_and(img, img, result, mask=mask);
    return result;
}
