#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <fstream>
#include <ctime>
#include <chrono>

#include "SparseStereoMethods.hpp"


/*
 * Contains methods for evaluating the performance of the two sparse stereo methods.
 *
 *
 */

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

void evaluate_ball_performance(float mean, std::vector<float> std_dev, const std::string test_pic_path, int num_test_pic, const std::string save_file_name_estimation, const::std::string save_file_name_time, cv::Mat proj_mat_left, cv::Mat proj_mat_right){
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
    std::ofstream myFileTime;
    myFileTime.open(save_file_name_time);
    myFile.open(save_file_name_estimation);
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

            //Logging time

            auto start = std::chrono::high_resolution_clock::now();
            ball_pose = find_ball_pose(left_eval_pic, right_eval_pic, proj_mat_left, proj_mat_right);
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start);



            ball_pose = TF_TABLE * ball_pose;
            myFile << cur_std_dev << " " << ball_pose.at<double>(0, 0) << " " << ball_pose.at<double>(0, 1) << " " << ball_pose.at<double>(0, 2) << std::endl;
            myFileTime << duration.count() << std::endl;
        }
    }
    myFileTime.close();
    myFile.close();
}

void evaluate_duck_performance(cv::Mat init_pic_left, cv::Mat init_pic_right, float mean, std::vector<float> std_dev, const std::string test_pic_path, int num_test_pic, const std::string save_file_name_estimation, const::std::string save_file_name_time, cv::Mat proj_mat_left, cv::Mat proj_mat_right)
{
    Eigen::Matrix4f duck_pose;

    // Loading images
    std::vector<cv::Mat> test_left_duck_pics;
    std::vector<cv::Mat> test_right_duck_pics;

    for(unsigned int i = 0; i < num_test_pic; i++)
    {
        test_left_duck_pics.push_back(cv::imread(test_pic_path + "Camera_Left" + std::to_string(i) + ".png"));
        test_right_duck_pics.push_back(cv::imread(test_pic_path + "Camera_Right" + std::to_string(i) + ".png"));
    }

    std::ofstream myFile;
    std::ofstream myFileTime;
    myFileTime.open(save_file_name_time);
    myFile.open(save_file_name_estimation);

    float cur_std_dev;
    cv::Mat left_eval_pic, right_eval_pic;
    for(unsigned int i = 0; i < std_dev.size(); i++)
    {
        cur_std_dev = std_dev[i];
        for(unsigned int j = 0; j < num_test_pic; j++)
        {
            if(j % 5 == 0)
            {
                std::cout << "Testing standard deviation " << cur_std_dev << " Picture " << j << " out of " << num_test_pic << std::endl;
            }
            left_eval_pic = add_gaussian_noise(0, cur_std_dev, test_left_duck_pics[j]);
            right_eval_pic = add_gaussian_noise(0, cur_std_dev, test_right_duck_pics[j]);

            //Logging time

            auto start = std::chrono::high_resolution_clock::now();
            duck_pose = estimate_6D_pose_dot_duck(init_pic_left, init_pic_right, left_eval_pic, right_eval_pic, proj_mat_left, proj_mat_right);
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start);



            myFile << duck_pose << std::endl;
            myFileTime << duration.count() << std::endl;
        }
    }
    myFileTime.close();
    myFile.close();

}
