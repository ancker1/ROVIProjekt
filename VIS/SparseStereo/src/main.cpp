#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include <rw/rw.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rwslibs/rwstudioapp/RobWorkStudioApp.hpp>

#include <iostream>
#include <fstream>

#include "SparseStereoMethods.hpp"
#include "TestingMethods.hpp"

// Function below is a modified example from Frederik from blackboard //
void printProjectionMatrix(std::string frameName, rw::models::WorkCell::Ptr wc, cv::Mat &proj_mat, cv::Mat &cam_mat) {
    rw::kinematics::State state = wc->getDefaultState();
    rw::kinematics::Frame* cameraFrame = wc->findFrame(frameName);
    if (cameraFrame != NULL) {
        if (cameraFrame->getPropertyMap().has("Camera")) {
            // Read the dimensions and field of view
            double fovy;
            int width, height;
            std::string camParam = cameraFrame->getPropertyMap().get<std::string>("Camera");
            std::istringstream iss (camParam, std::istringstream::in);
            iss >> fovy >> width >> height;

            double fovy_pixel = height / 2 / tan(fovy * (2*M_PI) / 360.0 / 2.0 );

            Eigen::Matrix<double, 3, 4> KA;
            KA << fovy_pixel, 0, width / 2.0, 0,
                  0, fovy_pixel, height / 2.0, 0,
                  0, 0, 1, 0;

            // OPENCV //
            cv::Mat KA_opencv = (cv::Mat_<double>(3, 4) << fovy_pixel, 0, width/2.0, 0,
                                                           0, fovy_pixel, height / 2.0, 0,
                                                           0, 0, 1, 0);

            cam_mat = KA_opencv.colRange(0, 3);

            std::cout << "Intrinsic parameters:" << std::endl;
            std::cout << KA << std::endl;

            rw::math::Transform3D<> camPosOGL = cameraFrame->wTf(state); // Transform world to camera
            rw::math::Transform3D<> openGLToVis = rw::math::Transform3D<>(rw::math::RPY<>(-rw::math::Pi, 0, rw::math::Pi).toRotation3D()); // Rotate camera to point towards the table
            rw::math::Transform3D<> H = inverse(camPosOGL * inverse(openGLToVis));

            std::cout << "Extrinsic parameters:" << std::endl;
            std::cout << H.e() << std::endl;

            cv::Mat H_opencv = (cv::Mat_<double>(4, 4) << H.R().getRow(0)[0], H.R().getRow(0)[1], H.R().getRow(0)[2], H.P()[0],
                                                          H.R().getRow(1)[0], H.R().getRow(1)[1], H.R().getRow(1)[2], H.P()[1],
                                                          H.R().getRow(2)[0], H.R().getRow(2)[1], H.R().getRow(2)[2], H.P()[2],
                                                                   0        ,           0       ,          0        ,     1   );
            // Calculates projection matrix opencv
            proj_mat = KA_opencv * H_opencv;
        }
    }
}

int main()
{
    // Load workcell
    static const std::string wc_path = "/home/mikkel/Desktop/Project_WorkCell_Cam/Project_WorkCell/Scene.wc.xml";
    const rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(wc_path);
    if ( wc.isNull() ){
        RW_THROW("Error loading workcell");
        return -1;
    }

    // Gets projection matrix for both cameras
    cv::Mat projection_mat_left, projection_mat_right, cam_left_mat, cam_right_mat;
    printProjectionMatrix("Camera_Left", wc, projection_mat_left, cam_left_mat);
    printProjectionMatrix("Camera_Right", wc, projection_mat_right, cam_right_mat);

    /****************************************************************************************
     **                Sparse stereo Ball center position estimation                       **
     ****************************************************************************************/
    cv::Mat pic_left_ball, pic_right_ball, ball_pose;
    pic_left_ball = cv::imread("/home/mikkel/Dropbox/Semester7/ROVIProjekt/VIS/SparseStereo/pictures/ball/Camera_Left_ball.png");
    pic_right_ball = cv::imread("/home/mikkel/Dropbox/Semester7/ROVIProjekt/VIS/SparseStereo/pictures/ball/Camera_Right_ball.png");
    // Correct ball position [0 0.45 0.175] (XYZ)
    ball_pose = find_ball_pose(pic_left_ball, pic_right_ball, projection_mat_left, projection_mat_right);
    std::cout << ball_pose << std::endl;

    //**************************Performance evaluation*************************************//

    float mean = 0;
    std::vector<float> std_dev{0, 1, 5, 20, 50, 100, 200, 255};
    const std::string test_pic_path = "/home/mikkel/Dropbox/Semester7/ROVIProjekt/VIS/SparseStereo/pictures/ball/performance_pic/";
    int num_test_pic = 30;
    const std::string file_name = "ball_eval_pose.txt";
    const std::string file_name_time = "ball_eval_time.txt";
    evaluate_ball_performance(mean, std_dev, test_pic_path, num_test_pic, file_name, file_name_time, projection_mat_left, projection_mat_right);

    /****************************************************************************************
     **               Sparse stereo duck transformation from base case                     **
     ****************************************************************************************/
    cv::Mat init_image_left, init_image_right, new_image_left, new_image_right;
    std::vector<cv::Point3d> triangulated_duck_dots;
    init_image_left = cv::imread("/home/mikkel/Dropbox/Semester7/ROVIProjekt/VIS/SparseStereo/pictures/duck/Camera_Left_dot_duck.png");
    init_image_right = cv::imread("/home/mikkel/Dropbox/Semester7/ROVIProjekt/VIS/SparseStereo/pictures/duck/Camera_Right_dot_duck.png");
    new_image_left =  cv::imread("/home/mikkel/Dropbox/Semester7/ROVIProjekt/VIS/SparseStereo/pictures/duck/Camera_Left_dot_duck_transformed.png");
    new_image_right = cv::imread("/home/mikkel/Dropbox/Semester7/ROVIProjekt/VIS/SparseStereo/pictures/duck/Camera_Right_dot_duck_transformed.png");

    // Difference in images is 0.1 in x
    std::cout << estimate_6D_pose_dot_duck(init_image_left, init_image_right, new_image_left, new_image_right, projection_mat_left, projection_mat_right) << std::endl;

    //**************************Performance evaluation*************************************//
    float mean_duck = 0;
    std::vector<float> std_dev_duck{0, 1, 5, 20, 50, 100, 200, 255};
    const std::string test_pic_path_duck = "/home/mikkel/Dropbox/Semester7/ROVIProjekt/VIS/SparseStereo/pictures/duck/performance_pic/";

    const std::string file_name_duck = "duck_eval_pose.txt";
    const std::string file_name_time_duck = "duck_eval_time.txt";
    evaluate_duck_performance(init_image_left, init_image_right, mean_duck, std_dev_duck, test_pic_path_duck, 30, file_name_duck, file_name_time_duck, projection_mat_left, projection_mat_right);

    return 0;

}
