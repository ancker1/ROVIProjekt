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


struct Camera {
    cv::Mat intrinsic;
    cv::Mat transformation;
    cv::Mat distortion;
    cv::Mat projection;
    cv::Mat translation;
    cv::Mat rotation;
    double image_width;
    double image_height;

    void printData() {
        std::cout << image_width << " " << image_height << "\n" << intrinsic << "\n"
                << distortion << "\n" << transformation << "\n" << projection
                << std::endl;
    }
};

void printProjectionMatrix(std::string frameName, rw::models::WorkCell::Ptr wc, cv::Mat &proj_mat) {
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
            std::cout << "test " << KA_opencv << std::endl;


            std::cout << "Intrinsic parameters:" << std::endl;
            std::cout << KA << std::endl;

            rw::math::Transform3D<> camPosOGL = cameraFrame->wTf(state);
            rw::math::Transform3D<> openGLToVis = rw::math::Transform3D<>(rw::math::RPY<>(-rw::math::Pi, 0, rw::math::Pi).toRotation3D());
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

cv::Mat color_threshold_smiley(cv::Mat pic){
    cv::Mat image_HSV, mask, mask_yellow, mask_blue, mask_red, smiley;
    cv::cvtColor(pic, image_HSV, cv::COLOR_BGR2HSV);
    cv::inRange(image_HSV, cv::Scalar(25, 102, 153), cv::Scalar(28, 255, 255), mask_yellow);
    cv::inRange(image_HSV, cv::Scalar(118, 250, 153), cv::Scalar(122, 255, 255), mask_blue);
    cv::inRange(image_HSV, cv::Scalar(0, 250, 153), cv::Scalar(4, 255, 255), mask_red);
    mask = mask_yellow + mask_red + mask_blue;
    cv::cvtColor(mask, mask, cv::COLOR_GRAY2BGR);

    cv::bitwise_and(pic, mask, smiley);
    return smiley;
}

cv::Mat color_threshold_duck(cv::Mat pic){

    cv::Mat image_HSV, mask, mask_body, mask_beak, duck;
    cv::cvtColor(pic, image_HSV, cv::COLOR_BGR2HSV);
    cv::inRange(image_HSV, cv::Scalar(15, 120, 65), cv::Scalar(20, 255, 255), mask_body);
    cv::inRange(image_HSV, cv::Scalar(20, 150, 55), cv::Scalar(36, 255, 255), mask_beak);
    mask = mask_body + mask_beak;

    // Opening and closing to get whole duck
    cv::Mat five_by_five( 2, 2, CV_8U, cv::Scalar(1) );
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, five_by_five);
    cv::Mat fifteen_by_fifteen( 8, 8, CV_8U, cv::Scalar(1) );
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, fifteen_by_fifteen);

    cv::cvtColor(mask, mask, cv::COLOR_GRAY2BGR);

    cv::bitwise_and(pic, mask, duck);
    return duck;

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
    // Load camera frames
//    rw::kinematics::Frame* const cam_left = wc->findFrame("Camera_Left");
//    rw::kinematics::Frame* const cam_right = wc->findFrame("Camera_Right");
//    if ( cam_left == nullptr || cam_right == nullptr ) {
//        RW_THROW("Error finding cameras");
//        return -1;
//    }
    // Gets projection matrix for both cameras
    cv::Mat projection_mat_left, projection_mat_right;
    printProjectionMatrix("Camera_Left", wc, projection_mat_left);
    printProjectionMatrix("Camera_Right", wc, projection_mat_right);
//    std::cout << "Proj Left: " << projection_mat_left << std::endl;
//    std::cout << "Proj Right: " << projection_mat_right << std::endl;


    /****************************************************************************************
     **                Find features in the right and left images                          **
     ****************************************************************************************/
    cv::Mat pic_left, pic_right, result;
    pic_left = cv::imread("/home/mikkel/Camera_Left.png");
    pic_right = cv::imread("/home/mikkel/Camera_Right.png");
    // Smiley
    //pic_left = color_threshold_smiley(pic_left);
    //pic_right = color_threshold_smiley(pic_right);
    // Duck
    pic_left = color_threshold_duck(pic_left);
    pic_right = color_threshold_duck(pic_right);
    /************************** SIFT ********************************************************/



    cv::Ptr<cv::xfeatures2d::SIFT> sift = cv::xfeatures2d::SIFT::create();
    std::vector<cv::KeyPoint> keypoint_left, keypoint_right;
    cv::Mat descriptor_left, descriptor_right;

    sift->detectAndCompute(pic_left, cv::noArray(), keypoint_left, descriptor_left);
    sift->detectAndCompute(pic_right, cv::noArray(), keypoint_right, descriptor_right);

    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    std::vector<std::vector<cv::DMatch>> knn_matches;
    matcher->knnMatch( descriptor_left, descriptor_right, knn_matches, 2 );

    const float ratio_thresh = 0.3f;
    std::vector<cv::DMatch> good_matches;
    for (size_t i = 0; i < knn_matches.size(); i++)
    {
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
        {
            good_matches.push_back(knn_matches[i][0]);

        }
    }

    //std::cout << keypoint_left[1].pt << std::endl;
//    cv::drawMatches(pic_left, keypoint_left, pic_right, keypoint_right, good_matches, result);
//    cv::imshow("sift matching result", result);

    //cv::drawKeypoints(pic_left, keypoint_left, pic_left);
    //cv::imshow("Left_img", pic_left);
//    cv::waitKey(0);
    /****************************************************************************************
     **                   Triangulates the found features mathces                          **
     ****************************************************************************************/
/*
    cv::Mat triangulate_point(1, 1, CV_64FC4);
    cv::Mat left_point(1, 1, CV_64FC2);
    cv::Mat right_point(1, 1, CV_64FC2);

    left_point.at<cv::Vec2d>(0)[0] = keypoint_left[1].pt.x;
    left_point.at<cv::Vec2d>(0)[1] = keypoint_left[1].pt.y;
    right_point.at<cv::Vec2d>(0)[0] = keypoint_right[1].pt.x;
    right_point.at<cv::Vec2d>(0)[1] = keypoint_right[1].pt.y;

    cv::triangulatePoints(projection_mat_left, projection_mat_right, left_point, right_point, triangulate_point);

    std::cout << triangulate_point << std::endl;
*/
    std::vector<cv::KeyPoint> good_keypoint_left, good_keypoint_right;
    for(int i = 0; i < good_matches.size(); i++)
    {
        good_keypoint_left.push_back(keypoint_left[good_matches[i].queryIdx]);
        good_keypoint_right.push_back(keypoint_right[good_matches[i].queryIdx]);
        //std::cout << good_matches[i].queryIdx << std::endl;
        std::cout << "Left " << good_keypoint_left[i].pt << std::endl;
        std::cout << "Right " <<good_keypoint_right[i].pt << std::endl;
    }
    std::cout <<
    cv::drawMatches(pic_left, keypoint_left, pic_right, keypoint_right, good_matches, result, cv::Scalar::all(-1),
            cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    cv::imshow("sift matching result", result);
    cv::waitKey(0);

    return 0;
}
