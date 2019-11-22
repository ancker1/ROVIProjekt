#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <rw/rw.hpp>
#include <rw/loaders/WorkCellLoader.hpp>

#include <rwslibs/rwstudioapp/RobWorkStudioApp.hpp>

#include <iostream>

void printProjectionMatrix(std::string frameName, rw::models::WorkCell::Ptr wc) {
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

            std::cout << "Ïntrinsic parameters:" << std::endl;
            std::cout << KA << std::endl;

            rw::math::Transform3D<> camPosOGL = cameraFrame->wTf(state);
            rw::math::Transform3D<> openGLToVis = rw::math::Transform3D<>(rw::math::RPY<>(-rw::math::Pi, 0, rw::math::Pi).toRotation3D());
            rw::math::Transform3D<> H = inverse(camPosOGL * inverse(openGLToVis));

            std::cout << "Extrinsic parameters:" << std::endl;
            std::cout << H.e() << std::endl;
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
    // Load camera frames
//    rw::kinematics::Frame* const cam_left = wc->findFrame("Camera_Left");
//    rw::kinematics::Frame* const cam_right = wc->findFrame("Camera_Right");
//    if ( cam_left == nullptr || cam_right == nullptr ) {
//        RW_THROW("Error finding cameras");
//        return -1;
//    }
    // Prints insentrics for both cameras
//    printProjectionMatrix("Camera_Left", wc);
//    printProjectionMatrix("Camera_Right", wc);

    /****************************************************************************************
     **                Find features in the right and left images                          **
     ****************************************************************************************/
    cv::Mat image;
    image = cv::imread("~/Camera_Left.png");
    cv::imshow( "Display window", image );
    cv::waitKey(0); // Wait for a keystroke in the window








    return 0;
}
