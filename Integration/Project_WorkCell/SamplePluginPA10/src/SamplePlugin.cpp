#include "SamplePlugin.hpp"

#include "pose_estimation.hpp"
#include "interpolator.hpp"
#include "rrt.hpp"

SamplePlugin::SamplePlugin():
	RobWorkStudioPlugin("SamplePluginUI", QIcon(":/pa_icon.png")) 
{
	setupUi(this);

	_timer = new QTimer(this);
    connect(_timer, SIGNAL(timeout()), this, SLOT(timer()));

	// now connect stuff from the ui component
	connect(_btn_im    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn_scan    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn1    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn_run    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_btn_m2    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_btn_showPose    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_btn_rrt    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );

	_framegrabber = NULL;
	
	_cameras = {"Camera_Right", "Camera_Left"};
	_cameras25D = {"Scanner25D"};
}

SamplePlugin::~SamplePlugin() 
{
    delete _textureRender;
    delete _bgRender;
}

void SamplePlugin::initialize() 
{
	log().info() << "INITALIZE" << "\n";

    getRobWorkStudio()->stateChangedEvent().add(std::bind(&SamplePlugin::stateChangedListener, this, std::placeholders::_1), this);

	// Auto load workcell
    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load("../../../Scene.wc.xml");
	getRobWorkStudio()->setWorkCell(wc);

}

void SamplePlugin::open(rw::models::WorkCell* workcell)
{
    log().info() << "OPEN" << "\n";
    _wc = workcell;
    _state = _wc->getDefaultState();

    log().info() << workcell->getFilename() << "\n";

    if (_wc != NULL) {
	// Add the texture render to this workcell if there is a frame for texture
    rw::kinematics::Frame* textureFrame = _wc->findFrame("MarkerTexture");
	if (textureFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->addRender("TextureImage",_textureRender,textureFrame);
	}
	// Add the background render to this workcell if there is a frame for texture
    rw::kinematics::Frame* bgFrame = _wc->findFrame("Background");
	if (bgFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->addRender("BackgroundImage",_bgRender,bgFrame);
	}

	// Create a GLFrameGrabber if there is a camera frame with a Camera property set
    rw::kinematics::Frame* cameraFrame = _wc->findFrame(_cameras[0]);
	if (cameraFrame != NULL) {
		if (cameraFrame->getPropertyMap().has("Camera")) {
			// Read the dimensions and field of view
			double fovy;
            int width, height;
			std::string camParam = cameraFrame->getPropertyMap().get<std::string>("Camera");
			std::istringstream iss (camParam, std::istringstream::in);
			iss >> fovy >> width >> height;
			// Create a frame grabber
            _framegrabber = new rwlibs::simulation::GLFrameGrabber(width,height,fovy);
            rw::graphics::SceneViewer::Ptr gldrawer = getRobWorkStudio()->getView()->getSceneViewer();
			_framegrabber->init(gldrawer);
		}
	}
	
    rw::kinematics::Frame* cameraFrame25D = _wc->findFrame(_cameras25D[0]);
	if (cameraFrame25D != NULL) {
		if (cameraFrame25D->getPropertyMap().has("Scanner25D")) {
			// Read the dimensions and field of view
			double fovy;
			int width,height;
			std::string camParam = cameraFrame25D->getPropertyMap().get<std::string>("Scanner25D");
			std::istringstream iss (camParam, std::istringstream::in);
			iss >> fovy >> width >> height;
			// Create a frame grabber
            _framegrabber25D = new rwlibs::simulation::GLFrameGrabber25D(width,height,fovy);
            rw::graphics::SceneViewer::Ptr gldrawer = getRobWorkStudio()->getView()->getSceneViewer();
			_framegrabber25D->init(gldrawer);
		}
	}

    _duck  = _wc->findFrame<rw::kinematics::MovableFrame>("Duck");
    if ( _duck == nullptr )
        RW_THROW("Duck frame not found.");
    
    _table = _wc->findFrame<rw::kinematics::Frame>("Table");
    if ( _table == nullptr )
        RW_THROW("Table frame not found.");
    
    _tcp   = _wc->findFrame<rw::kinematics::Frame>("UR-6-85-5-A.TCP");
    if ( _tcp == nullptr )
        RW_THROW("TCP frame not found.");

    _UR6 = _wc->findDevice<rw::models::SerialDevice>("UR-6-85-5-A");
    if ( _UR6 == nullptr )
        RW_THROW("Device UR6 not found.");

    _scanner25D = _wc->findFrame<rw::kinematics::MovableFrame>("Scanner25D");
    if ( _scanner25D == nullptr ){
        RW_THROW("Error finding frame: Scanner25D");
    }

    _target = _wc->findFrame<rw::kinematics::MovableFrame>("GraspTarget");
    if ( _target == nullptr )
        RW_THROW("Error finding frame: GraspTarget");
        

    _device = _wc->findDevice("UR-6-85-5-A");
    _step = -1;
	
    }
}


void SamplePlugin::close() {
    log().info() << "CLOSE" << "\n";

    // Stop the timer
    _timer->stop();
    // Remove the texture render
    rw::kinematics::Frame* textureFrame = _wc->findFrame("MarkerTexture");
	if (textureFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->removeDrawable("TextureImage",textureFrame);
	}
	// Remove the background render
    rw::kinematics::Frame* bgFrame = _wc->findFrame("Background");
	if (bgFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->removeDrawable("BackgroundImage",bgFrame);
	}
	// Delete the old framegrabber
	if (_framegrabber != NULL) {
		delete _framegrabber;
	}
	_framegrabber = NULL;
	_wc = NULL;
}

cv::Mat SamplePlugin::toOpenCVImage(const rw::sensor::Image& img) {
    cv::Mat res(img.getHeight(),img.getWidth(), CV_8SC3);
	res.data = (uchar*)img.getImageData();
	return res;
}


void SamplePlugin::btnPressed() {
    QObject *obj = sender();
     if(obj==_btn1){
        log().info() << "Button 1\n";
        std::cout << "Duck pick location: " << _duck->getTransform(_state).P() << std::endl;
        // Toggle the timer on and off
        if (!_timer->isActive()){
            _timer->start(10); // run 100 Hz
            _step = 0;
        }
        else
            _step = 0;

	} 
	else if( obj==_btn_im ){
		getImage();
	}
	else if( obj==_btn_scan ){
		get25DImage();
	}
	else if( obj==_btn_run ){
		runLinearInterpolation();	
	} else if ( obj == _btn_m2 ){
        poseEstimateM2();
    } else if ( obj == _btn_showPose ){
        showPoseEstimate();
    } else if ( obj == _btn_rrt )
    {
        runRRT();
    }
	
	
}
void SamplePlugin::runRRT()
{
    _target->moveTo(_scanner25D->getTransform(_state) * _pose * rw::math::Transform3D<>(rw::math::RPY<>(0,0,rw::math::Pi).toRotation3D()) * rw::math::Transform3D<>(rw::math::RPY<>(rw::math::Pi/2,0,0).toRotation3D()), _state);
    getRobWorkStudio()->setState(_state);
    std::cout << "RRT place: " << _place << std::endl;
    _path = RRT::rrt_path_calculate(_place, _wc, _state, _device, _UR6, _tcp, _target, _stepGrasp, _stepRelease);
}

void SamplePlugin::showPoseEstimate()
{
    std::cout << "--- Visualizing Pose Estimate ---" << std::endl;
    align::showTwoPointClouds(_scene, _object);  
}

void SamplePlugin::poseEstimateM2(){
    cout << "--- Initializing Pose Estimation M2 ---" << endl;
    std::string scenePath = get25DImage(std::string(std::getenv("HOME"))+"/Desktop/");
    
    /**** Read point clouds ****/
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_object (new pcl::PointCloud<pcl::PointXYZ>);  // Point cloud with XYZ
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scene  (new pcl::PointCloud<pcl::PointXYZ>);   // Point cloud with XYZ
    pcl::PCDReader reader;  // Read cloud data from PCD files
    reader.read("../../../parts/duck_voxel.pcd", *cloud_object);
    reader.read(scenePath, *cloud_scene );

    /**** Pose Estimation ****/
    Eigen::Matrix4f tf = poseEstimate::poseEstimate(cloud_scene, cloud_object);
    pcl::transformPointCloud(*cloud_object, *cloud_object, tf);
    _scene = cloud_scene;
    _object = cloud_object;

    std::cout << "Estimated pose: "  << endl << tf << std::endl;
    rw::math::Rotation3D<> R = rw::math::Rotation3D<>(tf.data()[0],tf.data()[4],tf.data()[8], tf.data()[1], tf.data()[5], tf.data()[9], tf.data()[2],tf.data()[6],tf.data()[10] );
    rw::math::Vector3D<> V = rw::math::Vector3D<>(tf.data()[12],tf.data()[13],tf.data()[14]);
    //rw::math::Transform3D<> pose(V,R);
    _pose = rw::math::Transform3D<>(V, R);
    std::cout << "Cast to RW:" << endl << _pose << std::endl;

    //_place = _table->getTransform(_state)*_duck->getTransform(_state);
    //_place.R() = _target->getTransform(_state).R();
    _target->moveTo(_scanner25D->getTransform(_state) * _pose * rw::math::Transform3D<>(rw::math::RPY<>(0,0,rw::math::Pi).toRotation3D()) * rw::math::Transform3D<>(rw::math::RPY<>(rw::math::Pi/2,0,0).toRotation3D()), _state);
    
    _place = _target->getTransform(_state);
    _place.P()[0] = 0.30;
    _place.P()[1] = -0.50;

    getRobWorkStudio()->setState(_state);
}

void SamplePlugin::runLinearInterpolation(){
    _target->moveTo(_scanner25D->getTransform(_state) * _pose * rw::math::Transform3D<>(rw::math::RPY<>(0,0,rw::math::Pi).toRotation3D()) * rw::math::Transform3D<>(rw::math::RPY<>(rw::math::Pi/2,0,0).toRotation3D()), _state);
	cout << "Initializing linear interpolation to calculate path" << endl;
    
    /******/

    auto PointTimes = interpolator::util::getPointTimes(_target->getTransform(_state), _UR6->baseTend(_state), _place);
    std::vector<rw::math::Transform3D<>> Points = std::get<0>(PointTimes);
    std::vector<float> Times = std::get<1>(PointTimes);
    rw::proximity::CollisionDetector::Ptr detector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));

    auto paths = interpolator::linearInterpolate(Points, Times, _target, _UR6, _wc, _state, detector);
    //std::vector<rw::math::Transform3D<>> blendPath = interpolator::parabolicBlend(Points, Times);
    std::vector<rw::math::Q> jointPath = std::get<0>(paths);
    std::vector<rw::math::Transform3D<>> cartPath = std::get<1>(paths);

    for ( unsigned int i = 0; i < cartPath.size(); i++ )
    {
        if ( cartPath[i].P() == Points[2].P() )
            _stepGrasp = i;

        if ( cartPath[i].P() == Points[6].P() )
            _stepRelease = i;

    }
    std::cout << "Grasp step: " << _stepGrasp << std::endl;
    std::cout << "Release step: " << _stepRelease << std::endl;

    std::cout << "Path length: " << jointPath.size() << std::endl;

    _path.clear();
    for ( rw::math::Q configuration : jointPath ) // configurationPath
    {
        if ( configuration.size() == 6 )
            _path.push_back(configuration);
    }
    std::cout << "Collision free path length: " << _path.size() << std::endl;
}

std::string SamplePlugin::get25DImage(std::string savePath) {
	if (_framegrabber25D != NULL) {
        // Create pcd file
        for (size_t i = 0; i < _cameras25D.size(); i++) {
			// Get the image as a RW image
            rw::kinematics::Frame* cameraFrame25D = _wc->findFrame(_cameras25D[i]); // "Camera");
			_framegrabber25D->grab(cameraFrame25D, _state);

			//const Image& image = _framegrabber->getImage();

			const rw::geometry::PointCloud* img = &(_framegrabber25D->getImage());
			std::string saved = savePath + _cameras25D[i] + ".pcd";
            std::ofstream output(saved);
            output << "# .PCD v.5 - Point Cloud Data file format\n";
            output << "FIELDS x y z\n";
            output << "SIZE 4 4 4\n";
            output << "TYPE F F F\n";
            output << "WIDTH " << img->getWidth() << "\n";
            output << "HEIGHT " << img->getHeight() << "\n";
            output << "POINTS " << img->getData().size() << "\n";
            output << "DATA ascii\n";
            for(const auto &p_tmp : img->getData()) {
                rw::math::Vector3D<float> p = p_tmp;
                output << p(0) << " " << p(1) << " " << p(2) << "\n";
            }
            output.close();
			return saved;
		}
	}
}

void SamplePlugin::get25DImage() {
	if (_framegrabber25D != NULL) {
        // Create pcd file
        for (size_t i = 0; i < _cameras25D.size(); i++) {
			// Get the image as a RW image
            rw::kinematics::Frame* cameraFrame25D = _wc->findFrame(_cameras25D[i]); // "Camera");
			_framegrabber25D->grab(cameraFrame25D, _state);

			//const Image& image = _framegrabber->getImage();

			const rw::geometry::PointCloud* img = &(_framegrabber25D->getImage());

            std::ofstream output(_cameras25D[i] + ".pcd");
            output << "# .PCD v.5 - Point Cloud Data file format\n";
            output << "FIELDS x y z\n";
            output << "SIZE 4 4 4\n";
            output << "TYPE F F F\n";
            output << "WIDTH " << img->getWidth() << "\n";
            output << "HEIGHT " << img->getHeight() << "\n";
            output << "POINTS " << img->getData().size() << "\n";
            output << "DATA ascii\n";
            for(const auto &p_tmp : img->getData()) {
                rw::math::Vector3D<float> p = p_tmp;
                output << p(0) << " " << p(1) << " " << p(2) << "\n";
            }
            output.close();
		}
	}
}

void SamplePlugin::getImage() {
	if (_framegrabber != NULL) {
        for (size_t i = 0; i < _cameras.size(); i++) {
			// Get the image as a RW image
            rw::kinematics::Frame* cameraFrame = _wc->findFrame(_cameras[i]); // "Camera");
			_framegrabber->grab(cameraFrame, _state);

			const rw::sensor::Image* rw_image = &(_framegrabber->getImage());

			// Convert to OpenCV matrix.
			cv::Mat image = cv::Mat(rw_image->getHeight(), rw_image->getWidth(), CV_8UC3, (rw::sensor::Image*)rw_image->getImageData());

			// Convert to OpenCV image
            cv::Mat imflip, imflip_mat;
			cv::flip(image, imflip, 1);
            cv::cvtColor( imflip, imflip_mat, cv::COLOR_RGB2BGR );

			cv::imwrite(_cameras[i] + ".png", imflip_mat );

			// Show in QLabel
			QImage img(imflip.data, imflip.cols, imflip.rows, imflip.step, QImage::Format_RGB888);
			QPixmap p = QPixmap::fromImage(img);
			unsigned int maxW = 480;
			unsigned int maxH = 640;
			_label->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));
		}
	}
}

void SamplePlugin::timer() {
    if(0 <= _step && _step < _path.size()){
        
        
        _device->setQ(_path.at(_step),_state);
        _step++;
        if (_step == _stepGrasp)
            rw::kinematics::Kinematics::gripFrame(_duck, _tcp, _state);

        if (_step == _stepRelease)
            rw::kinematics::Kinematics::gripFrame(_duck, _table, _state);
        
        getRobWorkStudio()->setState(_state);
        
        if ( _step >= _path.size() )
            std::cout << "Duck final (place) location: " << _duck->getTransform(_state).P() << std::endl;
    }
}

void SamplePlugin::stateChangedListener(const rw::kinematics::State& state) {
  _state = state;
}

