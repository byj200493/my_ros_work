/*
 * online_fusion_ros.cpp
 *
 *  Created on: Sep 29, 2015
 *      Author: karrer
 */

#include <math.h>
#include <stdio.h>

//#define PREPROCESS_IMAGES

//#define DEBUG_NO_MESHES
//#define DEBUG_NO_MESH_VISUALIZATION
#include "fastfusion_node/online_fusion_ros.hpp"

#include <opencv2/opencv.hpp>
#include "fastfusion_node/Simplify.hpp"

OnlineFusionROS::OnlineFusionROS(bool createMeshList):
	_currentMeshForSave(NULL),
	_currentMeshInterleaved(NULL),
	_fusion(NULL),
	_imageDepthScale(5000.0f), _maxCamDistance(MAXCAMDISTANCE),
	_threadFusion(false),
	_fusionThread(NULL),
	_newMesh(false),
	_fusionActive(true),
	_fusionAlive(true),
	_threadImageReading(false),
	_lastComputedFrame(-1),
	_verbose(true),
	_showCameraFrustum(true), _showDepthImage(true),
	_currentMesh(NULL),_currentNV(0),_currentNF(0), _currentMeshType(0),
	//_vertexBuffer(0), _faceBuffer(0), _edgeBuffer(0), _colorBuffer(0),
	_vertexBufferSize(0), _faceBufferSize(0), _edgeBufferSize(0),
	_onTrack(false), _onInterpolation(false), _saving(false),
	_runFusion(false), _createMeshList(createMeshList),
	_lightingEnabled(false),
	_colorEnabled(true),
	_isSetup(false)
{
	_meshNumber = 0;
	_fusionNumber = 0;
	_cx = 0.0f; _cy = 0.0f; _cz = 0.0f;
	_isReady = true;
	_frameCounter = 0;
	_modelCounter = 0;
	_fusionActive = true;
	_fusionAlive = true;
	_update = false;
	_runVisualization = true;
	pointIsClicked = false;
	sphereIsInitialized = true;
	numberClickedPoints = 0;
}

OnlineFusionROS::~OnlineFusionROS()
{
	std::cout << "in OnlineFusionROS destructor " << std::endl;
    //-- Delete Data
	if(_fusion) delete _fusion;
	if(_currentMeshForSave) delete _currentMeshForSave;
	if(_currentMeshInterleaved) delete _currentMeshInterleaved;

	fprintf(stderr,"\nEnd of OnlineFusionROS Destructor.");
#ifdef INTERACTIVE_MEMORY_MANAGEMENT
	if(INTERACTIVE_MEMORY_MANAGEMENT){
		fprintf(stderr,"\nInput:");
		char input[256];
		fprintf(stderr,"%s",fgets(input,256,stdin));
	}
#endif
}

void OnlineFusionROS::stop() {
    ROS_INFO("stop");
//-- Stop the fusion process and save the current mesh if required
	_runVisualization = false;
	if (_threadFusion) {
		if (_fusionThread) {
			std::unique_lock<std::mutex> updateLock(_fusionUpdateMutex);
			_newMesh = false;
			_runFusion = false;
			_newDataInQueue = true;
			_fusionThreadCondition.notify_one();
			updateLock.unlock();
			_fusionThread->join();
			delete _fusionThread;
			_fusionThread = NULL;
		}
	}
	_runFusion = false;
	//-- wait for last frame fusion is finished.
	while (_fusionActive);
	_fusionAlive = false;

	//-- Save current Mesh
	if (_saveMesh) {
        _currentMeshInterleaved->writePLY(_fileName,false);
        //_currentMeshInterleaved->writeOBJ(_fileName);//12/26/2018
        ROS_INFO("save mesh");
	}
	while(!_fusion->meshUpdateFinished()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
	if(_fusion) {
		_offset = _fusion->offset();
		delete _fusion;
		_fusion = NULL;
	}
	if(_currentMeshForSave) {
		delete _currentMeshForSave;
		_currentMeshForSave = NULL;
	}
	if(_currentMeshInterleaved) {
		delete _currentMeshInterleaved;
		_currentMeshInterleaved = NULL;
	}

	//-- End Visualization Thread
	if (_use_pcl_visualizer){
		//viewer->close();
		_visualizationThread->join();
		while (_update);
		delete _visualizationThread;
	}
    //decimateMesh();
    //textureMapping();
}
/*
created at 12/27/2018
*/
void OnlineFusionROS::decimateMesh()
{
    _fileName += ".obj";
    Simplify::load_obj(_fileName.c_str(), false);
    for(int loop = 0; loop < 4; ++loop)
    {
        int targCount = Simplify::getTriangleCount()/2;
        Simplify::simplify_mesh(targCount, _agressiveness, false);//12/26/2018
    }
    //Simplify::simplify_mesh_lossless(false);
    _simpliFileName += ".obj";
    Simplify::write_obj(_simpliFileName.c_str());
    ROS_INFO("agressiveness:%f",_agressiveness);
}

bool OnlineFusionROS::startNewMap() {
//-- Start a new map, only if initialization was performed before. Also Check if currently a process is running.  If any
//-- of these conditions is not met, return false
	//-- Check if a FusionMipMapCPU object exists
	if (_fusion || !_isSetup) {
		return false;
	}

	//-- Empty Queues
	if (!_queueDepth.empty()) {
		std::queue<cv::Mat> empty;
		std::swap( _queueDepth, empty );
	}
	if (!_queueRGB.empty()) {
		std::queue<cv::Mat> empty;
		std::swap( _queueRGB, empty );
	}
	if (!_queueNoise.empty()) {
		std::queue<cv::Mat> empty;
		std::swap( _queueNoise, empty );
	}
	if (!_queuePose.empty()) {
		std::queue<CameraInfo> empty;
		std::swap( _queuePose, empty );
	}
	if (!_queueTime.empty()) {
		std::queue<double> empty;
		std::swap( _queueTime, empty );
	}

	//-- Setup viewer if necessary
	if (_use_pcl_visualizer) {
		_visualizationThread = new std::thread(&OnlineFusionROS::visualize, this);
	}

	//-- Setup new fusion object and configure it
	_fusion = new FusionMipMapCPU(_offset.x,_offset.y,_offset.z,_scale, _distThreshold,0,true);
	_fusion->setThreadMeshing(_threadMeshing);
	_fusion->setIncrementalMeshing(true);
	_fusionActive = true;
	_update = false;
	_runFusion = true;
	return true;
}
/*
modified at 12/27/2018
*/
//void OnlineFusionROS::setupFusion(bool fusionThread, bool meshingThread,float imageScale, float scale, float distThreshold,
//								bool saveMesh, std::string fileName, bool usePclVisualizer){
void OnlineFusionROS::setupFusion(bool fusionThread, bool meshingThread,float imageScale, float scale, float distThreshold,
                                bool saveMesh, std::string fileName, std::string simpliFileName,
                                double agressiveness, bool usePclVisualizer){
//-- Initialize FusionMipMapCPU-class (_fusion)
	_imageDepthScale = imageScale;
	_scale = scale;
	_distThreshold = distThreshold;
	_threadMeshing = meshingThread;
	_threadFusion = fusionThread;
	_saveMesh = saveMesh;
	_fileName = fileName;
    _simpliFileName = simpliFileName;//added at 12/27/2018
    _agressiveness = agressiveness;
	_isSetup = true;
	_offset.x = _offset.y = _offset.z = 0.0f;
	_use_pcl_visualizer = usePclVisualizer;
    //startNewMap();
}

void OnlineFusionROS::fusionWrapperROS(void) {
//-- Fusion-Wrapper member to enable threading the fusion process. To buffer the data needed for the fusion
//-- (rgb-Image,depth-image, camera-pose), a std::queue with Mutex locking is used.
	//-- Initialize datatypes to store the current values
	_fusionActive = true;
	cv::Mat currImgRGB, currImgDepth, currNoise;
	CameraInfo currPose;
	double currTime, currDecayTime;
	unsigned int framesProcessed = 0;
	//-- Perform thread, as long as fusion is active
	while (_runFusion) {
		//-- Check if there is data available in the queue
		std::unique_lock<std::mutex> locker(_fusionUpdateMutex);
		while (!_newDataInQueue) {
			_fusionThreadCondition.wait(locker);
		}
		if (!_runFusion){
			std::cout << "tries break" << std::endl;
			locker.unlock();
			break;
		}
		std::queue<cv::Mat> queueRGB, queueDepth, queueNoise;
		std::queue<CameraInfo> queuePose;
		std::queue<double> queueTime, queueDecayTime;
		for (size_t i = 0; i < _queueRGB.size(); i++) {
			queueRGB.push(_queueRGB.front());
			_queueRGB.pop();
			queueDepth.push(_queueDepth.front());
			_queueDepth.pop();
			queuePose.push(_queuePose.front());
			_queuePose.pop();
			queueTime.push(_queueTime.front());
			_queueTime.pop();
			queueDecayTime.push(_queueDecayTime.front());
			_queueDecayTime.pop();
			if (!_queueNoise.empty()) {
				queueNoise.push(_queueNoise.front());
				_queueNoise.pop();
			}
		}
		_newDataInQueue = false;
		locker.unlock();
		for (size_t i = 0; i < queueRGB.size(); i++) {
			framesProcessed++;
			if (!queueNoise.empty()) {
				//-- Depth-Noise Data is available
				currImgRGB = queueRGB.front();
				queueRGB.pop();
				currImgDepth = queueDepth.front();
				queueDepth.pop();
				currPose = queuePose.front();
				queuePose.pop();
				currNoise = queueNoise.front();
				queueNoise.pop();
				currTime = queueTime.front();
				queueTime.pop();
				currDecayTime = queueDecayTime.front();
				queueDecayTime.pop();
				//-- Add Map and perform update
				_fusion->addMap(currImgDepth, currNoise,currPose,currImgRGB,1.0f/_imageDepthScale,_maxCamDistance, currTime,currDecayTime);
				_newMesh = _fusion->updateMeshes();
			} else {
				//-- No Depth Noise Data is available
				currImgRGB = queueRGB.front();
				queueRGB.pop();
				currImgDepth = queueDepth.front();
				queueDepth.pop();
				currPose = queuePose.front();
				queuePose.pop();
				currTime = queueTime.front();
				queueTime.pop();
				currDecayTime = queueDecayTime.front();
				queueDecayTime.pop();
				//updateLock.unlock();
				//-- Add Map and perform update
				_fusion->addMap(currImgDepth,currPose,currImgRGB,1.0f/_imageDepthScale,_maxCamDistance,currTime,currDecayTime);
				_newMesh = _fusion->updateMeshes();
			}
		}
	}
	_fusionActive = false;
	
}	

void OnlineFusionROS::drawCameraFrustum(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, cv::Mat &R_cv, cv::Mat &t_cv) {
	Eigen::Vector3f tl1,tr1,br1,bl1,c1,t;
	Eigen::Matrix3f R;
	double linewidth = 3.0;
	for (int i = 0;i < 3; i++) {
		for(int j = 0; j<3;j++) {
			R(i,j) = (float)R_cv.at<double>(i,j);
		}
	}
	t(0) = (float)t_cv.at<double>(0,0); t(1) = (float)t_cv.at<double>(1,0); t(2) = (float)t_cv.at<double>(2,0);
	tl1 = (R*cameraFrustum_.tl0 + t); tr1 = (R*cameraFrustum_.tr0 + t); br1 = (R*cameraFrustum_.br0 + t);
	bl1 = (R*cameraFrustum_.bl0 + t); c1 = (R*cameraFrustum_.c0 + t);
	//-- Draw Camera Frustum
	viewer->removeShape("t",0);
	viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(tl1(0),tl1(1),tl1(2)),
			pcl::PointXYZ(tr1(0),tr1(1),tr1(2)),0.0, 1.0, 0.0, "t", 0);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,linewidth,"t",0);
	viewer->removeShape("r",0);
	viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(tr1(0),tr1(1),tr1(2)),
			pcl::PointXYZ(br1(0),br1(1),br1(2)),0.0, 1.0, 0.0, "r", 0);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,linewidth,"r",0);
	viewer->removeShape("b",0);
	viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(br1(0),br1(1),br1(2)),
			pcl::PointXYZ(bl1(0),bl1(1),bl1(2)),0.0, 1.0, 0.0, "b", 0);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,linewidth,"b",0);
	viewer->removeShape("l",0);
	viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(bl1(0),bl1(1),bl1(2)),
			pcl::PointXYZ(tl1(0),tl1(1),tl1(2)),0.0, 1.0, 0.0, "l", 0);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,linewidth,"l",0);
	viewer->removeShape("tl_c",0);
	viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(tl1(0),tl1(1),tl1(2)),
			pcl::PointXYZ(c1(0),c1(1),c1(2)),0.0, 1.0, 0.0, "tl_c", 0);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,linewidth,"tl_c",0);
	viewer->removeShape("tr_c",0);
	viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(tr1(0),tr1(1),tr1(2)),
			pcl::PointXYZ(c1(0),c1(1),c1(2)),0.0, 1.0, 0.0, "tr_c", 0);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,linewidth,"tr_c",0);
	viewer->removeShape("bl_c",0);
	viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(bl1(0),bl1(1),bl1(2)),
			pcl::PointXYZ(c1(0),c1(1),c1(2)),0.0, 1.0, 0.0, "bl_c", 0);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,linewidth,"bl_c",0);
	viewer->removeShape("br_c",0);
	viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(br1(0),br1(1),br1(2)),
			pcl::PointXYZ(c1(0),c1(1),c1(2)),0.0, 1.0, 0.0, "br_c", 0);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,linewidth,"br_c",0);

}


void OnlineFusionROS::visualize() {
//-- Preliminary visualization: Generate colored Point cloud from the mesh vertexes
//-- Careful!: No check whether the mesh changes during the point cloud generation is performed--> may cause Problems!!
	//-- Initialize PCL-Viewer
	double linewidth = 3.0;
	bool pointcloudInit = false;

	//-- Setup the visualizer
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("visualization pc"));
	viewer->setShowFPS (false);
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 30);
	viewer->addCoordinateSystem (1);

	//-- Prepare camera parameters
	cv::Mat K_cv,Ext_cv, R_cv,t_cv;
	Eigen::Matrix4f Ext;
	Eigen::Matrix3f K,R;
	pcl::PointCloud<pcl::PointXYZ>::Ptr camera_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointXYZ camPointTemp;

	//-- Define Cornerpoints for camera frustum
	cameraFrustum_.tl0 << -0.24,-0.17,0.4;
	cameraFrustum_.tr0 << 0.24,-0.17,0.4;
	cameraFrustum_.br0 << 0.24,0.17,0.4;
	cameraFrustum_.bl0 << -0.24,0.17,0.4;
	cameraFrustum_.c0 << 0.0,0.0,0.0;

    while ((!viewer->wasStopped ()) && _runVisualization) {
    	viewer->spinOnce (10);
    	//-- Get Extrinsics
    	R_cv = _currentPose.getRotation();
    	t_cv = _currentPose.getTranslation();
    	drawCameraFrustum(viewer, R_cv, t_cv);

    	//-- Check if point cloud should be updated
    	if(_update) {
    		//-- Update Viewer
    		pcl::PointCloud<pcl::PointXYZRGB> points = _fusion->getCurrentPointCloud();
    		pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_ptr (new pcl::PointCloud<pcl::PointXYZRGB>(points));
    		if (points_ptr->points.size() > 0) {
				//-- KdTree for NN-search
				/*
				std::clock_t begin = std::clock();
				pcl::search::KdTree<pcl::PointXYZRGB> kdtree;
				kdtree.setInputCloud (points_ptr);
				//-- Search nearest Point
				pcl::PointXYZRGB searchPoint;
				searchPoint.x = 1.0; searchPoint.y = 2.1; searchPoint.z = 0; searchPoint.r = 1; searchPoint.g = 0; searchPoint.b = 0;
				std::vector<int> pointIdxNKNSearch(5);
				std::vector<float> pointNKNSquaredDistance(5);
				kdtree.nearestKSearch (searchPoint, 5, pointIdxNKNSearch, pointNKNSquaredDistance);
				std::clock_t end = std::clock();
				double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
				std::cout << "Time for kd-tree: " << elapsed_secs << std::endl;
				*/
				if (pointcloudInit) {
					viewer->updatePointCloud(points_ptr,"visualization pc");
				} else {
					viewer->addPointCloud(points_ptr, "visualization pc");
					pointcloudInit = true;
				}
    		}
            _update = false;
        }
    }
    viewer->removePointCloud("visualization pc",0);
    viewer->close();
}


void OnlineFusionROS::updateFusion(cv::Mat &rgbImg, cv::Mat &depthImg, CameraInfo &pose, double time, double decayTime, ros::Time timestamp) {
//-- Interface with fusion member which allows to query new depth data without noise information
	if (!_threadFusion) {
	//-- Not threaded Fusion
		_fusionActive = true;
		_frameCounter++;
		_isReady = false;
		_currentPose = pose;
		depthImg.copyTo(_currentDepthImg);
		//-- Lock visualization Mutex
		{
		std::lock_guard<std::mutex> updateLockVis(_visualizationUpdateMutex);
		//-- Add and update Map
        ROS_WARN("maxCamDistance:%f", _maxCamDistance);
		_fusion->addMap(depthImg,pose,rgbImg,1.0f/_imageDepthScale,_maxCamDistance,time, decayTime);
		_fusion->updateMeshes();
		if(!_pointermeshes.size()) _pointermeshes.resize(1,NULL);
		if(_pointermeshes[0]) delete _pointermeshes[0];
		if(!_currentMeshForSave) _currentMeshForSave = new MeshSeparate(3);
		if(!_currentMeshInterleaved) {
			std::cout << "Create New Mesh Interleaved" << std::endl;;
			_currentMeshInterleaved = new MeshInterleaved(3);
		}
		//-- Generate new Mesh
		*_currentMeshInterleaved = _fusion->getMeshInterleavedMarchingCubes();
		}
		if (_frameCounter > 10) {
			_update = true;
		}
		_isReady = true;
		_fusionActive = false;
	} else {
	//-- The fusion process is threaded
		if(!_fusionThread){
			//-- If not yet initialize --> initialize fusion thread
			_fusionThread = new std::thread(&OnlineFusionROS::fusionWrapperROS, this);
			_runFusion = true;
		}
		_currentPose = pose;

		//-- Lock and update data-queue
		{
		std::unique_lock<std::mutex> updateLock(_fusionUpdateMutex);
		_queueRGB.push(rgbImg);
		_queueDepth.push(depthImg);
		_queuePose.push(pose);
		_queueTime.push(time);
		_queueDecayTime.push(decayTime);
		_newDataInQueue = true;
		_fusionThreadCondition.notify_one();
		}

		_frameCounter++;
		//--Update Mesh
		if(_newMesh){
			_newMesh = false;
			{ // visualization scope begin
			std::lock_guard<std::mutex> updateLockVis(_visualizationUpdateMutex);
			if(!_currentMeshForSave) _currentMeshForSave = new MeshSeparate(3);
			if(!_currentMeshInterleaved) _currentMeshInterleaved = new MeshInterleaved(3);
			*_currentMeshInterleaved = _fusion->getMeshInterleavedMarchingCubes();
			} // visualization scope end
		}
		//-- Check whether to update Visualization
		if (_frameCounter > 10) {
			//-- Only update visualization after 10 frames fused
			_update = true;
		}
	}
}


void OnlineFusionROS::updateFusion(cv::Mat &rgbImg, cv::Mat &depthImg, cv::Mat &noiseImg,CameraInfo &pose, double time, double decayTime, ros::Time timestamp) {
//-- Update Fusion function when using it with noise data (from ToF camera)
	if (!_threadFusion) {
		//-- Unthreaded Fusion
		_fusionActive = true;
		_frameCounter++;
		_isReady = false;
		_currentPose = pose;
		depthImg.copyTo(_currentDepthImg);
		//-- Lock visualization Mutex
		{
		std::lock_guard<std::mutex> updateLockVis(_visualizationUpdateMutex);
		//-- Add and update Map
		_fusion->addMap(depthImg, noiseImg,pose,rgbImg,1.0f/_imageDepthScale,_maxCamDistance,time, decayTime);
		_fusion->updateMeshes();
		if(!_pointermeshes.size()) _pointermeshes.resize(1,NULL);
		if(_pointermeshes[0]) delete _pointermeshes[0];
		if(!_currentMeshForSave) _currentMeshForSave = new MeshSeparate(3);
		if(!_currentMeshInterleaved) {
			_currentMeshInterleaved = new MeshInterleaved(3);
		}
		//-- Generate new Mesh
		*_currentMeshInterleaved = _fusion->getMeshInterleavedMarchingCubes();
		}
		if (_frameCounter > 10) {
			_update = true;
		}
		_isReady = true;
		_fusionActive = false;
	} else {
		//-- The fusion process is threaded
		if(!_fusionThread){
			//-- If not yet initialize --> initialize fusion thread
			_fusionThread = new std::thread(&OnlineFusionROS::fusionWrapperROS, this);
			_runFusion = true;
		}
		_currentPose = pose;
		//-- Lock and update data-queue
		{
		std::unique_lock<std::mutex> updateLock(_fusionUpdateMutex);
		_queueRGB.push(rgbImg);
		_queueDepth.push(depthImg);
		_queueNoise.push(noiseImg);
		_queuePose.push(pose);
		_queueTime.push(time);
		_queueDecayTime.push(decayTime);
		_newDataInQueue = true;
		_fusionThreadCondition.notify_one();
		}
		_frameCounter++;
		//--Update Mesh
		if(_newMesh){
			_newMesh = false;
			{
			std::lock_guard<std::mutex> updateLockVis(_visualizationUpdateMutex);
			if(!_currentMeshForSave) _currentMeshForSave = new MeshSeparate(3);
			if(!_currentMeshInterleaved) _currentMeshInterleaved = new MeshInterleaved(3);
			*_currentMeshInterleaved = _fusion->getMeshInterleavedMarchingCubes();
			}
		}
		//-- Check whether to update Visualization
		if (_frameCounter > 10) {
			//-- Only update visualization after fusing 10 frames
			_update = true;
		}
	}
}
/*
created at 12/28/2018
*/
int OnlineFusionROS::getNearestIndex(std::vector<int> kIndices, std::vector<float> kDistances)
{
    int n = kIndices[0];
    float dist = kDistances[0];
    for(int i=1; i < kIndices.size(); ++i)
    {
        if(dist < kDistances[i])
        {
            n = kIndices[i];
            dist = kDistances[i];
        }
    }
    return n;
}
/*
created at 12/28/2018
*/
bool OnlineFusionROS::addSubTexture(cv::Mat &subMat)
{
    int subWidth = subMat.cols;
    int subHeight = subMat.rows;
    if(subWidth+_right > _texture.cols-1)
    {
        _top = _bottom;
        _right = 0;
        _bottom += subHeight;
    }
    else
    {
      if((_top+subHeight) > _bottom)
          _bottom = _top+subHeight;
    }
    if(_bottom > _texture.rows-1)
        return false;
    for(int y=0; y < subHeight; ++y)
    {
        for(int x=0; x < subWidth; ++x)
        {
            _texture.at<cv::Vec3b>(_top + y, _right + x) = subMat.at<cv::Vec3b>(y,x);
        }
    }
    _right += subWidth;
    return true;
}
/*
created at 12/28/2018
*/
void OnlineFusionROS::textureMapping()
{
    ROS_INFO("texture mapping start\n");
    ROS_INFO("path:%s",_simpliFileName.c_str());
    ROS_INFO("texture mapping start1");
    pcl::PolygonMesh sparseMesh;
    pcl::io::loadPLYFile(_simpliFileName, sparseMesh);
    pcl::PointCloud<pcl::PointXYZ> sparseCloud;
    pcl::fromPCLPointCloud2( sparseMesh.cloud, sparseCloud);
    ROS_INFO("sparse mesh face count:%d", sparseMesh.polygons.size());
    pcl::PolygonMesh denseMesh;
    _fileName += ".ply";
    pcl::io::loadPLYFile(_fileName,denseMesh);
    ROS_INFO("dense mesh face count:%d",denseMesh.polygons.size());
    pcl::PointCloud<pcl::PointXYZRGB> denseCloud;// = getCurrentPointCloud();
    pcl::fromPCLPointCloud2( denseMesh.cloud, denseCloud);
    ROS_INFO("dense point count:%d",denseCloud.size());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr denseCloudPtr (new pcl::PointCloud<pcl::PointXYZRGB>(denseCloud));
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>(false));
    tree->setInputCloud(denseCloudPtr);
    ROS_INFO("sparse point count:%d",sparseCloud.size());
    ROS_INFO("radius_search : %f", _radiusSearch);
    std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>> texCoordBuf;
    for(int i=0; i<sparseMesh.polygons.size(); ++i)
    {
        Eigen::Vector3f p[3];
        p[0] = sparseCloud.points[sparseMesh.polygons.at(i).vertices[0]].getVector3fMap();
        p[1] = sparseCloud.points[sparseMesh.polygons.at(i).vertices[1]].getVector3fMap();
        p[2] = sparseCloud.points[sparseMesh.polygons.at(i).vertices[2]].getVector3fMap();
        Eigen::Vector3f u = p[2]-p[0];
        Eigen::Vector3f v = p[1]-p[0];
        int Y = u.norm()/_sampleLen;
        int X = v.norm()/_sampleLen;
        float yLen = u.norm();
        float xLen = v.norm();
        //computing subtexture for triangle
        cv::Mat subMat(cvSize(X,Y),CV_8UC3,cvScalar(0,0,0));
        for(int iy=0; iy < Y; ++iy)
        {
            for(int ix=0; ix < X; ++ix)
            {
                float a = _sampleLen*ix/xLen;
                float b = _sampleLen*iy/yLen;
                if(a+b>1.0f)
                    continue;
                Eigen::Vector3f sample = p[0] + a*v+b*u;
                pcl::PointXYZRGB pt;
                pt.x = sample[0];
                pt.y = sample[1];
                pt.z = sample[2];
                std::vector<int> kIndices;
                std::vector<float> kDistances;
                int k = tree->radiusSearch(pt, _radiusSearch, kIndices, kDistances);
                if(k > 0)
                {
                    int id = getNearestIndex(kIndices, kDistances);
                    subMat.at<cv::Vec3b>(iy,ix)[0] = denseCloud.at(id).b;
                    subMat.at<cv::Vec3b>(iy,ix)[1] = denseCloud.at(id).g;
                    subMat.at<cv::Vec3b>(iy,ix)[2] = denseCloud.at(id).r;
                }
            }
        }
        //add subtexture to gloabal texture
        if(addSubTexture(subMat)==false)
            break;
        Eigen::Vector2f texCoords[3];
        texCoords[0][0] = _right - subMat.cols;
        texCoords[0][1] = _top + subMat.rows;
        texCoords[1][0] = _right;
        texCoords[1][1] = _top + subMat.rows;
        texCoords[2][0] = _right - subMat.cols;
        texCoords[2][1] = _top + subMat.rows;
        for(int i=0; i < 3; ++i)
        {
            texCoordBuf.push_back(texCoords[i]);
        }
    }
    cv::imwrite(_texLocation.c_str(),_texture);
    pcl::TextureMesh texMesh;
    texMesh.header = sparseMesh.header;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr ntree(new pcl::search::KdTree<pcl::PointXYZ>);
    ntree->setInputCloud((pcl::PointCloud<pcl::PointXYZ>::Ptr)&sparseCloud);
    n.setInputCloud((pcl::PointCloud<pcl::PointXYZ>::Ptr)&sparseCloud);
    n.setSearchMethod(ntree);
    n.setKSearch(20);
    n.compute(*normals);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointNormal);
    pcl::concatenateFields(sparseCloud,*normals,*cloud_with_normals);
    pcl::toPCLPointCloud2(*cloud_with_normals,texMesh.cloud);
    //pcl::toPCLPointCloud2(sparseCloud,texMesh.cloud);
    texMesh.tex_coordinates.push_back(texCoordBuf);
    texMesh.tex_polygons.push_back(sparseMesh.polygons);
    pcl::TexMaterial texMat;
    texMat.tex_Ka.r = 0.2f;
    texMat.tex_Ka.g = 0.2f;
    texMat.tex_Ka.b = 0.2f;
    texMat.tex_Kd.r = 0.8f;
    texMat.tex_Kd.g = 0.8f;
    texMat.tex_Kd.b = 0.8f;
    texMat.tex_Ks.r = 1.0f;
    texMat.tex_Ks.g = 1.0f;
    texMat.tex_Ks.b = 1.0f;
    texMat.tex_d = 1.0f;
    texMat.tex_Ns = 75.0f;
    texMat.tex_illum = 2;
    //std::stringstream texName;
    //texName << "material" << subMeshCount;
    //texMat.tex_name << texName;
    texMat.tex_name = "material";
    texMat.tex_file = _texLocation;
    texMesh.tex_materials.push_back(texMat);
    pcl::io::saveOBJFile("/home/dev/ply/tex_mesh.obj",texMesh);
    //pcl::io::savePolygonFile("mesh.ply",sparseMesh,false);
    ROS_INFO("texture mapping is finished");
}

void OnlineFusionROS::setupTexture(std::string &texLocation, int width, int height, float sampleLen, float radiusSearch)
{
   _texLocation = texLocation;
   _texture.create(cvSize(width,height),CV_8UC3);
   _left = 0;
   _top = 0;
   _right = 0;
   _bottom = 0;
   _sampleLen = sampleLen;
   _radiusSearch = radiusSearch;
}
