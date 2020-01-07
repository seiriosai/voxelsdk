/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "TOFPCLViewer.h"
#include "PCLGrabber.h"
#include "vtkRenderWindow.h"
#include <vtkImageViewer2.h>
#include <vtkJPEGReader.h>
#include "vtkRenderWindowInteractor.h"
#include "vtkImageCanvasSource2D.h"
#include "vtkImageActor.h"
#include "vtkImageReader.h"
#include "vtkImageData.h"
#include "vtkInformation.h"
#include "vtkCamera.h"
#include "vtkTransform.h"
#include "vtkTDxInteractorStyle.h"
#include "vtkInteractorStyleUnicam.h"
#include "vtkMapper2D.h"
#include "vtkImageMapper.h"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include <iostream>



#include <windows.h>

#define PCL_NO_PRECOMPILE
#include <pcl/visualization/pcl_visualizer.h>
#undef PCL_NO_PRECOMPILE
//extern HWND pwnd;
namespace Voxel
{
  
#define CLOUD_NAME "cloud"
vtkSmartPointer<vtkImageData> imageData;
vtkSmartPointer<vtkImageData> grayimageData;
vtkSmartPointer<vtkImageData> grbimageData;
cv::VideoCapture capture(0);    // 打开摄像头
PCLViewer::PCLViewer()
{

	int width = 320;
	int height = 240;
	imageData = vtkSmartPointer<vtkImageData>::New();
	vtkSmartPointer<vtkInformation> info = vtkSmartPointer<vtkInformation>::New();
	
	imageData->SetDimensions(width, height, 1);
	imageData->SetScalarType(VTK_DOUBLE, info);
	imageData->SetNumberOfScalarComponents(1, info);//每个像素需要表示的组份 =1是指标量图
	imageData->AllocateScalars(info);//很重要

	double *ptr = (double*)imageData->GetScalarPointer();

	for (int i = 0; i < height * width; i++)
	{
		*ptr++ = 100.0;
	}

	grayimageData = vtkSmartPointer<vtkImageData>::New();
	vtkSmartPointer<vtkInformation> grayinfo = vtkSmartPointer<vtkInformation>::New();

	grayimageData->SetDimensions(width, height, 1);
	grayimageData->SetScalarType(VTK_UNSIGNED_CHAR, grayinfo);
	grayimageData->SetNumberOfScalarComponents(1, grayinfo);//每个像素需要表示的组份 =1是指标量图
	grayimageData->AllocateScalars(grayinfo);//很重要

	unsigned char *grayptr = (unsigned char *)grayimageData->GetScalarPointer();

	for (int i = 0; i < height * width; i++)
	{
		*grayptr++ = 100;
	}

	grbimageData = vtkSmartPointer<vtkImageData>::New();
	vtkSmartPointer<vtkInformation> grbinfo = vtkSmartPointer<vtkInformation>::New();

	grbimageData->SetDimensions(640, 480, 1);
	grbimageData->SetScalarType(VTK_UNSIGNED_CHAR, grbinfo);
	grbimageData->SetNumberOfScalarComponents(3, grbinfo);//每个像素需要表示的组份 =1是指标量图
	grbimageData->AllocateScalars(grbinfo);//很重要

	unsigned char *grbptr = (unsigned char *)grbimageData->GetScalarPointer();

	for (int i = 0; i < 480 * 640; i++)
	{
		for (int j = 0; j < 3;j++)
		{
			*grbptr++ = 60*(j + 1);
		}
		
	}
	
	std::cout << "Built with OpenCV " << CV_VERSION << std::endl;

	
	//std::cout << capture.getBackendName() << std::endl;
	
	capture.set(CV_CAP_PROP_FRAME_WIDTH, 640);//宽度
	capture.set(CV_CAP_PROP_FRAME_HEIGHT, 480);//高度
	if (!capture.isOpened())    // 判断是否打开成功
	{
		cout << "open camera failed. " << endl;
		return;
	}

	while (0 && true)
	{
		cv::Mat frame;
		capture >> frame;    // 读取图像帧至frame
		if (!frame.empty())	// 判断是否为空
		{
			cv::imshow("camera", frame);
		}
		
		if (cv::waitKey(30) > 0)		// delay 30 ms等待按键
		{
			break;
		}
	}
	

}

void CvFrametoVTKImg(const cv::Mat &mat, vtkSmartPointer<vtkImageData> img)
{
	
	unsigned char *grbptr = (unsigned char *)img->GetScalarPointer();
	for (size_t nrow = 0; nrow < mat.rows; nrow++)
	{
		uchar* data = (uchar*)mat.ptr<uchar>(nrow);
		for (size_t ncol = 0; ncol < mat.cols*mat.channels(); ncol+=3)
		{
			*grbptr++ = int(data[ncol + 2]);
			*grbptr++ = int(data[ncol + 1]);
			*grbptr++ = int(data[ncol + 0]);
		}
		std::cout << std::endl;
	}
	
	
	
	return;
}

void PCLViewer::_renderLoop()
{

	vtkRenderer *ren0;
	vtkRenderer *ren1;
	vtkRenderer *ren2;
	vtkRenderer *ren3;
	vtkSmartPointer<vtkImageCanvasSource2D> canvas;
	vtkSmartPointer<vtkImageActor> redActor;
	vtkSmartPointer<vtkImageCanvasSource2D> graycanvas;
	vtkSmartPointer<vtkImageActor> grayredActor;
	vtkSmartPointer<vtkImageCanvasSource2D> grbcanvas;
	vtkSmartPointer<vtkImageActor> grbredActor;
	vtkRenderWindowInteractor* iren = vtkRenderWindowInteractor::New();
	vtkInteractorStyleUnicam* style = vtkInteractorStyleUnicam::New();
	double s[3] = { 2.0, 2, 2 };
	vtkSmartPointer<vtkTransform> trans;
  if(!_viewer)
  {
    _viewer =  Ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("UVCTOFViewer"));
    _viewer->setBackgroundColor(0, 0, 0);
    
    _viewer->addCoordinateSystem(1.0);
    _viewer->initCameraParameters();
    _viewer->setShowFPS(false);
    _viewer->resetCameraViewpoint();
    _viewer->setCameraPosition(15, 15, 20, 0, 0, 5, 0, 0, 1);
	_viewer->removeOrientationMarkerWidgetAxes();

	vtkSmartPointer<vtkRenderWindow> renWin = _viewer->getRenderWindow();
	ren0 = renWin->GetRenderers()->GetFirstRenderer();
	
	
	
	vtkCamera *rcamera = ren0->GetActiveCamera();
	rcamera->SetClippingRange(0.0, 6.0);
	rcamera->SetFocalPoint(0.0, 0.0, 5);
	rcamera->SetPosition(0, 0, -5);

	rcamera->ComputeViewPlaneNormal();

	rcamera->SetViewUp(0.0, 1, 0.0);
	canvas =
		vtkSmartPointer<vtkImageCanvasSource2D>::New();
	canvas->SetScalarTypeToDouble();
	canvas->SetNumberOfScalarComponents(1);
	canvas->SetExtent(0, 320, 0, 240, 0, 0);

	canvas->DrawImage(0, 0, imageData);

	canvas->Update();
	// Create actors
	redActor =
		vtkSmartPointer<vtkImageActor>::New();
	redActor->SetInputData(canvas->GetOutput());

	graycanvas =
		vtkSmartPointer<vtkImageCanvasSource2D>::New();
	graycanvas->SetScalarTypeToDouble();
	graycanvas->SetNumberOfScalarComponents(1);
	graycanvas->SetExtent(0, 320, 0, 240, 0, 0);

	graycanvas->DrawImage(0, 0, imageData);

	graycanvas->Update();
	// Create actors
	grayredActor =
		vtkSmartPointer<vtkImageActor>::New();
	grayredActor->SetInputData(graycanvas->GetOutput());

	grbcanvas =
		vtkSmartPointer<vtkImageCanvasSource2D>::New();
	grbcanvas->SetScalarTypeToUnsignedChar();
	grbcanvas->SetNumberOfScalarComponents(3);
	grbcanvas->SetExtent(0, 640, 0, 480, 0, 0);

	grbcanvas->DrawImage(0, 0, grbimageData);
	
	grbcanvas->Update();
	// Create actors
	
	grbredActor =
		vtkSmartPointer<vtkImageActor>::New();
	grbredActor->SetInputData(grbcanvas->GetOutput());
	trans =
		vtkSmartPointer<vtkTransform>::New();
	
	
	grbredActor->SetUserTransform(trans);
	

	ren1 = vtkRenderer::New();
	ren2 = vtkRenderer::New();
	ren3 = vtkRenderer::New();

	ren1->AddActor(redActor);
	ren2->AddActor(grayredActor);
	ren3->AddActor(grbredActor);
	//ren3->AddActor(actor2d);
	
	
	renWin->AddRenderer(ren1);
	renWin->AddRenderer(ren2);
	renWin->AddRenderer(ren3);
	renWin->AddRenderer(ren0);
	//iren->SetRenderWindow(renWin);



	// OK
	ren0->SetBackground(0.0, 0.0, 0.0);
	ren1->SetBackground(0.1, 0.2, 0.1);
	ren2->SetBackground(0.2, 0.1, 0.2);
	ren3->SetBackground(0.2, 0.2, 0.1);
	
	

	renWin->SetSize(800, 600);
	//renWin->SetFullScreen(1);     

	ren3->SetViewport(0, 0.5, 0.5, 1);
	ren1->SetViewport(0.5, 0.5, 1, 1);
	ren2->SetViewport(0, 0, 0.5, 0.5);
	ren0->SetViewport(0.5, 0, 1, 0.5);
	
	ren0->GetActiveCamera()->SetPosition(0, 0, -2);
	
	ren1->ResetCamera();
	ren1->GetActiveCamera()->Zoom(1.5);
	ren2->ResetCamera();
	ren2->GetActiveCamera()->Zoom(1.5);
	ren3->ResetCamera();
	ren3->GetActiveCamera()->Zoom(1.5);
	
	
	
  }
  
  if(_viewer->wasStopped())
    _viewer->resetStoppedFlag();
  
  bool firstTime = false;
  int updateCount = 0;
  
  
  while(!_stopLoop && !_viewer->wasStopped())
  {
    {
      Lock<Mutex> _(_cloudUpdateMutex);
	  //ren3->GetRenderWindow()->SetPixelData(10, 10, 109, 109, data, 1);
	  canvas->DrawImage(0, 0, imageData);
	  canvas->Update();
	  redActor->SetInputData(canvas->GetOutput());
	  graycanvas->DrawImage(0, 0, grayimageData);
	  graycanvas->Update();
	  grayredActor->SetInputData(graycanvas->GetOutput());
	  cv::Mat frame;
	  capture >> frame;    // 读取图像帧至frame
	  CvFrametoVTKImg(frame, grbimageData);
	  grbcanvas->DrawImage(0, 0, grbimageData);
	  grbcanvas->Update();
	  grbredActor->SetInputData(grbcanvas->GetOutput());
	  
	  double d[3];
	  
      if(_cloud && _handler)
      {
        double psize = 1.0, opacity = 1.0, linesize =1.0;
        _viewer->getPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, linesize, CLOUD_NAME);
        _viewer->getPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, CLOUD_NAME);
        _viewer->getPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, psize, CLOUD_NAME);
        
        if(!_viewer->updatePointCloud(_cloud, *_handler, CLOUD_NAME))
        {
          _viewer->addPointCloud(_cloud, *_handler, CLOUD_NAME);
          _viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, CLOUD_NAME);
          
          // Get the cloud mapper and set it not scale intensity to false color range dynamically
          auto cloudActorMap = _viewer->getCloudActorMap();
          auto actor = cloudActorMap->find(CLOUD_NAME);
          vtkPolyDataMapper *polyDataMapper = reinterpret_cast<vtkPolyDataMapper*>(actor->second.actor->GetMapper());
          polyDataMapper->UseLookupTableScalarRangeOn();
        }
        else
        {
          _viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, psize, CLOUD_NAME);
          _viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, linesize, CLOUD_NAME);
          _viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, CLOUD_NAME);
        }
      }
    }
    
    _viewer->spinOnce(10);
    std::this_thread::sleep_for(std::chrono::microseconds(10000));
    
    updateCount++;
    if(firstTime)
    {
      //_viewer->setCameraPosition(15, 15, 20, 0, 0, 5, 0, 0, 1);
      firstTime = false;
    }
  }
  
  
  _stopLoop = true;
}



void PCLViewer::setDepthCamera(DepthCameraPtr depthCamera)
{
  if(isRunning())
  {
    stop();
    _depthCamera = depthCamera;
    start();
  }
  else
    _depthCamera = depthCamera;
}

void PCLViewer::removeDepthCamera()
{
  if(isRunning())
  {
    stop();
  }
  
  _depthCamera = nullptr;
}


void PCLViewer::_cloudRenderCallback(const pcl::PointCloud<pcl::PointXYZI> &cloud)
{
	//return;
  if(_viewer && !_viewer->wasStopped())
  {
    Lock<Mutex> _(_cloudUpdateMutex);
	pcl::PointCloud<pcl::PointXYZI> cloud1 = cloud;
	
    _cloud = boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>>(new pcl::PointCloud<pcl::PointXYZI>(cloud));
    _handler = Ptr<pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>>(
      new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>(_cloud, "intensity"));
  }
}

void PCLViewer::_cloudRenderCallbackPCF(const Voxel::PointCloudFrame &cloud)
{
	//return;
	if (_viewer && !_viewer->wasStopped())
	{
		Lock<Mutex> _(_cloudUpdateMutex);

		
	}
}

void PCLViewer::_cloudRenderCallbackDepthF(const Voxel::DepthFrame &df)
{
	//return;
	if (_viewer && !_viewer->wasStopped())
	{
		Lock<Mutex> _(_cloudUpdateMutex);
		
		Voxel::DepthFrame f = df;
		
		double *ptr = (double*)imageData->GetScalarPointer();
		
		//std::cout << *ptr << std::endl;
		float dmaxv = 0.0;
		float dminv = 1000000.0;
		for (int i = 0; i < f.size.height * f.size.width; i++)
		{
			if (dmaxv < f.depth[i])dmaxv = f.depth[i];
			if (dminv > f.depth[i])dminv = f.depth[i];
		}
		float dmul = 256 / (dmaxv - dminv);
		ptr += f.size.height * f.size.width - 1;
		for (int i = 0; i < f.size.height * f.size.width; i++)
		{
			*ptr-- = (dmaxv - f.depth[i])*dmul;
		}

		unsigned char *grayptr = (unsigned char *)grayimageData->GetScalarPointer();
		//std::cout << *grayptr << std::endl;
		float mv = 0.0;
		for (int i = 0; i < f.size.height * f.size.width; i++)
		{
			if (mv < f.amplitude[i])mv = f.amplitude[i];

		}
		
		float mul = 256 / mv;
		grayptr += f.size.height * f.size.width-1;
		for (int i = 0; i < f.size.height * f.size.width; i++)
		{
			*grayptr-- = f.amplitude[i]*(mul-1000);
		}
		

	}
}
void PCLViewer::_cloudRenderCallbackRawF(const Voxel::RawFrame &f, const Voxel::DepthCamera::FrameType type)
{
	
}

void PCLViewer::start()
{
  if(!_depthCamera)
    return;
  
  _stopLoop = true;
  
  if(_renderThread.joinable())
    _renderThread.join();

#ifdef WINDOWS
  _viewer = nullptr;
#endif

  _stopLoop = false;
  
  _renderThread = std::thread(&PCLViewer::_renderLoop, this);
  
  _grabber = Ptr<pcl::Grabber>(new Voxel::PCLGrabber(*_depthCamera));
  
  boost::function<void (const pcl::PointCloud<pcl::PointXYZI> &)> f0 = boost::bind(&PCLViewer::_cloudRenderCallback, this, _1);
  boost::function<void(const Voxel::PointCloudFrame &)> f1 = boost::bind(&PCLViewer::_cloudRenderCallbackPCF, this, _1);
  boost::function<void(const Voxel::DepthFrame &)> f2 = boost::bind(&PCLViewer::_cloudRenderCallbackDepthF, this, _1);
  boost::function<void(const Voxel::RawFrame &, const Voxel::DepthCamera::FrameType)> f3 = boost::bind(&PCLViewer::_cloudRenderCallbackRawF, this, _1, _2);
  
  _grabber->registerCallback(f0);
  _grabber->registerCallback(f1);
  _grabber->registerCallback(f2);
  _grabber->registerCallback(f3);
  _grabber->start();
}

bool PCLViewer::isRunning()
{
  return _grabber && _grabber->isRunning() && !_stopLoop && !viewerStopped();
}

bool PCLViewer::viewerStopped()
{
  return !_viewer || _viewer->wasStopped();
}

void PCLViewer::stop()
{
  _stopLoop = true;
  
  if(_renderThread.joinable()) _renderThread.join();
  
  

#ifdef WINDOWS
  _viewer = nullptr;
#endif
  
  _grabber = nullptr;
  
  
}

}