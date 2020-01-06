/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include <CameraSystem.h>
#include <Common.h>

#include <thread>

#include "TOFPCLViewer.h"
 
int main ()
{
	
  //std::cout << "mainstart" << std::endl;
  //system("set VOXEL_SDK_PATH=D:\\vs13prj\\voxelsdk-0.6.10\\build");
  //Voxel::Configuration::setEnvironmentVariable("VOXEL_SDK_PATH", "D:\\vs13prj\\voxelsdk-0.6.10\\build");
  Voxel::logger.setDefaultLogLevel(Voxel::LOG_INFO);
  

  Voxel::CameraSystem sys;
  Voxel::DepthCameraPtr depthCamera;
  
  const Voxel::Vector<Voxel::DevicePtr> &devices = sys.scan();
  
  if(devices.size() > 0)
    depthCamera = sys.connect(devices[0]); // Connect to first available device
  else
  {
    std::cerr << "SimplePCLViewer: Could not find a compatible device." << std::endl;
    return -1;
  }
  
  if(!depthCamera)
  {
    std::cerr << "SimplePCLViewer: Could not open a depth camera." << std::endl;
    return -1;
  }
  /*
  Voxel::Map<Voxel::String,Voxel::ParameterPtr> m = depthCamera->getParameters();
  for (Voxel::Map<Voxel::String, Voxel::ParameterPtr>::iterator it = m.begin(); it != m.end();it++)
  {
	  Voxel::ParameterPtr pp = it->second;
	  Voxel::String pname = it->first;
	  std::cout << pname << std::endl;
	  uint32_t v;
	  std::cout << pp->address() << std::endl;
	  std::cout << pp->description() << std::endl;
	  std::cout << pp->displayName() << std::endl;
	  std::cout << "old value:" <<std::hex<< (int)pp->msb() <<" "<<std::hex<< (int)pp->lsb() << std::endl;
	  depthCamera->getProgrammer()->readRegister(pp->address(), v);
	  std::cout << "value:" << v << std::endl;
	 
  }
  */
  
  depthCamera->registerCallback(Voxel::DepthCamera::FRAME_DEPTH_FRAME, [&](Voxel::DepthCamera &dc, const Voxel::Frame &frame, Voxel::DepthCamera::FrameType c) {
	  const Voxel::DepthFrame *d = dynamic_cast<const Voxel::DepthFrame *>(&frame);

	  if (!d)
	  {
		  std::cout << "Null frame captured? or not of type DepthFrame" << std::endl;
		  return;
	  }
	  

  });
  depthCamera->registerCallback(Voxel::DepthCamera::FRAME_RAW_FRAME_PROCESSED, [&](Voxel::DepthCamera &dc, const Voxel::Frame &frame, Voxel::DepthCamera::FrameType c)
  {
	  std::cout << c << std::endl;
	  Voxel::ToFRawFrameTemplate<uint16_t, uint8_t> *d = (Voxel::ToFRawFrameTemplate<uint16_t, uint8_t> *)(&frame);
	  if (!d)
	  {
		  std::cout << "Null frame captured? or not of type DepthFrame" << std::endl;
		  return;
	  }
	  
  });
  depthCamera->start();
  Voxel::PCLViewer v;
  v.setDepthCamera(depthCamera);
  v.start();
  
  while(1||v.isRunning())
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  
  return 0;
}