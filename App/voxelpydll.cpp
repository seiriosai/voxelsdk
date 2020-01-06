#include "voxelpydll.h"
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include <CameraSystem.h>
#include <Common.h>

#include <thread>
#define USE_NEW 0
Voxel::CameraSystem* sys;
Voxel::DepthCameraPtr* depthCamera;
Voxel::DepthFrame deepframe;
static int voxelframedelayms = 150;
Voxel::ToFRawFrameTemplate<uint16_t, uint8_t> tofframe;
#define FRAMEBUF_NUM 30
static uint16_t array_phase[FRAMEBUF_NUM][320 * 240];
static float array_deepth[FRAMEBUF_NUM][320 * 240];
static float array_amplitude[FRAMEBUF_NUM][320 * 240];
static uint64_t array_deeptimeus[FRAMEBUF_NUM];
static uint64_t array_toftimeus[FRAMEBUF_NUM];
static uint64_t array_amptimeus[FRAMEBUF_NUM];
static int writedeeppos = 0;
static int readdeeppos = 0;
static int writeamppos = 0;
static int readamppos = 0;
static int writetofpos = 0;
static int readtofpos = 0;
static int deepframenum = 0;
static int ampframenum = 0;
static int tofframenum = 0;
Voxel::Mutex _frameMutex;
extern "C"
{
int tofSetDelay(int delayms)
{
	voxelframedelayms = delayms;
	return 0;
}

uint64_t getCurrentTime()
{
	timeval tv;


	gettimeofday(&tv, NULL);//获取当下精确的s和us的时间
	
	//printf("ts = %lld\n",uint64_t(tv.tv_sec*1000000 + tv.tv_usec));
	return (tv.tv_sec*1000000 + tv.tv_usec); //当前us数
}

int tofInit()
{
	printf(__FUNCTION__);
	Voxel::logger.setDefaultLogLevel(Voxel::LOG_ERROR);
	sys = new Voxel::CameraSystem();
	depthCamera = new Voxel::DepthCameraPtr();
	uint64_t curtime;
	

	const Voxel::Vector<Voxel::DevicePtr> &devices = sys->scan();

	if (devices.size() > 0)
		*depthCamera = sys->connect(devices[0]); // Connect to first available device
	else
	{
		std::cerr << "Could not find a compatible device." << std::endl;
		return -1;
	}

	if (!*depthCamera)
	{
		std::cerr << "Could not open a depth camera." << std::endl;
		return -1;
	}
	(*depthCamera)->registerCallback(Voxel::DepthCamera::FRAME_DEPTH_FRAME, [&](Voxel::DepthCamera &dc, const Voxel::Frame &frame, Voxel::DepthCamera::FrameType c) {
		 const Voxel::DepthFrame *d = dynamic_cast<const Voxel::DepthFrame *>(&frame);
		 
		     if(!d)
		     {
		       std::cout << "Null frame captured? or not of type DepthFrame" << std::endl;
		       return;
		     }
			 Voxel::Lock<Voxel::Mutex> _(_frameMutex);
			 
			 memcpy(array_deepth[writedeeppos%FRAMEBUF_NUM], d->depth.data(), sizeof(array_deepth[0]));
			 memcpy(array_amplitude[writeamppos%FRAMEBUF_NUM], d->amplitude.data(), sizeof(array_amplitude[0]));
			 array_deeptimeus[writedeeppos%FRAMEBUF_NUM] = d->timestamp;
			 array_amptimeus[writeamppos%FRAMEBUF_NUM] = d->timestamp;
			 writedeeppos++;
			 deepframenum++;
			 writeamppos++;
			 ampframenum++;
			 if (deepframenum > FRAMEBUF_NUM)
			 {
				 deepframenum = FRAMEBUF_NUM;
			 }
			 if (ampframenum > FRAMEBUF_NUM)
			 {
				 ampframenum = FRAMEBUF_NUM;
			 }
			 readdeeppos = writedeeppos + FRAMEBUF_NUM - deepframenum;
			 readamppos = writeamppos + FRAMEBUF_NUM - ampframenum;

			 //deepframe = *d;
		   
		   });
	(*depthCamera)->registerCallback(Voxel::DepthCamera::FRAME_RAW_FRAME_PROCESSED, [&](Voxel::DepthCamera &dc, const Voxel::Frame &frame, Voxel::DepthCamera::FrameType c) 
		{		
			//std::cout << c << std::endl;
			Voxel::ToFRawFrameTemplate<uint16_t, uint8_t> *d = (Voxel::ToFRawFrameTemplate<uint16_t, uint8_t> *)(&frame);
			 if(!d)
			 {
				 std::cout << "Null frame captured? or not of type DepthFrame" << std::endl;
				 return;
			 }
			 memcpy(array_phase[writetofpos%FRAMEBUF_NUM], d->phase(), sizeof(array_phase[0]));
			 array_toftimeus[writetofpos%FRAMEBUF_NUM] = d->timestamp;
			 writetofpos++;
			 tofframenum++;
			 if (tofframenum > FRAMEBUF_NUM)
			 {
				 tofframenum = FRAMEBUF_NUM;
			 }
			 readtofpos = writetofpos + FRAMEBUF_NUM - tofframenum;
			//tofframe = *d;
		});
	//(*depthCamera)->_initStartParams();
	//(*depthCamera)->refreshParams();
	(*depthCamera)->start();
	//(*depthCamera)->stop();
	//(*depthCamera)->start();
	return 0;
}

void tofDeinit()
{
	if (*depthCamera)
	{
		(*depthCamera)->stop();
	}
}
float* tofReadDepth()
{
	uint64_t curtime;
	curtime = getCurrentTime();
	static int pos = 0;
	int find = 0;
	while (deepframenum>0)
	{
		if ((curtime - array_deeptimeus[readdeeppos%FRAMEBUF_NUM]) < (voxelframedelayms * 1000))
		{
			if (find = 1)
			{
				break;
			}
			else{
				usleep(10000);
				curtime = getCurrentTime();
				continue;
			}

		}
		else
		{
			Voxel::Lock<Voxel::Mutex> _(_frameMutex);
			pos = readdeeppos;
			readdeeppos++;
			deepframenum--;
			find = 1;

		}
	}

	return  array_deepth[pos%FRAMEBUF_NUM];//deepframe.depth.data();
}

float* tofReadAmplitude()
{
	uint64_t curtime;
	curtime = getCurrentTime();
	static int pos = 0;
	int find = 0;
	while (ampframenum>0)
	{
		if ((curtime - array_amptimeus[readamppos%FRAMEBUF_NUM]) < (voxelframedelayms * 1000))
		{
			if (find = 1)
			{
				break;
			}
			else{
				usleep(10000);
				curtime = getCurrentTime();
				continue;
			}

		}
		else
		{
			Voxel::Lock<Voxel::Mutex> _(_frameMutex);
			pos = readamppos;
			readamppos++;
			ampframenum--;
			find = 1;

		}
	}

	return  array_amplitude[pos%FRAMEBUF_NUM]; //deepframe.amplitude.data();
}
unsigned char* tofReadPhase()
{
	uint64_t curtime;
	curtime = getCurrentTime();
	static int pos = 0;
	int find = 0;
	while (tofframenum>0)
	{
		if ((curtime - array_toftimeus[readtofpos%FRAMEBUF_NUM])< (voxelframedelayms*1000) )
		{
			if (find = 1)
			{
				break;
			}
			else{
				usleep(10000);
				curtime = getCurrentTime();
				continue;
			}
			
		}
		else
		{
			Voxel::Lock<Voxel::Mutex> _(_frameMutex);
			pos = readtofpos;
			readtofpos++;
			tofframenum--;
			find = 1;
			
		}
		
	}
	printf("delay = %d pos = %d fnum = %d\n",voxelframedelayms,pos%FRAMEBUF_NUM,tofframenum);
	return (unsigned char*)array_phase[pos%FRAMEBUF_NUM];
}

char* getTofProfiles()
{
	Voxel::Map<int, Voxel::String> profiles;
	profiles = (*depthCamera)->getCameraProfileNames();
       
	int selok = 0;
	int optid = 0;
	Voxel::String str;
	for(auto &p: profiles)
	{
		//std::cout << p.first << ", " << p.second;
		if(p.second == "Calibrated Lens Only" || p.second == "Calibrated High Ambient" || p.second == "High Ambient")
		{

		}
		else
		{
		  //continue;
		}
	
		char sz[50]={0};
		snprintf(sz,sizeof(sz),"%d",p.first);
		str += sz;
		str += ":";
		str += p.second;

		Voxel::ConfigurationFile *c = (*depthCamera)->configFile.getCameraProfile(p.first);

		if(c && c->getLocation() == Voxel::ConfigurationFile::IN_CAMERA)
		{
		
		str += " (HW)";

		}
		str+="\n";
	}
	static char* p = NULL;
	if(p)free(p);
	p = (char*)calloc(str.size()+sizeof(char),sizeof(char));
	memset(p,0,str.size()+sizeof(char));
	strcpy(p,str.c_str());
	return p;
}
int     getTofProfile()
{
	int curid = (*depthCamera)->configFile.getCurrentProfileID();
	return curid;
}
int     setTofProfile(int profileID)
{
	(*depthCamera)->stop();
	(*depthCamera)->setCameraProfile(profileID);
	(*depthCamera)->start();
	return 0;
}

int     setTofParrameter(char* parName, char* parValue)
{
	Voxel::Map<Voxel::String,Voxel::ParameterPtr>  m = (*depthCamera)->getParameters();
  	Voxel::String str;
	for (Voxel::Map<Voxel::String, Voxel::ParameterPtr>::iterator it = m.begin(); it != m.end();it++)
	{
		Voxel::ParameterPtr pp = it->second;
		if(strcmp(parName,pp->name().c_str()) != 0)
			continue;
		
		uint32_t v = atoi(parValue);
		uint32_t rv = 0;
		(*depthCamera)->getProgrammer()->readRegister(pp->address(), rv);
		uint32_t mask = pp->mask();
		int left = 0;
		for(left = 0;left<32;left++)
		    if(mask&(1<<left))continue;
		    else break;
		int right = pp->registerLen();
		for(;right>=0;right--)
		    if(mask&(1<<right))continue;
		    else break;
		v = v>>left;
		v = v<<left;
		v = v<<(31-right);
		v = v>>(31-right);
		v |= rv&mask;

		(*depthCamera)->getProgrammer()->writeRegister(pp->address(), v);
		break;
	}
	return 0;
}

char*  getTofParrameter()
{
	Voxel::Map<Voxel::String,Voxel::ParameterPtr>  m = (*depthCamera)->getParameters();
  	Voxel::String str;
	for (Voxel::Map<Voxel::String, Voxel::ParameterPtr>::iterator it = m.begin(); it != m.end();it++)
	{
		Voxel::ParameterPtr pp = it->second;
		
		str += pp->name();
		str += ":";
		uint32_t v=0;
		if(pp->address())
			(*depthCamera)->getProgrammer()->readRegister(pp->address(), v);
		uint32_t mask = pp->mask();
		int left = 0;
		for(left = 0;left<32;left++)
		if(mask&(1<<left))continue;
		else break;

		int right = pp->registerLen()-1;

		for(;right>=0;right--)
		if(mask&(1<<right))continue;
		else break;

		int realv = (v&~mask)>>left;

		if((right-left+1) == 16)
		{
		
		realv = (short)realv;
		}
		char sz[50]={0};
		snprintf(sz,sizeof(sz),"%d",realv);
		str+=sz;
		str+="\n";
		

	}
	static char* p = NULL;
	if(p)free(p);
	p = (char*)calloc(str.size()+sizeof(char),sizeof(char));
	memset(p,0,str.size()+sizeof(char));
	strcpy(p,str.c_str());
	return p;
	
}

char*     setTofConfParrameter(char* section, char* parName, char* parValue)
{
	printf("%s %s %s %s\n",__FUNCTION__,section,parName,parValue);
	//(*depthCamera)->stop();
	int id = getTofProfile();
	Voxel::ConfigurationFile *configFile = (*depthCamera)->configFile.getCameraProfile(id);
	configFile->set(section,parName,parValue);
	//(*depthCamera)->configFile.set(section,parName,parValue);
	//(*depthCamera)->start();
	static Voxel::String str = configFile->get(section,parName);
	return (char*)str.c_str();
}



}
