#include <iostream>

using namespace std;
#include <unistd.h>
#include <stdio.h>
#include <CameraSystem.h>
#include <Common.h>

#include <thread>
Voxel::CameraSystem* sys;
Voxel::DepthCameraPtr* depthCamera;
Voxel::DepthFrame deepframe;
Voxel::ToFRawFrameTemplate<uint16_t, uint8_t> tofframe;

int tofInit()
{
    printf(__FUNCTION__);
    Voxel::logger.setDefaultLogLevel(Voxel::LOG_INFO);
    sys = new Voxel::CameraSystem();
    depthCamera = new Voxel::DepthCameraPtr();
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
             deepframe = *d;

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
            tofframe = *d;
        });
    //(*depthCamera)->_initStartParam();
    (*depthCamera)->start();
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
    return  deepframe.depth.data();
}
unsigned char* tofReadPhase()
{

    //printf("%08X\n",tofframe.phase());
    return (unsigned char*)tofframe.phase();
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


int qttofmain()
{
    cout << "Hello World!" << endl;
    tofInit();

    while(1)
    {
        usleep(1000);
    }
    return 0;
}
