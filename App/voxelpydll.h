#ifndef _VOXEL_PYDLL_H
#define _VOXEL_PYDLL_H


extern "C"
{
int tofInit();
int tofSetDelay(int delayms);
void tofDeinit();
float* tofReadDepth();
float* tofReadAmplitude();
unsigned char* tofReadPhase();
char* getTofProfiles();
int     getTofProfile();
int     setTofProfile(int profileID);
int     setTofParrameter(char* parName, char* parValue);
char*  getTofParrameter();
char*     setTofConfParrameter(char* section, char* parName, char* parValue);
int uvcInit(int cameraidx,int width,int heigh);
int uvcDeinit();
unsigned char* getUvcFrame(int timeoutms);
}
#endif
