/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "DepthCameraLibrary.h"
#include "Logger.h"

#ifdef LINUX
#include <dlfcn.h>
#include <elfio/elfio.hpp>
#elif defined(WINDOWS)
#include <windows.h>
#endif

namespace Voxel
{

class DepthCameraLibraryPrivate
{
public:
#ifdef LINUX
  void *handle = 0;
#elif defined(WINDOWS)
  HINSTANCE handle;
#endif
};

String dynamicLoadError()
{
#ifdef WINDOWS
  DWORD error = GetLastError();
  if (error)
  {
    LPVOID lpMsgBuf;
    DWORD bufLen = FormatMessage(
      FORMAT_MESSAGE_ALLOCATE_BUFFER |
      FORMAT_MESSAGE_FROM_SYSTEM |
      FORMAT_MESSAGE_IGNORE_INSERTS,
      NULL,
      error,
      MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
      (LPTSTR)&lpMsgBuf,
      0, NULL);
    if (bufLen)
    {
      LPCSTR lpMsgStr = (LPCSTR)lpMsgBuf;
      String result(lpMsgStr, lpMsgStr + bufLen);

      LocalFree(lpMsgBuf);

      return result;
    }
  }
  return String();
#elif defined(LINUX)
  char *c;
  if(c = dlerror())
    return String(c);
  else
    return String();
#endif
}

DepthCameraLibrary::DepthCameraLibrary(const String &libName) : _libName(libName),
_libraryPrivate(Ptr<DepthCameraLibraryPrivate>(new DepthCameraLibraryPrivate())) {}
  
bool DepthCameraLibrary::load()
{
#ifdef LINUX
  _libraryPrivate->handle = dlopen(_libName.c_str(), RTLD_LAZY | RTLD_LOCAL);
#elif defined(WINDOWS)
  _libraryPrivate->handle = LoadLibraryExA(_libName.c_str(), NULL, LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR);
#endif

  if(!_libraryPrivate->handle) 
  {
    logger(LOG_ERROR) << "DepthCameraLibrary: Failed to load " << _libName << ". Error: " << GetLastError() << std::endl;
    return false;
  }
  return true;
}

bool DepthCameraLibrary::isLoaded() { return _libraryPrivate->handle; }
 
DepthCameraFactoryPtr DepthCameraLibrary::getDepthCameraFactory()
{
  if(!isLoaded())
    return 0;
  
  char symbol[] = "getDepthCameraFactory";

#ifdef LINUX
  GetDepthCameraFactory g = (GetDepthCameraFactory)dlsym(_libraryPrivate->handle, symbol);
#elif defined(WINDOWS)
  GetDepthCameraFactory g = (GetDepthCameraFactory)GetProcAddress(_libraryPrivate->handle, symbol);
#endif

  String error;
  if (!g || (error = dynamicLoadError()).size())  
  {
    logger(LOG_DEBUG) << "DepthCameraLibrary: Failed to load symbol " << symbol << " from library " << _libName << ". Error: " << error << std::endl;
    return 0;
  }
  
  DepthCameraFactoryPtr p;
  (*g)(p);

  return p;
}

FilterFactoryPtr DepthCameraLibrary::getFilterFactory()
{
  if(!isLoaded())
    return 0;
  
  char symbol[] = "getFilterFactory";
  
  #ifdef LINUX
  GetFilterFactory g = (GetFilterFactory)dlsym(_libraryPrivate->handle, symbol);
  #elif defined(WINDOWS)
  GetFilterFactory g = (GetFilterFactory)GetProcAddress(_libraryPrivate->handle, symbol);
  #endif
  
  String error;
  if(!g || (error = dynamicLoadError()).size())  
  {
    logger(LOG_DEBUG) << "DepthCameraLibrary: Failed to load symbol " << symbol << " from library " << _libName << ". Error: " << error << std::endl;
    return 0;
  }
  
  FilterFactoryPtr p;
  (*g)(p);
  
  return p;
}

DownloaderFactoryPtr DepthCameraLibrary::getDownloaderFactory()
{
  if(!isLoaded())
    return 0;
  
  char symbol[] = "getDownloaderFactory";
  
  #ifdef LINUX
  GetDownloaderFactory g = (GetDownloaderFactory)dlsym(_libraryPrivate->handle, symbol);
  #elif defined(WINDOWS)
  GetDownloaderFactory g = (GetDownloaderFactory)GetProcAddress(_libraryPrivate->handle, symbol);
  #endif
  
  String error;
  if(!g || (error = dynamicLoadError()).size())  
  {
    logger(LOG_DEBUG) << "DepthCameraLibrary: Failed to load symbol " << symbol << " from library " << _libName << ". Error: " << error << std::endl;
    return 0;
  }
  
  DownloaderFactoryPtr p;
  (*g)(p);
  
  return p;
}

int DepthCameraLibrary::getABIVersion()
{
#ifdef LINUX
  ELFIO::elfio reader;
  
  if(!reader.load(_libName))
  {
    logger(LOG_DEBUG) << "DepthCameraLibrary: Could not read library " << _libName << std::endl;
    return 0;
  }
  
  ELFIO::section *dynamicSection = 0;
  
  for(int i = 0; i < reader.sections.size(); i++)
    if(reader.sections[i]->get_type() == SHT_DYNAMIC)
    {
      dynamicSection = reader.sections[i];
      break;
    }
    
  if(!dynamicSection)
  {
    logger(LOG_DEBUG) << "DepthCameraLibrary: Could not find dynamic section in " << _libName << std::endl;
    return 0;
  }
  
  ELFIO::dynamic_section_accessor d(reader, dynamicSection);
    
  ELFIO::Elf_Xword tag = DT_SONAME, value;
    
  String soName;
    
  if(!d.get_entry(0, tag, value, soName))
  {
    logger(LOG_DEBUG) << "DepthCameraLibrary: Could not find SONAME in " << _libName << std::endl;
    return 0;
  }
    
  Vector<String> splits;
  
  split(soName, '.', splits);
  
  return atoi(splits[splits.size() - 1].c_str());
#elif defined(WINDOWS)
  DWORD  verHandle = NULL;
  UINT   size = 0;
  LPBYTE lpBuffer = NULL;
  DWORD  verSize = GetFileVersionInfoSizeA(_libName.c_str(), &verHandle);

  if (verSize != NULL)
  {
    Vector<char> verData;
    verData.resize(verSize);

    if (GetFileVersionInfoA(_libName.c_str(), verHandle, verSize, verData.data()))
    {
      if (VerQueryValueA(verData.data(), "\\", (VOID FAR* FAR*)&lpBuffer, &size))
      {
        if (size)
        {
          VS_FIXEDFILEINFO *verInfo = (VS_FIXEDFILEINFO *)lpBuffer;
          if (verInfo->dwSignature == 0xfeef04bd)
          {
            if ((verInfo->dwFileVersionMS & 0xFFFF) == 0 && verInfo->dwFileVersionLS == 0)
              return (verInfo->dwFileVersionMS >> 16);
            else
              return 0;
          }
        }
      }
    }
  }

  return 0;
#endif
}

DepthCameraLibrary::~DepthCameraLibrary()
{
  if(isLoaded())
  {
    logger(LOG_DEBUG) << "Unloading library '" << _libName << "'..." << std::endl;
#ifdef LINUX
    dlclose(_libraryPrivate->handle);
#elif defined(WINDOWS)
    FreeLibrary(_libraryPrivate->handle);
#endif
    _libraryPrivate->handle = 0;
  }
}


  
}