/*
 * capturing from UVC cam
 * requires: libjpeg-dev
 * build: gcc -std=c99 capture.c -ljpeg -o capture
 */
 
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
 
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <asm/types.h>
#include <linux/videodev2.h>
 #include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
 
//#include <jpeglib.h>
#include "uvccamera.h"
extern "C"
{
void quit(const char * msg)
{
  fprintf(stderr, "[%s] %d: %s\n", msg, errno, strerror(errno));
  exit(EXIT_FAILURE);
}
 
int xioctl(int fd, int request, void* arg)
{
  for (int i = 0; i < 100; i++) {
    int r = ioctl(fd, request, arg);
    if (r != -1 || errno != EINTR) return r;
  }
  return -1;
}
 
typedef struct {
  uint8_t* start;
  size_t length;
} buffer_t;
 
typedef struct {
  int fd;
  uint32_t width;
  uint32_t height;
  size_t buffer_count;
  buffer_t* buffers;
  buffer_t head;
} camera_t;
 
 
camera_t* camera_open(const char * device, uint32_t width, uint32_t height)
{
  int fd = open(device, O_RDWR | O_NONBLOCK, 0);
  if (fd == -1) quit("open");
  camera_t* camera = (camera_t*)malloc(sizeof (camera_t));
  camera->fd = fd;
  camera->width = width;
  camera->height = height;
  camera->buffer_count = 0;
  camera->buffers = NULL;
  camera->head.length = 0;
  camera->head.start = NULL;
  return camera;
}
 
 
void camera_init(camera_t* camera) {
  struct v4l2_capability cap;
  if (xioctl(camera->fd, VIDIOC_QUERYCAP, &cap) == -1) quit("VIDIOC_QUERYCAP");
  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) quit("no capture");
  if (!(cap.capabilities & V4L2_CAP_STREAMING)) quit("no streaming");
 
  struct v4l2_cropcap cropcap;
  memset(&cropcap, 0, sizeof cropcap);
  cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(camera->fd, VIDIOC_CROPCAP, &cropcap) == 0) {
    struct v4l2_crop crop;
    crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    crop.c = cropcap.defrect;
    if (xioctl(camera->fd, VIDIOC_S_CROP, &crop) == -1) {
      // cropping not supported
    }
  }
 
  struct v4l2_format format;
  memset(&format, 0, sizeof format);
  format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  format.fmt.pix.width = camera->width;
  format.fmt.pix.height = camera->height;
  format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
  format.fmt.pix.field = V4L2_FIELD_NONE;
  if (xioctl(camera->fd, VIDIOC_S_FMT, &format) == -1) quit("VIDIOC_S_FMT");
 
  struct v4l2_requestbuffers req;
  memset(&req, 0, sizeof req);
  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (xioctl(camera->fd, VIDIOC_REQBUFS, &req) == -1) quit("VIDIOC_REQBUFS");
  camera->buffer_count = req.count;
  camera->buffers = (buffer_t*)calloc(req.count, sizeof (buffer_t));
 
  size_t buf_max = 0;
  for (size_t i = 0; i < camera->buffer_count; i++) {
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (xioctl(camera->fd, VIDIOC_QUERYBUF, &buf) == -1)
      quit("VIDIOC_QUERYBUF");
    if (buf.length > buf_max) buf_max = buf.length;
    camera->buffers[i].length = buf.length;
    camera->buffers[i].start =
      (uint8_t*)mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED,
           camera->fd, buf.m.offset);
    if (camera->buffers[i].start == MAP_FAILED) quit("mmap");
  }
  camera->head.start = (uint8_t*)malloc(buf_max);
}
 
 
void camera_start(camera_t* camera)
{
  for (size_t i = 0; i < camera->buffer_count; i++) {
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (xioctl(camera->fd, VIDIOC_QBUF, &buf) == -1) quit("VIDIOC_QBUF");
  }
 
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(camera->fd, VIDIOC_STREAMON, &type) == -1)
    quit("VIDIOC_STREAMON");
}
 
void camera_stop(camera_t* camera)
{
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(camera->fd, VIDIOC_STREAMOFF, &type) == -1)
    quit("VIDIOC_STREAMOFF");
}
 
void camera_finish(camera_t* camera)
{
  for (size_t i = 0; i < camera->buffer_count; i++) {
    munmap(camera->buffers[i].start, camera->buffers[i].length);
  }
  free(camera->buffers);
  camera->buffer_count = 0;
  camera->buffers = NULL;
  free(camera->head.start);
  camera->head.length = 0;
  camera->head.start = NULL;
}
 
void camera_close(camera_t* camera)
{
  if (close(camera->fd) == -1) quit("close");
  free(camera);
}
 
 
int camera_capture(camera_t* camera)
{
  struct v4l2_buffer buf;
  memset(&buf, 0, sizeof buf);
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  if (xioctl(camera->fd, VIDIOC_DQBUF, &buf) == -1) return FALSE;
  memcpy(camera->head.start, camera->buffers[buf.index].start, buf.bytesused);
  camera->head.length = buf.bytesused;
  if (xioctl(camera->fd, VIDIOC_QBUF, &buf) == -1) return FALSE;
  return TRUE;
}
 
int camera_frame(camera_t* camera, struct timeval timeout) {
  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(camera->fd, &fds);
  int r = select(camera->fd + 1, &fds, 0, 0, &timeout);
  if (r == -1) quit("select");
  if (r == 0) return FALSE;
  return camera_capture(camera);
}
 
camera_t* guvccamera = NULL;
int uvcInit(int cameraidx,int width,int heigh)
 {
 	
	if(guvccamera)
	{
		camera_stop(guvccamera);
  		camera_finish(guvccamera);
  		camera_close(guvccamera);
	}
	char szcamid[100]={0};
	snprintf(szcamid,sizeof(szcamid),"/dev/video%d",cameraidx);
	guvccamera = camera_open(szcamid, width, heigh);
	if(guvccamera == NULL)return -1;
  	camera_init(guvccamera);
  	camera_start(guvccamera);
	return 0;
 	
 }

int uvcDeinit()
{
	if(guvccamera)
	{
		camera_stop(guvccamera);
  		camera_finish(guvccamera);
  		camera_close(guvccamera);
	}
	guvccamera = NULL;
	return 0;
}


unsigned char* getUvcFrame(int timeoutms)
{
	struct timeval timeout;
	static unsigned char* pframe = NULL;
	timeout.tv_sec = timeoutms/1000;
	timeout.tv_usec = (timeoutms%1000)*1000;
	int ret = camera_frame(guvccamera, timeout);
	if(ret <= 0)
	{
		perror("getFrame Error.\n");
	}
	if(pframe == NULL)
	{
		pframe = (unsigned char*)calloc(guvccamera->width* guvccamera->height*3,sizeof(unsigned char));
	}
	cv::Mat srcImaged = cv::Mat(guvccamera->width, guvccamera->height, CV_8UC2);	 
	 
	for (int i = 0; i < (guvccamera->width* guvccamera->height* 2); i++)	 
	{		  
	
	srcImaged.data[i] = guvccamera->head.start[i];	  
	}	  
	
	cv::Mat dstImaged = cv::Mat(480, 640, CV_8UC3);	  
	cv::cvtColor(srcImaged, dstImaged, CV_YUV2BGR_YUYV);
	memcpy(pframe,dstImaged.data,guvccamera->width* guvccamera->height*3);
	return pframe;
}
 #if 0
int main(int argc, char** argv)
{
  int width = 1920;
  int heigh = 1080;
  if(argc >= 3)width = atoi(argv[2]);
  if(argc >= 4)heigh = atoi(argv[3]);
  camera_t* camera = camera_open(argv[1], width, heigh);
  camera_init(camera);
  camera_start(camera);
 
  struct timeval timeout;
  timeout.tv_sec = 1;
  timeout.tv_usec = 0;
  /* skip 5 frames for booting a cam */
  for (int i = 0; i < 50; i++) {
    camera_frame(camera, timeout);
    for(int i = 0;i<20;i++)
    {
      printf("%02X ",camera->head.start[i]);
    }
    printf("\n");
    unsigned char* rgb =
    yuyv2rgb(camera->head.start, camera->width, camera->height);
    char szfn[100]={0};
    snprintf(szfn,sizeof(szfn),"r%02d.jpg",i);
    FILE* out = fopen(szfn, "w");
    jpeg(out, rgb, camera->width, camera->height, 100);
    fclose(out);
    free(rgb);
  }
  camera_frame(camera, timeout);
  for(int i = 0;i<20;i++)
  {
    printf("%02X ",camera->head.start[i]);
  }
  printf("\n");
  unsigned char* rgb =
    yuyv2rgb(camera->head.start, camera->width, camera->height);
  FILE* out = fopen("result.jpg", "w");
  jpeg(out, rgb, camera->width, camera->height, 100);
  fclose(out);
  free(rgb);
 
  camera_stop(camera);
  camera_finish(camera);
  camera_close(camera);
  return 0;
}
 #endif
}