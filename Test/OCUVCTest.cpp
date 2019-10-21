#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include <iostream>

using namespace cv;
using namespace std;


int main()
{
	cout << "Built with OpenCV " << CV_VERSION << endl;

	VideoCapture capture(0);    // 打开摄像头
	std::cout << capture.getBackendName() << std::endl;
	std::cout << capture.getBackendName() << std::endl;
	std::cout << capture.getBackendName() << std::endl;
	capture.set(CV_CAP_PROP_FRAME_WIDTH, 320);//宽度

	capture.set(CV_CAP_PROP_FRAME_HEIGHT, 240);//高度
	if (!capture.isOpened())    // 判断是否打开成功
	{
		cout << "open camera failed. " << endl;
		return -1;
	}

	while (true)
	{
		Mat frame;
		capture >> frame;    // 读取图像帧至frame
		if (!frame.empty())	// 判断是否为空
		{
			imshow("camera", frame);
		}

		if (waitKey(30) > 0)		// delay 30 ms等待按键
		{
			break;
		}
	}

	return 0;
}
