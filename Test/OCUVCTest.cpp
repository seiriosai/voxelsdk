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

	VideoCapture capture(0);    // ������ͷ
	std::cout << capture.getBackendName() << std::endl;
	std::cout << capture.getBackendName() << std::endl;
	std::cout << capture.getBackendName() << std::endl;
	capture.set(CV_CAP_PROP_FRAME_WIDTH, 320);//���

	capture.set(CV_CAP_PROP_FRAME_HEIGHT, 240);//�߶�
	if (!capture.isOpened())    // �ж��Ƿ�򿪳ɹ�
	{
		cout << "open camera failed. " << endl;
		return -1;
	}

	while (true)
	{
		Mat frame;
		capture >> frame;    // ��ȡͼ��֡��frame
		if (!frame.empty())	// �ж��Ƿ�Ϊ��
		{
			imshow("camera", frame);
		}

		if (waitKey(30) > 0)		// delay 30 ms�ȴ�����
		{
			break;
		}
	}

	return 0;
}
