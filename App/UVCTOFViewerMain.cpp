#include "vtkCommand.h"
#include "vtkSliderWidget.h"
#include "vtkImagePlaneWidget.h"
#include "vtkOutlineFilter.h"
#include "vtkVolume16Reader.h"
#include "vtkPolyDataMapper.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkCellPicker.h"
#include "vtkSliderRepresentation2D.h"
#include "vtkProperty.h"
#include "vtkImageActor.h"
#include "vtkCamera.h"
#include "vtkCameraActor.h"
#include "vtkImageMapToColors.h"
#include "vtkSmartPointer.h"
#include "vtkTransform.h"
#include "vtkAxesActor.h"
#include <vtkImageViewer2.h>
#include <vtkJPEGReader.h>
#include <vtkBMPReader.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <thread>
class vtkSliderCallback : public vtkCommand
{
public:
	static vtkSliderCallback *New()
	{
		return new vtkSliderCallback;
	}
	virtual void Execute(vtkObject *caller, unsigned long, void*)
	{
		vtkSliderWidget *sliderWidget =
			reinterpret_cast<vtkSliderWidget*>(caller);
		if (!sliderWidget) return;
		int value = static_cast<int>(static_cast<vtkSliderRepresentation *>
			(sliderWidget->GetRepresentation())->GetValue());

		if (this->WidgetX)
		{
			this->WidgetX->SetSliceIndex(value);
		}
		else if (this->WidgetY)
		{
			this->WidgetY->SetSliceIndex(value);
		}
		else if (this->WidgetZ)
		{
			this->WidgetZ->SetSliceIndex(value);
		}
	}

	vtkSliderCallback() :WidgetX(0), WidgetY(0), WidgetZ(0) {}

	vtkImagePlaneWidget* WidgetX;
	vtkImagePlaneWidget* WidgetY;
	vtkImagePlaneWidget* WidgetZ;
};

class vtkWidgetWindowLevelCallback : public vtkCommand
{
public:
	static vtkWidgetWindowLevelCallback *New()
	{
		return new vtkWidgetWindowLevelCallback;
	}

	void Execute(vtkObject *caller, unsigned long vtkNotUsed(event),
		void *callData)
	{
		vtkImagePlaneWidget* self =
			reinterpret_cast<vtkImagePlaneWidget*>(caller);
		if (!self) return;

		double* wl = static_cast<double*>(callData);

		if (self == this->WidgetX)
		{
			this->WidgetY->SetWindowLevel(wl[0], wl[1]);
			this->WidgetZ->SetWindowLevel(wl[0], wl[1]);
		}
		else if (self == this->WidgetY)
		{
			this->WidgetX->SetWindowLevel(wl[0], wl[1]);
			this->WidgetZ->SetWindowLevel(wl[0], wl[1]);
		}
		else if (self == this->WidgetZ)
		{
			this->WidgetX->SetWindowLevel(wl[0], wl[1]);
			this->WidgetY->SetWindowLevel(wl[0], wl[1]);
		}
	}

	vtkWidgetWindowLevelCallback() :WidgetX(0), WidgetY(0), WidgetZ(0) {}

	vtkImagePlaneWidget* WidgetX;
	vtkImagePlaneWidget* WidgetY;
	vtkImagePlaneWidget* WidgetZ;
};

extern int TOFViewer();
unsigned data[10000] = { 0 };

vtkRenderWindow* pren;
void thread1(int x){
	std::cout << "x= " << x << std::endl;
	boost::this_thread::sleep_for(boost::chrono::milliseconds(1000));
	TOFViewer();
}
#include "vtkImageCanvasSource2D.h"
int main()
{
	
	
	TOFViewer();
	
	while (true)
	{
		std::this_thread::sleep_for(std::chrono::microseconds(10000));

	}


	return 0;
}
