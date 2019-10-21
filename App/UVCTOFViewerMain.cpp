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

int main0()
{
	///文件读取的操作  
	//TOFViewer();
	vtkVolume16Reader* pReader = vtkVolume16Reader::New();
	pReader->SetDataDimensions(64, 64);
	pReader->SetDataByteOrderToLittleEndian();
	pReader->SetImageRange(1, 93);
	pReader->SetDataSpacing(3.2, 3.2, 1.5);
	//pReader->SetFilePrefix("D:\\VTKITK\\vtkdata-5.6.0\\Data\\headsq\\quarter");
	pReader->SetFilePrefix("D:\\headsq\\quarter");

	pReader->SetDataMask(0x7fff);
	pReader->Update();

	vtkOutlineFilter *DicomOutline = vtkOutlineFilter::New();
	DicomOutline->SetInputConnection(pReader->GetOutputPort());

	vtkPolyDataMapper *pMapper = vtkPolyDataMapper::New();
	pMapper->SetInputConnection(DicomOutline->GetOutputPort());

	vtkActor *DicomActor = vtkActor::New();
	DicomActor->SetMapper(pMapper);

	vtkRenderer *ren0 = vtkRenderer::New();
	vtkRenderer *ren1 = vtkRenderer::New();
	vtkRenderer *ren2 = vtkRenderer::New();
	vtkRenderer *ren3 = vtkRenderer::New();

	vtkRenderWindow *renWin = vtkRenderWindow::New();
	vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();

	renWin->AddRenderer(ren1);
	renWin->AddRenderer(ren2);
	renWin->AddRenderer(ren3);
	renWin->AddRenderer(ren0);
	iren->SetRenderWindow(renWin);

	vtkCellPicker *picker = vtkCellPicker::New();
	picker->SetTolerance(0.005);

	vtkImagePlaneWidget *planeWidgetX = vtkImagePlaneWidget::New();
	planeWidgetX->SetInteractor(iren);
	planeWidgetX->SetKeyPressActivationValue('x');
	planeWidgetX->SetPicker(picker);
	planeWidgetX->RestrictPlaneToVolumeOn();
	planeWidgetX->GetPlaneProperty()->SetColor(0.0, 0.0, 1.0);
	planeWidgetX->DisplayTextOn();
	//planeWidgetX->TextureInterpolateOn();
	planeWidgetX->TextureInterpolateOff();
	planeWidgetX->SetResliceInterpolateToLinear();
	planeWidgetX->SetInputData((vtkDataSet*)pReader->GetOutput());
	planeWidgetX->SetPlaneOrientationToXAxes();//冠
	planeWidgetX->SetSliceIndex(32);
	planeWidgetX->GetTexturePlaneProperty()->SetOpacity(1);
	planeWidgetX->On();

	vtkImagePlaneWidget *planeWidgetY = vtkImagePlaneWidget::New();
	planeWidgetY->SetInteractor(iren);
	planeWidgetY->SetKeyPressActivationValue('y');
	planeWidgetY->SetPicker(picker);
	planeWidgetY->RestrictPlaneToVolumeOn();
	planeWidgetY->GetPlaneProperty()->SetColor(1.0, 0.0, 0.0);
	planeWidgetY->DisplayTextOn();
	planeWidgetY->TextureInterpolateOn();
	planeWidgetY->SetResliceInterpolateToLinear();
	planeWidgetY->SetInputData((vtkDataSet*)pReader->GetOutput());
	planeWidgetY->SetPlaneOrientationToYAxes();//矢
	planeWidgetY->SetSliceIndex(25);
	planeWidgetY->On();

	vtkImagePlaneWidget *planeWidgetZ = vtkImagePlaneWidget::New();
	planeWidgetZ->SetInteractor(iren);
	planeWidgetZ->DisplayTextOn();
	planeWidgetZ->RestrictPlaneToVolumeOn();
	planeWidgetZ->SetKeyPressActivationValue('z');
	planeWidgetZ->SetPicker(picker);
	planeWidgetZ->GetPlaneProperty()->SetColor(0.0, 1.0, 0.0);
	planeWidgetZ->TextureInterpolateOn();
	planeWidgetZ->SetResliceInterpolateToLinear();
	planeWidgetZ->SetInputData((vtkDataSet*)pReader->GetOutput());
	planeWidgetZ->SetPlaneOrientationToZAxes(); //横断面
	planeWidgetZ->SetSliceIndex(50);
	planeWidgetZ->On();

	//////////////////////////////////////////////////////
	vtkWidgetWindowLevelCallback *cbk = vtkWidgetWindowLevelCallback::New();
	cbk->WidgetX = planeWidgetX;
	cbk->WidgetY = planeWidgetY;
	cbk->WidgetZ = planeWidgetZ;
	planeWidgetX->AddObserver(vtkCommand::EndWindowLevelEvent, cbk);
	planeWidgetY->AddObserver(vtkCommand::EndWindowLevelEvent, cbk);
	planeWidgetZ->AddObserver(vtkCommand::EndWindowLevelEvent, cbk);
	cbk->Delete();
	//////////////////////////////////////////////////////
	vtkSliderRepresentation2D *sliderRep1 = vtkSliderRepresentation2D::New();
	sliderRep1->SetMinimumValue(1);
	sliderRep1->SetMaximumValue(50);
	sliderRep1->SetValue(planeWidgetX->GetSliceIndex());
	sliderRep1->GetPoint1Coordinate()->SetCoordinateSystemToNormalizedDisplay();
	sliderRep1->GetPoint1Coordinate()->SetValue(.0, .02);
	sliderRep1->GetPoint2Coordinate()->SetCoordinateSystemToNormalizedDisplay();
	sliderRep1->GetPoint2Coordinate()->SetValue(1.0, .02);

	vtkSliderWidget *sliderWidget1 = vtkSliderWidget::New();
	sliderWidget1->SetInteractor(iren);
	sliderWidget1->SetRepresentation(sliderRep1);
	sliderWidget1->SetAnimationModeToAnimate();
	sliderWidget1->EnabledOn();

	vtkSliderCallback  *slidercbk = vtkSliderCallback::New();
	slidercbk->WidgetX = planeWidgetX;
	sliderWidget1->AddObserver(vtkCommand::InteractionEvent, slidercbk);
	//////////////////////////////////////////////////////
	//增加2D图像测试获取切片
	vtkImageMapToColors *colorMap1 = vtkImageMapToColors::New();
	colorMap1->SetInputConnection(pReader->GetOutputPort());
	
	//colorMap1->PassAlphaToOutputOff(); //use in RGBA
	//colorMap1->SetActiveComponent(0);
	//colorMap1->SetOutputFormatToLuminance();
	//colorMap1->SetInputData((vtkDataObject*)planeWidgetX->GetResliceOutput());
	//colorMap1->SetLookupTable((vtkScalarsToColors *)planeWidgetX->GetLookupTable());
	vtkImageActor *imageActor1 = vtkImageActor::New();
	imageActor1->PickableOff();
	//ren1->GetRenderWindow()->SetTileViewport(colorMap1->GetOutputPort());

	vtkImageMapToColors *colorMap2 = vtkImageMapToColors::New();
	colorMap2->PassAlphaToOutputOff();
	colorMap2->SetActiveComponent(0); // for muti-component
	colorMap2->SetOutputFormatToLuminance();
	colorMap2->SetInputData((vtkDataObject*)planeWidgetY->GetResliceOutput());
	colorMap2->SetLookupTable((vtkScalarsToColors *)planeWidgetX->GetLookupTable());
	vtkImageActor *imageActor2 = vtkImageActor::New();
	imageActor2->PickableOff();
	imageActor2->SetInputData(colorMap2->GetOutput());

	vtkImageMapToColors *colorMap3 = vtkImageMapToColors::New();
	colorMap3->PassAlphaToOutputOff();
	colorMap3->SetActiveComponent(0);
	colorMap3->SetOutputFormatToLuminance();
	colorMap3->SetInputData((vtkDataObject*)planeWidgetZ->GetResliceOutput());
	colorMap3->SetLookupTable((vtkScalarsToColors *)planeWidgetX->GetLookupTable());
	vtkImageActor *imageActor3 = vtkImageActor::New();
	imageActor3->PickableOff();
	imageActor3->SetInputData(colorMap3->GetOutput());
	/////////////////////////////////////////////////////////////////////
	vtkSmartPointer<vtkTransform> transform =
		vtkSmartPointer<vtkTransform>::New();
	transform->Translate(-10.0, -10.0, -10.0);

	vtkSmartPointer<vtkAxesActor> axes =
		vtkSmartPointer<vtkAxesActor>::New();

	axes->SetUserTransform(transform);

	ren0->AddActor(axes);
	//////////////////////////////////////////////////////

	ren0->AddActor(DicomActor); //outline  DicomActor
	ren1->AddActor(imageActor1);
	ren2->AddActor(imageActor2);
	ren3->AddActor(imageActor3);

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

	ren0->GetActiveCamera()->Dolly(1.15);
	ren1->GetActiveCamera()->Dolly(1.15);
	ren2->GetActiveCamera()->Dolly(1.15);
	ren3->GetActiveCamera()->Dolly(1.15);

	ren0->ResetCamera();
	ren1->ResetCamera();
	ren2->ResetCamera();
	ren3->ResetCamera();

	iren->Initialize();
	renWin->SetWindowName("vtkSuperViewer by winston");
	iren->Start();
	renWin->Render();

	return 0;
}
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
	/*
	vtkSmartPointer<vtkImageCanvasSource2D> canvas = vtkSmartPointer<vtkImageCanvasSource2D>::New();
	
	vtkSmartPointer<vtkJPEGReader> jpegReader = vtkSmartPointer<vtkJPEGReader>::New();
	jpegReader->SetFileName("C:\\tmp\\1.jpg");

	// Visualize
	vtkSmartPointer<vtkImageViewer2> imageViewer = vtkSmartPointer<vtkImageViewer2>::New();
	imageViewer->SetInputConnection(jpegReader->GetOutputPort());
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	imageViewer->SetupInteractor(renderWindowInteractor);
	imageViewer->Render();
	
	imageViewer->GetRenderer()->ResetCamera();
	imageViewer->Render();
	vtkBMPReader *r = vtkBMPReader::New();
	unsigned char p[10000] = { 0 };

	r->SetMemoryBuffer(p);
	
	imageViewer->SetInputConnection(r->GetOutputPort());
	
	//imageViewer->GetRenderWindow()->SetWindowName("read and show jpg test");
	//imageViewer->GetRenderWindow()->SetPixelData();

	renderWindowInteractor->Start();
	*/
	while (true)
	{
		std::this_thread::sleep_for(std::chrono::microseconds(10000));

	}
	vtkRenderer *ren0 = vtkRenderer::New();
	vtkRenderer *ren1 = vtkRenderer::New();
	vtkRenderer *ren2 = vtkRenderer::New();
	vtkRenderer *ren3 = vtkRenderer::New();

	vtkRenderWindow *renWin = vtkRenderWindow::New();
	vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();

	renWin->AddRenderer(ren1);
	renWin->AddRenderer(ren2);
	renWin->AddRenderer(ren3);
	renWin->AddRenderer(ren0);
	iren->SetRenderWindow(renWin);

	

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
	pren = ren0->GetRenderWindow();
	pren = renWin;
	

	ren0->GetActiveCamera()->Dolly(1.15);
	ren1->GetActiveCamera()->Dolly(1.15);
	ren2->GetActiveCamera()->Dolly(1.15);
	ren3->GetActiveCamera()->Dolly(1.15);

	ren0->ResetCamera();
	ren1->ResetCamera();
	ren2->ResetCamera();
	ren3->ResetCamera();

	iren->Initialize();
	renWin->SetWindowName("vtkSuperViewer by winston");
	
	boost::thread thrd1(boost::bind(&thread1, 1));
	
	iren->Start();
	renWin->Render();

	return 0;
}
